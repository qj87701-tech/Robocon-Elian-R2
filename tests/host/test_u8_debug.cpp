#include <cmath>
#include <cstdint>
#include <initializer_list>
#include "test_assert.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "3_Module/3swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};
int fake_sequence = 0;
Struct_Remote_Chassis_Command Remote_Chassis_Command{};

namespace
{
constexpr float RPM_TO_RAD_S = 0.10471975511965977f;
constexpr uint8_t Expected_Mask = TEST_EXPECTED_U8_MASK;
Class_Motor_VESC *const Motors[] = {&VESC_Chassis_LF, &VESC_Chassis_RF, &VESC_Chassis_LB};
Class_Motor_DM_Normal *const Steers[] = {&DM_Chassis_LF, &DM_Chassis_RF, &DM_Chassis_LB, &DM_Chassis_RB};
bool Fresh_Feedback = true;

void Near(float actual, float expected) { assert(std::fabs(actual - expected) < 0.002f); }

void Reset()
{
    for (auto *motor : Motors) { motor->Get_Fake_State() = {}; }
    for (auto *motor : Steers) { motor->Get_Fake_State() = {}; }
    Motor_Init(); Chassis_Init();
    for (auto *motor : Motors) { motor->Get_Fake_State().status_1_valid = true; }
    Remote_Chassis_Command = {};
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Mode = 1U;
    Remote_Chassis_Command.Motor_Enable = 1U;
    Fresh_Feedback = true;
}

void Cycle(float y)
{
    if (Fresh_Feedback) { for (auto *motor : Motors) { motor->Get_Fake_State().status_1_update_count++; } }
    const bool enabled = Remote_Chassis_Command.Online != 0U && Remote_Chassis_Command.Motor_Enable != 0U;
    Chassis_Set_Command(enabled ? Struct_Chassis_Command{16.0f, y, 30.0f, true, Chassis_Command_Source_DEBUG_U8} : Struct_Chassis_Command{});
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Chassis_VESC_10ms_PeriodElapsedCallback();
}

void All_Off()
{
    for (auto *motor : Motors) { assert(!motor->Get_Motor_Enable()); Near(motor->Get_Control_Omega(), 0.0f); }
    for (auto *motor : Steers) { assert(!motor->Get_Motor_Enable()); }
}

void Only_Selected(float expected_omega)
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        assert(Motors[i]->Get_Motor_Enable() == ((Expected_Mask & (1U << i)) != 0U));
        Near(Motors[i]->Get_Control_Omega(), (Expected_Mask & (1U << i)) != 0U ? expected_omega : 0.0f);
    }
    for (auto *motor : Steers)
    {
        assert(!motor->Get_Motor_Enable());
    }
}

void Test_ID_Selection_And_No_DM_Homing()
{
    Reset();
    for (auto *motor : Steers) { motor->Get_Fake_State().motor_enable = true; }
    VESC_Chassis_LF.Set_Motor_Enable(true); VESC_Chassis_LF.Set_Control_Omega(10.0f);
    Cycle(4.0f);
    Only_Selected(VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
    for (auto *motor : Steers) { assert(motor->Get_Fake_State().exit_count == 1); }
    for (auto *motor : Steers) { assert(motor->Get_Fake_State().enter_count == 0 && motor->Get_Fake_State().send_count == 0); }
    assert(!Chassis_Get_Swerve_Ready());
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if ((Expected_Mask & (1U << i)) == 0U) { Motors[i]->Get_Fake_State().status_1_valid = false; }
    }
    for (int i = 0; i < 60; i++) { Cycle(4.0f); }
    Only_Selected(4.0f / SWERVE_WHEEL_RADIUS_M);
}

void Test_Normal_Speed_Ceiling_And_Ramp()
{
    Reset();
    for (int i = 0; i < 200; i++) { Cycle(16.0f); }
    Only_Selected(1440.0f * RPM_TO_RAD_S);
    Cycle(0.0f);
    Only_Selected((1440.0f - VESC_DECEL_RPM_PER_S * 0.01f) * RPM_TO_RAD_S);
    Reset();
    Cycle(-4.0f);
    Only_Selected(-VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
}

// 回中先走完减速斜坡, 零速期间持续释放; 再次同向操作从零恢复RPM。
void Test_Neutral_Releases_Selected_U8_And_Restarts_From_Zero()
{
    for (float direction : {1.0f, -1.0f})
    {
        Reset();
        for (int i = 0; i < 60; i++) { Cycle(direction * 4.0f); }
        for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = direction * 60.0f * RPM_TO_RAD_S; }
        Cycle(0.0f);
        Only_Selected(direction * (4.0f / SWERVE_WHEEL_RADIUS_M - VESC_DECEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S));
        for (uint8_t i = 0U; i < 3U; i++)
        {
            if ((Expected_Mask & (1U << i)) != 0U) { assert(Motors[i]->Get_Fake_State().control_method == Motor_VESC_Control_Method_RPM); }
        }
        for (int cycle = 0; cycle < 600; cycle++)
        {
            if (cycle == 80) { for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 0; }
            Cycle(0.0f);
            for (uint8_t i = 0U; i < 3U; i++)
            {
                if ((Expected_Mask & (1U << i)) == 0U || Motors[i]->Get_Control_Omega() != 0.0f) { continue; }
                assert(Motors[i]->Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
                Near(Motors[i]->Get_Fake_State().control_current, 0.0f);
            }
        }
        assert(!Chassis_Get_Swerve_Fault());
        Only_Selected(0.0f);
        Cycle(direction * 4.0f);
        Only_Selected(direction * VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
        for (uint8_t i = 0U; i < 3U; i++)
        {
            if ((Expected_Mask & (1U << i)) != 0U) { assert(Motors[i]->Get_Fake_State().control_method == Motor_VESC_Control_Method_RPM); }
        }
    }
}


void Test_Neutral_Waits_For_Fresh_Stopped_Feedback()
{
    Reset();
    Cycle(4.0f);
    for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 40.0f * RPM_TO_RAD_S;
    for (int i = 0; i < 20; ++i) Cycle(0.0f);
    for (unsigned i = 0; i < 3; ++i)
    {
        if (!(Expected_Mask & (1U << i))) continue;
        assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
        assert(Motors[i]->Get_Control_Method() == Motor_VESC_Control_Method_CURRENT);
    }
    for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 0;
    Fresh_Feedback = false;
    for (int i = 0; i < 15; ++i) Cycle(-4.0f);
    for (unsigned i = 0; i < 3; ++i)
        if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Stable_Cycles == 0);
    Fresh_Feedback = true;
    for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES - 1; ++i) Cycle(-4.0f);
    for (auto *motor : Motors) motor->Get_Fake_State().now_omega = -40 * RPM_TO_RAD_S;
    Cycle(-4.0f); // 短暂反向漂移也须重新计数
    for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 0;
    for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES; ++i) Cycle(-4.0f);
    for (unsigned i = 0; i < 3; ++i)
        if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_STOPPED);
    Cycle(-4.0f);
    Only_Selected(-VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
}



void Test_Stop_Hysteresis_And_Restart_Guard()
{
    for (float direction : {1.0f, -1.0f})
    {
        Reset();
        Cycle(direction * 4.0f);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 15.0f * RPM_TO_RAD_S;
        for (int i = 0; i < 30; ++i) Cycle(0);
        for (unsigned i = 0; i < 3; ++i)
        {
            if (!(Expected_Mask & (1U << i))) continue;
            assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
            assert(U8_Control.Get_Drive(i).Stable_Cycles == 0);
        }
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 9.0f * RPM_TO_RAD_S;
        for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES - 1; ++i) Cycle(0);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 11.0f * RPM_TO_RAD_S;
        Cycle(0);
        for (unsigned i = 0; i < 3; ++i)
            if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Stable_Cycles == 0);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 10.0f * RPM_TO_RAD_S;
        for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES; ++i) Cycle(0);
        for (float rpm : {15.0f, 20.0f})
        {
            for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * rpm * RPM_TO_RAD_S;
            Cycle(0);
            for (unsigned i = 0; i < 3; ++i)
                if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_STOPPED);
        }
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 21.0f * RPM_TO_RAD_S;
        Cycle(0);
        for (unsigned i = 0; i < 3; ++i)
            if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 0;
        for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES; ++i) Cycle(0);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = direction * 15.0f * RPM_TO_RAD_S;
        Cycle(0);
        // 停稳状态允许灰区，但收到反向目标必须重新完成10RPM确认。
        Cycle(-direction * 4.0f);
        Only_Selected(0);
        for (unsigned i = 0; i < 3; ++i)
            if (Expected_Mask & (1U << i)) assert(U8_Control.Get_Drive(i).Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
        for (auto *motor : Motors) motor->Get_Fake_State().now_omega = 0;
        Fresh_Feedback = false;
        for (int i = 0; i < 15; ++i) Cycle(-direction * 4.0f);
        Only_Selected(0);
        Fresh_Feedback = true;
        for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES; ++i) Cycle(-direction * 4.0f);
        Cycle(-direction * 4.0f);
        Only_Selected(-direction * VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
    }
}

void Test_Optional_Brake_Is_Low_Speed_Only()
{
    Reset();
    Struct_U8_Control_Parameter parameter;
    parameter.Low_Speed_Brake_Enable = true;
    U8_Control.Init(parameter);
    auto &state = VESC_Chassis_LF.Get_Fake_State();
    U8_Control.Set_Speed(Swerve_Wheel_F, 20 * RPM_TO_RAD_S);
    state.now_omega = 150 * RPM_TO_RAD_S;
    for (int i = 0; i < 5; ++i) { ++state.status_1_update_count; U8_Control.Set_Speed(Swerve_Wheel_F, 0); }
    assert(state.control_method == Motor_VESC_Control_Method_CURRENT);
    state.now_omega = 50 * RPM_TO_RAD_S;
    ++state.status_1_update_count; U8_Control.Set_Speed(Swerve_Wheel_F, 0);
    assert(state.control_method == Motor_VESC_Control_Method_CURRENT_BRAKE);
    Near(state.control_current, VESC_LOW_SPEED_BRAKE_CURRENT_A);
    state.now_omega = 0;
    for (unsigned i = 0; i < VESC_REVERSE_STABLE_CYCLES; ++i)
    { ++state.status_1_update_count; U8_Control.Set_Speed(Swerve_Wheel_F, 0); }
    assert(state.control_method == Motor_VESC_Control_Method_CURRENT);
    Near(state.control_current, 0);
}

void Test_Reverse_Requires_New_Stopped_Feedback()
{
    Reset();
    for (int i = 0; i < 60; i++) { Cycle(4.0f); }
    for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = 100.0f * RPM_TO_RAD_S; }
    for (int i = 0; i < 100; i++) { Cycle(-4.0f); }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if ((Expected_Mask & (1U << i)) == 0U) { continue; }
        assert(Motors[i]->Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
        assert(Chassis_Swerve_Diagnostic[i].Drive_Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
        Near(Motors[i]->Get_Fake_State().control_current, 0.0f);
    }
    for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = 0.0f; }
    Fresh_Feedback = false;
    for (int i = 0; i < 20; i++) { Cycle(-4.0f); }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if ((Expected_Mask & (1U << i)) != 0U) { assert(Chassis_Swerve_Diagnostic[i].Drive_Stop_Stable_Cycles == 0U); }
    }
    Fresh_Feedback = true;
    for (uint8_t i = 0U; i < VESC_REVERSE_STABLE_CYCLES; i++) { Cycle(-4.0f); }
    Cycle(-4.0f);
    Only_Selected(-VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
}

void Test_Unfinished_Reverse_Survives_All_Mode_Transitions()
{
    // 0 MANUAL回零, 1 AUTO尚无有效速度, 2退出控制页, 3遥控离线。
    for (uint8_t transition = 0U; transition < 4U; transition++)
    {
        Reset();
        for (int i = 0; i < 60; i++) { Cycle(4.0f); }
        for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = 100.0f * RPM_TO_RAD_S; }
        for (int i = 0; i < 100; i++) { Cycle(-4.0f); }
        for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = 0.0f; }
        Cycle(-4.0f); // 仅收到一次新的停稳反馈, 还远不足10次。
        Fresh_Feedback = false;
        Remote_Chassis_Command.Mode = transition == 0U ? REMOTE_MODE_MANUAL : (transition == 1U ? REMOTE_MODE_AUTOMATIC : 0U);
        if (transition == 3U) { Remote_Chassis_Command.Online = 0U; Remote_Chassis_Command.Motor_Enable = 0U; }
        Chassis_Set_Command(transition == 0U ? Struct_Chassis_Command{0.0f, -4.0f, 0.0f, true, Chassis_Command_Source_MANUAL} : Struct_Chassis_Command{});
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        Chassis_VESC_10ms_PeriodElapsedCallback();
        Remote_Chassis_Command.Mode = REMOTE_MODE_DEBUG;
        Remote_Chassis_Command.Online = 1U; Remote_Chassis_Command.Motor_Enable = 1U;
        Cycle(-4.0f);
        for (uint8_t i = 0U; i < 3U; i++)
        {
            if ((Expected_Mask & (1U << i)) == 0U) { continue; }
            assert(Chassis_Swerve_Diagnostic[i].Drive_Status == Chassis_Swerve_Drive_Status_WAIT_STOP);
            assert(Chassis_Swerve_Diagnostic[i].Drive_Stop_Stable_Cycles == 1U);
            assert(Chassis_Swerve_Diagnostic[i].Drive_Wait_Elapsed_Time >= 1000U);
            assert(Motors[i]->Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
            Near(Motors[i]->Get_Control_Omega(), 0.0f);
        }
        Fresh_Feedback = true;
        for (uint8_t i = 1U; i < VESC_REVERSE_STABLE_CYCLES; i++) { Cycle(-4.0f); }
        Cycle(-4.0f);
        Only_Selected(-VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
    }
}

void Test_Feedback_Fault_Is_Global_And_Only_Enable_Off_Clears()
{
    Reset(); Cycle(4.0f);
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if ((Expected_Mask & (1U << i)) != 0U) { Motors[i]->Get_Fake_State().status_1_valid = false; break; }
    }
    Cycle(4.0f); All_Off();
    assert(Chassis_Get_Swerve_Fault());
    for (auto *motor : Motors) { motor->Get_Fake_State().status_1_valid = true; }
    Cycle(4.0f); All_Off();
    Remote_Chassis_Command.Online = 0U; Remote_Chassis_Command.Motor_Enable = 0U;
    Cycle(0.0f); All_Off();
    Remote_Chassis_Command.Online = 1U; Remote_Chassis_Command.Motor_Enable = 1U;
    Cycle(4.0f); All_Off();
    // 切回正常遥控/上位机也不能绕过ID2或ID3的U8故障锁。
    for (auto source : {Chassis_Command_Source_MANUAL, Chassis_Command_Source_UPPER})
    {
        Chassis_Set_Command({0.0f, 4.0f, 0.0f, true, source});
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        Chassis_VESC_10ms_PeriodElapsedCallback();
        All_Off();
        assert(Chassis_Get_Swerve_Fault());
    }
    Remote_Chassis_Command.Motor_Enable = 0U; Cycle(0.0f);
    assert(!Chassis_Get_Swerve_Fault());
    Remote_Chassis_Command.Motor_Enable = 1U; Cycle(4.0f);
    Only_Selected(VESC_ACCEL_RPM_PER_S * 0.01f * RPM_TO_RAD_S);
}

void Test_Reverse_Timeout_And_Exit_Keep_Stopping_Test_Motors()
{
    Reset();
    for (int i = 0; i < 60; i++) { Cycle(4.0f); }
    for (auto *motor : Motors) { motor->Get_Fake_State().now_omega = 100.0f * RPM_TO_RAD_S; }
    for (uint32_t i = 0U; i < VESC_REVERSE_TIMEOUT_CYCLES + 5U; i++) { Cycle(-4.0f); }
    assert(Chassis_Get_Swerve_Fault()); All_Off();

    Reset();
    for (int i = 0; i < 20; i++) { Cycle(4.0f); }
    Remote_Chassis_Command.Mode = 0U; Remote_Chassis_Command.Motor_Enable = 0U;
    int counts[3]{};
    for (uint8_t i = 0U; i < 3U; i++) { counts[i] = Motors[i]->Get_Fake_State().send_count; }
    for (int i = 0; i < 5; i++) { Cycle(0.0f); All_Off(); }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if ((Expected_Mask & (1U << i)) != 0U) { assert(Motors[i]->Get_Fake_State().send_count == counts[i] + 5); }
    }
}
}

int main()
{
    if (Expected_Mask == 0U)
    {
        Reset();
        for (int i = 0; i < 20; i++) { Cycle(16.0f); All_Off(); }
        return 0;
    }
    Test_Stop_Hysteresis_And_Restart_Guard();
    Test_Optional_Brake_Is_Low_Speed_Only();
    Test_Neutral_Waits_For_Fresh_Stopped_Feedback();
    Test_Neutral_Releases_Selected_U8_And_Restarts_From_Zero();
    Test_ID_Selection_And_No_DM_Homing();
    Test_Unfinished_Reverse_Survives_All_Mode_Transitions();
    Test_Normal_Speed_Ceiling_And_Ramp();
    Test_Reverse_Requires_New_Stopped_Feedback();
    Test_Feedback_Fault_Is_Global_And_Only_Enable_Off_Clears();
    Test_Reverse_Timeout_And_Exit_Keep_Stopping_Test_Motors();
    return 0;
}
