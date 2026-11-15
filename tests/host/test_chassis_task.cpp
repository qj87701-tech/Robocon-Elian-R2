#include <cmath>
#include <cstdint>
#include <cstdio>
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
bool Keep_Drive_Feedback_Fresh = true;
constexpr float Accel_RPM_Step = VESC_ACCEL_RPM_PER_S * 0.01f;
constexpr float Decel_RPM_Step = VESC_DECEL_RPM_PER_S * 0.01f;

Struct_Chassis_Command Test_Manual_Command()
{
    return {Remote_Chassis_Command.Speed_X, Remote_Chassis_Command.Speed_Y, Remote_Chassis_Command.Speed_W,
            Remote_Chassis_Command.Online != 0U && Remote_Chassis_Command.Motor_Enable != 0U && Remote_Chassis_Command.Mode == REMOTE_MODE_MANUAL, Chassis_Command_Source_MANUAL};
}
void Run_Chassis_Cycle()
{
    if (Keep_Drive_Feedback_Fresh)
    {
        Class_Motor_VESC *motors[] = {&VESC_Chassis_LF, &VESC_Chassis_RF, &VESC_Chassis_LB};
        for (auto *motor : motors) { ++motor->Get_Fake_State().status_1_update_count; }
    }
    Chassis_Set_Command(Test_Manual_Command());
    ::Chassis_Swerve_10ms_PeriodElapsedCallback();
}
void Assert_Near(const float value, const float expected)
{
    if (std::fabs(value - expected) >= 1.0e-4f)
    {
        std::fprintf(stderr, "Expected %.6f, got %.6f\n", static_cast<double>(expected), static_cast<double>(value));
    }
    assert(std::fabs(value - expected) < 1.0e-4f);
}

float Get_LF_Drive_RPM()
{
    return VESC_Chassis_LF.Get_Fake_State().control_omega * 9.54929659f;
}

void Assert_LF_Drive_RPM(const float expected)
{
    const float actual = Get_LF_Drive_RPM();
    if (std::fabs(actual - expected) >= 0.001f)
    {
        std::fprintf(stderr, "Expected drive %.4f rpm, got %.4f rpm\n", static_cast<double>(expected), static_cast<double>(actual));
    }
    assert(std::fabs(actual - expected) < 0.001f);
}

void Reset_Chassis()
{
    fake_sequence = 0;
    DM_Chassis_LF.Get_Fake_State() = {};
    DM_Chassis_RF.Get_Fake_State() = {};
    DM_Chassis_LB.Get_Fake_State() = {};
    DM_Chassis_RB.Get_Fake_State() = {};
    VESC_Chassis_LF.Get_Fake_State() = {};
    VESC_Chassis_RF.Get_Fake_State() = {};
    VESC_Chassis_LB.Get_Fake_State() = {};
    Motor_Init();
    Keep_Drive_Feedback_Fresh = true;
    VESC_Chassis_LF.Get_Fake_State().status_1_valid = true;
    VESC_Chassis_RF.Get_Fake_State().status_1_valid = true;
    VESC_Chassis_LB.Get_Fake_State().status_1_valid = true;
    Chassis_Init();
    Remote_Chassis_Command = {};
}

void Set_Valid_Command(const float speed_x, const float speed_y, const float speed_w)
{
    Remote_Chassis_Command.Speed_X = speed_x;
    Remote_Chassis_Command.Speed_Y = speed_y;
    Remote_Chassis_Command.Speed_W = speed_w;
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Mode = 2U;
    Remote_Chassis_Command.Motor_Enable = 1U;
}

void Set_LF_Target_RPM(const float rpm)
{
    Set_Valid_Command(0.0f, rpm * 0.01047197551f, 0.0f); // 轮半径0.1m, 将机械rpm转换为m/s
}

void Run_Swerve_Cycles(const int cycles)
{
    for (int i = 0; i < cycles; i++)
    {
        Run_Chassis_Cycle();
    }
}

void Check_LF_Ramp_Cycles(const int cycles, float &previous_rpm)
{
    for (int i = 0; i < cycles; i++)
    {
        Run_Chassis_Cycle();
        const float rpm = Get_LF_Drive_RPM();
        const float acceleration = (rpm - previous_rpm) / 0.01f;
        assert(std::isfinite(rpm));
        assert(std::fabs(rpm) <= 1440.01f);
        assert(std::fabs(acceleration) <= std::fmax(VESC_ACCEL_RPM_PER_S, VESC_DECEL_RPM_PER_S) + 0.1f);
        previous_rpm = rpm;
    }
}

void Feed_DM(Class_Motor_DM_Normal &motor, const float angle, const float omega)
{
    motor.Get_Fake_State().now_total_angle = angle;
    motor.Get_Fake_State().now_omega = omega;
    motor.Get_Fake_State().total_angle_initialized = true;
    ++motor.Get_Fake_State().rx_update_count;
}

void Feed_LF_DM(const float angle, const float omega)
{
    Feed_DM(DM_Chassis_LF, angle, omega);
    // 其余两组提供正常的零位反馈, 单独考察前中轮的回零和速度斜坡。
    Feed_DM(DM_Chassis_RF, 0.0f, 0.0f);
    Feed_DM(DM_Chassis_LB, 0.0f, 0.0f);
}

void Reach_LF_Homing_Ready()
{
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Run_Chassis_Cycle();
    for (int i = 0; i < 10; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Run_Chassis_Cycle();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
    Run_Swerve_Cycles(VESC_REVERSE_STABLE_CYCLES + 1); // 等待零目标的新反馈停稳确认
}

void Assert_All_Targets_Zero()
{
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_RB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 0.0f);
}


void Test_100ms_Callback_Checks_Three_Active_Swerve_Pairs()
{
    Reset_Chassis();
    Chassis_100ms_PeriodElapsedCallback();

    assert(DM_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_LB.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().alive_count == 0);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().alive_count == 1);
    assert(VESC_Chassis_LB.Get_Fake_State().alive_count == 1);
}


void Test_Swerve_Three_DM_And_VESC_Pairs_Are_Active()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Run_Chassis_Cycle();

    assert(DM_Chassis_LF.Get_Fake_State().enter_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == 1);
    assert(DM_Chassis_LB.Get_Fake_State().enter_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().enter_count == 0);
    assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(DM_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().send_count == 1);
    assert(DM_Chassis_LB.Get_Fake_State().send_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().send_count == 0);
}

void Test_Three_Wheel_Command_Uses_Confirmed_Radius_And_ID_Order()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Run_Swerve_Cycles(20);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 10.0f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 10.0f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 10.0f);

    DM_Chassis_LF.Get_Fake_State().now_total_angle = -1.57079633f;
    DM_Chassis_RF.Get_Fake_State().now_total_angle = -0.52359878f;
    DM_Chassis_LB.Get_Fake_State().now_total_angle = 0.52359878f;
    Set_Valid_Command(0.0f, 0.0f, 2.0f);
    Run_Swerve_Cycles(40);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, -1.57079633f);
    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_angle, -0.52359878f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_angle, 0.52359878f);
    // 边长0.45m对应外接圆半径0.259807621m; W=2rad/s、轮半径0.1m。
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 5.19615242f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, -5.19615242f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 5.19615242f);
    assert(DM_Chassis_LF.Get_Fake_State().init_tx_id == 1U);
    assert(DM_Chassis_RF.Get_Fake_State().init_tx_id == 2U);
    assert(DM_Chassis_LB.Get_Fake_State().init_tx_id == 3U);
    assert(VESC_Chassis_LF.Get_Fake_State().vesc_id == 1U);
    assert(VESC_Chassis_RF.Get_Fake_State().vesc_id == 2U);
    assert(VESC_Chassis_LB.Get_Fake_State().vesc_id == 3U);
    assert(DM_Chassis_RB.Get_Fake_State().send_count == 0);

    Remote_Chassis_Command.Online = 0U;
    Run_Swerve_Cycles(1);
    Assert_All_Targets_Zero();
    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LB.Get_Fake_State().motor_enable);
}

void Test_Any_Unhomed_Wheel_Blocks_All_Three_Drive_Motors()
{
    Class_Motor_DM_Normal *motors[] = {&DM_Chassis_LF, &DM_Chassis_RF, &DM_Chassis_LB};
    for (uint8_t missing_wheel = 0U; missing_wheel < 3U; missing_wheel++)
    {
        Reset_Chassis();
        Set_Valid_Command(0.0f, 1.0f, 0.0f);
        Run_Swerve_Cycles(1);
        for (uint8_t cycle = 0U; cycle < 10U; cycle++)
        {
            for (uint8_t wheel = 0U; wheel < 3U; wheel++)
            {
                if (wheel != missing_wheel)
                {
                    Feed_DM(*motors[wheel], 0.0f, 0.0f);
                }
            }
            Run_Swerve_Cycles(1);
            Assert_All_Targets_Zero();
        }
        assert(Chassis_Swerve_Diagnostic[missing_wheel].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
        for (uint8_t cycle = 0U; cycle < 10U; cycle++)
        {
            Feed_DM(*motors[missing_wheel], 0.0f, 0.0f);
            Run_Swerve_Cycles(1);
        }
        Run_Swerve_Cycles(20);
        Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 10.0f);
        Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 10.0f);
        Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 10.0f);
    }
}

void Test_Enable_Waits_For_A_New_DM_Feedback_Frame()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);

    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);

    Feed_LF_DM(0.2f, 0.0f);
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);
}

void Test_Homing_Holds_VESC_At_Zero()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Run_Chassis_Cycle();
    Feed_LF_DM(1.0f, 2.0f);
    Run_Chassis_Cycle();

    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
}

void Test_Homing_Requires_Ten_Consecutive_Stable_Cycles()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Run_Chassis_Cycle();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Run_Chassis_Cycle();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);

    Feed_LF_DM(0.04f, 0.4f);
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
}

void Test_Homing_Stability_Count_Resets_When_Error_Returns()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Run_Chassis_Cycle();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Run_Chassis_Cycle();
    }
    Feed_LF_DM(0.06f, 0.4f);
    Run_Chassis_Cycle();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Run_Chassis_Cycle();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);

    Feed_LF_DM(0.04f, 0.4f);
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
}

void Test_Homing_Reaches_Ready_And_Uses_Direct_Drive_Angle()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Feed_LF_DM(1.57079633f, 1.0f);
    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Run_Chassis_Cycle();

    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Target_Angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Now_Angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Now_Omega, 1.0f);
}

void Test_Homing_Times_Out_After_Three_Seconds()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 300; i++)
    {
        Run_Chassis_Cycle();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_TIMEOUT);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
}

void Test_Disable_Then_Enable_Retries_After_Timeout()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 300; i++)
    {
        Run_Chassis_Cycle();
    }
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_DISABLED);

    Remote_Chassis_Command.Motor_Enable = 1U;
    Run_Chassis_Cycle();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    assert(DM_Chassis_LF.Get_Fake_State().enter_count == 2);
}

void Test_MIT_RAM_Setters_Validate_Ranges_And_Apply_To_Active_Motor()
{
    Reset_Chassis();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_F, 8.5f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_F, 0.35f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_F, 0.8f));
    assert(!Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_F, 50.1f));
    assert(!Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_F, 5.1f));
    assert(!Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_F, -2.1f));
    assert(!Chassis_Swerve_Set_MIT_K_P(static_cast<Enum_Swerve_Wheel>(Swerve_Wheel_Number), 1.0f));

    Reach_LF_Homing_Ready();
    Run_Chassis_Cycle();
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
}

void Test_Control_Class_Callbacks_Preserve_Homing_Ramp_And_Offline_Stop()
{
    Reset_Chassis();
    Class_Chassis_Control chassis;
    chassis.Init();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    chassis.Set_Command(Test_Manual_Command());
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    for (int i = 0; i < 10; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        chassis.Set_Command(Test_Manual_Command());
        chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
    for (unsigned i = 0; i <= VESC_REVERSE_STABLE_CYCLES; ++i)
    {
        ++VESC_Chassis_LF.Get_Fake_State().status_1_update_count;
        ++VESC_Chassis_RF.Get_Fake_State().status_1_update_count;
        ++VESC_Chassis_LB.Get_Fake_State().status_1_update_count;
        chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    }
    assert(chassis.Set_MIT_K_P(Swerve_Wheel_F, 8.5f));
    assert(chassis.Set_MIT_K_D(Swerve_Wheel_F, 0.35f));
    assert(chassis.Set_MIT_Feedforward_Torque(Swerve_Wheel_F, 0.8f));
    assert(!chassis.Set_MIT_K_P(Swerve_Wheel_F, 50.1f));
    Set_LF_Target_RPM(200.0f);
    chassis.Set_Command(Test_Manual_Command());
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    Assert_LF_Drive_RPM(Accel_RPM_Step);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
    chassis.TIM_10ms_VESC_PeriodElapsedCallback();
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 1);
    chassis.TIM_100ms_PeriodElapsedCallback();
    assert(DM_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().alive_count == 1);
    Remote_Chassis_Command.Online = 0U;
    chassis.Set_Command(Test_Manual_Command());
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    Assert_LF_Drive_RPM(0.0f);
    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
}

void Test_MIT_Parameters_Are_Isolated_Between_Wheels()
{
    Reset_Chassis();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_LB, 12.0f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_RB, 0.8f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_RB, -0.6f));
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 7.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.2f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Assert_Near(DM_Chassis_RF.Get_Fake_State().k_p, 12.0f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().k_d, 0.8f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_torque, -0.6f);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_LB.Get_Fake_State().enter_count == 1);
}

void Test_Disable_Preserves_MIT_Tuning_But_Init_Restores_Defaults()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_F, 8.5f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_F, 0.35f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_F, 0.8f));
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(200.0f);
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(0.0f);
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
    Chassis_Init();
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 7.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.2f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(Accel_RPM_Step);
}

void Test_Swerve_Drive_Linear_Ramp_Reaches_Target_Without_Startup_Delay()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(Accel_RPM_Step); // 首周期立即按配置的加速度增加目标转速
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(2.0f * Accel_RPM_Step);
    Run_Swerve_Cycles(98);
    Assert_LF_Drive_RPM(std::fmin(1200.0f, 100.0f * Accel_RPM_Step));
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(1200.0f);
}

void Test_Swerve_Drive_Negative_Start_Uses_The_Same_Acceleration()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(-1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(-std::fmin(1200.0f, 50.0f * Accel_RPM_Step));
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(-std::fmin(1200.0f, 100.0f * Accel_RPM_Step));
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(-1200.0f);
}

void Test_Swerve_Drive_Stop_And_Restart_Use_Linear_Ramp()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(120);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(0.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(200.0f - Decel_RPM_Step);
    Run_Swerve_Cycles(19);
    Assert_LF_Drive_RPM(0.0f);
    Run_Swerve_Cycles(VESC_REVERSE_STABLE_CYCLES);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(Accel_RPM_Step);
}

void Test_Swerve_Drive_Does_Not_Overshoot_A_Small_Target()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_Valid_Command(0.0f, 0.20f, 0.0f); // 高于运动进入阈值, 半径0.1m对应轮速2rad/s
    for (int i = 0; i < 80; i++)
    {
        Run_Chassis_Cycle();
        assert(VESC_Chassis_LF.Get_Fake_State().control_omega <= 2.00001f);
    }
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 2.0f);
}

void Test_Swerve_Drive_Nonzero_Speed_Changes_Use_Linear_Ramp()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(200);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(200.0f + Accel_RPM_Step);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(200.0f + 2.0f * Accel_RPM_Step);
    Run_Swerve_Cycles(18);
    Assert_LF_Drive_RPM(400.0f);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(400.0f - Decel_RPM_Step);
    Run_Swerve_Cycles(19);
    Assert_LF_Drive_RPM(200.0f);
}

void Test_Swerve_Drive_Lower_Target_Decelerates_On_The_Next_Cycle()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(50.0f * Accel_RPM_Step);
    Set_LF_Target_RPM(1400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(51.0f * Accel_RPM_Step);
    Set_LF_Target_RPM(400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(51.0f * Accel_RPM_Step - Decel_RPM_Step); // 降低目标后立即减速
    Run_Swerve_Cycles(40);
    Assert_LF_Drive_RPM(400.0f);
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(10);
    Assert_LF_Drive_RPM(400.0f + 10.0f * Accel_RPM_Step);
    Set_LF_Target_RPM(0.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(400.0f + 10.0f * Accel_RPM_Step - Decel_RPM_Step); // 加速途中回中立即减速
}

void Test_Swerve_Drive_Acceleration_Bounds_During_Retargets()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    float previous_rpm = 0.0f;
    Set_LF_Target_RPM(1200.0f);
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Set_LF_Target_RPM(200.0f);
    Check_LF_Ramp_Cycles(230, previous_rpm);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(1400.0f);
    Check_LF_Ramp_Cycles(50, previous_rpm);
    Set_LF_Target_RPM(210.0f);
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Assert_LF_Drive_RPM(210.0f);
    const float targets[] = {300.0f, 250.0f, 450.0f, 350.0f, 600.0f, 300.0f};
    for (float target : targets)
    {
        Set_LF_Target_RPM(target);
        Check_LF_Ramp_Cycles(15, previous_rpm);
    }
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Assert_LF_Drive_RPM(300.0f);
    Set_LF_Target_RPM(0.0f);
    Check_LF_Ramp_Cycles(150, previous_rpm);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Rapid_Retargets_Keep_Bounded_Continuous_Output()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    float previous_rpm = 0.0f;
    std::uint32_t seed = 0x12345678U;
    for (int i = 0; i < 300; i++)
    {
        seed = seed * 1664525U + 1013904223U;
        const float target = static_cast<float>(static_cast<int>(seed % 2801U) - 1400);
        Set_LF_Target_RPM(target);
        Check_LF_Ramp_Cycles(7 + static_cast<int>(seed % 23U), previous_rpm);
    }
    Set_LF_Target_RPM(0.0f);
    Check_LF_Ramp_Cycles(500, previous_rpm);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Steering_Stop_Immediately_Starts_Ramp_Down()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(50.0f * Accel_RPM_Step);
    DM_Chassis_LF.Get_Fake_State().now_total_angle = 1.0f; // 舵向误差超过45°, 限速比例必须为0
    Run_Swerve_Cycles(1);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Speed_Ratio, 0.0f);
    Assert_LF_Drive_RPM(50.0f * Accel_RPM_Step - Decel_RPM_Step); // 舵向保护触发后立即减速
    Run_Swerve_Cycles(150);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Invalid_Command_Resets_Ramp()
{
    for (int reason = 0; reason < 3; reason++)
    {
        Reset_Chassis();
        Reach_LF_Homing_Ready();
        Set_Valid_Command(0.0f, 10.0f, 0.0f);
        for (int i = 0; i < 60; i++)
        {
            Run_Chassis_Cycle();
        }
        if (reason == 0) { Remote_Chassis_Command.Online = 0U; }
        if (reason == 1) { Remote_Chassis_Command.Motor_Enable = 0U; }
        if (reason == 2) { Remote_Chassis_Command.Mode = 1U; }
        Run_Chassis_Cycle();
        Assert_LF_Drive_RPM(0.0f);
        assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
        Reach_LF_Homing_Ready();
        Set_Valid_Command(0.0f, 10.0f, 0.0f);
        Run_Chassis_Cycle();
        Assert_LF_Drive_RPM(Accel_RPM_Step);
    }
}

void Test_Swerve_Drive_Reverses_Only_After_Ramping_To_Zero()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(120);
    Set_LF_Target_RPM(-200.0f);
    float previous_rpm = Get_LF_Drive_RPM();
    bool crossed_zero = false;
    for (int i = 0; i < 250; i++)
    {
        Run_Swerve_Cycles(1);
        const float rpm = Get_LF_Drive_RPM();
        if (!crossed_zero && rpm < -0.001f)
        {
            Assert_Near(previous_rpm, 0.0f); // 必须至少有一次零速下发, 再向相反方向起步
            assert(std::fabs(rpm + Accel_RPM_Step) < 0.001f);
            crossed_zero = true;
        }
        previous_rpm = rpm;
    }
    assert(crossed_zero);
    Assert_LF_Drive_RPM(-200.0f);
}

// 复现故障: 目标已经归零, 但反馈仍正转时不得下发负转速; 两个方向均检查。
void Test_Swerve_Reversal_Waits_For_Actual_Stop()
{
    const float directions[] = {1.0f, -1.0f};
    for (float direction : directions)
    {
        Reset_Chassis();
        Reach_LF_Homing_Ready();
        auto &feedback = VESC_Chassis_LF.Get_Fake_State();
        feedback.status_1_valid = true;
        Set_LF_Target_RPM(direction * 200.0f);
        Run_Swerve_Cycles(20);
        feedback.now_omega = direction * 200.0f * 0.10471975512f;
        Set_LF_Target_RPM(-direction * 200.0f);
        for (int i = 0; i < 80; i++)
        {
            ++feedback.status_1_update_count;
            Run_Chassis_Cycle();
            assert(Get_LF_Drive_RPM() * direction >= -0.001f);
        }
        Assert_LF_Drive_RPM(0.0f);
        assert(feedback.control_method == Motor_VESC_Control_Method_CURRENT);
        Assert_Near(feedback.control_current, 0.0f);

        feedback.now_omega = 0.0f;
        Run_Swerve_Cycles(9);
        Assert_LF_Drive_RPM(0.0f);
        // 高于10rpm的一帧必须打断稳定计数，即使仍处于10~20rpm灰区。
        feedback.now_omega = direction * 11.0f * 0.10471975512f;
        Run_Swerve_Cycles(1);
        feedback.now_omega = direction * 9.0f * 0.10471975512f;
        Run_Swerve_Cycles(9);
        Assert_LF_Drive_RPM(0.0f);
        Run_Swerve_Cycles(1);
        Assert_LF_Drive_RPM(0.0f); // 最后一帧确认时仍保持释放。
        Run_Swerve_Cycles(1);
        Assert_LF_Drive_RPM(-direction * Accel_RPM_Step);
        assert(feedback.control_method == Motor_VESC_Control_Method_RPM);
    }
}

void Start_LF_Reversal_Wait()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(20);
    VESC_Chassis_LF.Get_Fake_State().now_omega = 200.0f * 0.10471975512f;
    Set_LF_Target_RPM(-200.0f);
    Run_Swerve_Cycles(21); // 包含浮点斜坡归零后的等待状态入口。
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Reversal_Does_Not_Count_Cached_Zero_Feedback()
{
    Start_LF_Reversal_Wait();
    auto &feedback = VESC_Chassis_LF.Get_Fake_State();
    feedback.now_omega = 0.0f;
    Run_Swerve_Cycles(1);
    Keep_Drive_Feedback_Fresh = false;
    Run_Swerve_Cycles(30);
    Assert_LF_Drive_RPM(0.0f);
    assert(feedback.control_method == Motor_VESC_Control_Method_CURRENT);
    Keep_Drive_Feedback_Fresh = true;
    Run_Swerve_Cycles(9);
    Assert_LF_Drive_RPM(0.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(-Accel_RPM_Step);
}

void Test_Reversal_Timeout_Latches_And_Disables_All_Wheels()
{
    Start_LF_Reversal_Wait();
    Run_Swerve_Cycles(478);
    assert(!Chassis_Get_Swerve_Fault()); // 499个10ms周期仍可等待。
    assert(VESC_Chassis_LF.Get_Motor_Enable());
    Run_Swerve_Cycles(1); // 第500周期锁定, 不再无限等待。
    assert(Chassis_Get_Swerve_Fault());
    assert(!Chassis_Get_Swerve_Ready());
    assert(!VESC_Chassis_LF.Get_Motor_Enable());
    assert(!VESC_Chassis_RF.Get_Motor_Enable());
    assert(!VESC_Chassis_LB.Get_Motor_Enable());
    assert(!DM_Chassis_LF.Get_Motor_Enable());
    VESC_Chassis_LF.Get_Fake_State().now_omega = 0.0f;
    Run_Swerve_Cycles(20);
    assert(Chassis_Get_Swerve_Fault());
    Assert_LF_Drive_RPM(0.0f);
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Chassis_Cycle();
    assert(!Chassis_Get_Swerve_Fault());
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(VESC_REVERSE_STABLE_CYCLES + 1); // 普通零目标也须先确认停稳
    Set_LF_Target_RPM(-200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(-Accel_RPM_Step);
}

void Test_Reversal_Feedback_Loss_And_Nonfinite_Disable()
{
    for (int reason = 0; reason < 2; reason++)
    {
        Start_LF_Reversal_Wait();
        auto &feedback = VESC_Chassis_LF.Get_Fake_State();
        if (reason == 0) { feedback.status_1_valid = false; }
        else { feedback.now_omega = NAN; }
        Run_Swerve_Cycles(1);
        assert(Chassis_Get_Swerve_Fault());
        assert(!feedback.motor_enable);
        Assert_LF_Drive_RPM(0.0f);
        feedback.status_1_valid = true;
        feedback.now_omega = 0.0f;
        Run_Swerve_Cycles(20);
        assert(!feedback.motor_enable); // 反馈恢复不能自行重启。
    }
}

void Test_Reversal_Uses_Latest_Command_And_Remembers_Coasting_Direction()
{
    Start_LF_Reversal_Wait();
    Set_LF_Target_RPM(0.0f);
    VESC_Chassis_LF.Get_Fake_State().now_omega = 0.0f;
    Run_Swerve_Cycles(30);
    Assert_LF_Drive_RPM(0.0f); // 等待期间回中, 停稳后不能执行旧反转请求。

    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(20);
    VESC_Chassis_LF.Get_Fake_State().now_omega = 200.0f * 0.10471975512f;
    Set_LF_Target_RPM(0.0f);
    Run_Swerve_Cycles(25);
    Set_LF_Target_RPM(-200.0f);
    Run_Swerve_Cycles(30);
    Assert_LF_Drive_RPM(0.0f); // 目标回中不等于实际停稳。
}

void Test_Reversal_Offline_Stop_And_Reenable_While_Coasting()
{
    Start_LF_Reversal_Wait();
    Remote_Chassis_Command.Online = 0U;
    Run_Chassis_Cycle();
    assert(!VESC_Chassis_LF.Get_Motor_Enable());
    Assert_LF_Drive_RPM(0.0f);
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(-200.0f);
    Run_Swerve_Cycles(30);
    // 失能清除了方向记忆, 但真实反馈仍正转, 重新使能也不能直接反向。
    Assert_LF_Drive_RPM(0.0f);
    assert(VESC_Chassis_LF.Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
}

void Test_No_Drive_Feedback_Blocks_First_Drive_After_Homing()
{
    Reset_Chassis();
    VESC_Chassis_LF.Get_Fake_State().status_1_valid = false;
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    assert(Chassis_Get_Swerve_Fault());
    assert(!VESC_Chassis_LF.Get_Motor_Enable());
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Auto_Stop_And_Offline_Do_Not_Acknowledge_Drive_Fault()
{
    Start_LF_Reversal_Wait();
    Run_Swerve_Cycles(479);
    assert(Chassis_Get_Swerve_Fault());
    assert(Remote_Chassis_Command.Motor_Enable == 1U);
    Chassis_Set_Command({}); // 模拟上位机速度故障退出或切AUTO页, 遥控总使能仍开。
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Get_Swerve_Fault());
    Remote_Chassis_Command.Online = 0U;
    Remote_Chassis_Command.Motor_Enable = 0U; // 遥控掉线路径也会清许可, 不等于用户确认。
    Run_Chassis_Cycle();
    assert(Chassis_Get_Swerve_Fault());
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Motor_Enable = 1U;
    Run_Chassis_Cycle();
    assert(Chassis_Get_Swerve_Fault());
    assert(!VESC_Chassis_LF.Get_Motor_Enable());
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Chassis_Cycle();
    assert(!Chassis_Get_Swerve_Fault());
}

void Test_Swerve_Drive_Waits_Until_Steer_Is_Near_Target()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_Valid_Command(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 60; i++)
    {
        Run_Chassis_Cycle();
    }
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);

    DM_Chassis_LF.Get_Fake_State().now_total_angle = 1.57079633f;
    Run_Chassis_Cycle();
    Assert_LF_Drive_RPM(Accel_RPM_Step); // 舵向等待结束后从零速按线性斜坡起步
}

void Test_Swerve_Diagonal_Command_Normalizes_All_Wheels_To_1440_RPM()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    DM_Chassis_LF.Get_Fake_State().now_total_angle = 0.78539816f;
    Set_Valid_Command(15.1f, 15.1f, 0.0f);

    for (int i = 0; i < 500; i++)
    {
        Run_Chassis_Cycle();
    }

    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 150.79644737f);
}

void Test_Swerve_Rejects_Invalid_Remote_Commands()
{
    Reset_Chassis();
    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Online = 0U;
    Run_Chassis_Cycle();
    Assert_All_Targets_Zero();

    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Mode = 1U;
    Run_Chassis_Cycle();
    Assert_All_Targets_Zero();

    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Chassis_Cycle();
    Assert_All_Targets_Zero();
}

void Test_10ms_VESC_Callback_Sends_Only_Active_VESC_Object()
{
    Reset_Chassis();
    Chassis_VESC_10ms_PeriodElapsedCallback();

    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 1);
}
}

static void Test_Auto_Command_Uses_Only_Selected_Command()
{
    Reset_Chassis();
    Chassis_Set_Command({0.0f, 0.2f, 0.0f, true, Chassis_Command_Source_UPPER});
    ::Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(!Chassis_Get_Swerve_Ready());
    for (int i = 0; i < 10; i++) { Feed_LF_DM(0.0f, 0.0f); ::Chassis_Swerve_10ms_PeriodElapsedCallback(); }
    assert(Chassis_Get_Swerve_Ready() && !Chassis_Get_Swerve_Fault());
    for (int i = 0; i < 20; i++) { ::Chassis_Swerve_10ms_PeriodElapsedCallback(); }
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 2.0f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 2.0f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 2.0f);
    Chassis_Set_Command({}); ::Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_All_Targets_Zero(); assert(!Chassis_Get_Swerve_Ready());
}

// MANUAL和上位机共用回中停车: 所有活动轮减至零后释放, 不清除滑行方向记忆。
void Test_Neutral_Releases_All_Active_Wheels_For_Manual_And_Upper()
{
    const Enum_Chassis_Command_Source sources[] = {Chassis_Command_Source_MANUAL, Chassis_Command_Source_UPPER};
    Class_Motor_VESC *const motors[] = {&VESC_Chassis_LF, &VESC_Chassis_RF, &VESC_Chassis_LB};
    for (auto source : sources)
    {
        for (float direction : {1.0f, -1.0f})
        {
            Reset_Chassis();
            Reach_LF_Homing_Ready();
            auto cycle = [source](float speed_y)
            {
                ++VESC_Chassis_LF.Get_Fake_State().status_1_update_count;
                ++VESC_Chassis_RF.Get_Fake_State().status_1_update_count;
                ++VESC_Chassis_LB.Get_Fake_State().status_1_update_count;
                Chassis_Set_Command({0.0f, speed_y, 0.0f, true, source});
                Chassis_Swerve_10ms_PeriodElapsedCallback();
                Chassis_VESC_10ms_PeriodElapsedCallback();
            };
            for (int i = 0; i < 120; i++) { cycle(direction * 2.0f); }
            const float previous_omega = VESC_Chassis_LF.Get_Control_Omega();
            for (auto *motor : motors) { motor->Get_Fake_State().now_omega = direction * 60.0f * 0.10471975512f; }
            cycle(0.0f);
            Assert_Near(VESC_Chassis_LF.Get_Control_Omega(), previous_omega - direction * VESC_DECEL_RPM_PER_S * 0.01f * 0.10471975512f);
            for (int i = 0; i < 120; i++) { cycle(0.0f); }
            for (auto *motor : motors)
            {
                assert(motor->Get_Motor_Enable());
                Assert_Near(motor->Get_Control_Omega(), 0.0f);
                assert(motor->Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
                Assert_Near(motor->Get_Fake_State().control_current, 0.0f);
            }
            // 仍在原方向滑行时反向操作, 必须保留既有的实际停稳等待。
            cycle(-direction * 2.0f);
            for (auto *motor : motors)
            {
                assert(motor->Get_Fake_State().control_method == Motor_VESC_Control_Method_CURRENT);
                Assert_Near(motor->Get_Control_Omega(), 0.0f);
                motor->Get_Fake_State().now_omega = 0.0f;
            }
            for (uint8_t i = 0U; i < VESC_REVERSE_STABLE_CYCLES; i++) { cycle(-direction * 2.0f); }
            cycle(-direction * 2.0f);
            for (auto *motor : motors)
            {
                assert(motor->Get_Fake_State().control_method == Motor_VESC_Control_Method_RPM);
                Assert_Near(motor->Get_Control_Omega(), -direction * VESC_ACCEL_RPM_PER_S * 0.01f * 0.10471975512f);
            }
        }
    }
}

int main()
{
    Test_Neutral_Releases_All_Active_Wheels_For_Manual_And_Upper();
    Test_Swerve_Reversal_Waits_For_Actual_Stop();
    Test_Reversal_Does_Not_Count_Cached_Zero_Feedback();
    Test_Reversal_Timeout_Latches_And_Disables_All_Wheels();
    Test_Reversal_Feedback_Loss_And_Nonfinite_Disable();
    Test_Reversal_Uses_Latest_Command_And_Remembers_Coasting_Direction();
    Test_Reversal_Offline_Stop_And_Reenable_While_Coasting();
    Test_No_Drive_Feedback_Blocks_First_Drive_After_Homing();
    Test_Auto_Stop_And_Offline_Do_Not_Acknowledge_Drive_Fault();
    Test_Auto_Command_Uses_Only_Selected_Command();
    Test_Three_Wheel_Command_Uses_Confirmed_Radius_And_ID_Order();
    Test_Any_Unhomed_Wheel_Blocks_All_Three_Drive_Motors();
    Test_Control_Class_Callbacks_Preserve_Homing_Ramp_And_Offline_Stop();
    Test_MIT_Parameters_Are_Isolated_Between_Wheels();
    Test_Disable_Preserves_MIT_Tuning_But_Init_Restores_Defaults();
    Test_Swerve_Drive_Linear_Ramp_Reaches_Target_Without_Startup_Delay();
    Test_Swerve_Drive_Steering_Stop_Immediately_Starts_Ramp_Down();
    Test_Swerve_Drive_Negative_Start_Uses_The_Same_Acceleration();
    Test_Swerve_Drive_Stop_And_Restart_Use_Linear_Ramp();
    Test_Swerve_Drive_Does_Not_Overshoot_A_Small_Target();
    Test_Swerve_Drive_Nonzero_Speed_Changes_Use_Linear_Ramp();
    Test_Swerve_Drive_Lower_Target_Decelerates_On_The_Next_Cycle();
    Test_Swerve_Drive_Acceleration_Bounds_During_Retargets();
    Test_Swerve_Drive_Rapid_Retargets_Keep_Bounded_Continuous_Output();
    Test_Swerve_Drive_Invalid_Command_Resets_Ramp();
    Test_100ms_Callback_Checks_Three_Active_Swerve_Pairs();
    Test_Swerve_Three_DM_And_VESC_Pairs_Are_Active();
    Test_Enable_Waits_For_A_New_DM_Feedback_Frame();
    Test_Homing_Holds_VESC_At_Zero();
    Test_Homing_Requires_Ten_Consecutive_Stable_Cycles();
    Test_Homing_Stability_Count_Resets_When_Error_Returns();
    Test_Homing_Reaches_Ready_And_Uses_Direct_Drive_Angle();
    Test_Homing_Times_Out_After_Three_Seconds();
    Test_Disable_Then_Enable_Retries_After_Timeout();
    Test_MIT_RAM_Setters_Validate_Ranges_And_Apply_To_Active_Motor();
    Test_Swerve_Drive_Reverses_Only_After_Ramping_To_Zero();
    Test_Swerve_Drive_Waits_Until_Steer_Is_Near_Target();
    Test_Swerve_Diagonal_Command_Normalizes_All_Wheels_To_1440_RPM();
    Test_Swerve_Rejects_Invalid_Remote_Commands();
    Test_10ms_VESC_Callback_Sends_Only_Active_VESC_Object();
    return 0;
}
