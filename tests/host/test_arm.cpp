#include "test_assert.h"
#include <cmath>
#include <initializer_list>
#include <limits>
#include "3_Module/Arm/mod_arm_control.h"

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};
int fake_sequence = 0;
static std::uint32_t tick = 0;
std::uint32_t HAL_GetTick() { return tick; }

static void Feedback()
{
    for (auto motor : {&DM_Arm_Big, &DM_Arm_Small, &DM_Arm_Rotate})
    {
        auto &s = motor->Get_Fake_State();
        ++s.rx_update_count;
        s.control_status = Motor_DM_Control_Status_ENABLE;
        s.now_angle = s.control_angle;
        s.now_omega = 0.0f;
    }
}

int main()
{
    float y, z, a, b;
    Class_Arm_Kinematics::Forward(0.0f, 1.57079632679f, y, z);
    assert(std::fabs(y - 400.0f) < 0.01f && std::fabs(z - 324.0f) < 0.01f);
    assert(Class_Arm_Kinematics::Inverse(400.0f, 324.0f, 0.0f, 1.57f, a, b));
    assert(std::fabs(a) < 0.001f && std::fabs(b - 1.5707963f) < 0.001f);
    assert(!Class_Arm_Kinematics::Inverse(800.0f, 0.0f, 0, 0, a, b));
    assert(!Class_Arm_Kinematics::Inverse(NAN, 0.0f, 0, 0, a, b));
    assert(!Class_Arm_Kinematics::Inverse(724.0f, 0.0f, 0, 0, a, b));
    Motor_Arm_Init();
    assert(DM_Arm_Big.Get_Fake_State().init_tx_id == 6U);
    assert(DM_Arm_Small.Get_Fake_State().init_rx_id == 0x77U);
    Class_Arm_Control arm;
    arm.Init();
    assert(arm.Set_Target_Point(1) == Arm_Result_INVALID_POINT);
    assert(arm.Set_Target_Position(400, 324) == Arm_Result_NOT_READY);
    Struct_Arm_Parameter p;
    p.Calibration_Ready = true;
    p.Big_Direction = p.Small_Direction = 1.0f;
    p.Big_Zero = 1.0f; // 配置用度，电机反馈用rad
    p.Big_Min = p.Small_Min = -120.0f;
    p.Big_Max = p.Small_Max = 120.0f;
    p.Relative_Min = -140.0f; p.Relative_Max = 140.0f;
    p.Rotate_Ready = true;
    p.Rotate_Min = -90.0f;
    p.Rotate_Max = 90.0f;
    DM_Arm_Big.Set_Control_Angle(0.0174532925f);
    DM_Arm_Small.Set_Control_Angle(0.2f); // 从正肘角开始，固定测试使用的逆解分支
    arm.Init(p);
    Feedback(); arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Set_Enable(true) == Arm_Result_ACCEPTED);
    assert(std::fabs(DM_Arm_Big.Get_Control_Omega() - 0.0174533f) < 0.000001f);
    // Enter之后允许反馈确认延迟，不能立即故障。
    DM_Arm_Big.Get_Fake_State().control_status = Motor_DM_Control_Status_DISABLE;
    tick += 10; arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Get_Status() == Arm_Status_IDLE);
    Feedback(); tick += 10; arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Set_Target_Position(400, 324) == Arm_Result_ACCEPTED);
    assert(arm.Set_Target_Position(450, 200) == Arm_Result_BUSY);
    float previous = DM_Arm_Small.Get_Control_Angle(), omega = 0.0f;
    for (int i = 0; i < 15000 && arm.Get_Status() == Arm_Status_MOVING; ++i)
    {
        Feedback(); tick += 10; arm.TIM_10ms_PeriodElapsedCallback();
        float next = DM_Arm_Small.Get_Control_Angle();
        float speed = (next - previous) / 0.01f;
        assert(std::fabs(speed) < 0.0175f);
        assert(std::fabs(speed - omega) < 0.0019f);
        previous = next; omega = speed;
    }
    assert(arm.Get_Status() == Arm_Status_ARRIVED);
    assert(std::fabs(DM_Arm_Big.Get_Control_Angle() - 0.0174532925f) < 0.000001f);
    assert(arm.Set_Rotate_Target(2.0f, 0.08f) == Arm_Result_INVALID_TARGET);
    assert(arm.Set_Rotate_Target(1.0f, 0.09f) == Arm_Result_INVALID_TARGET);
    assert(arm.Set_Rotate_Target(1.0f, 0.08f) == Arm_Result_ACCEPTED);
    assert(arm.Set_Target_Position(800, 0) == Arm_Result_INVALID_TARGET);
    tick += 250; arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Get_Status() == Arm_Status_FAULT);
    assert(!DM_Arm_Big.Get_Motor_Enable());
    assert(arm.Set_Enable(true) == Arm_Result_NOT_READY);
    arm.Set_Enable(false);
    Feedback(); arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Set_Enable(true) == Arm_Result_ACCEPTED);
    p.Speed_Max = 0.5f;
    arm.Init(p);
    assert(!DM_Arm_Big.Get_Motor_Enable());
    // 即使位置相同，反馈速度过大也不能到位，规划后3s超时退出。
    Feedback(); arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Set_Enable(true) == Arm_Result_ACCEPTED);
    assert(std::fabs(DM_Arm_Big.Get_Control_Omega() - 0.00872665f) < 0.000001f);
    assert(arm.Set_Target_Position(400, 324) == Arm_Result_ACCEPTED);
    for (int i = 0; i < 310; ++i)
    {
        Feedback(); DM_Arm_Big.Get_Fake_State().now_omega = 0.1f;
        tick += 10; arm.TIM_10ms_PeriodElapsedCallback();
    }
    assert(arm.Get_Status() == Arm_Status_FAULT);
    arm.Set_Enable(false);
    Feedback(); arm.TIM_10ms_PeriodElapsedCallback();
    assert(arm.Set_Enable(true) == Arm_Result_ACCEPTED);
    // 新反馈仍报告失能，使能确认超时后锁定故障。
    for (int i = 0; i < 21; ++i)
    {
        Feedback(); DM_Arm_Big.Get_Fake_State().control_status = Motor_DM_Control_Status_DISABLE;
        tick += 10; arm.TIM_10ms_PeriodElapsedCallback();
    }
    assert(arm.Get_Status() == Arm_Status_FAULT);
    return 0;
}
