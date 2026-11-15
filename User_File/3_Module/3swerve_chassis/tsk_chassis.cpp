/**
 * @file tsk_chassis.cpp
 * @brief 底盘控制任务
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 三舵轮底盘任务：回零、解算调用、DM控制及U8速度斜坡和反转保护。
 * @note Task_Init中初始化，前台每10ms更新控制并发送，100ms检查在线状态。
 * @note 常改参数在同名头文件：活动轮组、DEBUG测试ID、U8加减速及回零/MIT参数。
 * @note 运动学和暂定几何见tsk_chassis_resolve；正常与DEBUG共用同一U8保护状态。
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_chassis.h"

#include "4_Task/tsk_remote/tsk_remote.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 生产调度只使用此控制对象, 不在头文件中定义全局实例
static Class_Chassis_Control Chassis_Control;
static Class_U8_Test U8_Test_Control;

/* Exported variables --------------------------------------------------------*/

// 对外兼容的诊断快照, 不替代类内部的控制状态
Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number] = {};
Class_U8_Control U8_Control;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化底盘控制
 */
void Class_Chassis_Control::Init()
{
    Final_Command = {};
    Struct_U8_Control_Parameter parameter;
    parameter.Accel_RPM_Per_S = Chassis_Swerve_Config.Accel_RPM_Per_S;
    parameter.Decel_RPM_Per_S = Chassis_Swerve_Config.Decel_RPM_Per_S;
    parameter.Wheel_Omega_Max = Chassis_Swerve_Config.Wheel_Omega_Max;
    parameter.Control_Period_MS = Chassis_Swerve_Config.Control_Period_MS;
    parameter.Send_Wheel_Mask = Chassis_Swerve_Config.Active_Wheel_Mask;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        parameter.Drive_Direction[i] = Swerve_Wheel_Config[i].Drive_Direction;
    }
    U8_Control.Init(parameter);
    U8_Test_Init();
    Chassis_Swerve.Init(Chassis_Swerve_Config.Wheel_Radius, Chassis_Swerve_Config.Chassis_Radius, Chassis_Swerve_Config.Steer_Angle_Limit);
    Chassis_Swerve.Set_Wheel_Omega_Max(Chassis_Swerve_Config.Wheel_Omega_Max);
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Chassis_Swerve_Diagnostic[i] = {};
        Swerve_Wheel_Control[i] = {};
        Swerve_MIT_Parameter[i] = {DM_CHASSIS_MIT_K_P_DEFAULT, DM_CHASSIS_MIT_K_D_DEFAULT, DM_CHASSIS_MIT_TORQUE_DEFAULT};
    }
}

/**
 * @brief 接收控制入口生成的唯一车体速度指令, 不直接发送电机报文
 *
 * @param command 车体X/Y速度m/s、角速度rad/s及总使能
 * @note 任一速度为NaN或Inf时清空命令, 后续控制周期按失能路径处理
 */
void Class_Chassis_Control::Set_Command(const Struct_Chassis_Command &command)
{
    Final_Command = command;
    if (!isfinite(command.Speed_X) || !isfinite(command.Speed_Y) || !isfinite(command.Speed_W)) { Final_Command = {}; }
}

/**
 * @brief 检查启用轮组的内部回零状态是否全部就绪
 * @return bool 至少启用一个轮组且启用轮组均为READY时返回true
 * @note 读取控制状态而非VOFA诊断快照, 不触发回零或电机发送
 */
bool Class_Chassis_Control::Get_Swerve_Ready() const
{
    if (U8_Control.Get_Fault()) { return false; }
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        if ((Chassis_Swerve_Config.Active_Wheel_Mask & (1U << i)) != 0U && Swerve_Wheel_Control[i].Homing.Status != Chassis_Swerve_Homing_Status_READY) { return false; }
    }
    return Chassis_Swerve_Config.Active_Wheel_Mask != 0U;
}

/**
 * @brief 检查启用轮组的回零超时及行进保护故障
 * @return bool 回零超时、行进反馈失效或反转超时时返回true
 * @note 不读取VESC内部fault码, 电调硬件故障仍需通过VESC Tool诊断
 */
bool Class_Chassis_Control::Get_Swerve_Fault() const
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        if ((Chassis_Swerve_Config.Active_Wheel_Mask & (1U << i)) != 0U && Swerve_Wheel_Control[i].Homing.Status == Chassis_Swerve_Homing_Status_TIMEOUT) { return true; }
    }
    return U8_Control.Get_Fault();
}

/**
 * @brief 向唯一底盘对象提交本周期车体命令
 * @param command 控制入口生成的最终速度与使能, 不从此入口发送CAN
 */
void Chassis_Set_Command(const Struct_Chassis_Command &command) { Chassis_Control.Set_Command(command); }

/**
 * @brief 向任务层提供启用轮组的回零就绪状态
 * @return bool 所有启用轮组均就绪时返回true
 */
bool Chassis_Get_Swerve_Ready() { return Chassis_Control.Get_Swerve_Ready(); }

/**
 * @brief 向任务层提供启用轮组的回零及行进保护故障状态
 * @return bool 任一启用轮组存在故障时返回true
 */
bool Chassis_Get_Swerve_Fault() { return Chassis_Control.Get_Swerve_Fault(); }

/**
 * @brief 获取轮组对应的DM6220对象
 *
 * @param Wheel 舵轮轮组编号
 * @return Class_Motor_DM_Normal* 对应舵向电机
 */
Class_Motor_DM_Normal *Class_Chassis_Control::Swerve_Get_Steer_Motor(Enum_Swerve_Wheel Wheel)
{
    return (Wheel < Swerve_Wheel_Number) ? Swerve_Wheel_Config[Wheel].Steer_Motor : nullptr;
}

/**
 * @brief 获取轮组对应的VESC对象
 */
Class_Motor_VESC *Class_Chassis_Control::Swerve_Get_Drive_Motor(Enum_Swerve_Wheel Wheel)
{
    return (Wheel < Swerve_Wheel_Number) ? Swerve_Wheel_Config[Wheel].Drive_Motor : nullptr;
}

/**
 * @brief 判断轮组是否已接入本轮硬件测试
 */
bool Class_Chassis_Control::Swerve_Is_Wheel_Active(Enum_Swerve_Wheel Wheel)
{
    return ((Chassis_Swerve_Config.Active_Wheel_Mask & (1U << static_cast<uint8_t>(Wheel))) != 0U);
}

/**
 * @brief 将DM连续电机轴角度换算为舵轮机构连续角度
 */
float Class_Chassis_Control::Swerve_Motor_To_Mechanism_Angle(Enum_Swerve_Wheel Wheel, float Motor_Angle)
{
    return (Motor_Angle * Swerve_Wheel_Config[Wheel].Steer_Direction - Swerve_Wheel_Config[Wheel].Steer_Offset);
}

/**
 * @brief 将舵轮机构目标角度换算为DM多圈电机轴目标角度
 */
float Class_Chassis_Control::Swerve_Mechanism_To_Motor_Angle(Enum_Swerve_Wheel Wheel, float Mechanism_Angle)
{
    return ((Mechanism_Angle + Swerve_Wheel_Config[Wheel].Steer_Offset) * Swerve_Wheel_Config[Wheel].Steer_Direction);
}

/**
 * @brief 将DM电机轴角速度换算为舵轮机构角速度
 */
float Class_Chassis_Control::Swerve_Motor_To_Mechanism_Omega(Enum_Swerve_Wheel Wheel, float Motor_Omega)
{
    return (Motor_Omega * Swerve_Wheel_Config[Wheel].Steer_Direction);
}

/**
 * @brief 设置单个DM6220的MIT位置增益
 *
 * @param Wheel 舵轮轮组编号
 * @param K_P MIT位置增益, 0.0~50.0
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P)
{
    if (Wheel >= Swerve_Wheel_Number || K_P < Chassis_Swerve_Config.MIT_Limit.K_P_Min || K_P > Chassis_Swerve_Config.MIT_Limit.K_P_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].K_P = K_P;
    return (true);
}

/**
 * @brief 设置单个DM6220的MIT速度阻尼增益
 *
 * @param Wheel 舵轮轮组编号
 * @param K_D MIT速度阻尼增益, 0.0~5.0
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D)
{
    if (Wheel >= Swerve_Wheel_Number || K_D < Chassis_Swerve_Config.MIT_Limit.K_D_Min || K_D > Chassis_Swerve_Config.MIT_Limit.K_D_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].K_D = K_D;
    return (true);
}

/**
 * @brief 设置单个DM6220的MIT前馈转矩
 *
 * @param Wheel 舵轮轮组编号
 * @param Torque 前馈转矩, -2.0~2.0N·m
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque)
{
    if (Wheel >= Swerve_Wheel_Number || Torque < Chassis_Swerve_Config.MIT_Limit.Torque_Min || Torque > Chassis_Swerve_Config.MIT_Limit.Torque_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].Feedforward_Torque = Torque;
    return (true);
}

/**
 * @brief 回零期间持续发送保存零点后的0rad目标
 */
void Class_Chassis_Control::Swerve_Hold_Homing_Target(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    motor->Set_Control_Angle(Swerve_Mechanism_To_Motor_Angle(Wheel, 0.0f));
    motor->Set_Control_Omega(0.0f);
    motor->Set_Control_Torque(0.0f);
    motor->Set_K_P(Swerve_MIT_Parameter[Wheel].K_P);
    motor->Set_K_D(Swerve_MIT_Parameter[Wheel].K_D);
}

/**
 * @brief 更新单个活动舵轮的10ms回零状态机
 *
 * @return true 回零完成, 可以进入正常底盘控制
 */
bool Class_Chassis_Control::Swerve_Update_Homing(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    Struct_Chassis_Swerve_Homing_Control &homing = Swerve_Wheel_Control[Wheel].Homing;

    if (homing.Status == Chassis_Swerve_Homing_Status_DISABLED)
    {
        homing.Feedback_Update_Count = motor->Get_Rx_Update_Count();
        homing.Elapsed_Cycles = 1U;
        homing.Stable_Cycles = 0U;
        homing.Status = Chassis_Swerve_Homing_Status_WAIT_FEEDBACK;
        if (!motor->Get_Motor_Enable())
        {
            motor->CAN_Send_Enter();
        }
        Swerve_Hold_Homing_Target(Wheel);
        return (false);
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_READY)
    {
        return (true);
    }

    if (homing.Elapsed_Cycles < Chassis_Swerve_Config.Homing.Timeout_Cycles)
    {
        homing.Elapsed_Cycles++;
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK && motor->Get_Rx_Update_Count() != homing.Feedback_Update_Count)
    {
        homing.Status = Chassis_Swerve_Homing_Status_HOMING;
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_HOMING)
    {
        if (fabsf(motor->Get_Now_Total_Angle()) <= Chassis_Swerve_Config.Homing.Position_Limit && fabsf(motor->Get_Now_Omega()) <= Chassis_Swerve_Config.Homing.Omega_Limit)
        {
            homing.Stable_Cycles++;
        }
        else
        {
            homing.Stable_Cycles = 0U;
        }

        if (homing.Stable_Cycles >= Chassis_Swerve_Config.Homing.Stable_Cycles)
        {
            // DM上位机已保存的机械零点始终是0rad, 不得用回零完成时的实际角度重定义零点
            Chassis_Swerve.Reset_Wheel(Wheel, 0.0f);
            homing.Status = Chassis_Swerve_Homing_Status_READY;
            return (true);
        }
    }

    if (homing.Elapsed_Cycles >= Chassis_Swerve_Config.Homing.Timeout_Cycles)
    {
        homing.Status = Chassis_Swerve_Homing_Status_TIMEOUT;
        homing.Stable_Cycles = 0U;
    }

    Swerve_Hold_Homing_Target(Wheel);
    return (false);
}

/**
 * @brief 更新单个舵轮的低频诊断快照
 */
void Class_Chassis_Control::Swerve_Update_Diagnostic(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    Struct_Chassis_Swerve_Diagnostic &diagnostic = Chassis_Swerve_Diagnostic[Wheel];
    diagnostic.Steer_Target_Angle = Chassis_Swerve.Get_Target_Angle(Wheel);
    diagnostic.Steer_Now_Angle = Swerve_Motor_To_Mechanism_Angle(Wheel, motor->Get_Now_Total_Angle());
    diagnostic.Steer_Angle_Error = Chassis_Swerve.Get_Steer_Angle_Error(Wheel);
    diagnostic.Steer_Now_Omega = Swerve_Motor_To_Mechanism_Omega(Wheel, motor->Get_Now_Omega());
    diagnostic.Steer_Target_Omega_Limit = 0.0f;
    diagnostic.Steer_Speed_Ratio = (Swerve_Wheel_Control[Wheel].Homing.Status == Chassis_Swerve_Homing_Status_READY) ? Swerve_Get_Steer_Speed_Ratio(Wheel) : 0.0f;
    diagnostic.Total_Angle_Initialized = motor->Get_Total_Angle_Initialized();
    diagnostic.Steer_Homing_Status = Swerve_Wheel_Control[Wheel].Homing.Status;
    diagnostic.Steer_Homing_Elapsed_Time = static_cast<uint32_t>(Swerve_Wheel_Control[Wheel].Homing.Elapsed_Cycles) * Chassis_Swerve_Config.Control_Period_MS;
    diagnostic.Steer_Target_Enable = motor->Get_Motor_Enable();
    diagnostic.Drive_Status = U8_Control.Get_Drive(Wheel).Status;
    diagnostic.Drive_Wait_Elapsed_Time = static_cast<uint32_t>(U8_Control.Get_Drive(Wheel).Elapsed_Cycles) * Chassis_Swerve_Config.Control_Period_MS;
    diagnostic.Drive_Stop_Stable_Cycles = U8_Control.Get_Drive(Wheel).Stable_Cycles;
}

/**
 * @brief 根据舵向误差限制行进轮速度
 *
 * @param Wheel 舵轮轮组编号
 * @return float 允许的速度比例, 0.0~1.0
 * @note 误差小于5°允许全速, 大于45°停止驱动, 中间线性过渡
 */
float Class_Chassis_Control::Swerve_Get_Steer_Speed_Ratio(Enum_Swerve_Wheel Wheel)
{
    float angle_error = fabsf(Chassis_Swerve.Get_Steer_Angle_Error(Wheel));
    if (angle_error <= Chassis_Swerve_Config.Steer_Full_Speed_Angle)
    {
        return (1.0f);
    }
    if (angle_error >= Chassis_Swerve_Config.Steer_Stop_Angle)
    {
        return (0.0f);
    }
    return ((Chassis_Swerve_Config.Steer_Stop_Angle - angle_error) / (Chassis_Swerve_Config.Steer_Stop_Angle - Chassis_Swerve_Config.Steer_Full_Speed_Angle));
}

/**
 * @brief 仅向当前已接入的舵向电机发送MIT控制报文
 */
void Class_Chassis_Control::Swerve_Motor_Send()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (Swerve_Is_Wheel_Active(wheel))
        {
            Swerve_Get_Steer_Motor(wheel)->TIM_Send_PeriodElapsedCallback();
        }
    }
}

/**
 * @brief 控制已经完成回零的DM6220舵向电机
 */
void Class_Chassis_Control::Swerve_Steer_Motor_Control()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(wheel);
        motor->Set_Control_Angle(Swerve_Mechanism_To_Motor_Angle(wheel, Chassis_Swerve.Get_Target_Angle(wheel)));
        motor->Set_Control_Omega(0.0f);
        motor->Set_Control_Torque(Swerve_MIT_Parameter[wheel].Feedforward_Torque);
        motor->Set_K_P(Swerve_MIT_Parameter[wheel].K_P);
        motor->Set_K_D(Swerve_MIT_Parameter[wheel].K_D);
    }
}

/**
 * @brief 控制当前已接入的VESC行进电机
 * @note 回零完成且STATUS_1有效后才允许速度控制, 反馈失效锁定退出
 * @note 舵向误差达到停止阈值时, 行进轮目标立即按减速斜坡下降
 */
void Class_Chassis_Control::Swerve_Drive_Motor_Control()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        float drive_direction = Swerve_Wheel_Config[wheel].Drive_Direction;
        float steer_speed_ratio = Swerve_Get_Steer_Speed_Ratio(wheel);
        float target_omega = Chassis_Swerve.Get_Wheel_Omega(wheel) * drive_direction * steer_speed_ratio;
        Class_Motor_VESC *motor = Swerve_Get_Drive_Motor(wheel);
        motor->Set_Motor_Enable(true);
        U8_Control.Set_Speed(wheel, target_omega);
        Swerve_Update_Diagnostic(wheel);
    }
}

/**
 * @brief 清零并失能三轮电机, 同时撤销DEBUG曾启用的非正式活动U8
 * @param Reset_Drive_State 总失能时清除反转故障; 故障退出时保留诊断及锁定状态
 */
void Class_Chassis_Control::Swerve_Motor_Disable(bool Reset_Drive_State)
{
    Chassis_Swerve.Set_Speed_X(0.0f);
    Chassis_Swerve.Set_Speed_Y(0.0f);
    Chassis_Swerve.Set_Speed_W(0.0f);
    Chassis_Swerve.Calculate();
    const bool reset_homing = Reset_Drive_State || !U8_Control.Get_Fault();
    U8_Control.Disable(7U,Reset_Drive_State);
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);

        Class_Motor_DM_Normal *steer_motor = Swerve_Get_Steer_Motor(wheel);
        Class_Motor_VESC *drive_motor = Swerve_Get_Drive_Motor(wheel);
        steer_motor->Set_Control_Angle(0.0f);
        steer_motor->Set_Control_Omega(0.0f);
        steer_motor->Set_Control_Torque(0.0f);
        drive_motor->Set_Control_Omega(0.0f);
        drive_motor->Set_Motor_Enable(false);
        if (steer_motor->Get_Motor_Enable())
        {
            steer_motor->CAN_Send_Exit();
        }
        if (Reset_Drive_State)
        {
            Swerve_Wheel_Control[i].Homing = {};
            Chassis_Swerve_Diagnostic[i] = {};
        }
        else
        {
            if (reset_homing)
            {
                Swerve_Wheel_Control[i].Homing = {};
            }
            Swerve_Update_Diagnostic(wheel);
        }
    }
}

/**
 * @brief 独立U8测试时, 四台DM均失能, 返回正常底盘后重新回零
 */
void Class_Chassis_Control::Swerve_Steer_Motor_Disable()
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        auto *motor = Swerve_Get_Steer_Motor(static_cast<Enum_Swerve_Wheel>(i));
        motor->Set_Control_Angle(0.0f);
        motor->Set_Control_Omega(0.0f);
        motor->Set_Control_Torque(0.0f);
        if (motor->Get_Motor_Enable())
        {
            motor->CAN_Send_Exit();
        }
        Swerve_Wheel_Control[i].Homing = {};
    }
    if (DM_Chassis_RB.Get_Motor_Enable())
    {
        DM_Chassis_RB.CAN_Send_Exit();
    }
}

/**
 * @brief 舵轮底盘回零、速度解算和电机控制
 * @note 当前10ms生产入口使用本函数
 */
void Class_Chassis_Control::Swerve_Control()
{
    if (Final_Command.Source == Chassis_Command_Source_DEBUG_U8)
    {
        Swerve_Steer_Motor_Disable();
        U8_Test_Update(Final_Command.Speed_Y, Final_Command.Motor_Enable,
                       Remote_Chassis_Command.Online != 0U && Remote_Chassis_Command.Motor_Enable == 0U);
        for (uint8_t i = 0U; i < 3U; i++)
        {
            Swerve_Update_Diagnostic(static_cast<Enum_Swerve_Wheel>(i));
        }
        return;
    }
    U8_Test_Exit();
    // 正常底盘不再使用的测试U8立即失能; 发送掩码仍保留, 防止电调残留旧速度。
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        if (Swerve_Is_Wheel_Active(static_cast<Enum_Swerve_Wheel>(i))) { continue; }
        Swerve_Get_Drive_Motor(static_cast<Enum_Swerve_Wheel>(i))->Set_Motor_Enable(false);
    }
    if (!Final_Command.Motor_Enable)
    {
        // 速度控制退出、来源切换和遥控掉线都可能撤销最终命令, 不能代替用户确认故障。
        // 仅在线遥控明确关闭总使能时清锁; 普通停止仍按原路径复位回零和斜坡。
        bool reset_drive_state = Remote_Chassis_Command.Online != 0U && Remote_Chassis_Command.Motor_Enable == 0U;
        Swerve_Motor_Disable(reset_drive_state);
        return;
    }

    if (U8_Control.Get_Fault())
    {
        Swerve_Motor_Disable(false);
        return;
    }

    bool all_active_wheels_ready = true;
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        Class_Motor_DM_Normal *steer_motor = Swerve_Get_Steer_Motor(wheel);
        Class_Motor_VESC *drive_motor = Swerve_Get_Drive_Motor(wheel);
        Chassis_Swerve.Set_Now_Angle(wheel, Swerve_Motor_To_Mechanism_Angle(wheel, steer_motor->Get_Now_Total_Angle()));
        drive_motor->Set_Motor_Enable(true);
        if (!Swerve_Update_Homing(wheel))
        {
            all_active_wheels_ready = false;
            drive_motor->Set_Control_Omega(0.0f);
        }
        Swerve_Update_Diagnostic(wheel);
    }

    if (!all_active_wheels_ready)
    {
        U8_Control.Reset_Ramp(false);
        return;
    }

    Chassis_Swerve.Set_Speed_X(Final_Command.Speed_X);
    Chassis_Swerve.Set_Speed_Y(Final_Command.Speed_Y);
    Chassis_Swerve.Set_Speed_W(Final_Command.Speed_W);
    Chassis_Swerve.Calculate();

    Swerve_Steer_Motor_Control();
    Swerve_Drive_Motor_Control();
    if (U8_Control.Get_Fault()) { Swerve_Motor_Disable(false); }
}

/**
 * @brief 每10ms处理一次舵轮底盘控制
 * @note 当前生产入口使用本函数
 */
void Class_Chassis_Control::TIM_10ms_Swerve_PeriodElapsedCallback()
{
    Swerve_Control();
    if (Final_Command.Source != Chassis_Command_Source_DEBUG_U8 && Remote_Chassis_Command.Mode != REMOTE_MODE_DEBUG) { Swerve_Motor_Send(); }
}

/**
 * @brief 每100ms检查一次底盘电机在线状态
 */
void Class_Chassis_Control::TIM_100ms_PeriodElapsedCallback()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (Swerve_Is_Wheel_Active(wheel))
        {
            Swerve_Get_Steer_Motor(wheel)->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
    U8_Control.Alive_100ms();
}

/* Task entry wrappers ------------------------------------------------------*/

/**
 * @brief 初始化底盘解算及运行状态, 需先执行Motor_Init, 不在此处使能电机
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_Init()
{
    Chassis_Control.Init();
}

/**
 * @brief 舵轮10ms控制入口: 安全检查、回零、解算、控制并发送DM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_Swerve_10ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_10ms_Swerve_PeriodElapsedCallback();
}

/**
 * @brief 舵轮控制更新后每10ms发送VESC报文, 发送频率100Hz
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_VESC_10ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_10ms_VESC_PeriodElapsedCallback();
}

/**
 * @brief 每100ms检查当前活动轮组的电机在线状态
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_100ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_100ms_PeriodElapsedCallback();
}

/**
 * @brief 校验并保存单轮MIT位置增益, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P)
{
    return Chassis_Control.Set_MIT_K_P(Wheel, K_P);
}

/**
 * @brief 校验并保存单轮MIT速度阻尼, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D)
{
    return Chassis_Control.Set_MIT_K_D(Wheel, K_D);
}

/**
 * @brief 校验并保存单轮MIT前馈转矩, 单位N·m, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque)
{
    return Chassis_Control.Set_MIT_Feedforward_Torque(Wheel, Torque);
}

/**
 * @brief 每10ms通过同一U8对象发送正常控制或失能, 不重复发送
 */
void Class_Chassis_Control::TIM_10ms_VESC_PeriodElapsedCallback()
{
    U8_Control.Send_10ms();
}

/* U8行进速度控制、反转保护与DEBUG速度测试 --------------------------------------*/

/**
 * @brief 初始化唯一U8状态, 不使能电机
 */
void Class_U8_Control::Init(const Struct_U8_Control_Parameter &parameter)
{
    Parameter = parameter;
    Send_Wheel_Mask = parameter.Send_Wheel_Mask & 7U;
    Reset_Ramp(true);
}
/**
 * @brief 按前中、左后、右后获取既有VESC对象
 */
Class_Motor_VESC *Class_U8_Control::Get_Motor(Enum_Swerve_Wheel wheel) const
{
    return wheel < Swerve_Wheel_Number ? Motor[wheel] : nullptr;
}
/**
 * @brief 读取反转状态, 不刷新反馈
 */
const Struct_Chassis_Swerve_Drive_Control &Class_U8_Control::Get_Drive(uint8_t wheel) const
{
    return Wheel[wheel < 3U ? wheel : 0U].Drive;
}
/**
 * @brief 所选电机清零失能; 只有调用方确认在线关总使能时才允许复位保护
 */
void Class_U8_Control::Disable(uint8_t mask, bool reset)
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!(mask & (1U << i)))
        {
            continue;
        }
        Motor[i]->Set_Control_Omega(0.0f);
        Motor[i]->Set_Motor_Enable(false);
        Wheel[i].Drive_Control_Omega = 0.0f;
        if (reset)
        {
            Wheel[i].Drive = {};
        }
    }
}
/**
 * @brief 本次上电曾使用的U8始终得到周期控制或失能报文
 */
void Class_U8_Control::Send_10ms()
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (Send_Wheel_Mask & (1U << i))
        {
            Motor[i]->TIM_Send_PeriodElapsedCallback();
        }
    }
}
/**
 * @brief 检查本次上电曾控制的U8在线状态
 */
void Class_U8_Control::Alive_100ms()
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (Send_Wheel_Mask & (1U << i))
        {
            Motor[i]->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}
/**
 * @brief 原速度斜坡与反转停稳逻辑迁入, 正常和DEBUG共用同一份状态
 */
void Class_U8_Control::Set_Speed(Enum_Swerve_Wheel Wheel, float Target_Omega)
{
    if (Wheel >= Swerve_Wheel_Number)
    {
        return;
    }
    if (Get_Fault())
    {
        Disable();
        return;
    }
    Mark_Used(1U << Wheel);
    Class_Motor_VESC *motor = Get_Motor(Wheel);
    // 极短临界区只读取同一帧的转速、计数和有效性，避免CAN中断造成跨帧组合。
    const uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t feedback_count = motor->Get_Status_1_Update_Count();
    const float now_omega = motor->Get_Now_Omega();
    const bool feedback_valid = motor->Get_Status_1_Valid();
    __set_PRIMASK(interrupt_state);
    motor->Set_Motor_Enable(true);
    if (Target_Omega > Parameter.Wheel_Omega_Max)
    {
        Target_Omega = Parameter.Wheel_Omega_Max;
    }
    else if (Target_Omega < -Parameter.Wheel_Omega_Max)
    {
        Target_Omega = -Parameter.Wheel_Omega_Max;
    }
    Struct_U8_Wheel_Control &wheel = this->Wheel[Wheel];
    Struct_Chassis_Swerve_Drive_Control &drive = wheel.Drive;
    const float enter_omega = VESC_STOP_ENTER_RPM * RPM_TO_RAD_S;
    const float exit_omega = VESC_STOP_EXIT_RPM * RPM_TO_RAD_S;
    if (!feedback_valid || !isfinite(now_omega))
    {
        drive.Status = Chassis_Swerve_Drive_Status_FEEDBACK_LOST;
    }
    if (drive.Status == Chassis_Swerve_Drive_Status_FEEDBACK_LOST || drive.Status == Chassis_Swerve_Drive_Status_TIMEOUT)
    {
        wheel.Drive_Control_Omega = 0.0f;
        motor->Set_Motor_Enable(false);
        return;
    }

    if (drive.Status == Chassis_Swerve_Drive_Status_STOPPED)
    {
        if (fabsf(now_omega) > exit_omega || (Target_Omega != 0.0f && fabsf(now_omega) > enter_omega))
        {
            drive.Status = Chassis_Swerve_Drive_Status_WAIT_STOP;
            drive.Stable_Cycles = 0U;
            drive.Elapsed_Cycles = 0U;
            drive.Feedback_Update_Count = feedback_count;
        }
        else if (Target_Omega == 0.0f)
        {
            Set_Stop_Output(motor, true, now_omega);
            return;
        }
        else
        {
            drive = {};
        }
    }

    if (drive.Status == Chassis_Swerve_Drive_Status_NORMAL)
    {
        // 最近下发方向跨零后仍保留, 防止“回中一瞬间再反向”绕过等待。
        if (Target_Omega == 0.0f || Target_Omega * drive.Last_Direction < 0.0f || (Target_Omega * now_omega < 0.0f && fabsf(now_omega) > enter_omega))
        {
            drive.Status = Chassis_Swerve_Drive_Status_DECELERATING;
            drive.Elapsed_Cycles = 0U;
            drive.Stable_Cycles = 0U;
        }
    }

    if (drive.Status == Chassis_Swerve_Drive_Status_DECELERATING || drive.Status == Chassis_Swerve_Drive_Status_WAIT_STOP)
    {
        if (++drive.Elapsed_Cycles >= VESC_REVERSE_TIMEOUT_CYCLES)
        {
            drive.Status = Chassis_Swerve_Drive_Status_TIMEOUT;
            wheel.Drive_Control_Omega = 0.0f;
            motor->Set_Motor_Enable(false);
            return;
        }
        if (drive.Status == Chassis_Swerve_Drive_Status_DECELERATING)
        {
            motor->Set_Control_Method(Motor_VESC_Control_Method_RPM);
            motor->Set_Control_Omega(Update_Drive_Omega(Wheel, 0.0f));
            if (wheel.Drive_Control_Omega != 0.0f)
            {
                return;
            }
            drive.Status = Chassis_Swerve_Drive_Status_WAIT_STOP;
            drive.Feedback_Update_Count = feedback_count;
        }
        Set_Stop_Output(motor, false, now_omega);
        // 等待状态严格累计10RPM以内的新帧，10~20RPM灰区也打断连续计数。
        if (feedback_count != drive.Feedback_Update_Count)
        {
            drive.Feedback_Update_Count = feedback_count;
            if (fabsf(now_omega) <= enter_omega)
            {
                drive.Stable_Cycles++;
            }
            else
            {
                drive.Stable_Cycles = 0U;
            }
        }
        if (drive.Stable_Cycles >= VESC_REVERSE_STABLE_CYCLES)
        {
            // 确认周期仍释放驱动，下一周期才允许按最新目标从零加速。
            drive.Status = Chassis_Swerve_Drive_Status_STOPPED;
            drive.Last_Direction = 0;
            Set_Stop_Output(motor, true, now_omega);
        }
        return;
    }

    float control_omega = Update_Drive_Omega(Wheel, Target_Omega);
    motor->Set_Control_Method(Motor_VESC_Control_Method_RPM);
    motor->Set_Control_Omega(control_omega);
    if (wheel.Drive_Control_Omega != 0.0f)
    {
        drive.Last_Direction = wheel.Drive_Control_Omega > 0.0f ? 1 : -1;
    }
}

/**
 * @brief 停车等待默认零电流滑行；主动制动仅在显式开启且低速时使用正电流幅值。
 * @param Stopped 已确认停稳时一律释放驱动，避免持续零速闭环造成反向微动。
 * @param Now_Omega 本次控制使用的同一帧机械角速度，rad/s
 */
void Class_U8_Control::Set_Stop_Output(Class_Motor_VESC *motor, bool Stopped, float Now_Omega)
{
    float rpm = fabsf(Now_Omega) / RPM_TO_RAD_S;
    if (!Stopped && Parameter.Low_Speed_Brake_Enable && isfinite(Parameter.Low_Speed_Brake_RPM) &&
        Parameter.Low_Speed_Brake_RPM > 0.0f && rpm <= Parameter.Low_Speed_Brake_RPM &&
        isfinite(Parameter.Low_Speed_Brake_Current_A) && Parameter.Low_Speed_Brake_Current_A > 0.0f)
    {
        motor->Set_Control_Method(Motor_VESC_Control_Method_CURRENT_BRAKE);
        motor->Set_Control_Current_Brake(Parameter.Low_Speed_Brake_Current_A);
    }
    else
    {
        motor->Set_Control_Method(Motor_VESC_Control_Method_CURRENT);
        motor->Set_Control_Current(0.0f);
    }
}

/**
 * @brief 按轮端机械rpm/s限加减速, 目标跨零时先减到零
 */
float Class_U8_Control::Update_Drive_Omega(Enum_Swerve_Wheel Wheel, float Target_Omega)
{
    float &control_omega = this->Wheel[Wheel].Drive_Control_Omega;

    if (control_omega * Target_Omega < 0.0f)
    {
        Target_Omega = 0.0f;
    }

    float ramp = (fabsf(Target_Omega) > fabsf(control_omega)) ? Parameter.Accel_RPM_Per_S : Parameter.Decel_RPM_Per_S;
    float step = ramp * RPM_TO_RAD_S * (Parameter.Control_Period_MS / 1000.0f);
    float difference = Target_Omega - control_omega;
    if (fabsf(difference) <= step)
    {
        control_omega = Target_Omega;
    }
    else
    {
        control_omega += (difference > 0.0f) ? step : -step;
    }
    return (control_omega);
}

/**
 * @brief 正常回零仅清斜坡, 只有在线关总使能才清保护状态
 */
void Class_U8_Control::Reset_Ramp(bool Reset_Drive_State)
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        this->Wheel[i].Drive_Control_Omega = 0.0f;
        if (Reset_Drive_State)
        {
            this->Wheel[i].Drive = {};
        }
    }
}

/**
 * @brief 汇总所有U8保护锁定, 测试列表不影响故障保持
 */
bool Class_U8_Control::Get_Fault() const
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Chassis_Swerve_Drive_Status status = this->Wheel[i].Drive.Status;
        if (status == Chassis_Swerve_Drive_Status_FEEDBACK_LOST || status == Chassis_Swerve_Drive_Status_TIMEOUT)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查测试ID, 拒绝非法或重复配置
 */
uint8_t Class_U8_Test::Get_Motor_Mask() const
{
    uint8_t mask = 0U;
    for (uint8_t id : Motor_ID)
    {
        uint8_t bit = 0U;
        switch (id)
        {
        case VESC_CHASSIS_LF_ID:
            bit = 1U << Swerve_Wheel_F;
            break;
        case VESC_CHASSIS_RF_ID:
            bit = 1U << Swerve_Wheel_LB;
            break;
        case VESC_CHASSIS_LB_ID:
            bit = 1U << Swerve_Wheel_RB;
            break;
        default:
            return 0U;
        }
        if (mask & bit)
        {
            return 0U;
        }
        mask |= bit;
    }
    return mask;
}
/**
 * @brief 初始化测试ID及输入状态, 不清除共用驱动故障
 */
void Class_U8_Test::Init()
{
    Result = {};
    Result.Motor_Mask = Get_Motor_Mask();
    Active = false;
}

/**
 * @brief 退出时清零失能所选U8, 保留原有反转等待和故障
 */
void Class_U8_Test::Exit()
{
    if (Active) { U8_Control.Disable(Result.Motor_Mask, false); }
    Active = false;
    Result.Status = U8_Test_Status_DISABLED;
}

/**
 * @brief 前台10ms将右Y速度交给共用U8控制, 其余摇杆和功能键不参与
 */
void Class_U8_Test::Update(float y, bool enable, bool acknowledge)
{
    if (!enable || Result.Motor_Mask == 0U)
    {
        Exit();
        if (acknowledge) { U8_Control.Disable(7U, true); }
        return;
    }
    U8_Control.Mark_Used(Result.Motor_Mask);
    U8_Control.Disable(uint8_t((~Result.Motor_Mask) & 7U), false);
    Active = true;
    if (U8_Control.Get_Fault())
    {
        Result.Status = U8_Test_Status_FAULT;
        U8_Control.Disable();
        return;
    }
    if (!isfinite(y)) { Exit(); return; }
    const auto &parameter = U8_Control.Get_Parameter();
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        if ((Result.Motor_Mask & (1U << i)) != 0U)
        {
            U8_Control.Set_Speed(static_cast<Enum_Swerve_Wheel>(i), y / SWERVE_WHEEL_RADIUS_M * parameter.Drive_Direction[i]);
        }
    }
    Result.Status = U8_Control.Get_Fault() ? U8_Test_Status_FAULT : U8_Test_Status_RUNNING;
    if (U8_Control.Get_Fault()) { U8_Control.Disable(); }
}

/**
 * @brief 唯一DEBUG速度测试对象的初始化入口
 */
void U8_Test_Init() { U8_Test_Control.Init(); }

/**
 * @brief 前台10ms输入入口, 不自行发送CAN
 */
void U8_Test_Update(float y, bool enable, bool acknowledge) { U8_Test_Control.Update(y, enable, acknowledge); }

/**
 * @brief 离开DEBUG时撤销测试输出
 */
void U8_Test_Exit() { U8_Test_Control.Exit(); }

/**
 * @brief 获取当前测试状态及ID掩码供调试输出使用
 */
const Struct_U8_Test_Result &U8_Test_Get_Result() { return U8_Test_Control.Get_Result(); }
