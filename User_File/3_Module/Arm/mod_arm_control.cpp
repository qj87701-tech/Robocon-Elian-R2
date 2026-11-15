/**
 * @file mod_arm_control.cpp
 * @brief 共用归一化三次曲线实现两臂同时到位；不接受运动中的新目标。
 * @note 位置速度模式的速度字段作为电机内部限速，轨迹加减速由本模块生成。
 */

/* Includes ------------------------------------------------------------------*/

#include "mod_arm_control.h"

/* Private macros ------------------------------------------------------------*/
//1 rad/s = 180/π °/s ≈ 57.30°/s
#define ARM_DEG_TO_RAD (3.14159265358979323846f / 180.0f)

/* Exported variables --------------------------------------------------------*/

Class_Arm_Control Arm;

/* Private variables ---------------------------------------------------------*/

// 数组顺序：大臂、小臂、独立旋转轴
static Class_Motor_DM_Normal *const Motor[3] = {&DM_Arm_Big, &DM_Arm_Small, &DM_Arm_Rotate};

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 清空运行状态；配置不完整时禁止进入运动。
 *
 * @param Value 标定、软限位和预设点，角度为°，角速度为°/s，角加速度为°/s²
 */
void Class_Arm_Control::Init(const Struct_Arm_Parameter &Value)
{
    if (Enabled || Rotate_Active)
    {
        Disable(Arm_Status_DISABLED);
    }
    Previous_Tick = HAL_GetTick();
    Parameter = Value;
    // 配置用角度制，运动学、反馈和电机命令统一使用弧度制
    Parameter.Big_Zero *= ARM_DEG_TO_RAD;
    Parameter.Small_Zero *= ARM_DEG_TO_RAD;
    Parameter.Big_Min *= ARM_DEG_TO_RAD;
    Parameter.Big_Max *= ARM_DEG_TO_RAD;
    Parameter.Small_Min *= ARM_DEG_TO_RAD;
    Parameter.Small_Max *= ARM_DEG_TO_RAD;
    Parameter.Relative_Min *= ARM_DEG_TO_RAD;
    Parameter.Relative_Max *= ARM_DEG_TO_RAD;
    Parameter.Speed_Max *= ARM_DEG_TO_RAD;
    Parameter.Acceleration_Max *= ARM_DEG_TO_RAD;
    Parameter.Position_Tolerance *= ARM_DEG_TO_RAD;
    Parameter.Speed_Tolerance *= ARM_DEG_TO_RAD;
    Parameter.Rotate_Min *= ARM_DEG_TO_RAD;
    Parameter.Rotate_Max *= ARM_DEG_TO_RAD;
    Parameter.Rotate_Speed_Max *= ARM_DEG_TO_RAD;

    Enabled = Rotate_Active = false;
    Status = Arm_Status_DISABLED;
    Elapsed = Stable = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        Rx_Count[i] = Motor[i]->Get_Rx_Update_Count();
        Rx_Seen[i] = false;
        Rx_Tick[i] = HAL_GetTick();
    }
}

/**
 * @brief 检查软件参数、零偏及角度限位，不能用占位参数启用机械臂。
 *
 * @return bool 所有初始化参数是否有效
 */
bool Class_Arm_Control::Configuration_Valid() const
{
    // 标定完成、方向和零偏
    if (!(Parameter.Calibration_Ready && fabsf(Parameter.Big_Direction) == 1.0f && fabsf(Parameter.Small_Direction) == 1.0f &&
          isfinite(Parameter.Big_Zero) && isfinite(Parameter.Small_Zero)))
    {
        return false;
    }

    // 大臂软限位
    if (!(isfinite(Parameter.Big_Min) && isfinite(Parameter.Big_Max) && Parameter.Big_Min < Parameter.Big_Max))
    {
        return false;
    }

    // 小臂软限位
    if (!(isfinite(Parameter.Small_Min) && isfinite(Parameter.Small_Max) && Parameter.Small_Min < Parameter.Small_Max))
    {
        return false;
    }

    // 相对肘角软限位
    if (!(isfinite(Parameter.Relative_Min) && isfinite(Parameter.Relative_Max) && Parameter.Relative_Min < Parameter.Relative_Max))
    {
        return false;
    }

    // 速度上限
    if (!(isfinite(Parameter.Speed_Max) && Parameter.Speed_Max > 0 && Parameter.Speed_Max <= DM_ARM_JOINT_OMEGA_MAX))
    {
        return false;
    }

    // 加速度上限
    if (!(isfinite(Parameter.Acceleration_Max) && Parameter.Acceleration_Max > 0))
    {
        return false;
    }

    // 到位角度误差
    if (!(isfinite(Parameter.Position_Tolerance) && Parameter.Position_Tolerance > 0))
    {
        return false;
    }

    // 到位速度阈值
    if (!(isfinite(Parameter.Speed_Tolerance) && Parameter.Speed_Tolerance > 0))
    {
        return false;
    }
    return true;
}

/**
 * @brief 检查关节、相对肘角和电机协议位置范围；不代表完成实体碰撞检测。
 *
 * @param Big 大臂绝对角，rad
 * @param Small 小臂绝对角，rad
 * @return bool 是否满足关节、相对角和协议量程限制
 */
bool Class_Arm_Control::Joint_Valid(float Big, float Small) const
{
    float d = Small - Big;
    return isfinite(Big) && isfinite(Small) && Big >= Parameter.Big_Min && Big <= Parameter.Big_Max &&
        Small >= Parameter.Small_Min && Small <= Parameter.Small_Max && d >= Parameter.Relative_Min && d <= Parameter.Relative_Max &&
        fabsf(Parameter.Big_Zero + Parameter.Big_Direction * Big) < DM_ARM_ANGLE_MAX &&
        fabsf(Parameter.Small_Zero + Parameter.Small_Direction * Small) < DM_ARM_ANGLE_MAX;
}

/**
 * @brief 新反馈必须在200ms期限内，驱动器故障状态不能触发自动清错重启。
 *
 * @param Index 电机索引：0大臂、1小臂、2旋转轴
 * @return bool 是否存在期限内的无故障反馈
 */
bool Class_Arm_Control::Feedback_Valid(unsigned Index) const
{
    return Rx_Seen[Index] && unsigned(HAL_GetTick() - Rx_Tick[Index]) < ARM_FEEDBACK_TIMEOUT_MS && isfinite(Motor[Index]->Get_Now_Angle()) && isfinite(Motor[Index]->Get_Now_Omega()) &&
        Motor[Index]->Get_Control_Status() <= Motor_DM_Control_Status_ENABLE;
}

/**
 * @brief 撤销许可及目标速度并失能三个电机；故障不会自行恢复。
 *
 * @param Next 失能后保留的状态，故障状态不自动清除
 */
void Class_Arm_Control::Disable(Enum_Arm_Status Next)
{
    Enabled = Rotate_Active = false;
    Status = Next;
    for (auto motor : Motor)
    {
        motor->Set_Control_Omega(0);
        motor->CAN_Send_Exit();
    }
}

/**
 * @brief 显式使能时从当前位置接管，不自动回零；故障须先撤销许可再申请。
 *
 * @param Enable true申请使能，false撤销许可并失能
 * @return Enum_Arm_Result 已接受或未就绪
 */
Enum_Arm_Result Class_Arm_Control::Set_Enable(bool Enable)
{
    if (!Enable)
    {
        Disable(Arm_Status_DISABLED);
        return Arm_Result_ACCEPTED;
    }
    if (Enabled)
    {
        return Arm_Result_ACCEPTED;
    }
    if (Status == Arm_Status_FAULT || !Configuration_Valid() || !Feedback_Valid(0) || !Feedback_Valid(1))
    {
        return Arm_Result_NOT_READY;
    }
    float now_big = Parameter.Big_Direction * (Motor[0]->Get_Now_Angle() - Parameter.Big_Zero);
    float now_small = Parameter.Small_Direction * (Motor[1]->Get_Now_Angle() - Parameter.Small_Zero);
    if (!Joint_Valid(now_big, now_small))
    {
        return Arm_Result_NOT_READY;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        Motor[i]->Set_Control_Angle(Motor[i]->Get_Now_Angle());
        Motor[i]->Set_Control_Omega(Parameter.Speed_Max);
        Enable_Tick[i] = HAL_GetTick();
        Motor[i]->CAN_Send_Enter();
    }
    Enabled = true;
    Status = Arm_Status_IDLE;
    return Arm_Result_ACCEPTED;
}

/**
 * @brief 预设点编号1~9；未填写的点不会变成零坐标命令。
 *
 * @param ID 预设点编号，1~9
 * @return Enum_Arm_Result 已接受、忙碌、无效点位或未就绪
 */
Enum_Arm_Result Class_Arm_Control::Set_Target_Point(unsigned ID)
{
    if (ID < 1 || ID > 9 || !Parameter.Point[ID - 1].Configured)
    {
        return Arm_Result_INVALID_POINT;
    }
    return Set_Target_Position(Parameter.Point[ID - 1].Y, Parameter.Point[ID - 1].Z);
}

/**
 * @brief 接受新坐标，按当前装配分支求解并生成速度/加速度受限的统一运动时长。
 *
 * @param Y 向前的末端轴心坐标，mm
 * @param Z 向上的末端轴心坐标，mm
 * @return Enum_Arm_Result 已接受、忙碌、无效目标或未就绪
 */
Enum_Arm_Result Class_Arm_Control::Set_Target_Position(float Y, float Z)
{
    if (Status == Arm_Status_MOVING)
    {
        return Arm_Result_BUSY;
    }
    if (!Enabled || !Configuration_Valid() || !Feedback_Valid(0) || !Feedback_Valid(1))
    {
        return Arm_Result_NOT_READY;
    }
    float now_big = Parameter.Big_Direction * (Motor[0]->Get_Now_Angle() - Parameter.Big_Zero);
    float now_small = Parameter.Small_Direction * (Motor[1]->Get_Now_Angle() - Parameter.Small_Zero);
    float target_big;
    float target_small;
    if (!Joint_Valid(now_big, now_small) || !Class_Arm_Kinematics::Inverse(Y, Z, now_big, now_small, target_big, target_small) || !Joint_Valid(target_big, target_small))
    {
        return Arm_Result_INVALID_TARGET;
    }
    // 相对角不得越过kπ的伸直/折叠奇异位置，防止闭链翻支。
    if (floorf((now_small - now_big) / 3.14159265359f) != floorf((target_small - target_big) / 3.14159265359f) && fabsf(sinf(now_small - now_big)) > 0.001f)
    {
        return Arm_Result_INVALID_TARGET;
    }
    Start[0] = Motor[0]->Get_Now_Angle();
    Start[1] = Motor[1]->Get_Now_Angle();
    Target[0] = Parameter.Big_Zero + Parameter.Big_Direction * target_big;
    Target[1] = Parameter.Small_Zero + Parameter.Small_Direction * target_small;
    float distance = fmaxf(fabsf(Target[0] - Start[0]), fabsf(Target[1] - Start[1]));
    Duration = fmaxf(ARM_CONTROL_PERIOD_S, fmaxf(1.5f * distance / Parameter.Speed_Max, sqrtf(6.0f * distance / Parameter.Acceleration_Max)));
    if (!isfinite(Duration) || Duration > ARM_MOVE_DURATION_MAX_S)
    {
        return Arm_Result_INVALID_TARGET;
    }
    Elapsed = Stable = 0;
    Status = Arm_Status_MOVING;
    return Arm_Result_ACCEPTED;
}

/**
 * @brief 独立设置DM2325输出轴位置与正的速度上限，不参与Y/Z逆解。
 *
 * @param Angle DM2325输出轴位置，rad
 * @param Speed 正的电机速度上限，rad/s
 * @return Enum_Arm_Result 已接受、无效目标或未就绪
 */
Enum_Arm_Result Class_Arm_Control::Set_Rotate_Target(float Angle, float Speed)
{
    if (!Enabled || !Parameter.Rotate_Ready || !Feedback_Valid(2) || !isfinite(Parameter.Rotate_Min) || !isfinite(Parameter.Rotate_Max) ||
        Parameter.Rotate_Min >= Parameter.Rotate_Max || !isfinite(Parameter.Rotate_Speed_Max) || Parameter.Rotate_Speed_Max <= 0)
    {
        return Arm_Result_NOT_READY;
    }
    if (!isfinite(Angle) || !isfinite(Speed) || Speed <= 0 || Angle < Parameter.Rotate_Min || Angle > Parameter.Rotate_Max ||
        fabsf(Angle) >= DM_ARM_ANGLE_MAX || Speed > Parameter.Rotate_Speed_Max || Speed > DM_ARM_ROTATE_OMEGA_MAX)
    {
        return Arm_Result_INVALID_TARGET;
    }
    Motor[2]->Set_Control_Angle(Angle);
    Motor[2]->Set_Control_Omega(Speed);
    Enable_Tick[2] = HAL_GetTick();
    Motor[2]->CAN_Send_Enter();
    Rotate_Active = true;
    return Arm_Result_ACCEPTED;
}

/**
 * @brief 从反馈关节角计算当前位置，供遥控显示或串口调试，mm。
 *
 * @param Y 输出向前坐标，mm；反馈无效时为NAN
 * @param Z 输出向上坐标，mm；反馈无效时为NAN
 */
void Class_Arm_Control::Get_Now_Position(float &Y, float &Z) const
{
    if (!Configuration_Valid() || !Feedback_Valid(0) || !Feedback_Valid(1))
    {
        Y = Z = NAN;
        return;
    }
    Class_Arm_Kinematics::Forward(Parameter.Big_Direction * (Motor[0]->Get_Now_Angle() - Parameter.Big_Zero),
        Parameter.Small_Direction * (Motor[1]->Get_Now_Angle() - Parameter.Small_Zero), Y, Z);
}

/**
 * @brief 更新两臂共用轨迹进度，并根据反馈判断稳定到位
 *
 * @return bool true允许继续发送，false表示超时已失能
 * @note 仅在MOVING状态下每10ms调用；三次曲线两端速度为零
 */
bool Class_Arm_Control::Update_Motion()
{
    ++Elapsed;
    float t = fminf(Elapsed * ARM_CONTROL_PERIOD_S / Duration, 1.0f);
    float progress = t * t * (3.0f - 2.0f * t);
    for (unsigned i = 0; i < 2; ++i)
    {
        Motor[i]->Set_Control_Angle(Start[i] + (Target[i] - Start[i]) * progress);
    }
    if (t >= 1.0f && fabsf(Target[0] - Motor[0]->Get_Now_Angle()) <= Parameter.Position_Tolerance &&
        fabsf(Target[1] - Motor[1]->Get_Now_Angle()) <= Parameter.Position_Tolerance &&
        fabsf(Motor[0]->Get_Now_Omega()) < Parameter.Speed_Tolerance && fabsf(Motor[1]->Get_Now_Omega()) < Parameter.Speed_Tolerance)
    {
        ++Stable;
    }
    else
    {
        Stable = 0;
    }
    if (Stable >= ARM_ARRIVE_STABLE_CYCLES)
    {
        Status = Arm_Status_ARRIVED;
    }
    else if (Elapsed * ARM_CONTROL_PERIOD_S > Duration + ARM_MOVE_TIMEOUT_MARGIN_S)
    {
        Disable(Arm_Status_FAULT);
        return false;
    }
    return true;
}

/**
 * @brief 每10ms更新反馈期限、轨迹及真实到位状态；不缓存运动期间的新目标。
 */
void Class_Arm_Control::TIM_10ms_PeriodElapsedCallback()
{
    // 先判断调度停顿，再刷新接收计数，避免新反馈掩盖失控时间
    unsigned now = HAL_GetTick();
    if (Enabled && unsigned(now - Previous_Tick) >= ARM_FEEDBACK_TIMEOUT_MS)
    {
        Disable(Arm_Status_FAULT);
        Previous_Tick = now;
        return;
    }
    Previous_Tick = now;
    for (unsigned i = 0; i < 3; ++i)
    {
        unsigned count = Motor[i]->Get_Rx_Update_Count();
        if (count != Rx_Count[i])
        {
            Rx_Count[i] = count;
            Rx_Tick[i] = now;
            Rx_Seen[i] = true;
        }
    }
    if (!Enabled)
    {
        return;
    }
    if (!Feedback_Valid(0) || !Feedback_Valid(1) || (Rotate_Active && !Feedback_Valid(2)))
    {
        Disable(Arm_Status_FAULT);
        return;
    }
    float now_big = Parameter.Big_Direction * (Motor[0]->Get_Now_Angle() - Parameter.Big_Zero);
    float now_small = Parameter.Small_Direction * (Motor[1]->Get_Now_Angle() - Parameter.Small_Zero);
    if (!Joint_Valid(now_big, now_small) || (Rotate_Active && (Motor[2]->Get_Now_Angle() < Parameter.Rotate_Min || Motor[2]->Get_Now_Angle() > Parameter.Rotate_Max)))
    {
        Disable(Arm_Status_FAULT);
        return;
    }
    // Enter确认最多等待200ms，期间保持当前位置，禁止调用自动清错路径。
    for (unsigned i = 0; i < (Rotate_Active ? 3U : 2U); ++i)
    {
        if (Motor[i]->Get_Control_Status() != Motor_DM_Control_Status_ENABLE)
        {
            if (unsigned(now - Enable_Tick[i]) >= ARM_FEEDBACK_TIMEOUT_MS)
            {
                Disable(Arm_Status_FAULT);
            }
            return;
        }
    }
    // 两臂轨迹计算、稳定到位与运动超时检查
    if (Status == Arm_Status_MOVING && !Update_Motion())
    {
        return;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        Motor[i]->TIM_Send_PeriodElapsedCallback();
    }
    if (Rotate_Active)
    {
        if (Motor[2]->Get_Control_Status() != Motor_DM_Control_Status_ENABLE)
        {
            Disable(Arm_Status_FAULT);
            return;
        }
        Motor[2]->TIM_Send_PeriodElapsedCallback();
    }
}

/**
 * @brief 每100ms检查在线；失能帧用于获取初始反馈，掉线前先撤销使能防止驱动自动重启。
 */
void Class_Arm_Control::TIM_100ms_PeriodElapsedCallback()
{
    for (unsigned i = 0; i < 3; ++i)
    {
        if (!Motor[i]->Get_Motor_Enable())
        {
            Motor[i]->CAN_Send_Exit();
        }
        // 在线判断由本模块反馈计数完成，不调用会自动重使能的通用Alive函数。
    }
}
