/**
 * @file mod_arm_control.h
 * @brief 机械臂预设点、协调点到点运动与独立DM2325接口。
 * @note 当前仅保留模块。后续接入时先初始化，再按10ms和100ms周期调用。
 * @note 常改参数及预设点集中在本文件。默认标定未就绪，所有运动请求被拒绝。
 */
#ifndef MOD_ARM_CONTROL_H
#define MOD_ARM_CONTROL_H

/* Includes ------------------------------------------------------------------*/

#include "mod_arm_kinematics.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

#define ARM_CONTROL_PERIOD_S 0.01f          // 10ms前台控制周期，s
#define ARM_FEEDBACK_TIMEOUT_MS 200U        // 反馈/使能确认及前台停顿期限，ms
#define ARM_ARRIVE_STABLE_CYCLES 30U        // 连续30个10ms周期稳定到位
#define ARM_MOVE_TIMEOUT_MARGIN_S 3.0f     // 规划结束后的额外等待时间，s
#define ARM_MOVE_DURATION_MAX_S 3600.0f    // 拒绝异常长的规划时间，s

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 目标提交结果，调用方根据结果决定是否提示忙碌或重新提交
 */
enum Enum_Arm_Result
{
    Arm_Result_ACCEPTED = 0,     // 已接受本次指令
    Arm_Result_BUSY,             // 正在运动，本次指令不缓存
    Arm_Result_INVALID_POINT,    // 点位编号错误或尚未配置
    Arm_Result_INVALID_TARGET,   // 坐标不可达或超出限位
    Arm_Result_NOT_READY,        // 标定、反馈或使能条件不满足
};

/**
 * @brief 机械臂运行状态；故障需显式失能后重新申请使能
 */
enum Enum_Arm_Status
{
    Arm_Status_DISABLED = 0,
    Arm_Status_IDLE,
    Arm_Status_MOVING,
    Arm_Status_ARRIVED,
    Arm_Status_FAULT,
};

/**
 * @brief 预设点，坐标相对肩轴；Configured为false时拒绝执行
 */
struct Struct_Arm_Point
{
    float Y = 0.0f;                                  // 向车头的坐标，mm
    float Z = 0.0f;                                  // 向上的坐标，mm
    bool Configured = false;                         // 填写真实点位后单独置true
};

/**
 * @brief 机械臂初始化参数，实际方向和限位确认后再填写
 * @note 角度、角速度和角加速度分别填写°、°/s和°/s²，Init统一换算
 */
struct Struct_Arm_Parameter
{
    bool Calibration_Ready = false;                  // 保存水平零位、核对方向和实际限位后置true
    float Big_Zero = 0.0f;                           // 水平向前时的电机反馈，°
    float Small_Zero = 0.0f;                         // 水平向前时的电机反馈，°
    float Big_Direction = 0.0f;                      // 正角向上，对应电机方向，实测填+1或-1
    float Small_Direction = 0.0f;                    // 正角向上，对应电机方向，实测填+1或-1
    float Big_Min = NAN;                             // 大臂绝对角软限位，°
    float Big_Max = NAN;                             // 大臂绝对角软限位，°
    float Small_Min = NAN;                           // 小臂绝对角软限位，°
    float Small_Max = NAN;                           // 小臂绝对角软限位，°
    float Relative_Min = NAN;                        // 小臂角减大臂角，防止闭链跨分支，°
    float Relative_Max = NAN;                        // 小臂角减大臂角，防止闭链跨分支，°
    float Speed_Max = 1.0f;                          // 两臂最大速度，°/s
    float Acceleration_Max = 1.0f;                 // 两臂加减速度，°/s²
    float Position_Tolerance = 1.0f;                // 到位角度误差，°
    float Speed_Tolerance = 1.0f;                   // 到位速度阈值，°/s
    bool Rotate_Ready = false;                       // 独立旋转轴完成零位和限位核对
    float Rotate_Min = NAN;                          // DM2325输出轴位置范围，°
    float Rotate_Max = NAN;                          // DM2325输出轴位置范围，°
    float Rotate_Speed_Max = 5.0f;                  // 独立调试速度上限，°/s
    Struct_Arm_Point Point[9]{};                     // 遥控点位1~9，默认全部未配置
};

/**
 * @brief 大小臂点到点控制，独立旋转轴通过Set_Rotate_Target操作
 */
class Class_Arm_Control
{
public:
    // 初始化相关函数

    void Init(const Struct_Arm_Parameter &Parameter = Struct_Arm_Parameter{});

    // 周期回调函数

    void TIM_10ms_PeriodElapsedCallback();
    void TIM_100ms_PeriodElapsedCallback();

    // 读变量

    inline Enum_Arm_Status Get_Status() const;
    void Get_Now_Position(float &Y, float &Z) const;

    // 写变量：仅在前台调用，许可恢复后须重新显式使能
    Enum_Arm_Result Set_Enable(bool Enable);
    Enum_Arm_Result Set_Target_Point(unsigned ID);
    Enum_Arm_Result Set_Target_Position(float Y, float Z);
    Enum_Arm_Result Set_Rotate_Target(float Motor_Angle, float Speed);

private:
    // 初始化相关常量
    Struct_Arm_Parameter Parameter{};               // 初始化参数副本，内部统一使用弧度制

    // 内部变量
    Enum_Arm_Status Status = Arm_Status_DISABLED;    // 运行状态
    bool Enabled = false;                            // 两臂控制许可
    bool Rotate_Active = false;                      // 独立旋转轴已接管
    float Start[2]{};                                // 本次轨迹起点，电机输出轴rad
    float Target[2]{};                               // 本次轨迹终点，电机输出轴rad
    float Duration = 0.0f;                           // 两臂共用轨迹时长，s
    unsigned Elapsed = 0;                            // 已执行的10ms轨迹周期数
    unsigned Stable = 0;                             // 连续满足到位条件的10ms周期数
    unsigned Rx_Count[3]{};                          // 各电机上次接收计数
    unsigned Rx_Tick[3]{};                           // 各电机最近一次反馈时刻，ms
    unsigned Enable_Tick[3]{};                       // 各电机发送使能指令的时刻，ms
    bool Rx_Seen[3]{};                               // 是否已取得本次初始化后的反馈
    unsigned Previous_Tick = 0;                      // 上次10ms回调时刻，ms

    // 内部函数

    bool Update_Motion();
    bool Configuration_Valid() const;
    bool Joint_Valid(float Big, float Small) const;
    bool Feedback_Valid(unsigned Index) const;
    void Disable(Enum_Arm_Status Next);
};

/* Exported variables --------------------------------------------------------*/

extern Class_Arm_Control Arm;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取机械臂当前运行状态
 *
 * @return Enum_Arm_Status 待机、运动、到位或故障状态
 */
inline Enum_Arm_Status Class_Arm_Control::Get_Status() const
{
    return (Status);
}

#endif
