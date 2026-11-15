/**
 * @file tsk_chassis_resolve.h
 * @brief 正三角形三舵轮底盘速度解算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 三舵轮运动学：车体速度转换为各轮舵向角度和行进角速度。
 * @note tsk_chassis设置VX/VY/W和实际舵角后调用Calculate，再读取轮组目标。
 * @note X向右、Y向前、W逆时针；轮径和正三角形边长在本头文件修改，当前仍为暂定值。
 * @note 本文件只计算，不发送CAN，不负责回零或电机使能。
 */

#ifndef TSK_CHASSIS_RESOLVE_H
#define TSK_CHASSIS_RESOLVE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// 几何仍为暂定值, 并非实测尺寸。实测后只改轮径和三轮中心正三角形边长。
#define SWERVE_WHEEL_DIAMETER_M 0.200f
#define SWERVE_WHEEL_RADIUS_M (SWERVE_WHEEL_DIAMETER_M / 2.0f)
#define SWERVE_CHASSIS_SIDE_LENGTH_M 0.450f
#define SWERVE_CHASSIS_RADIUS_M (SWERVE_CHASSIS_SIDE_LENGTH_M / 1.73205080757f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵轮轮组编号
 */
enum Enum_Swerve_Wheel : uint8_t
{
    Swerve_Wheel_F = 0, // 前中, ID1
    Swerve_Wheel_LB,    // 左后, ID2
    Swerve_Wheel_RB,    // 右后, ID3
    Swerve_Wheel_Number,
};

/**
 * @brief 正三角形三舵轮底盘解算类
 */
class Class_Chassis_Swerve
{
public:
    // 初始化相关函数

    void Init(float Wheel_Radius, float Chassis_Radius, float Steer_Angle_Limit = 12.0f);

    // 写变量: 车体速度、实际舵角和轮速限制

    inline void Set_Speed_X(float Speed_X);
    inline void Set_Speed_Y(float Speed_Y);
    inline void Set_Speed_W(float Speed_W);
    inline void Set_Now_Angle(Enum_Swerve_Wheel Wheel, float Angle);
    inline void Set_Wheel_Omega_Max(float Wheel_Omega_Max);

    void Reset_Wheel(Enum_Swerve_Wheel Wheel, float Hold_Angle);

    // 读变量: 解算输出

    inline float Get_Target_Angle(Enum_Swerve_Wheel Wheel) const;
    inline float Get_Wheel_Omega(Enum_Swerve_Wheel Wheel) const;
    float Get_Steer_Angle_Error(Enum_Swerve_Wheel Wheel) const;

    // 计算入口, 调用前先更新车体速度和实际舵角

    void Calculate();

private:
    /**
     * @brief 底盘机械参数
     */
    struct Struct_Chassis_Swerve_Parameter
    {
        float Wheel_Radius = 0.0f;   // 行进轮半径, m
        float Chassis_Radius = 0.0f; // 轮组投影点到底盘中心的距离, m
        float Wheel_Omega_Max = 0.0f; // 行进轮最大机械角速度, rad/s, 0表示不限制
        float Steer_Angle_Limit = 0.0f; // 舵向目标软限位, rad, 0表示不限制
    } Parameter;

    /**
     * @brief 底盘目标速度
     */
    struct Struct_Chassis_Swerve_Velocity
    {
        float Speed_X = 0.0f; // 向右为正, m/s
        float Speed_Y = 0.0f; // 向前为正, m/s
        float Speed_W = 0.0f; // 逆时针为正, rad/s
    } Velocity;

    /**
     * @brief 单个舵轮轮组状态
     */
    struct Struct_Chassis_Swerve_Wheel
    {
        float Now_Angle = 0.0f;
        float Last_Target_Angle = 0.0f;
        float Target_Angle = 0.0f;
        float Wheel_Omega = 0.0f;
        bool Now_Angle_Initialized = false;
        bool Motion_Target_Initialized = false;
        bool Motion_Command_Active = false;
        bool Drive_Reversed = false;
    } Wheel[Swerve_Wheel_Number];
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置底盘X轴目标速度
 *
 * @param Speed_X X轴目标速度, 向右为正, m/s
 */
inline void Class_Chassis_Swerve::Set_Speed_X(float Speed_X)
{
    Velocity.Speed_X = Speed_X;
}

/**
 * @brief 设置底盘Y轴目标速度
 *
 * @param Speed_Y Y轴目标速度, 向前为正, m/s
 */
inline void Class_Chassis_Swerve::Set_Speed_Y(float Speed_Y)
{
    Velocity.Speed_Y = Speed_Y;
}

/**
 * @brief 设置底盘旋转目标速度
 *
 * @param Speed_W 旋转目标角速度, 逆时针为正, rad/s
 */
inline void Class_Chassis_Swerve::Set_Speed_W(float Speed_W)
{
    Velocity.Speed_W = Speed_W;
}

/**
 * @brief 设置舵向电机当前角度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @param Angle 舵向电机当前角度, rad
 */
inline void Class_Chassis_Swerve::Set_Now_Angle(Enum_Swerve_Wheel Wheel_ID, float Angle)
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return;
    }

    Wheel[Wheel_ID].Now_Angle = Angle;
    if (!Wheel[Wheel_ID].Now_Angle_Initialized)
    {
        Wheel[Wheel_ID].Last_Target_Angle = Angle;
        Wheel[Wheel_ID].Target_Angle = Angle;
        Wheel[Wheel_ID].Now_Angle_Initialized = true;
    }
}

/**
 * @brief 设置行进轮最大机械角速度
 *
 * @param Wheel_Omega_Max 最大机械角速度, rad/s, 小于等于0表示不限制
 */
inline void Class_Chassis_Swerve::Set_Wheel_Omega_Max(float Wheel_Omega_Max)
{
    Parameter.Wheel_Omega_Max = Wheel_Omega_Max;
}

/**
 * @brief 获取舵向电机目标角度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @return float 舵向电机目标角度, rad
 */
inline float Class_Chassis_Swerve::Get_Target_Angle(Enum_Swerve_Wheel Wheel_ID) const
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return 0.0f;
    }

    return Wheel[Wheel_ID].Target_Angle;
}

/**
 * @brief 获取行进轮机械角速度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @return float 行进轮机械角速度, rad/s
 */
inline float Class_Chassis_Swerve::Get_Wheel_Omega(Enum_Swerve_Wheel Wheel_ID) const
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return 0.0f;
    }

    return Wheel[Wheel_ID].Wheel_Omega;
}

#endif
