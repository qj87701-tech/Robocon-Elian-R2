/**
 * @file mod_arm_kinematics.h
 * @brief 大小臂平面运动学，肩轴为原点，Y向前、Z向上，坐标mm、角度rad。
 * @note 两臂均使用相对水平的绝对角；新版STEP为400/324mm，外部四杆等角传动。
 */
#ifndef MOD_ARM_KINEMATICS_H
#define MOD_ARM_KINEMATICS_H

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 双连杆平面运动学；小臂使用绝对角，四杆等角关系由零偏与方向处理
 */
class Class_Arm_Kinematics
{
public:
    // 几何常量

    static constexpr float Big_Length = 400.0f;   // 肩轴到肘轴，mm
    static constexpr float Small_Length = 324.0f; // 肘轴到末端轴，mm

    // 正逆运动学函数

    static void Forward(float Big, float Small, float &Y, float &Z);
    static bool Inverse(float Y, float Z, float Now_Big, float Now_Small, float &Big, float &Small);
};

#endif
