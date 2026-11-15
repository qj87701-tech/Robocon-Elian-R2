/**
 * @file mod_arm_kinematics.cpp
 * @brief 平面正逆解；逆解选择靠近当前角度的装配分支，不在运动中切换分支。
 */

/* Includes ------------------------------------------------------------------*/

#include "mod_arm_kinematics.h"
#include <math.h>

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 用两臂绝对角计算小臂末端轴心坐标，mm。
 *
 * @param Big 大臂相对水平的绝对角，rad
 * @param Small 小臂相对水平的绝对角，rad
 * @param Y 输出向前坐标，mm
 * @param Z 输出向上坐标，mm
 */
void Class_Arm_Kinematics::Forward(float Big, float Small, float &Y, float &Z)
{
    Y = Big_Length * cosf(Big) + Small_Length * cosf(Small);
    Z = Big_Length * sinf(Big) + Small_Length * sinf(Small);
}

/**
 * @brief 求最近的连续角分支；拒绝不可达、非有限数及接近完全折叠/伸直的目标。
 *
 * @param Y 目标向前坐标，mm
 * @param Z 目标向上坐标，mm
 * @param Now_Big 当前大臂绝对角，rad
 * @param Now_Small 当前小臂绝对角，rad
 * @param Big 输出大臂目标角，rad
 * @param Small 输出小臂目标角，rad
 * @return bool 是否求得远离伸直/折叠奇异位置的解
 */
bool Class_Arm_Kinematics::Inverse(float Y, float Z, float Now_Big, float Now_Small, float &Big, float &Small)
{
    if (!isfinite(Y) || !isfinite(Z) || !isfinite(Now_Big) || !isfinite(Now_Small))
    {
        return false;
    }
    // 余弦定理求相对肘角，排除不可达与接近奇异位置的目标
    float c = (Y * Y + Z * Z - Big_Length * Big_Length - Small_Length * Small_Length) / (2.0f * Big_Length * Small_Length);
    if (!isfinite(c) || fabsf(c) >= 0.9999f)
    {
        return false;
    }
    float delta = acosf(c);
    float best = 1.0e30f;
    // 两个逆解分支分别展开到当前角度附近，选择关节角变化较小的一支
    for (int sign = -1; sign <= 1; sign += 2)
    {
        float d = sign * delta;
        float a = atan2f(Z, Y) - atan2f(Small_Length * sinf(d), Big_Length + Small_Length * cosf(d));
        float b = a + d;
        a += roundf((Now_Big - a) / 6.28318530718f) * 6.28318530718f;
        b += roundf((Now_Small - b) / 6.28318530718f) * 6.28318530718f;
        float cost = fabsf(a - Now_Big) + fabsf(b - Now_Small);
        if (cost < best)
        {
            best = cost;
            Big = a;
            Small = b;
        }
    }
    return true;
}
