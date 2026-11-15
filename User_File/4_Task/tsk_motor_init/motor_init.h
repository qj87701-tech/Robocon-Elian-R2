/**
 * @file motor_init.h
 * @brief 电机对象和初始化配置
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 电机绑定与初值：集中定义DM、VESC对象及CAN节点参数。
 * @note Task_Init先调用Motor_Init，再初始化底盘，默认保持电机失能。
 * @note 常改参数见本头文件：ID、DM量程/MIT初值、VESC极对数及命令限幅。
 */

#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

/* Includes ------------------------------------------------------------------*/

#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Motor/Motor_VESC/dvc_motor_vesc.h"

/* Exported macros -----------------------------------------------------------*/

// 底盘达妙DM6220反馈报文ID
#define DM_CHASSIS_LF_MASTER_ID 0x11
#define DM_CHASSIS_RF_MASTER_ID 0x22
#define DM_CHASSIS_LB_MASTER_ID 0x33
#define DM_CHASSIS_RB_MASTER_ID 0x44

// 底盘达妙DM6220电机ID
#define DM_CHASSIS_LF_MOTOR_ID 0x01
#define DM_CHASSIS_RF_MOTOR_ID 0x02
#define DM_CHASSIS_LB_MOTOR_ID 0x03
#define DM_CHASSIS_RB_MOTOR_ID 0x04

// DM6220 MIT协议量程
#define DM_CHASSIS_ANGLE_MAX 251.3f
#define DM_CHASSIS_OMEGA_MAX 45.0f
#define DM_CHASSIS_TORQUE_MAX 10.0f

// DM6220 MIT舵向初始参数，后续可通过USART9在RAM中调整
#define DM_CHASSIS_MIT_K_P_DEFAULT 7.0f
#define DM_CHASSIS_MIT_K_D_DEFAULT 0.20f
#define DM_CHASSIS_MIT_TORQUE_DEFAULT 0.0f

// 底盘VESC节点ID
#define VESC_CHASSIS_LF_ID 1U
#define VESC_CHASSIS_RF_ID 2U
#define VESC_CHASSIS_LB_ID 3U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四个DM6220共用的MIT协议量程和初值, 轮组ID沿用上方定义
 */
struct Struct_Motor_DM_Init_Parameter
{
    float Angle_Max = DM_CHASSIS_ANGLE_MAX;                    // 位置协议范围, ±rad
    float Omega_Max = DM_CHASSIS_OMEGA_MAX;                    // 速度协议范围, ±rad/s
    float Torque_Max = DM_CHASSIS_TORQUE_MAX;                  // 转矩协议范围, ±N·m
    float K_P = DM_CHASSIS_MIT_K_P_DEFAULT;                    // 初始位置增益
    float K_D = DM_CHASSIS_MIT_K_D_DEFAULT;                    // 初始速度阻尼
    float Torque_Feedforward = DM_CHASSIS_MIT_TORQUE_DEFAULT;  // 初始前馈转矩, N·m
};

/**
 * @brief U8 KV100的STM32侧VESC驱动初始化配置, 不改写VESC内部Flash参数
 * @note 保留21极对及1440机械rpm对应30240ERPM的保护上限, 不代表16m/s一定可达
 * @note 实际车速取决于待实测轮径、传动及电调配置; 不随遥控挡位擅自提高电机限幅
 */
struct Struct_Motor_VESC_Init_Parameter
{
    uint8_t Pole_Pairs = 21U;        // 电机极对数
    float Gearbox_Rate = 1.0f;       // 驱动内转速换算比
    int32_t ERPM_Max = 30240;        // STM32侧目标电转速限幅, ERPM
    float Current_Max = 5.0f;        // STM32侧电流命令限幅, A
    float Current_Brake_Max = 5.0f;  // STM32侧制动电流命令限幅, A
};

/* Exported variables --------------------------------------------------------*/

// 底盘达妙DM6220舵向电机
extern Class_Motor_DM_Normal DM_Chassis_LF;
extern Class_Motor_DM_Normal DM_Chassis_RF;
extern Class_Motor_DM_Normal DM_Chassis_LB;
extern Class_Motor_DM_Normal DM_Chassis_RB;

// 舵轮底盘行进电机VESC
extern Class_Motor_VESC VESC_Chassis_LF;
extern Class_Motor_VESC VESC_Chassis_RF;
extern Class_Motor_VESC VESC_Chassis_LB;

/* Exported function declarations --------------------------------------------*/

// 机械臂CAN1：2325旋转轴、4340P大臂/小臂，位置速度模式
#define DM_ARM_ROTATE_ID 0x05U
#define DM_ARM_BIG_ID 0x06U
#define DM_ARM_SMALL_ID 0x07U
#define DM_ARM_ANGLE_MAX 12.5f
#define DM_ARM_JOINT_OMEGA_MAX 10.0f
#define DM_ARM_JOINT_TORQUE_MAX 28.0f
#define DM_ARM_ROTATE_OMEGA_MAX 200.0f
#define DM_ARM_ROTATE_TORQUE_MAX 10.0f
#define DM_ARM_ROTATE_MASTER_ID 0x55U
#define DM_ARM_BIG_MASTER_ID 0x66U
#define DM_ARM_SMALL_MASTER_ID 0x77U
extern Class_Motor_DM_Normal DM_Arm_Rotate;
extern Class_Motor_DM_Normal DM_Arm_Big;
extern Class_Motor_DM_Normal DM_Arm_Small;
void Motor_Init();
void Motor_Arm_Init(); // 机械臂接入时单独调用

#endif
