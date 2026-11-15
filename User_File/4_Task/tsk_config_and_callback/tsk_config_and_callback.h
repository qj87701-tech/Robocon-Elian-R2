/**
 * @file tsk_config_and_callback.h
 * @brief 存放用户定义的回调函数以及任务
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 *
 * @note 工程入口：设备初始化、CAN/UART回调分发和裸机周期任务调度。
 * @note main先Task_Init，再在while循环调用Task_Loop，周期参数见Struct_Task_Parameter。
 * @note 10ms处理接收/控制/VOFA，100ms检查电机并打印；500ms/1s保留按需诊断入口。
 * @note AUTOMATIC需先Upper_Configure_Speed配置缩放、限速和超时，默认保持失能。
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "3_Module/3swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_debug/tsk_debug.h"
#include "4_Task/tsk_remote/tsk_remote.h"

#include "fdcan.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裸机周期调度参数, 单位ms
 */
struct Struct_Task_Parameter
{
    uint32_t Task_10ms_Period = 10U;    // 控制与VOFA调度周期, ms
    uint32_t Task_100ms_Period = 100U;  // 在线检查与诊断周期, ms
    uint32_t Task_500ms_Period = 500U;  // 预留低频调度周期, ms
    uint32_t Task_1s_Period = 1000U;    // 预留低频调度周期, ms
};

/**
 * @brief UART7速度字段的显式物理单位与控制约束, 默认未配置
 * @note 缩放=每个原始整数代表的m/s或rad/s; 坐标必须由上位机转换成车体系
 */
struct Struct_Upper_Speed_Parameter
{
    float Linear_Scale = 0.0f;          // Speed_X/Y原始整数到m/s的倍率, 由双方协议约定
    float Angular_Scale = 0.0f;         // Speed_W原始整数到rad/s的倍率, 由双方协议约定
    float Translation_Speed_Max = 0.0f; // 允许的合平移速度, m/s, 不大于遥控极速上限
    float Angular_Speed_Max = 0.0f;     // 允许的角速度幅值, rad/s, 不大于原控制上限
    uint32_t Timeout_MS = 0U;           // 控制断流超时, ms; 按实际发送周期配置, 不大于协议在线有效期
};

enum Enum_Upper_Control_Status : uint8_t
{
    Upper_Control_Status_DISABLED = 0,
    Upper_Control_Status_WAIT_CONFIG,
    Upper_Control_Status_WAIT_FRAME,
    Upper_Control_Status_RUNNING,
    Upper_Control_Status_FAULT,
};

enum Enum_Upper_Control_Fault : uint8_t
{
    Upper_Control_Fault_NONE = 0,
    Upper_Control_Fault_FRAME,
    Upper_Control_Fault_TIMEOUT,
    Upper_Control_Fault_SPEED,
    Upper_Control_Fault_CHASSIS,
};

/**
 * @brief 上位机速度接入状态与最终命令快照, 可供后续上位机应答和现有诊断读取
 */
struct Struct_Upper_Control_State
{
    bool Configured = false;
    Enum_Upper_Control_Status Status = Upper_Control_Status_DISABLED;
    Enum_Upper_Control_Fault Fault = Upper_Control_Fault_NONE;
    Struct_Chassis_Command Command{};
};

/**
 * @brief 裸机前台任务调度, 硬件接收回调仍使用普通函数入口
 * @note 全工程仅使用一个调度对象; 不引入RTOS和补发追赶
 */
class Class_Task_Control
{
public:
    // 初始化相关函数

    void Init();

    void Loop();

    // 写变量: 失能时配置上位机速度接口

    bool Configure_Upper_Speed(const Struct_Upper_Speed_Parameter &Speed_Parameter);

    // 读变量

    const Struct_Upper_Control_State &Get_Upper_Control_State() const { return Upper_Control; }

    // 有效帧回调, 在前台逐帧检查

    void Check_Upper_Frame(const Struct_Upper_Data &Data);

private:
    // 内部函数: 各周期任务

    void Task10ms_Callback();

    void Task100ms_Callback();

    void Task500ms_Callback();

    void Task1s_Callback();

    // 内部函数: 控制来源与命令选择

    void Update_Chassis_Command();

    Struct_Chassis_Command Select_Chassis_Command();

    // 初始化相关常量

    const Struct_Task_Parameter Parameter{};
    Struct_Upper_Speed_Parameter Upper_Speed_Parameter{};
    // 内部变量: 许可、故障和接收帧边界

    Struct_Upper_Control_State Upper_Control{};
    bool Upper_Request_Active = false;            // 当前已进入上位机控制许可窗口
    uint32_t Upper_Frame_Barrier = 0U;            // 切入/重新使能前已有帧不得直接驱动
    uint32_t Upper_Invalid_Frame_Count = 0U;
    uint32_t Upper_CRC_Error_Count = 0U;
    uint32_t Upper_Overflow_Count = 0U;
    uint32_t Task_10ms_Previous_Tick = 0U;     // 上次调度时刻, ms
    uint32_t Task_100ms_Previous_Tick = 0U;
    uint32_t Task_500ms_Previous_Tick = 0U;
    uint32_t Task_1s_Previous_Tick = 0U;
};

/* Exported variables --------------------------------------------------------*/

// HAL接收与驱动初始化使用的兼容标志
extern bool init_finished;

/**
 * @brief 前台配置UART7速度字段缩放、物理限速及断流超时, 必须先关闭总使能
 * @return bool 参数是否合法并已应用; 默认不配置, 不会自动启用上位机运动
 */
bool Upper_Configure_Speed(const Struct_Upper_Speed_Parameter &Speed_Parameter);

/**
 * @brief 读取接入状态及最终命令, 用于前台应答和诊断
 */
const Struct_Upper_Control_State &Upper_Get_Control_State();
extern "C" {
#endif

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化用户设备、通信总线和裸机任务时间基准
 */
void Task_Init();

/**
 * @brief 裸机前台循环, 按时间基准调度各周期回调
 */
void Task_Loop();

#ifdef __cplusplus
}
#endif

#endif
