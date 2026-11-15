/**
 * @file tsk_remote.h
 * @brief 遥控器接收和底盘指令换算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 遥控任务：接收NRF报文、换算摇杆速度并选择MANUAL/AUTOMATIC/DEBUG。
 * @note Task_Init调用Remote_Init，前台每10ms更新Remote_Chassis_Command。
 * @note F2平移四挡4/8/12/16m/s，F1独立旋转挡位；DEBUG只使用右摇杆Y。
 * @note 通道范围和死区见Struct_Remote_Parameter，报文布局见pro_sbus。
 */

#ifndef TSK_REMOTE_H
#define TSK_REMOTE_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Protocol/pro_sbus/pro_sbus.h"
#include "2_Device/NRF24L01/dvc_nrf24l01.h"
#include "3_Module/3swerve_chassis/tsk_chassis_resolve.h"

/* Exported macros -----------------------------------------------------------*/

#define REMOTE_MODE_DEBUG 1
#define REMOTE_MODE_MANUAL 2
#define REMOTE_MODE_AUTOMATIC 3

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控屏选择的底盘控制权, 独立于原始页面模式
 */
enum Enum_Remote_Control_Source : uint8_t
{
    Remote_Control_Source_NONE = 0,
    Remote_Control_Source_MANUAL, // MANUAL页面: 摇杆驾驶
    Remote_Control_Source_AUTO,   // AUTOMATIC页面: 上位机提供车体速度, STM32执行三舵轮解算
    Remote_Control_Source_DEBUG_U8 // DEBUG页面: 右摇杆Y单独测试所选U8
};

/**
 * @brief 遥控器底盘控制指令
 */
struct Struct_Remote_Chassis_Command
{
    // 车体左右速度, m/s, 向右为正
    float Speed_X;
    // 车体前后速度, m/s, 向前为正
    float Speed_Y;
    // 车体旋转角速度, rad/s, 逆时针为正
    float Speed_W;
    // 有效遥控报文是否仍在在线期限内
    uint8_t Online;
    // 遥控页面模式, 对应REMOTE_MODE宏
    uint8_t Mode;
    // 遥控总使能, 0撤销电机控制
    uint8_t Motor_Enable;
    Enum_Remote_Control_Source Control_Source = Remote_Control_Source_NONE;
    uint32_t Control_Request_Count = 0U; // 控制权变化计数, 同一页面不重复申请
};

/**
 * @brief 遥控速度换算参数, 四挡比例由遥控端映射, 主控只换算一次
 */
struct Struct_Remote_Parameter
{
    uint16_t Channel_Min = 200U;              // 通道下限
    uint16_t Channel_Middle = 1000U;          // 通道中值
    uint16_t Channel_Max = 1800U;             // 通道上限
    float Translation_Speed_Max = 16.0f;      // 极速档合平移命令上限, m/s; F2四挡为4/8/12/16
    float Angular_Speed_Max = Translation_Speed_Max / SWERVE_CHASSIS_RADIUS_M; // F1独立换挡, 角速度上限rad/s=同级轮周速度/暂定中心半径
    float Speed_Dead_Zone = 0.05f;            // 平移死区, m/s
    float Angular_Speed_Dead_Zone = 0.12f;    // 旋转死区, rad/s
};

/**
 * @brief 10ms遥控接收与指令换算, 复用全局NRF24L01和SBUS_Receive
 * @note Remote_Chassis_Command保留为各任务读取的指令快照
 */
class Class_Remote_Control
{
public:
    // 初始化相关函数

    void Init();

    // 周期回调函数

    void TIM_10ms_PeriodElapsedCallback();

private:
    // 内部函数: 通道换算与控制来源选择

    float Channel_To_Speed(uint16_t Channel);
    float Channel_To_Angular_Speed(uint16_t Channel);
    void Update_Chassis_Command();

    void Update_Control_Source();

    void Select_Control_Source(Enum_Remote_Control_Source Source);

    // 初始化相关常量

    const Struct_Remote_Parameter Parameter{};
};

/* Exported variables --------------------------------------------------------*/

extern Class_NRF24L01 NRF24L01;
extern Struct_Remote_Chassis_Command Remote_Chassis_Command;

/* Exported function declarations --------------------------------------------*/

void Remote_Init();

void Remote_10ms_PeriodElapsedCallback();

#endif
