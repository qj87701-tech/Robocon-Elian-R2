/**
 * @file pro_sbus.h
 * @brief NRF24L01遥控数据接收协议
 * @version 0.1
 * @date 2026-10-03 0.1 新建文档
 *
 * @note 遥控协议：从NRF24L01的31字节报文提取通道、模式、使能和功能键。
 * @note tsk_remote调用Data_Process解包，并每10ms检查是否掉线。
 * @note 这里只保留原始通道；物理速度换算与控制来源选择见tsk_remote。
 */

#ifndef __PRO_SBUS_H
#define __PRO_SBUS_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Exported macros -----------------------------------------------------------*/

#define SBUS_CHANNEL_NUM 16

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint16_t Channel[SBUS_CHANNEL_NUM];
    uint8_t Mode;
    uint8_t Motor_Enable;
    uint8_t Function[16];
    uint8_t Online;
} Struct_SBUS_Data;

/**
 * @brief 遥控原始帧解包与掉线计数, 不在此处换算底盘速度
 */
class Class_SBUS_Receive
{
private:
    // 内部变量: 未收到有效遥控帧的10ms计数

    uint8_t Alive_Count;

    // 内部函数

    void Data_Reset();

public:
    // 读变量: 原始遥控数据快照

    Struct_SBUS_Data SBUS_Data;

    // 初始化相关函数

    void Init();
    // 解帧及周期回调函数

    uint8_t Data_Process(const uint8_t *data);
    void TIM_10ms_PeriodElapsedCallback();
};

/* Exported variables --------------------------------------------------------*/

extern Class_SBUS_Receive SBUS_Receive;

#endif
