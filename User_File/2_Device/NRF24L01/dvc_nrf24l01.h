/**
 * @file dvc_nrf24l01.h
 * @brief NRF24L01无线驱动：GPIO模拟SPI、寄存器配置和原始报文收发。
 * @note tsk_remote先初始化模块，再每10ms查询并取出接收报文。
 * @note 本工程使用31字节载荷；引脚见实现文件，协议解释见pro_sbus。
 */

#ifndef __DVC_NRF24L01_H
#define __DVC_NRF24L01_H

#include "main.h"
#include "nrf24l01_define.h"

/*外部可调用全局数组***********/
#define NRF24L01_TX_PACKET_WIDTH 31 // 发送数据包宽度，范围：1~32字节
#define NRF24L01_RX_PACKET_WIDTH 31 // 接收通道0数据包宽度，范围：1~32字节

typedef struct
{
    uint8_t NRF24L01_TxAddress[5];
    uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];
    uint8_t NRF24L01_RxAddress[5];
    uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];

} NRF24L01_Type;

/***********外部可调用全局数组*/

/*函数声明*********************/

/*********************函数声明*/

/**
 * @brief NRF24L01原始载荷收发, 报文内容交给pro_sbus解释
 */
class Class_NRF24L01
{
private:
    // 内部函数: GPIO模拟SPI
    void NRF24L01_W_CE(uint8_t BitValue);
    void NRF24L01_W_CSN(uint8_t BitValue);
    void NRF24L01_W_SCK(uint8_t BitValue);
    void NRF24L01_W_MOSI(uint8_t BitValue);
    void NRF24L01_GPIO_Init(void);
    uint8_t NRF24L01_R_MISO(void);
    uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte);
    // 内部函数: 芯片指令与载荷收发
    uint8_t NRF24L01_ReadReg(uint8_t RegAddress);
    void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
    void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data);
    void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
    void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count);
    void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count);
    void NRF24L01_FlushTx(void);
    void NRF24L01_FlushRx(void);
    uint8_t NRF24L01_ReadStatus(void);

    // 内部函数: 电源与收发状态切换
    void NRF24L01_PowerDown(void);
    void NRF24L01_StandbyI(void);
    void NRF24L01_Rx(void);
    void NRF24L01_Tx(void);

public:
    // 读写变量: 收发地址及载荷缓存

    NRF24L01_Type NRF24L01_data;

    // 初始化与收发接口

    void NRF24L01_Init(void);
    uint8_t NRF24L01_Send(void);
    uint8_t NRF24L01_Receive(void);
    void NRF24L01_UpdateRxAddress(void);
};

#endif /* __DVC_NRF24L01_H */

