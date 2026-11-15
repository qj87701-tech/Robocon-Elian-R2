/**
 * @file tsk_debug.h
 * @brief 调试数据输出任务
 * @version 0.1
 * @date 2026-10-11 0.1 新建文档
 *
 * @note 调试任务：UART9输出27路VOFA波形和接收DM调参，USART10输出文字诊断。
 * @note 10ms应用调参并发送波形；100ms按正常底盘或DEBUG模式选择文字打印。
 * @note I0~I7为U8/VESC，I8~I23为DM及回零状态，I24~I26为U8停车诊断，通道明细见Update_Vofa_Data。
 * @note 观察轮组、调参名字与发送缓冲见本头文件，DMA忙时本次打印可能跳过。
 */

#ifndef TSK_DEBUG_H
#define TSK_DEBUG_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "3_Module/3swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "3_Module/3swerve_chassis/tsk_chassis_resolve.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define DEBUG_VOFA_DATA_NUMBER 27U
#define DEBUG_VOFA_COMMAND_NUMBER 3U
#define DEBUG_UPPER_UART10_BUFFER_SIZE 256U

#define DEBUG_RAD_TO_DEG 57.29577951308232f
#define DEBUG_RAD_S_TO_RPM 9.54929658551372f
#define DEBUG_CELSIUS_TO_KELVIN 273.15f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 调试数据与打印封装, 生产对象须持续存活以保持DMA缓冲区有效
 * @note UART9接收仅暂存命令; 前台10ms应用命令并独立发送VOFA快照
 */
class Class_Debug_Control
{
public:
    // 初始化相关函数

    void Init();

    // 串口接收回调, 只暂存调参命令

    void UART9_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

    // 周期回调函数: 应用调参及发送VOFA

    void TIM_10ms_PeriodElapsedCallback();

    void Vofa_TIM_10ms_PeriodElapsedCallback();

    // 写变量: 正常模式的观察轮组

    void Set_Swerve_Wheel(Enum_Swerve_Wheel Wheel);

    // 文字打印接口, 在低频前台按需选择

    void UART10_Print_Remote_Data();

    void UART10_Print_Swerve_Data();

    void UART10_Print_U8_Test_Data();

    void UART10_Print_Upper_Control_Data();

    void UART10_Print_IMU_Data();

    void UART10_Print_IMU_Attitude_Data();

    void UART10_Print_IMU_Acceleration_Data();

    void UART10_Print_IMU_Gyroscope_Data();

    void UART10_Print_IMU_Status_Data();

    void UART10_Print_Upper_Data();

    void UART10_Print_Upper_Status_Data();

private:
    // 内部函数

    void Update_Vofa_Data();
    bool Command_Is_Complete(const uint8_t *Buffer, uint16_t Length);
    void UART10_Float_To_String(float Value, char *String);

    // 初始化相关常量: 调参字典和设备绑定

    enum Enum_Debug_Vofa_Command : uint8_t
    {
        Debug_Vofa_Command_DM_K_P = 0,
        Debug_Vofa_Command_DM_K_D,
        Debug_Vofa_Command_DM_Torque_Feedforward,
    };

    // 按前中ID1、左后ID2、右后ID3绑定motor_init既有对象。
    Class_Vofa_UART Debug_Vofa_UART9;

    Class_Motor_DM_Normal *Debug_DM_Motor[Swerve_Wheel_Number] = {
        &DM_Chassis_LF,
        &DM_Chassis_RF,
        &DM_Chassis_LB,
    };

    Class_Motor_VESC *Debug_VESC_Motor[Swerve_Wheel_Number] = {
        &VESC_Chassis_LF,
        &VESC_Chassis_RF,
        &VESC_Chassis_LB,
    };

    // 内部变量: 波形快照与暂存命令

    Enum_Swerve_Wheel Debug_Swerve_Wheel = Swerve_Wheel_F;
    float Debug_Vofa_Data[DEBUG_VOFA_DATA_NUMBER]{};
    const char *Debug_Vofa_Command_Name[DEBUG_VOFA_COMMAND_NUMBER] = {"dm_kp", "dm_kd", "dm_tau_ff"};
    volatile int32_t Debug_Pending_Command_Index = -1;
    volatile float Debug_Pending_Command_Value = 0.0f;
    volatile uint32_t Debug_Pending_Command_Update_Count = 0U;
    uint32_t Debug_Applied_Command_Update_Count = 0U;

    // 各打印函数独立持有原尺寸缓冲区; Init不重置在途DMA缓冲区
    char UART10_Upper_Data_Tx_Data[2][DEBUG_UPPER_UART10_BUFFER_SIZE]{};
    uint8_t UART10_Upper_Data_Tx_Index = 0U;
    char UART10_Upper_Status_Data_Tx_Data[2][DEBUG_UPPER_UART10_BUFFER_SIZE]{};
    uint8_t UART10_Upper_Status_Data_Tx_Index = 0U;
    char UART10_IMU_Attitude_Data_Tx_Data[2][96]{};
    uint8_t UART10_IMU_Attitude_Data_Tx_Index = 0U;
    char UART10_IMU_Acceleration_Data_Tx_Data[2][96]{};
    uint8_t UART10_IMU_Acceleration_Data_Tx_Index = 0U;
    char UART10_IMU_Gyroscope_Data_Tx_Data[2][96]{};
    uint8_t UART10_IMU_Gyroscope_Data_Tx_Index = 0U;
    char UART10_IMU_Status_Data_Tx_Data[2][160]{};
    uint8_t UART10_IMU_Status_Data_Tx_Index = 0U;
    char UART10_Remote_Data_Tx_Data[160]{};
    char UART10_Swerve_Data_Tx_Data[512]{};
    // 仅在发送成功后切换缓冲, BUSY不覆盖仍由DMA使用的内容。
    char UART10_Upper_Control_Tx_Data[2][192]{};
    uint8_t UART10_Upper_Control_Tx_Index = 0U;
    char UART10_U8_Test_Tx_Data[2][640]{};
    uint8_t UART10_U8_Test_Tx_Index = 0U;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Debug_Init();

void Debug_UART9_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

void Debug_10ms_PeriodElapsedCallback();

void Debug_Vofa_10ms_PeriodElapsedCallback();

void Debug_Set_Swerve_Wheel(Enum_Swerve_Wheel Wheel);

void Debug_UART10_Print_Remote_Data();

void Debug_UART10_Print_Swerve_Data();
/**
 * @brief 前台100ms调用, DEBUG模式下替代正常底盘诊断打印
 */
void Debug_UART10_Print_U8_Test_Data();

void Debug_UART10_Print_IMU_Data();

void Debug_UART10_Print_IMU_Attitude_Data();

void Debug_UART10_Print_IMU_Acceleration_Data();

void Debug_UART10_Print_IMU_Gyroscope_Data();

void Debug_UART10_Print_IMU_Status_Data();

/**
 * @brief USART10打印上位机原始速度整数及数据字节, 按需在低频前台回调调用
 */
void Debug_UART10_Print_Upper_Data();

/**
 * @brief USART10打印上位机在线、数据年龄及接收统计, 默认不自动调用
 */
void Debug_UART10_Print_Upper_Status_Data();

/**
 * @brief 按需从前台打印上位机速度接入状态, 与其他USART10输出错开
 */
void Debug_UART10_Print_Upper_Control_Data();

#endif
