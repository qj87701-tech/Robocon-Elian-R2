/**
 * @file tsk_remote.cpp
 * @brief 遥控器接收和底盘指令换算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 遥控任务：接收NRF报文、换算摇杆速度并选择MANUAL/AUTOMATIC/DEBUG。
 * @note Task_Init调用Remote_Init，前台每10ms更新Remote_Chassis_Command。
 * @note F2平移四挡4/8/12/16m/s，F1独立旋转挡位；DEBUG只使用右摇杆Y。
 * @note 通道范围和死区见Struct_Remote_Parameter，报文布局见pro_sbus。
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_remote.h"

#include <math.h>

/* Private variables ---------------------------------------------------------*/

static Class_Remote_Control Remote_Control;

/* Exported variables --------------------------------------------------------*/

Class_NRF24L01 NRF24L01;
Struct_Remote_Chassis_Command Remote_Chassis_Command;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 将遥控通道值换算为底盘速度
 *
 * @param Channel 遥控通道值, 200~1800
 * @return float 单轴底盘目标速度, m/s; 未满推按偏量线性输出
 * @note 遥控端F2已经把四挡满推偏量设为200/400/600/800, 此处不得再次乘挡位
 */
float Class_Remote_Control::Channel_To_Speed(uint16_t Channel)
{
    if (Channel < Parameter.Channel_Min)
    {
        Channel = Parameter.Channel_Min;
    }
    else if (Channel > Parameter.Channel_Max)
    {
        Channel = Parameter.Channel_Max;
    }

    float speed = (static_cast<float>(Channel) - Parameter.Channel_Middle) * Parameter.Translation_Speed_Max / (Parameter.Channel_Max - Parameter.Channel_Middle);
    if (fabsf(speed) < Parameter.Speed_Dead_Zone)
    {
        speed = 0.0f;
    }

    return speed;
}

/**
 * @brief 将遥控旋转通道值换算为底盘角速度
 *
 * @param Channel 遥控通道值, 200~1800
 * @return float 底盘目标角速度, rad/s
 * @note 遥控端F1独立缩放旋转通道; 满推角速度等于同级线速度/轮组中心半径
 */
float Class_Remote_Control::Channel_To_Angular_Speed(uint16_t Channel)
{
    if (Channel < Parameter.Channel_Min)
    {
        Channel = Parameter.Channel_Min;
    }
    else if (Channel > Parameter.Channel_Max)
    {
        Channel = Parameter.Channel_Max;
    }

    float speed = (static_cast<float>(Channel) - Parameter.Channel_Middle) * Parameter.Angular_Speed_Max / (Parameter.Channel_Max - Parameter.Channel_Middle);
    if (fabsf(speed) < Parameter.Angular_Speed_Dead_Zone)
    {
        speed = 0.0f;
    }

    return speed;
}

/**
 * @brief 根据遥控在线、模式和使能状态更新底盘控制指令
 */
void Class_Remote_Control::Update_Chassis_Command()
{
    Remote_Chassis_Command.Online = SBUS_Receive.SBUS_Data.Online;
    Remote_Chassis_Command.Mode = SBUS_Receive.SBUS_Data.Mode;
    Remote_Chassis_Command.Motor_Enable = SBUS_Receive.SBUS_Data.Motor_Enable;

    Update_Control_Source();
    if (Remote_Chassis_Command.Online == 0 || Remote_Chassis_Command.Motor_Enable == 0 ||
        (Remote_Chassis_Command.Control_Source != Remote_Control_Source_MANUAL && Remote_Chassis_Command.Control_Source != Remote_Control_Source_DEBUG_U8))
    {
        Remote_Chassis_Command.Speed_X = 0.0f;
        Remote_Chassis_Command.Speed_Y = 0.0f;
        Remote_Chassis_Command.Speed_W = 0.0f;
        return;
    }

    Remote_Chassis_Command.Speed_Y = Channel_To_Speed(SBUS_Receive.SBUS_Data.Channel[0]);
    if (Remote_Chassis_Command.Control_Source == Remote_Control_Source_DEBUG_U8)
    {
        // DEBUG沿用正常Y轴换算及F2挡位, 其余摇杆不影响U8单独测试。
        Remote_Chassis_Command.Speed_X = 0.0f;
        Remote_Chassis_Command.Speed_W = 0.0f;
        return;
    }
    Remote_Chassis_Command.Speed_X = Channel_To_Speed(SBUS_Receive.SBUS_Data.Channel[1]);
    // 遥控端保持原方形通道范围; 合速度取较大轴幅值, 方向仍按原X/Y比例。
    // 该映射不需要识别挡位: 四挡最大轴幅值自然对应4/8/12/16m/s。
    // 45°时等效两轴除√2, 其他角度连续同比缩放; 同一方向半幅输入得到半速。
    float speed = hypotf(Remote_Chassis_Command.Speed_X, Remote_Chassis_Command.Speed_Y);
    if (speed > 0.0f)
    {
        float ratio = fmaxf(fabsf(Remote_Chassis_Command.Speed_X), fabsf(Remote_Chassis_Command.Speed_Y)) / speed;
        Remote_Chassis_Command.Speed_X *= ratio;
        Remote_Chassis_Command.Speed_Y *= ratio;
    }
    Remote_Chassis_Command.Speed_W = Channel_To_Angular_Speed(SBUS_Receive.SBUS_Data.Channel[3]);
}

/**
 * @brief 保存本次控制权选择及事件编号, 不直接使能电机
 * @param Source 遥控或自主控制来源
 */
void Class_Remote_Control::Select_Control_Source(Enum_Remote_Control_Source Source)
{
    Remote_Chassis_Command.Control_Source = Source;
    Remote_Chassis_Command.Control_Request_Count++;
}

/**
 * @brief 根据遥控页面选择控制权, MANUAL驾驶、AUTOMATIC上位机速度、DEBUG测试U8
 * @note 只在来源变化时生成事件; 功能键不参与选择, 离线或其他页面撤销许可
 */
void Class_Remote_Control::Update_Control_Source()
{
    const auto &remote = SBUS_Receive.SBUS_Data;
    Enum_Remote_Control_Source source = Remote_Control_Source_NONE;
    if (remote.Online != 0U)
    {
        if (remote.Mode == REMOTE_MODE_MANUAL) { source = Remote_Control_Source_MANUAL; }
        else if (remote.Mode == REMOTE_MODE_AUTOMATIC) { source = Remote_Control_Source_AUTO; }
        else if (remote.Mode == REMOTE_MODE_DEBUG) { source = Remote_Control_Source_DEBUG_U8; }
    }
    if (source != Remote_Chassis_Command.Control_Source) { Select_Control_Source(source); }
}

/**
 * @brief 初始化遥控器接收及控制权安全状态
 */
void Class_Remote_Control::Init()
{
    SBUS_Receive.Init();
    Remote_Chassis_Command = {};
    NRF24L01.NRF24L01_Init();
    Update_Chassis_Command();
}

/**
 * @brief 每10ms接收NRF24L01数据、解析SBUS并更新底盘指令
 */
void Class_Remote_Control::TIM_10ms_PeriodElapsedCallback()
{
    SBUS_Receive.TIM_10ms_PeriodElapsedCallback();

    if (NRF24L01.NRF24L01_Receive() == 1)
    {
        SBUS_Receive.Data_Process(NRF24L01.NRF24L01_data.NRF24L01_RxPacket);
    }

    Update_Chassis_Command();
}

/* 兼容原有任务入口 ----------------------------------------------------------*/

void Remote_Init()
{
    Remote_Control.Init();
}

void Remote_10ms_PeriodElapsedCallback()
{
    Remote_Control.TIM_10ms_PeriodElapsedCallback();
}
