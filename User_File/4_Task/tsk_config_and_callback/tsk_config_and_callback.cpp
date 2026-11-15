/**
 * @file tsk_config_and_callback.cpp
 * @brief 存放用户定义的回调函数以及任务
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 *
 * @note 工程入口：设备初始化、CAN/UART回调分发和裸机周期任务调度。
 * @note main先Task_Init，再在while循环调用Task_Loop，周期参数见Struct_Task_Parameter。
 * @note 10ms处理接收/控制/VOFA，100ms检查电机并打印；500ms/1s保留按需诊断入口。
 * @note AUTOMATIC需先Upper_Configure_Speed配置缩放、限速和超时，默认保持失能。
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/

static Class_Task_Control Task_Control;

// 全局初始化完成标志位, 保留HAL和驱动兼容入口
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

static void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

static void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

static void UART4_Callback(uint8_t *Buffer, uint16_t Length);

static void UART7_Callback(uint8_t *Buffer, uint16_t Length);

static void Upper_Frame_Check_Callback(const Struct_Upper_Data &Data);

static void UART9_Vofa_Callback(uint8_t *Buffer, uint16_t Length);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 失能时配置UART7原始速度字段的物理单位、限速和控制超时
 * @param parameter 双方协议约定的缩放与限制; 默认全零, 不提供猜测值
 * @return bool 参数及失能条件均满足时返回true
 * @note 只启用速度接口, 不启动运动, 不改变原始32字节线协议
 */
bool Class_Task_Control::Configure_Upper_Speed(const Struct_Upper_Speed_Parameter &parameter)
{
    if (Remote_Chassis_Command.Motor_Enable != 0U ||
        DM_Chassis_LF.Get_Motor_Enable() || DM_Chassis_RF.Get_Motor_Enable() || DM_Chassis_LB.Get_Motor_Enable() ||
        VESC_Chassis_LF.Get_Motor_Enable() || VESC_Chassis_RF.Get_Motor_Enable() || VESC_Chassis_LB.Get_Motor_Enable()) { return false; }
    const float positive[] = {parameter.Linear_Scale, parameter.Angular_Scale, parameter.Translation_Speed_Max, parameter.Angular_Speed_Max};
    for (float value : positive) { if (!isfinite(value) || value <= 0.0f) { return false; } }
    const Struct_Remote_Parameter limit{};
    if (parameter.Translation_Speed_Max > limit.Translation_Speed_Max || parameter.Angular_Speed_Max > limit.Angular_Speed_Max ||
        parameter.Timeout_MS == 0U || parameter.Timeout_MS > UPPER_OFFLINE_TIMEOUT) { return false; }
    Upper_Speed_Parameter = parameter;
    const Enum_Upper_Control_Fault fault = Upper_Control.Fault;
    Upper_Control = {};
    Upper_Control.Fault = fault; // 配置不能代替在线遥控关闭总使能确认故障。
    Upper_Control.Configured = true;
    Upper_Request_Active = false;
    Upper.Set_CRC_Check_Enable(true);
    Chassis_Set_Command({});
    return true;
}

/**
 * @brief 在原上位机前台解帧路径逐帧检查速度, 一条超限帧即锁定
 */
void Class_Task_Control::Check_Upper_Frame(const Struct_Upper_Data &data)
{
    if (!Upper_Request_Active || !Upper_Control.Configured || Upper_Control.Fault != Upper_Control_Fault_NONE) { return; }
    if (uint32_t(HAL_GetTick() - data.First_Rx_Tick) >= Upper_Speed_Parameter.Timeout_MS)
    {
        Upper_Control.Fault = Upper_Control_Fault_TIMEOUT;
        return;
    }
    const float x = data.Speed_X * Upper_Speed_Parameter.Linear_Scale;
    const float y = data.Speed_Y * Upper_Speed_Parameter.Linear_Scale;
    const float w = data.Speed_W * Upper_Speed_Parameter.Angular_Scale;
    if (!isfinite(x) || !isfinite(y) || !isfinite(w) || hypotf(x, y) > Upper_Speed_Parameter.Translation_Speed_Max || fabsf(w) > Upper_Speed_Parameter.Angular_Speed_Max)
    {
        Upper_Control.Fault = Upper_Control_Fault_SPEED;
    }
}

/**
 * @brief 唯一任务对象的上位机有效帧检查适配, 在前台临界区外执行
 */
static void Upper_Frame_Check_Callback(const Struct_Upper_Data &Data)
{
    Task_Control.Check_Upper_Frame(Data);
}

/**
 * @brief 每10ms选择车体速度来源, 上位机仅在许可窗口内使用新鲜且校验通过的速度
 * @return Struct_Chassis_Command 默认零速失能; 所有速度均为车体m/s、rad/s
 * @note 上位机帧异常/断流/越界后锁定上位机控制, 在线遥控明确关闭总使能才清锁
 * @note 手动接管可执行新的遥控指令, 不清除上位机锁定或原底盘反转故障
 */
Struct_Chassis_Command Class_Task_Control::Select_Chassis_Command()
{
    const auto &remote = Remote_Chassis_Command;
    const auto &data = Upper.Get_Data();
    if (remote.Online != 0U && remote.Motor_Enable == 0U) { Upper_Control.Fault = Upper_Control_Fault_NONE; }
    if (remote.Online == 0U || remote.Motor_Enable == 0U)
    {
        Upper_Request_Active = false;
        Upper_Control.Status = Upper_Control_Status_DISABLED;
        return {};
    }
    if (remote.Control_Source == Remote_Control_Source_MANUAL)
    {
        Upper_Request_Active = false;
        Upper_Control.Status = Upper_Control_Status_DISABLED;
        return {remote.Speed_X, remote.Speed_Y, remote.Speed_W, true, Chassis_Command_Source_MANUAL};
    }
    if (remote.Control_Source == Remote_Control_Source_DEBUG_U8)
    {
        Upper_Request_Active = false;
        Upper_Control.Status = Upper_Control_Status_DISABLED;
        return {0.0f, remote.Speed_Y, 0.0f, true, Chassis_Command_Source_DEBUG_U8};
    }
    if (remote.Control_Source != Remote_Control_Source_AUTO)
    {
        Upper_Request_Active = false;
        Upper_Control.Status = Upper_Control_Status_DISABLED;
        return {};
    }
    if (Upper_Control.Fault != Upper_Control_Fault_NONE)
    {
        Upper_Control.Status = Upper_Control_Status_FAULT;
        return {};
    }
    if (!Upper_Control.Configured)
    {
        Upper_Control.Status = Upper_Control_Status_WAIT_CONFIG;
        return {};
    }
    if (Chassis_Get_Swerve_Fault())
    {
        Upper_Control.Fault = Upper_Control_Fault_CHASSIS;
        Upper_Control.Status = Upper_Control_Status_FAULT;
        return {};
    }
    if (!Upper_Request_Active)
    {
        // 不使用切入之前的缓存帧; 同周期到达的帧也保守地等待下一帧。
        Upper.Discard_Pending_Data(); // 残帧也属于旧窗口, 不能靠切入后的新尾字节启动。
        Upper_Request_Active = true;
        Upper_Frame_Barrier = data.Frame_Count;
        Upper_Invalid_Frame_Count = data.Invalid_Frame_Count;
        Upper_CRC_Error_Count = data.CRC_Error_Count;
        Upper_Overflow_Count = data.Buffer_Overflow_Count;
        Upper_Control.Status = Upper_Control_Status_WAIT_FRAME;
        return {};
    }
    if (!Upper.Get_CRC_Check_Enable() || data.Invalid_Frame_Count != Upper_Invalid_Frame_Count ||
        data.CRC_Error_Count != Upper_CRC_Error_Count || data.Buffer_Overflow_Count != Upper_Overflow_Count)
    {
        Upper_Control.Fault = Upper_Control_Fault_FRAME;
    }
    else if (!data.Valid || Upper.Get_Status() != Upper_Status_ONLINE || uint32_t(HAL_GetTick() - data.Last_Rx_Tick) >= Upper_Speed_Parameter.Timeout_MS)
    {
        // 首次尚无新速度时等待, 已经接入有效速度流后才将断流作为锁定故障。
        if (Upper_Control.Status == Upper_Control_Status_RUNNING) { Upper_Control.Fault = Upper_Control_Fault_TIMEOUT; }
        else { Upper_Control.Status = Upper_Control_Status_WAIT_FRAME; }
    }
    else if (data.Frame_Count != Upper_Frame_Barrier)
    {
        float x = data.Speed_X * Upper_Speed_Parameter.Linear_Scale;
        float y = data.Speed_Y * Upper_Speed_Parameter.Linear_Scale;
        float w = data.Speed_W * Upper_Speed_Parameter.Angular_Scale;
        if (isfinite(x) && isfinite(y) && isfinite(w) && hypotf(x, y) <= Upper_Speed_Parameter.Translation_Speed_Max && fabsf(w) <= Upper_Speed_Parameter.Angular_Speed_Max)
        {
            Upper_Control.Status = Upper_Control_Status_RUNNING;
            return {x, y, w, true, Chassis_Command_Source_UPPER};
        }
        Upper_Control.Fault = Upper_Control_Fault_SPEED;
    }
    if (Upper_Control.Fault != Upper_Control_Fault_NONE) { Upper_Control.Status = Upper_Control_Status_FAULT; }
    return {};
}

/**
 * @brief 在原10ms任务内向底盘提交唯一车体命令, 不计算位置或比赛步骤
 */
void Class_Task_Control::Update_Chassis_Command()
{
    Upper_Control.Command = Select_Chassis_Command();
    Chassis_Set_Command(Upper_Control.Command);
}

/**
 * @brief 上位机速度参数的全局前台配置入口
 */
bool Upper_Configure_Speed(const Struct_Upper_Speed_Parameter &parameter) { return Task_Control.Configure_Upper_Speed(parameter); }

/**
 * @brief 读取最终命令及上位机速度接入状态, 不刷新任何接收时间戳
 */
const Struct_Upper_Control_State &Upper_Get_Control_State() { return Task_Control.Get_Upper_Control_State(); }

/**
 * @brief CAN1接收回调函数
 *
 * @param Header CAN接收报文头
 * @param Buffer CAN接收数据
 */
static void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    if (Header.IdType != FDCAN_STANDARD_ID || Header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    switch (Header.Identifier)
    {
    case (DM_CHASSIS_LF_MASTER_ID):
    {
        DM_Chassis_LF.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_RF_MASTER_ID):
    {
        DM_Chassis_RF.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_LB_MASTER_ID):
    {
        DM_Chassis_LB.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_RB_MASTER_ID):
    {
        DM_Chassis_RB.CAN_RxCpltCallback();
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief CAN3接收回调函数
 *
 * @param Header CAN接收报文头
 * @param Buffer CAN接收数据
 */
static void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    if (Header.IdType != FDCAN_EXTENDED_ID || Header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    switch (Header.Identifier & 0xffU)
    {
    case (VESC_CHASSIS_LF_ID):
    {
        VESC_Chassis_LF.CAN_RxCpltCallback();
        break;
    }
    case (VESC_CHASSIS_RF_ID):
    {
        VESC_Chassis_RF.CAN_RxCpltCallback();
        break;
    }
    case (VESC_CHASSIS_LB_ID):
    {
        VESC_Chassis_LB.CAN_RxCpltCallback();
        break;
    }

    default:
    {
        break;
    }
    }
}

/**
 * @brief UART4接收回调函数
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度
 */
static void UART4_Callback(uint8_t *Buffer, uint16_t Length)
{
    HI06.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief UART7接收上位机字节, 协议对象只缓存, 由10ms前台完成解析
 * @param Buffer 本次DMA接收区
 * @param Length 本次接收字节数, 不代表32字节帧边界
 */
static void UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    Upper.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief USART9接收VOFA调参指令
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度, Byte
 */
static void UART9_Vofa_Callback(uint8_t *Buffer, uint16_t Length)
{
    Debug_UART9_RxCpltCallback(Buffer, Length);
}

/**
 * @brief 每10ms调用一次
 */
void Class_Task_Control::Task10ms_Callback()
{
    // 先检查上一速度流的期限, 新解析的数据不能掩盖已经发生的断流或前台长时间停顿。
    if (Upper_Control.Status == Upper_Control_Status_RUNNING &&
        uint32_t(HAL_GetTick() - Upper.Get_Data().Last_Rx_Tick) >= Upper_Speed_Parameter.Timeout_MS)
    {
        Upper_Control.Fault = Upper_Control_Fault_TIMEOUT;
    }

    HI06.TIM_10ms_PeriodElapsedCallback();            // 解析HI06数据并更新在线状态
    Upper.TIM_10ms_PeriodElapsedCallback();           // 解析上位机数据并更新在线状态

    Remote_10ms_PeriodElapsedCallback();              // 接收遥控数据并更新底盘指令

    // USART9调参命令只在前台应用, 并先于本周期舵轮控制生效
    Debug_10ms_PeriodElapsedCallback();               // 应用串口9暂存的MIT调参命令

    Update_Chassis_Command();                       // 选择遥控/上位机速度并提交唯一底盘命令

    Chassis_Swerve_10ms_PeriodElapsedCallback();      // 舵轮回零、运动学解算及DM控制
    Chassis_VESC_10ms_PeriodElapsedCallback();        // 发送VESC行进电机控制指令
    Debug_Vofa_10ms_PeriodElapsedCallback();          // 串口9发送VOFA调试数据
}

/**
 * @brief 每100ms调用一次
 */
void Class_Task_Control::Task100ms_Callback()
{
    Chassis_100ms_PeriodElapsedCallback();            // 检查活动轮组DM及VESC在线状态
    // if (Remote_Chassis_Command.Mode == REMOTE_MODE_DEBUG) { Debug_UART10_Print_U8_Test_Data(); }
    // else { Debug_UART10_Print_Swerve_Data(); }         // 正常底盘诊断与DEBUG独立U8诊断错开
}

/**
 * @brief 每500ms调用一次
 */
void Class_Task_Control::Task500ms_Callback()
{
    Debug_UART10_Print_IMU_Data();                 // HI06姿态：Roll/Pitch/Yaw角度和温度
    // 上位机速度接口; 按需开启一种打印, 与其他USART10输出错开
    // Debug_UART10_Print_Upper_Data();               // 上位机原始速度整数、数据区和扩展字节
    // Debug_UART10_Print_Upper_Status_Data();        // 上位机在线状态、数据年龄和接收统计
}

/**
 * @brief 每1s调用一次
 */
void Class_Task_Control::Task1s_Callback()
{
    // Debug_UART10_Print_Remote_Data();              // 遥控在线状态、模式、通道和功能按键
}

/**
 * @brief 初始化任务
 */
void Class_Task_Control::Init()
{
    init_finished = false;
    Upper_Speed_Parameter = {};
    Upper_Control = {};
    Upper_Request_Active = false;

    // 时间戳
    SYS_Timestamp.Init(&htim5);

    // HI06和UART4空闲中断DMA接收
    HI06.Init();
    UART_Init(&huart4, UART4_Callback);

    // UART7上位机速度接收, 控制链默认校验CRC; 缩放和超时未配置时保持失能
    Upper.Init();
    Upper.Set_CRC_Check_Enable(true);
    Upper.Set_Frame_Callback(Upper_Frame_Check_Callback);
    UART_Init(&huart7, UART7_Callback);
    // 其余UART空闲中断DMA接收
    UART_Init(&huart9, UART9_Vofa_Callback);
    UART_Init(&huart10, nullptr);

    // USART9 VOFA调试任务, 默认观察前中轮DM ID 1和VESC ID 1
    Debug_Init();

    // 遥控数据和NRF24L01
    Remote_Init();

    // 电机初始化
    Motor_Init();

    // 底盘初始化
    Chassis_Init();

    // 电机CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Extended_Init(&hfdcan3, CAN3_Callback);

    // TIM5用于时间戳溢出计时
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t now_tick = HAL_GetTick();                // 读取当前毫秒时基
    Task_10ms_Previous_Tick = now_tick;
    Task_100ms_Previous_Tick = now_tick;
    Task_500ms_Previous_Tick = now_tick;
    Task_1s_Previous_Tick = now_tick;

    // 标记初始化完成
    init_finished = true;

    // 默认失能达妙电机
    DM_Chassis_LF.CAN_Send_Exit();
    DM_Chassis_RF.CAN_Send_Exit();
    DM_Chassis_LB.CAN_Send_Exit();
    DM_Chassis_RB.CAN_Send_Exit();
}

/**
 * @brief 前台循环任务
 */
void Class_Task_Control::Loop()
{
    uint32_t now_tick = HAL_GetTick();

    if ((uint32_t)(now_tick - Task_10ms_Previous_Tick) >= Parameter.Task_10ms_Period)
    {
        Task_10ms_Previous_Tick = now_tick;
        Task10ms_Callback();                          // 执行10ms控制与通信任务
    }

    if ((uint32_t)(now_tick - Task_100ms_Previous_Tick) >= Parameter.Task_100ms_Period)
    {
        Task_100ms_Previous_Tick = now_tick;
        Task100ms_Callback();                         // 执行100ms在线检查与诊断任务
    }

    if ((uint32_t)(now_tick - Task_500ms_Previous_Tick) >= Parameter.Task_500ms_Period)
    {
        Task_500ms_Previous_Tick = now_tick;
        Task500ms_Callback();                         // 执行500ms低频打印任务
    }

    if ((uint32_t)(now_tick - Task_1s_Previous_Tick) >= Parameter.Task_1s_Period)
    {
        Task_1s_Previous_Tick = now_tick;
        Task1s_Callback();                            // 执行1s预留打印任务
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim 定时器句柄
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
    }
}

/* 兼容Core/main.c中的C入口 ---------------------------------------------------*/

void Task_Init()
{
    Task_Control.Init();
}

void Task_Loop()
{
    Task_Control.Loop();
}
