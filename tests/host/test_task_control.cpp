#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "test_assert.h"
#include "1_Middleware/Hipnuc/hipnuc_sample.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_sbus/pro_sbus.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Motor/Motor_VESC/dvc_motor_vesc.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "3_Module/3swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_config_and_callback/tsk_config_and_callback.h"
#include "4_Task/tsk_debug/tsk_debug.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

GPIO_TypeDef Fake_GPIOF_Instance{0xF00DU};
FDCAN_HandleTypeDef hfdcan1{reinterpret_cast<void *>(0x10000U)};
FDCAN_HandleTypeDef hfdcan2{reinterpret_cast<void *>(0x20000U)};
FDCAN_HandleTypeDef hfdcan3{reinterpret_cast<void *>(0x30000U)};
TIM_HandleTypeDef htim5{TIM5};
UART_HandleTypeDef huart1{USART1};
UART_HandleTypeDef huart4{UART4};
UART_HandleTypeDef huart7{UART7};
UART_HandleTypeDef huart9{UART9};
UART_HandleTypeDef huart10{USART10};
Class_Timestamp SYS_Timestamp;

int fake_sequence = 0;
int fake_timestamp_init_sequence = 0;
int fake_timestamp_overflow_count = 0;
TIM_HandleTypeDef *fake_timestamp_handler = nullptr;
std::uint64_t fake_timestamp_now = 0U;

void Call_TIM_Callback(TIM_HandleTypeDef *htim);

namespace
{
std::uint32_t Fake_Tick = 0U;
GPIO_PinState Key1_Level = GPIO_PIN_SET;
GPIO_PinState Key2_Level = GPIO_PIN_SET;
CAN_Callback Registered_CAN1_Callback = nullptr;
CAN_Callback Registered_CAN3_Callback = nullptr;
bool CAN1_Init_Saw_Init_Finished = true;
bool CAN3_Init_Saw_Init_Finished = true;
int CAN1_Init_Sequence = 0;
int CAN3_Init_Sequence = 0;
int CAN1_Standard_Init_Count = 0;
int CAN3_Extended_Init_Count = 0;
int TIM5_Start_Sequence = 0;
int TIM5_Start_Count = 0;
HAL_StatusTypeDef TIM5_Start_Result = HAL_OK;
HAL_StatusTypeDef UART_Transmit_Result = HAL_OK;
int Error_Handler_Count = 0;
int UART_Transmit_Count = 0;
std::string UART_Output;
int Vofa_Transmit_Count = 0;
std::vector<std::uint8_t> Vofa_Output;
int UART4_Receive_Start_Count = 0;
int UART7_Receive_Start_Count = 0;
int UART9_Receive_Start_Count = 0;
int UART10_Receive_Start_Count = 0;
bool Keep_Remote_Alive = false;

const std::uint8_t Default_Remote_Packet[31] = {
    0x0F, 0xC8, 0x40, 0x1F, 0xC2, 0x01, 0xF0, 0xFF,
    0xAA, 0xA8, 0xAA, 0xAA, 0xFF, 0x01, 0xD0, 0xFF,
    0x00, 0x08, 0xE0, 0x84, 0x49, 0x53, 0x05, 0xFF,
    0xAF, 0x05, 0x00, 0x00, 0x88, 0x00, 0x00};
std::uint8_t Remote_Packet[31]{};

void Reset_Fakes(const std::uint32_t initial_tick = 0U)
{
    Fake_Tick = initial_tick;
    Key1_Level = GPIO_PIN_SET;
    Key2_Level = GPIO_PIN_SET;
    Registered_CAN1_Callback = nullptr;
    Registered_CAN3_Callback = nullptr;
    CAN1_Init_Saw_Init_Finished = true;
    CAN3_Init_Saw_Init_Finished = true;
    CAN1_Init_Sequence = 0;
    CAN3_Init_Sequence = 0;
    CAN1_Standard_Init_Count = 0;
    CAN3_Extended_Init_Count = 0;
    TIM5_Start_Sequence = 0;
    TIM5_Start_Count = 0;
    TIM5_Start_Result = HAL_OK;
    UART_Transmit_Result = HAL_OK;
    Error_Handler_Count = 0;
    UART_Transmit_Count = 0;
    UART_Output.clear();
    Vofa_Transmit_Count = 0;
    Vofa_Output.clear();
    UART4_Receive_Start_Count = 0;
    UART7_Receive_Start_Count = 0;
    UART9_Receive_Start_Count = 0;
    UART10_Receive_Start_Count = 0;
    Keep_Remote_Alive = false;
    std::memcpy(Remote_Packet, Default_Remote_Packet, sizeof(Remote_Packet));
    fake_sequence = 0;
    fake_timestamp_init_sequence = 0;
    fake_timestamp_overflow_count = 0;
    fake_timestamp_handler = nullptr;
    fake_timestamp_now = 0U;
    DM_Chassis_LF.Get_Fake_State() = {};
    DM_Chassis_RF.Get_Fake_State() = {};
    DM_Chassis_LB.Get_Fake_State() = {};
    DM_Chassis_RB.Get_Fake_State() = {};
    VESC_Chassis_LF.Get_Fake_State() = {};
    VESC_Chassis_RF.Get_Fake_State() = {};
    VESC_Chassis_LB.Get_Fake_State() = {};
    // 正常任务测试提供有效的STATUS_1, 无反馈/掉线由底盘保护用例单独验证。
    VESC_Chassis_LF.Get_Fake_State().status_1_valid = true;
    VESC_Chassis_RF.Get_Fake_State().status_1_valid = true;
    VESC_Chassis_LB.Get_Fake_State().status_1_valid = true;
    Fake_NRF24L01 = {};
    Fake_HI06 = {};
}

void Run_At(const std::uint32_t tick)
{
    Fake_Tick = tick;
    Task_Loop();
}

void Advance_10ms()
{
    if (Keep_Remote_Alive)
    {
        assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
    }
    Run_At(Fake_Tick + 10U);
}

void Set_Remote_Function(const std::uint8_t index, const bool pressed)
{
    assert(index < 16U);
    std::uint8_t &data = Remote_Packet[26U + index / 8U];
    const std::uint8_t mask = static_cast<std::uint8_t>(1U << (index % 8U));
    if (pressed)
    {
        data |= mask;
    }
    else
    {
        data &= static_cast<std::uint8_t>(~mask);
    }
    assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
}

void Test_Initialization_Contract()
{
    Reset_Fakes();
    Task_Init();

    assert(init_finished);
    assert(fake_timestamp_handler == &htim5);
    assert(!CAN1_Init_Saw_Init_Finished);
    assert(!CAN3_Init_Saw_Init_Finished);
    assert(Registered_CAN1_Callback != nullptr);
    assert(Registered_CAN3_Callback != nullptr);
    assert(CAN1_Standard_Init_Count == 1);
    assert(CAN3_Extended_Init_Count == 1);
    assert(TIM5_Start_Count == 1);
    assert(Fake_HI06.Init_Count == 1);
    assert(UART4_Receive_Start_Count == 1);
    assert(UART4_Manage_Object.Callback_Function != nullptr);
    assert(UART7_Receive_Start_Count == 1);
    assert(UART9_Receive_Start_Count == 1);
    assert(UART9_Manage_Object.Callback_Function != nullptr);
    assert(UART10_Receive_Start_Count == 1);
    assert(Error_Handler_Count == 0);
    assert(DM_Chassis_LF.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_RF.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_LB.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_RB.Get_Fake_State().can_handler == &hfdcan1);
    assert(VESC_Chassis_LF.Get_Fake_State().can_handler == &hfdcan3);
    assert(VESC_Chassis_RF.Get_Fake_State().can_handler == &hfdcan3);
    assert(VESC_Chassis_LB.Get_Fake_State().can_handler == &hfdcan3);
    assert(DM_Chassis_LF.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_LB.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().exit_count == 1);
    assert(fake_timestamp_init_sequence < DM_Chassis_LF.Get_Fake_State().init_sequence);
    assert(DM_Chassis_LF.Get_Fake_State().set_omega_sequence < CAN1_Init_Sequence);
    assert(CAN1_Init_Sequence < CAN3_Init_Sequence);
    assert(CAN3_Init_Sequence < TIM5_Start_Sequence);
}

void Test_UART4_Callback_Forwards_To_HI06()
{
    Reset_Fakes();
    Task_Init();
    std::uint8_t data[3] = {0x5AU, 0xA5U, 0x01U};

    UART4_Manage_Object.Callback_Function(data, sizeof(data));

    assert(Fake_HI06.UART_Callback_Count == 1);
    assert(Fake_HI06.UART_Buffer == data);
    assert(Fake_HI06.UART_Length == sizeof(data));
}

void Receive_Upper_Frame()
{
    std::uint8_t frame[32] = {0x55U, 0xaaU, 0xffU, 0x7fU, 0x00U, 0x80U, 0x18U, 0xfcU};
    const uint16_t crc = Upper_CRC16_Calculate(frame, 29U);
    frame[29] = static_cast<uint8_t>(crc); frame[30] = static_cast<uint8_t>(crc >> 8U);
    frame[31] = 0x77U;
    assert(UART7_Manage_Object.Callback_Function != nullptr);
    UART7_Manage_Object.Callback_Function(frame, sizeof(frame));
}

void Test_UART7_Init_And_Foreground_Receive()
{
    Reset_Fakes();
    Task_Init();
    assert(Upper.Get_Status() == Upper_Status_OFFLINE);
    assert(!Upper.Get_Data().Valid && Upper.Get_CRC_Check_Enable());
    Receive_Upper_Frame();
    assert(Upper.Get_Data().Frame_Count == 0U);
    Run_At(9U);
    assert(Upper.Get_Data().Frame_Count == 0U);
    Run_At(10U);
    assert(Upper.Get_Data().Frame_Count == 1U && Upper.Get_Data().Valid);
    assert(Upper.Get_Data().Speed_X == -32768 && Upper.Get_Data().Speed_Y == 32767 && Upper.Get_Data().Speed_W == -1000);
    assert(Remote_Chassis_Command.Speed_X == 0.0f && Remote_Chassis_Command.Speed_Y == 0.0f && Remote_Chassis_Command.Speed_W == 0.0f);
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
}

void Test_Upper_Print_Not_Automatic()
{
    Reset_Fakes();
    Task_Init();
    Receive_Upper_Frame();
    Run_At(10U);
    Run_At(500U);
    assert(UART_Output.find("UPPER") == std::string::npos);
    Debug_UART10_Print_Upper_Data();
    assert(UART_Output.find("raw_xyw=-32768,32767,-1000") != std::string::npos);
    Debug_UART10_Print_Upper_Status_Data();
    assert(UART_Output.find("crc_enable=1") != std::string::npos);
}

void Test_HI06_Processes_Every_10ms()
{
    Reset_Fakes();
    Task_Init();

    Run_At(9U);
    assert(Fake_HI06.TIM_10ms_Count == 0);
    Run_At(10U);
    assert(Fake_HI06.TIM_10ms_Count == 1);
    Run_At(20U);
    assert(Fake_HI06.TIM_10ms_Count == 2);
}

void Test_VESC_And_Vofa_Send_Run_Every_10ms()
{
    Reset_Fakes();
    Task_Init();

    assert(UART9_Manage_Object.UART_Handler == &huart9);
    assert(UART9_Manage_Object.Callback_Function != nullptr);
    Run_At(9U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 0);
    Run_At(10U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(19U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(20U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 2);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 2);
    assert(Vofa_Output.size() == 27U * sizeof(float) + sizeof(std::uint32_t));

    for (std::uint32_t tick = 21U; tick <= 1000U; ++tick)
    {
        Run_At(tick);
    }
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 100);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 100);
}

void Test_Enabled_VESC_Send_Runs_Every_10ms()
{
    Reset_Fakes();
    Task_Init();
    Remote_Packet[25] = 0x06U;
    Keep_Remote_Alive = true;

    for (int cycle = 1; cycle <= 100; ++cycle)
    {
        Advance_10ms();
        assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
        assert(VESC_Chassis_LF.Get_Fake_State().send_count == cycle);
    }
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 100);
}

void Test_Vofa_Output_Contains_VESC_Waveform_Channels()
{
    Reset_Fakes();
    Task_Init();

    Fake_VESC_State &vesc = VESC_Chassis_LF.Get_Fake_State();
    vesc.control_omega = 6.28318531f;
    vesc.now_omega = -3.14159265f;
    vesc.motor_current = 8.5f;
    vesc.duty = 0.375f;
    vesc.input_current = 3.25f;
    vesc.input_voltage = 48.0f;
    vesc.mos_temperature = 52.25f;
    vesc.status_1_valid = true;
    vesc.status_4_valid = true;
    vesc.status_5_valid = true;
    vesc.status_5_update_count = 1U;
    Debug_Vofa_10ms_PeriodElapsedCallback();

    assert(Vofa_Transmit_Count == 1);
    assert(Vofa_Output.size() == 27U * sizeof(float) + sizeof(std::uint32_t));

    float channel[27]{};
    std::uint32_t frame_tail = 0U;
    std::memcpy(channel, Vofa_Output.data(), sizeof(channel));
    std::memcpy(&frame_tail, Vofa_Output.data() + sizeof(channel), sizeof(frame_tail));
    assert(std::fabs(channel[0] - 60.0f) < 0.001f);
    assert(std::fabs(channel[1] + 30.0f) < 0.001f);
    assert(std::fabs(channel[2] - 90.0f) < 0.001f);
    assert(std::fabs(channel[3] - 8.5f) < 0.0001f);
    assert(std::fabs(channel[4] - 37.5f) < 0.0001f);
    assert(std::fabs(channel[5] - 3.25f) < 0.0001f);
    assert(std::fabs(channel[6] - 48.0f) < 0.0001f);
    assert(std::fabs(channel[7] - 52.25f) < 0.0001f);
    assert(frame_tail == 0x7F800000U);
}

void Test_Vofa_Output_Uses_Fixed_10ms_Period_When_Feedback_Is_Invalid()
{
    Reset_Fakes();
    Task_Init();
    Fake_VESC_State &vesc = VESC_Chassis_LF.Get_Fake_State();

    vesc.status_1_valid = true;
    vesc.status_4_valid = true;
    vesc.status_5_valid = false;
    vesc.status_5_update_count = 1U;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 1);
    float channel[27]{};
    std::memcpy(channel, Vofa_Output.data(), sizeof(channel));
    assert(std::isnan(channel[6]));

    vesc.status_5_valid = true;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 2);
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 3);

    vesc.status_5_update_count = 2U;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 4);
}

void Test_USART9_MIT_Command_Applies_Before_Chassis_Control()
{
    Reset_Fakes();
    Task_Init();
    const std::uint8_t command[] = "dm_kp=8.5#";
    UART9_Manage_Object.Callback_Function(const_cast<std::uint8_t *>(command), sizeof(command) - 1U);

    Remote_Packet[25] = 0x06U;
    for (std::uint8_t i = 0U; i < 31U; i++)
    {
        Fake_NRF24L01.Rx_Packet[i] = Remote_Packet[i];
    }
    Fake_NRF24L01.Receive_Result = 1U;
    Run_At(10U);

    assert(std::fabs(DM_Chassis_LF.Get_Fake_State().k_p - 8.5f) < 0.0001f);
}

void Test_Periodic_Output_Disables_100ms_Swerve_Print_And_Prints_IMU_Every_500ms()
{
    Reset_Fakes();
    Task_Init();
    SBUS_Receive.SBUS_Data.Online = 1U;
    SBUS_Receive.SBUS_Data.Mode = 2U;
    SBUS_Receive.SBUS_Data.Motor_Enable = 1U;
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Mode = 2U;
    Remote_Chassis_Command.Motor_Enable = 1U;
    Remote_Chassis_Command.Speed_X = 0.0f;
    Remote_Chassis_Command.Speed_Y = 1.0f;
    Remote_Chassis_Command.Speed_W = 0.0f;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_LF.Get_Fake_State().control_status = Motor_DM_Control_Status_ENABLE;
    DM_Chassis_LF.Get_Fake_State().now_angle = 0.0f;
    VESC_Chassis_LF.Get_Fake_State().status = Motor_VESC_Status_DISABLE;
    VESC_Chassis_LF.Get_Fake_State().now_erpm = -123;
    Chassis_Set_Command({Remote_Chassis_Command.Speed_X, Remote_Chassis_Command.Speed_Y, Remote_Chassis_Command.Speed_W, true, Chassis_Command_Source_MANUAL});
    Chassis_Swerve_10ms_PeriodElapsedCallback();

    Debug_UART10_Print_Swerve_Data();
    assert(UART_Transmit_Count == 1);
    assert(UART_Output ==
           "SW RC=1 mode=2 motor_enable=1 speed=0.00,1.00,0.00 "
           "DM enable=1 online=1 control=1 target=0.00 target_omega=0.00 now=0.00 error=0.00 ratio=0.00 "
           "VESC enable=1 online=0 target_erpm=0 now_erpm=-123 select=0 source=0 home=1,0,0 dm_rx=0,0,0 dm_on=1,0,0 dm_ctl=1,0,0 drive=0,0,0 wait_ms=0 stop=0\r\n");
    assert(UART_Output.find("IMU ") == std::string::npos);

    UART_Transmit_Count = 0;
    UART_Output.clear();

    Run_At(50U);
    assert(UART_Transmit_Count == 0);
    Run_At(99U);
    assert(UART_Transmit_Count == 0);
    Run_At(100U);
    assert(UART_Transmit_Count == 0);

    Run_At(499U);
    assert(UART_Transmit_Count == 0);
    Run_At(500U);
    assert(UART_Transmit_Count == 1);
    assert(UART_Output.rfind("IMU ", 0U) == 0U);
}

void Test_IMU_Data_Has_Independent_Print_Functions()
{
    Reset_Fakes();
    Task_Init();
    Fake_HI06.Status = HI06_Status_ONLINE;
    Fake_HI06.Data.Valid = 0x080019BDU;
    Fake_HI06.Data.Main_Status = 512U;
    Fake_HI06.Data.Accel_X = 1.25f;
    Fake_HI06.Data.Accel_Y = -2.50f;
    Fake_HI06.Data.Accel_Z = 3.75f;
    Fake_HI06.Data.Gyro_X = 0.50f;
    Fake_HI06.Data.Gyro_Y = -1.25f;
    Fake_HI06.Data.Gyro_Z = 2.00f;
    Fake_HI06.Data.Temperature = 27.50f;
    Fake_HI06.Data.Rx_Byte_Count = 82U;
    Fake_HI06.Data.Frame_Count = 3U;
    Fake_HI06.Data.CRC_Error_Count = 1U;
    Fake_HI06.Data.Invalid_Frame_Count = 2U;
    Fake_HI06.Data.Buffer_Overflow_Count = 4U;

    Debug_UART10_Print_IMU_Acceleration_Data();
    assert(UART_Output == "IMU online=1 acc=1.25,-2.50,3.75 temp=27.50\r\n");

    UART_Output.clear();
    Debug_UART10_Print_IMU_Gyroscope_Data();
    assert(UART_Output == "IMU online=1 gyr=0.50,-1.25,2.00 temp=27.50\r\n");

    UART_Output.clear();
    Debug_UART10_Print_IMU_Status_Data();
    assert(UART_Output == "IMU online=1 valid=0x080019BD status=512 rx=82 frame=3 crc_err=1 invalid=2 overflow=4\r\n");
}

void Test_Independent_IMU_Print_Skips_When_USART10_DMA_Is_Busy()
{
    Reset_Fakes();
    Task_Init();
    Fake_HI06.Status = HI06_Status_ONLINE;
    UART_Transmit_Result = HAL_BUSY;

    Debug_UART10_Print_IMU_Data();
    assert(UART_Transmit_Count == 1);
    assert(UART_Output.empty());
}

void Test_Scheduler_Wrap_And_No_Catchup_Burst()
{
    Reset_Fakes();
    Task_Init();
    Run_At(9U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 0);
    Run_At(10U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    Run_At(100U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 2);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 1);
    Run_At(1100U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 3);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 2);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 3);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 2);
    assert(Vofa_Transmit_Count == 3);

    Reset_Fakes(std::numeric_limits<std::uint32_t>::max() - 5U);
    Task_Init();
    Run_At(3U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 0);
    Run_At(4U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(13U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(14U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 2);
    assert(Vofa_Transmit_Count == 2);
}

void Test_CAN1_And_CAN3_Routing()
{
    Reset_Fakes();
    Task_Init();

    std::uint8_t buffer[8]{};
    FDCAN_RxHeaderTypeDef header{};
    header.IdType = FDCAN_STANDARD_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    // 机械臂暂不接入，CAN1收到完整反馈时也不分发给机械臂电机。
    header.DataLength = FDCAN_DLC_BYTES_8;
    const unsigned arm_ids[] = {DM_ARM_ROTATE_MASTER_ID, DM_ARM_BIG_MASTER_ID, DM_ARM_SMALL_MASTER_ID};
    for (unsigned id : arm_ids)
    {
        header.Identifier = id;
        Registered_CAN1_Callback(header, buffer);
    }
    Run_At(Fake_Tick + 100U);
    Class_Motor_DM_Normal *const arm_motors[] = {&DM_Arm_Rotate, &DM_Arm_Big, &DM_Arm_Small};
    for (auto motor : arm_motors)
    {
        const auto &state = motor->Get_Fake_State();
        assert(state.receive_count == 0);
        assert(state.enter_count == 0 && state.exit_count == 0 && state.send_count == 0);
    }
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 0);
    header.Identifier = 0x11U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x22U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_RF.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x33U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LB.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x44U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_RB.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x56U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 1);

    header.IdType = FDCAN_EXTENDED_ID;
    header.Identifier = (9U << 8U) | 1U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.Identifier = (9U << 8U) | 2U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_RF.Get_Fake_State().receive_count == 1);
    header.Identifier = (16U << 8U) | 3U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LB.Get_Fake_State().receive_count == 1);
    header.Identifier = (27U << 8U) | 4U;
    Registered_CAN3_Callback(header, buffer);
    header.Identifier = (9U << 8U) | 5U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().receive_count == 1);
    assert(VESC_Chassis_LB.Get_Fake_State().receive_count == 1);

    header.IdType = FDCAN_EXTENDED_ID;
    header.Identifier = 0x11U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.IdType = FDCAN_STANDARD_ID;
    header.Identifier = 1U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
}

void Test_Remote_Updates_Before_Chassis_Control()
{
    Reset_Fakes();
    Task_Init();

    Remote_Packet[25] = 0x06U;
    for (std::uint8_t i = 0U; i < 31U; i++)
    {
        Fake_NRF24L01.Rx_Packet[i] = Remote_Packet[i];
    }
    Fake_NRF24L01.Receive_Result = 1U;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_RF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_LB.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_RB.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    VESC_Chassis_LF.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    VESC_Chassis_RF.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    VESC_Chassis_LB.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    Run_At(10U);

    assert(Remote_Chassis_Command.Online == 1U);
    assert(Remote_Chassis_Command.Mode == 2U);
    assert(DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_RB.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(DM_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
    // 行进轮速度需要等待舵向对准，本测试只验证遥控命令先于底盘控制更新以及电机使能状态。
    // 舵向误差门控和VESC速度输出由底盘任务的独立测试覆盖。
}

void Test_TIM_Routing()
{
    Reset_Fakes();
    Task_Init();

    Call_TIM_Callback(&htim5);
    assert(fake_timestamp_overflow_count == 1);
    TIM_HandleTypeDef other_timer{reinterpret_cast<void *>(0x50000U)};
    Call_TIM_Callback(&other_timer);
    assert(fake_timestamp_overflow_count == 1);
}
}

extern "C" GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, const std::uint16_t gpio_pin)
{
    if (gpio_port == GPIOG)
    {
        assert(gpio_pin == NRF_IRQ_Pin);
        return GPIO_PIN_SET;
    }

    assert(gpio_port == GPIOF);
    if (gpio_pin == Key1_Pin)
    {
        return Key1_Level;
    }
    assert(gpio_pin == Key2_Pin);
    return Key2_Level;
}

extern "C" std::uint32_t HAL_GetTick()
{
    return Fake_Tick;
}

extern "C" HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    assert(htim == &htim5);
    ++TIM5_Start_Count;
    TIM5_Start_Sequence = ++fake_sequence;
    return TIM5_Start_Result;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                                std::uint8_t *data,
                                                const std::uint16_t size,
                                                const std::uint32_t timeout)
{
    (void)huart;
    (void)data;
    (void)size;
    (void)timeout;
    assert(false && "打印数据不应继续使用USART1阻塞发送");
    return HAL_ERROR;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                                    std::uint8_t *data,
                                                    const std::uint16_t size)
{
    if (huart == &huart9)
    {
        ++Vofa_Transmit_Count;
        Vofa_Output.assign(data, data + size);
        return UART_Transmit_Result;
    }

    assert(huart == &huart10);
    ++UART_Transmit_Count;
    if (UART_Transmit_Result == HAL_OK)
    {
        UART_Output.append(reinterpret_cast<const char *>(data), size);
    }
    return UART_Transmit_Result;
}

extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                                           std::uint8_t *,
                                                           const std::uint16_t size)
{
    assert(size == UART_BUFFER_SIZE);
    if (huart == &huart4)
    {
        assert(!init_finished);
        ++UART4_Receive_Start_Count;
    }
    else if (huart == &huart7)
    {
        ++UART7_Receive_Start_Count;
    }
    else if (huart == &huart9)
    {
        ++UART9_Receive_Start_Count;
    }
    else
    {
        assert(huart == &huart10);
        ++UART10_Receive_Start_Count;
    }
    return HAL_OK;
}

extern "C" void Error_Handler()
{
    ++Error_Handler_Count;
}

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, const CAN_Callback callback_function)
{
    assert(callback_function != nullptr);
    if (hfdcan == &hfdcan1)
    {
        Registered_CAN1_Callback = callback_function;
        CAN1_Init_Saw_Init_Finished = init_finished;
        CAN1_Init_Sequence = ++fake_sequence;
        ++CAN1_Standard_Init_Count;
    }
    else
    {
        assert(false && "CAN3必须使用扩展帧初始化");
    }
}

void CAN_Extended_Init(FDCAN_HandleTypeDef *hfdcan, const CAN_Callback callback_function)
{
    assert(hfdcan == &hfdcan3);
    assert(callback_function != nullptr);
    Registered_CAN3_Callback = callback_function;
    CAN3_Init_Saw_Init_Finished = init_finished;
    CAN3_Init_Sequence = ++fake_sequence;
    ++CAN3_Extended_Init_Count;
}

std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *, std::uint16_t, std::uint8_t *, std::uint16_t)
{
    return HAL_OK;
}

std::uint8_t CAN_Transmit_Extended_Data(FDCAN_HandleTypeDef *, std::uint32_t, std::uint8_t *, std::uint16_t)
{
    return HAL_OK;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Call_TIM_Callback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PeriodElapsedCallback(htim);
}

// 以下缩放和超时只供主机测试, 不作为固件/未来上位机的默认协议参数。
static Struct_Upper_Speed_Parameter Upper_Test_Parameter()
{
    return {0.001f, 0.001f, 5.0f, 2.0f, 100U};
}

static void Feed_Upper_Speed(int16_t x, int16_t y, int16_t w, uint8_t error = 0U, uint16_t length = 32U)
{
    uint8_t frame[32] = {0x55U, 0xaaU};
    const int16_t values[3] = {y, x, w};
    for (uint8_t i = 0U; i < 3U; i++)
    {
        const uint16_t raw = static_cast<uint16_t>(values[i]);
        frame[2U + i * 2U] = static_cast<uint8_t>(raw);
        frame[3U + i * 2U] = static_cast<uint8_t>(raw >> 8U);
    }
    const uint16_t crc = Upper_CRC16_Calculate(frame, 29U);
    frame[29] = static_cast<uint8_t>(crc); frame[30] = static_cast<uint8_t>(crc >> 8U);
    frame[31] = error == 2U ? 0x00U : 0x77U;
    if (error == 1U) { frame[29] ^= 1U; }
    UART7_Manage_Object.Callback_Function(frame, length);
}

static void Assert_Command_Disabled()
{
    const auto &command = Upper_Get_Control_State().Command;
    assert(!command.Motor_Enable && command.Speed_X == 0.0f && command.Speed_Y == 0.0f && command.Speed_W == 0.0f);
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
}

static void Begin_Upper_Control(uint32_t tick = 0U)
{
    Reset_Fakes(tick); Task_Init();
    assert(Upper_Configure_Speed(Upper_Test_Parameter()));
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x07U;
    Advance_10ms();
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_WAIT_FRAME);
    Assert_Command_Disabled();
}

static void Test_Upper_Configuration_And_Default_Disable()
{
    Reset_Fakes(); Task_Init();
    assert(!Upper_Get_Control_State().Configured);
    assert(!Upper_Configure_Speed({}));
    for (uint8_t test = 0U; test < 8U; test++)
    {
        auto parameter = Upper_Test_Parameter();
        switch (test)
        {
        case 0U: parameter.Linear_Scale = std::numeric_limits<float>::quiet_NaN(); break;
        case 1U: parameter.Angular_Scale = std::numeric_limits<float>::infinity(); break;
        case 2U: parameter.Linear_Scale = -0.001f; break;
        case 3U: parameter.Timeout_MS = 0U; break;
        case 4U: parameter.Timeout_MS = UPPER_OFFLINE_TIMEOUT + 1U; break;
        case 5U: parameter.Translation_Speed_Max = 17.0f; break;
        case 6U: parameter.Angular_Speed_Max = Struct_Remote_Parameter{}.Angular_Speed_Max + 1.0f; break;
        default: parameter.Angular_Speed_Max = 0.0f; break;
        }
        assert(!Upper_Configure_Speed(parameter));
    }
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x07U;
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_WAIT_CONFIG);
    Assert_Command_Disabled();
    assert(!Upper_Configure_Speed(Upper_Test_Parameter())); // 总使能开时禁止改变缩放。
    Remote_Packet[25] = 0x03U; Advance_10ms();
    assert(Upper_Configure_Speed(Upper_Test_Parameter()));
    assert(Upper_Get_Control_State().Configured);
}

static void Test_Upper_Fresh_Frame_And_Body_Velocity()
{
    Reset_Fakes(); Task_Init();
    assert(Upper_Configure_Speed(Upper_Test_Parameter()));
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms(); // 失能期间的缓存不得用于启动。
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x07U; Advance_10ms();
    Assert_Command_Disabled();
    Advance_10ms(); Assert_Command_Disabled();
    Feed_Upper_Speed(1000, -2000, 500); Advance_10ms();
    const auto &control = Upper_Get_Control_State();
    assert(control.Status == Upper_Control_Status_RUNNING);
    assert(control.Command.Source == Chassis_Command_Source_UPPER && control.Command.Motor_Enable);
    assert(std::fabs(control.Command.Speed_X - 1.0f) < 1e-5f);
    assert(std::fabs(control.Command.Speed_Y + 2.0f) < 1e-5f);
    assert(std::fabs(control.Command.Speed_W - 0.5f) < 1e-5f);
    assert(Fake_HI06.Status != HI06_Status_ONLINE); // 车体速度不依赖本地定位或IMU解算。
    assert(!Upper_Configure_Speed(Upper_Test_Parameter()));
    Remote_Packet[25] = 0x03U; Advance_10ms(); Assert_Command_Disabled();
    Remote_Packet[25] = 0x07U; Advance_10ms(); Assert_Command_Disabled();
    Feed_Upper_Speed(0, 0, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Command.Motor_Enable); // 合法零速流允许原底盘回零。
}

static void Test_Upper_Reaches_Original_Drive_And_Timeout_Locks()
{
    Begin_Upper_Control();
    for (uint32_t i = 0U; i < 40U; i++)
    {
        auto &dm = DM_Chassis_LF.Get_Fake_State();
        dm.total_angle_initialized = true; dm.now_total_angle = 0.0f; dm.now_omega = 0.0f; dm.rx_update_count++;
        Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    }
    assert(Chassis_Get_Swerve_Ready());
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega > 0.0f);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable && !VESC_Chassis_LB.Get_Fake_State().motor_enable);
    for (uint32_t i = 0U; i < 8U; i++) { Advance_10ms(); }
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_RUNNING);
    Advance_10ms();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Assert_Command_Disabled();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms(); Assert_Command_Disabled();
    Remote_Packet[25] = 0x06U; Advance_10ms(); // 人工可接管, 不清除上位机故障。
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_MANUAL);
    assert(DM_Chassis_LF.Get_Fake_State().motor_enable);
    Remote_Packet[25] = 0x07U; Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    Assert_Command_Disabled();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Remote_Packet[25] = 0x03U; Advance_10ms();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_NONE);
    Remote_Packet[25] = 0x07U; Advance_10ms(); Assert_Command_Disabled();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_RUNNING);
}

static void Test_Upper_Bad_Frames_And_Speed_Reject()
{
    for (uint8_t error = 1U; error <= 6U; error++)
    {
        Begin_Upper_Control();
        Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
        assert(Upper_Get_Control_State().Status == Upper_Control_Status_RUNNING);
        if (error <= 2U) { Feed_Upper_Speed(0, 1000, 0, error); }
        else if (error == 3U)
        {
            uint8_t overflow[1024]{};
            UART7_Manage_Object.Callback_Function(overflow, sizeof(overflow));
        }
        else if (error == 4U) { Feed_Upper_Speed(4000, 4000, 0); } // 单轴未超限, 合速度超限。
        else if (error == 5U) { Feed_Upper_Speed(0, 0, -2001); }
        else { Upper.Set_CRC_Check_Enable(false); }
        Advance_10ms();
        assert(Upper_Get_Control_State().Fault == (error == 4U || error == 5U ? Upper_Control_Fault_SPEED : Upper_Control_Fault_FRAME));
        Assert_Command_Disabled();
        Upper.Set_CRC_Check_Enable(true);
        Feed_Upper_Speed(0, 1000, 0); Advance_10ms(); Assert_Command_Disabled();
    }
    Begin_Upper_Control();
    Feed_Upper_Speed(3000, 4000, 2000); Advance_10ms(); // 合速度及角速度边界均允许。
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_RUNNING);
    Begin_Upper_Control(std::numeric_limits<uint32_t>::max() - 35U);
    Feed_Upper_Speed(0, -1000, 0); Advance_10ms();
    for (uint8_t i = 0U; i < 10U; i++) { Advance_10ms(); }
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT); // Tick回绕不延长控制期限。
}

static void Test_Upper_Batch_Stale_Data_And_Reconfigure_Lock()
{
    Begin_Upper_Control();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    Feed_Upper_Speed(4000, 4000, 0); Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_SPEED);
    Assert_Command_Disabled();

    Begin_Upper_Control();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    Feed_Upper_Speed(0, 1000, 0); // 字节已缓存, 前台暂停后不能把旧帧重新标鲜。
    Run_At(Fake_Tick + 150U);
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Assert_Command_Disabled();

    Begin_Upper_Control();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    Fake_Tick += 150U; // 断流后刚收到新帧也不能掩盖已发生的超时。
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Keep_Remote_Alive = false;
    for (uint8_t i = 0U; i < 100U; i++) { Advance_10ms(); }
    assert(Upper_Configure_Speed(Upper_Test_Parameter()));
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Keep_Remote_Alive = true; Advance_10ms();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms(); Assert_Command_Disabled();
    Remote_Packet[25] = 0x03U; Advance_10ms(); // 仅在线遥控关总使能确认故障。
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_NONE);
}

static void Test_Upper_Partial_Frames_Cannot_Revive_Old_Velocity()
{
    uint8_t tail = 0x77U;
    Reset_Fakes(); Task_Init();
    assert(Upper_Configure_Speed(Upper_Test_Parameter()));
    Feed_Upper_Speed(0, 1000, 0, 0U, 31U); Advance_10ms();
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x07U; Advance_10ms();
    UART7_Manage_Object.Callback_Function(&tail, 1U); Advance_10ms();
    Assert_Command_Disabled(); // 切入前的残帧补尾不能启动。
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Status == Upper_Control_Status_RUNNING);

    Begin_Upper_Control();
    Feed_Upper_Speed(0, 1000, 0, 0U, 31U); Advance_10ms();
    Fake_Tick += 150U;
    UART7_Manage_Object.Callback_Function(&tail, 1U); Advance_10ms();
    assert(Upper_Get_Control_State().Fault == Upper_Control_Fault_TIMEOUT);
    Assert_Command_Disabled(); // 同一等待窗口内, 过期速度残帧不能靠新尾字节续期。
}

static void Test_Page_Control_Ownership()
{
    Begin_Upper_Control();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    Remote_Packet[25] = 0x06U; Advance_10ms();
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_MANUAL);
    assert(DM_Chassis_LF.Get_Fake_State().motor_enable);
    Set_Remote_Function(0U, true); Set_Remote_Function(1U, true); Advance_10ms();
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_MANUAL);
    Remote_Packet[25] = 0x07U; Advance_10ms(); Assert_Command_Disabled();
    Set_Remote_Function(0U, false); Set_Remote_Function(1U, false); Advance_10ms();
    Assert_Command_Disabled();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_UPPER);
    Keep_Remote_Alive = false;
    for (uint8_t i = 0U; i < 100U; i++) { Feed_Upper_Speed(0, 1000, 0); Advance_10ms(); }
    Assert_Command_Disabled(); // 即使上位机持续发速度, 遥控离线也必须失能。
    Keep_Remote_Alive = true; Advance_10ms(); Assert_Command_Disabled();
    Feed_Upper_Speed(0, 1000, 0); Advance_10ms();
    assert(Upper_Get_Control_State().Command.Motor_Enable);
    Remote_Packet[25] = 0x05U; Advance_10ms();
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_DEBUG_U8);
    assert(Upper_Get_Control_State().Command.Motor_Enable);
    assert(!DM_Chassis_LF.Get_Motor_Enable());
    assert(VESC_Chassis_LF.Get_Motor_Enable());
    Remote_Packet[25] = 0x04U; Advance_10ms(); Assert_Command_Disabled();
}

static void Test_Debug_U8_Works_Through_Real_Remote_And_Task_Selection()
{
    Reset_Fakes(); Task_Init();
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x05U;
    std::memset(Remote_Packet + 1U, 0, 22U);
    for (uint8_t channel = 0U; channel < 16U; channel++)
    {
        const uint16_t value = channel <= 1U ? 1800U : (channel == 3U ? 200U : 1000U);
        for (uint8_t bit = 0U; bit < 11U; bit++)
        {
            const uint16_t index = channel * 11U + bit;
            if ((value & (1U << bit)) != 0U) { Remote_Packet[1U + index / 8U] |= 1U << (index % 8U); }
        }
    }
    Advance_10ms();
    const auto &command = Upper_Get_Control_State().Command;
    assert(command.Source == Chassis_Command_Source_DEBUG_U8 && command.Motor_Enable);
    assert(command.Speed_X == 0.0f && command.Speed_Y == 16.0f && command.Speed_W == 0.0f);
    assert(!Upper_Get_Control_State().Configured); // DEBUG不要求上位机配置或定位。
    for (int i = 0; i < 160; i++) { Advance_10ms(); }
    assert(std::fabs(VESC_Chassis_LF.Get_Control_Omega() - 150.79644737f) < 0.002f);
    assert(!VESC_Chassis_LF.Get_Status_4_Valid()); // 速度测试不要求位置反馈。
    for (uint8_t key = 0U; key < 9U; key++)
    {
        Set_Remote_Function(key, true); Advance_10ms();
        assert(VESC_Chassis_LF.Get_Control_Method() == Motor_VESC_Control_Method_RPM);
        assert(std::fabs(VESC_Chassis_LF.Get_Control_Omega() - 150.79644737f) < 0.002f);
        Set_Remote_Function(key, false); Advance_10ms();
    }
    assert(!VESC_Chassis_RF.Get_Motor_Enable() && !VESC_Chassis_LB.Get_Motor_Enable());
    for (auto *motor : {&DM_Chassis_LF, &DM_Chassis_RF, &DM_Chassis_LB, &DM_Chassis_RB})
    {
        assert(!motor->Get_Motor_Enable());
        assert(motor->Get_Fake_State().enter_count == 0 && motor->Get_Fake_State().send_count == 0);
    }
    Remote_Packet[25] = 0x01U; Advance_10ms(); Assert_Command_Disabled();
    Remote_Packet[25] = 0x05U; Advance_10ms();
    assert(VESC_Chassis_LF.Get_Control_Omega() > 0.0f);
    Keep_Remote_Alive = false;
    for (int i = 0; i < 100; i++) { Advance_10ms(); }
    Assert_Command_Disabled();
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x06U; Advance_10ms();
    assert(Upper_Get_Control_State().Command.Source == Chassis_Command_Source_MANUAL);
    assert(DM_Chassis_LF.Get_Motor_Enable());
    assert(!Chassis_Get_Swerve_Ready());
    assert(VESC_Chassis_LF.Get_Control_Omega() == 0.0f); // 返回正常底盘必须重新回零。
    Remote_Packet[25] = 0x07U; Advance_10ms(); Assert_Command_Disabled();
}

static void Test_Only_ID1_Feedback_Allows_Manual_Drive()
{
    Reset_Fakes(); Task_Init();
    Keep_Remote_Alive = true; Remote_Packet[25] = 0x06U;
    Advance_10ms();
    for (int i = 0; i < 40; i++)
    {
        auto &state = DM_Chassis_LF.Get_Fake_State();
        state.total_angle_initialized = true;
        state.now_total_angle = 0.0f; state.now_omega = 0.0f; state.rx_update_count++;
        Advance_10ms();
    }
    assert(Chassis_Get_Swerve_Ready());
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LB].Steer_Homing_Status == Chassis_Swerve_Homing_Status_DISABLED);
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_RB].Steer_Homing_Status == Chassis_Swerve_Homing_Status_DISABLED);
    assert(std::fabs(VESC_Chassis_LF.Get_Fake_State().control_omega) > 0.0f);
    for (auto *motor : {&DM_Chassis_RF, &DM_Chassis_LB})
    {
        assert(!motor->Get_Fake_State().motor_enable);
        assert(motor->Get_Fake_State().enter_count == 0);
        assert(motor->Get_Fake_State().send_count == 0);
        assert(motor->Get_Fake_State().alive_count == 0);
    }
    for (auto *motor : {&VESC_Chassis_RF, &VESC_Chassis_LB})
    {
        assert(!motor->Get_Fake_State().motor_enable);
        assert(motor->Get_Fake_State().send_count == 0);
        assert(motor->Get_Fake_State().alive_count == 0);
        assert(motor->Get_Fake_State().control_omega == 0.0f);
    }
    Remote_Packet[25] = 0x02U; Advance_10ms();
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
}

int main()
{
    Reset_Fakes(); Task_Init();
    assert(Upper.Get_CRC_Check_Enable()); // 上位机将参与运动控制, 任务入口必须启用CRC。
    Test_Debug_U8_Works_Through_Real_Remote_And_Task_Selection();
    Test_Only_ID1_Feedback_Allows_Manual_Drive();
    Test_Upper_Batch_Stale_Data_And_Reconfigure_Lock();
    Test_Upper_Partial_Frames_Cannot_Revive_Old_Velocity();
    Test_Page_Control_Ownership();
    Test_Upper_Configuration_And_Default_Disable();
    Test_Upper_Fresh_Frame_And_Body_Velocity();
    Test_Upper_Reaches_Original_Drive_And_Timeout_Locks();
    Test_Upper_Bad_Frames_And_Speed_Reject();
    // 类入口保持10ms调度, 同一时刻不重复发送。
    Reset_Fakes();
    Class_Task_Control task;
    task.Init();
    Fake_Tick = 9U;
    task.Loop();
    assert(Vofa_Transmit_Count == 0);
    Fake_Tick = 10U;
    task.Loop();
    assert(Vofa_Transmit_Count == 1);
    task.Loop();
    assert(Vofa_Transmit_Count == 1);
    Test_Initialization_Contract();
    Test_UART4_Callback_Forwards_To_HI06();
    Test_UART7_Init_And_Foreground_Receive();
    Test_Upper_Print_Not_Automatic();
    Test_HI06_Processes_Every_10ms();
    Test_VESC_And_Vofa_Send_Run_Every_10ms();
    Test_Enabled_VESC_Send_Runs_Every_10ms();
    Test_Vofa_Output_Contains_VESC_Waveform_Channels();
    Test_Vofa_Output_Uses_Fixed_10ms_Period_When_Feedback_Is_Invalid();
    Test_USART9_MIT_Command_Applies_Before_Chassis_Control();
    Test_Periodic_Output_Disables_100ms_Swerve_Print_And_Prints_IMU_Every_500ms();
    Test_IMU_Data_Has_Independent_Print_Functions();
    Test_Independent_IMU_Print_Skips_When_USART10_DMA_Is_Busy();
    Test_Scheduler_Wrap_And_No_Catchup_Burst();
    Test_CAN1_And_CAN3_Routing();
    Test_Remote_Updates_Before_Chassis_Control();
    Test_TIM_Routing();
    std::puts("Task control: existing safety/scheduling and upper integration tests passed");
    return 0;
}
