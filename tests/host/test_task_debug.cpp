#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "test_assert.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "3_Module/3swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_debug/tsk_debug.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "4_Task/tsk_config_and_callback/tsk_config_and_callback.h"
#include "usart.h"

bool init_finished = false;
int fake_sequence = 0;

UART_HandleTypeDef huart9{UART9};
UART_HandleTypeDef huart10{USART10};
Struct_UART_Manage_Object UART1_Manage_Object{};
Struct_UART_Manage_Object UART2_Manage_Object{};
Struct_UART_Manage_Object UART3_Manage_Object{};
Struct_UART_Manage_Object UART4_Manage_Object{};
Struct_UART_Manage_Object UART5_Manage_Object{};
Struct_UART_Manage_Object UART6_Manage_Object{};
Struct_UART_Manage_Object UART7_Manage_Object{};
Struct_UART_Manage_Object UART8_Manage_Object{};
Struct_UART_Manage_Object UART9_Manage_Object{};
Struct_UART_Manage_Object UART10_Manage_Object{};

Class_Motor_DM_Normal DM_Chassis_LF;
Class_Motor_DM_Normal DM_Chassis_RF;
Class_Motor_DM_Normal DM_Chassis_LB;
Class_Motor_DM_Normal DM_Chassis_RB;
Class_Motor_VESC VESC_Chassis_LF;
Class_Motor_VESC VESC_Chassis_RF;
Class_Motor_VESC VESC_Chassis_LB;
Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number]{};
Class_SBUS_Receive SBUS_Receive;
Struct_Remote_Chassis_Command Remote_Chassis_Command{};

static Struct_Upper_Control_State Test_Upper_Control{};
const Struct_Upper_Control_State &Upper_Get_Control_State() { return Test_Upper_Control; }
static Struct_U8_Test_Result Test_U8_Result{};
const Struct_U8_Test_Result &U8_Test_Get_Result() { return Test_U8_Result; }

namespace
{
std::uint8_t Transmit_Data[128]{};
std::uint16_t Transmit_Length = 0U;
int Transmit_Count = 0;
int Set_K_P_Count = 0;
int Set_K_D_Count = 0;
int Set_Torque_Count = 0;
Enum_Swerve_Wheel Last_Set_Wheel = Swerve_Wheel_LB;
float Last_Set_Value = 0.0f;
std::uint32_t Test_Tick = 0U;
std::string UART10_Output;
std::uint8_t *UART10_DMA_Buffer = nullptr;
std::uint16_t UART10_DMA_Length = 0U;
int UART10_Attempts = 0;
bool UART10_Busy = false;

void Assert_Near(const float value, const float expected)
{
    assert(std::fabs(value - expected) < 1.0e-4f);
}

void Reset_Test()
{
    UART9_Manage_Object = {};
    UART9_Manage_Object.UART_Handler = &huart9;
    DM_Chassis_LF.Get_Fake_State() = {};
    DM_Chassis_RF.Get_Fake_State() = {};
    DM_Chassis_LB.Get_Fake_State() = {};
    DM_Chassis_RB.Get_Fake_State() = {};
    VESC_Chassis_LF.Get_Fake_State() = {};
    VESC_Chassis_RF.Get_Fake_State() = {};
    VESC_Chassis_LB.Get_Fake_State() = {};
    for (std::uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Chassis_Swerve_Diagnostic[i] = {};
    }
    Transmit_Length = 0U;
    Transmit_Count = 0;
    Set_K_P_Count = 0;
    Set_K_D_Count = 0;
    Set_Torque_Count = 0;
    Last_Set_Wheel = Swerve_Wheel_LB;
    Last_Set_Value = 0.0f;
    Test_Tick = 0U;
    UART10_Output.clear();
    UART10_DMA_Buffer = nullptr;
    UART10_DMA_Length = 0U;
    UART10_Attempts = 0;
    UART10_Busy = false;
    Test_Upper_Control = {};
    Test_U8_Result = {};
    Remote_Chassis_Command = {};
    Upper.Init();
    Debug_Init();
}

void Configure_Wheel(Fake_VESC_State &vesc, Fake_Motor_State &dm, Struct_Chassis_Swerve_Diagnostic &diagnostic, const float base)
{
    vesc.status_1_valid = true;
    vesc.status_4_valid = true;
    vesc.status_5_valid = true;
    vesc.control_omega = 2.0f * 3.14159265358979323846f * base;
    vesc.now_omega = 3.14159265358979323846f * base;
    vesc.motor_current = -4.0f * base;
    vesc.duty = 0.25f * base;
    vesc.input_current = 2.0f * base;
    vesc.input_voltage = 24.0f * base;
    vesc.mos_temperature = 40.0f * base;

    dm.control_angle = 0.10f * base;
    dm.now_total_angle = 0.08f * base;
    dm.control_omega = 0.0f;
    dm.now_omega = 0.50f * base;
    dm.control_torque = 0.70f * base;
    dm.now_torque = 1.50f * base;
    dm.k_p = 7.0f * base;
    dm.k_d = 0.20f * base;
    dm.control_status = Motor_DM_Control_Status_ENABLE;
    dm.mos_temperature = 303.15f + base;
    dm.rotor_temperature = 313.15f + base;
    dm.status = Motor_DM_Status_ENABLE;

    diagnostic.Steer_Target_Angle = dm.control_angle;
    diagnostic.Steer_Now_Angle = dm.now_total_angle;
    diagnostic.Steer_Angle_Error = diagnostic.Steer_Target_Angle - diagnostic.Steer_Now_Angle;
    diagnostic.Steer_Now_Omega = dm.now_omega;
    diagnostic.Steer_Homing_Status = Chassis_Swerve_Homing_Status_READY;
    diagnostic.Steer_Homing_Elapsed_Time = 120U;
    diagnostic.Steer_Target_Enable = true;
    diagnostic.Drive_Status = Chassis_Swerve_Drive_Status_WAIT_STOP;
    diagnostic.Drive_Stop_Stable_Cycles = 3;
    diagnostic.Drive_Wait_Elapsed_Time = 200;
}

void Read_Frame(float *channels)
{
    assert(Transmit_Length == 27U * sizeof(float) + sizeof(std::uint32_t));
    std::memcpy(channels, Transmit_Data, 27U * sizeof(float));
    std::uint32_t tail = 0U;
    std::memcpy(&tail, Transmit_Data + 27U * sizeof(float), sizeof(tail));
    assert(tail == 0x7f800000U);
}

void Test_Debug_Emits_27_Channel_Frame_Every_10ms()
{
    Reset_Test();
    Configure_Wheel(VESC_Chassis_LF.Get_Fake_State(), DM_Chassis_LF.Get_Fake_State(), Chassis_Swerve_Diagnostic[Swerve_Wheel_F], 1.0f);

    Debug_Vofa_10ms_PeriodElapsedCallback();
    float channels[27]{};
    Read_Frame(channels);
    Assert_Near(channels[0], 60.0f);
    Assert_Near(channels[1], 30.0f);
    Assert_Near(channels[2], 30.0f);
    Assert_Near(channels[3], -4.0f);
    Assert_Near(channels[4], 25.0f);
    Assert_Near(channels[5], 2.0f);
    Assert_Near(channels[6], 24.0f);
    Assert_Near(channels[7], 40.0f);
    Assert_Near(channels[8], 0.10f);
    Assert_Near(channels[9], 0.08f);
    Assert_Near(channels[10], 0.02f);
    Assert_Near(channels[11], 0.0f);
    Assert_Near(channels[12], 0.50f);
    Assert_Near(channels[13], 0.70f);
    Assert_Near(channels[14], 1.50f);
    Assert_Near(channels[15], 7.0f);
    Assert_Near(channels[16], 0.20f);
    Assert_Near(channels[17], 31.0f);
    Assert_Near(channels[18], 41.0f);
    Assert_Near(channels[19], 1.0f);
    Assert_Near(channels[20], 1.0f);
    Assert_Near(channels[21], 1.0f);
    Assert_Near(channels[22], 3.0f);
    Assert_Near(channels[23], 120.0f);
    Assert_Near(channels[24], 2.0f);
    Assert_Near(channels[25], 3.0f);
    Assert_Near(channels[26], static_cast<float>(VESC_Chassis_LF.Get_Control_Method()));

    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Transmit_Count == 2);
}

void Test_Debug_Uses_NaN_For_Invalid_Continuous_Feedback()
{
    Reset_Test();
    Configure_Wheel(VESC_Chassis_LF.Get_Fake_State(), DM_Chassis_LF.Get_Fake_State(), Chassis_Swerve_Diagnostic[Swerve_Wheel_F], 1.0f);
    VESC_Chassis_LF.Get_Fake_State().status_1_valid = false;
    VESC_Chassis_LF.Get_Fake_State().status_4_valid = false;
    VESC_Chassis_LF.Get_Fake_State().status_5_valid = false;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_DISABLE;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Target_Enable = false;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status = Chassis_Swerve_Homing_Status_WAIT_FEEDBACK;

    Debug_Vofa_10ms_PeriodElapsedCallback();
    float channels[27]{};
    Read_Frame(channels);
    assert(std::isnan(channels[1]));
    assert(std::isnan(channels[2]));
    assert(std::isnan(channels[3]));
    assert(std::isnan(channels[4]));
    assert(std::isnan(channels[5]));
    assert(std::isnan(channels[6]));
    assert(std::isnan(channels[7]));
    assert(std::isnan(channels[9]));
    assert(std::isnan(channels[10]));
    assert(std::isnan(channels[12]));
    assert(std::isnan(channels[14]));
    assert(std::isnan(channels[17]));
    assert(std::isnan(channels[18]));
    Assert_Near(channels[20], 0.0f);
    Assert_Near(channels[21], 0.0f);
    Assert_Near(channels[22], 1.0f);
}

void Send_Command(const char *command)
{
    Debug_UART9_RxCpltCallback(reinterpret_cast<std::uint8_t *>(const_cast<char *>(command)), static_cast<std::uint16_t>(std::strlen(command)));
    Debug_10ms_PeriodElapsedCallback();
}

void Test_Debug_Stages_And_Applies_MIT_Commands_Once()
{
    Reset_Test();
    Send_Command("dm_kp=8.5#");
    assert(Set_K_P_Count == 1 && Last_Set_Wheel == Swerve_Wheel_F);
    Assert_Near(Last_Set_Value, 8.5f);
    Debug_10ms_PeriodElapsedCallback();
    assert(Set_K_P_Count == 1);

    Send_Command("dm_kd=0.35#");
    assert(Set_K_D_Count == 1);
    Assert_Near(Last_Set_Value, 0.35f);

    Send_Command("dm_tau_ff=0.8#");
    assert(Set_Torque_Count == 1);
    Assert_Near(Last_Set_Value, 0.8f);
}

void Test_Debug_Rejects_Invalid_And_Malformed_MIT_Commands()
{
    Reset_Test();
    Send_Command("dm_kp=50.1#");
    Send_Command("dm_kd=5.1#");
    Send_Command("dm_tau_ff=-2.1#");
    Send_Command("unknown=1.0#");
    Send_Command("dm_kp=#");
    Send_Command("dm_kp=abc#");
    assert(Set_K_P_Count == 0);
    assert(Set_K_D_Count == 0);
    assert(Set_Torque_Count == 0);
}

void Feed_Upper_Frame()
{
    std::uint8_t frame[32] = {0x55U, 0xaaU, 0xffU, 0x7fU, 0x00U, 0x80U, 0x18U, 0xfcU};
    for (std::uint8_t i = 8U; i <= 28U; ++i) { frame[i] = i; }
    frame[31] = 0x77U;
    Upper.UART_RxCpltCallback(frame, 32U);
    Upper.TIM_10ms_PeriodElapsedCallback();
}

void Test_Upper_Print_Raw_Data()
{
    Reset_Test();
    Feed_Upper_Frame();
    Debug_UART10_Print_Upper_Data();
    assert(UART10_Output.find("raw_xyw=-32768,32767,-1000") != std::string::npos);
    assert(UART10_Output.find("field=08 09 0A 0B 0C 0D 0E 0F 10 11 12 13") != std::string::npos);
    assert(UART10_Output.find("extend=14 15 16 17 18 19 1A 1B 1C") != std::string::npos);
    assert(UART10_Output.find("online=1 valid=1") != std::string::npos);
    assert(UART10_Output.find("rpm") == std::string::npos);
}

void Test_Upper_Print_Invalid_And_CRC_State()
{
    Reset_Test();
    Test_Tick = 1000U;
    Debug_UART10_Print_Upper_Status_Data();
    assert(UART10_Output.find("online=0 valid=0 age_ms=NA crc_enable=0") != std::string::npos);
    assert(UART10_Output.find("rx=0 frame=0 invalid=0 crc_err=0 overflow=0") != std::string::npos);
    UART10_Busy = false;
    UART10_Output.clear();
    Feed_Upper_Frame();
    Test_Tick = 1500U;
    Upper.TIM_10ms_PeriodElapsedCallback();
    Upper.Set_CRC_Check_Enable(true);
    Debug_UART10_Print_Upper_Status_Data();
    assert(UART10_Output.find("online=0 valid=0 age_ms=500 crc_enable=1") != std::string::npos);
    assert(UART10_Output.find("rx=32 frame=1") != std::string::npos);
    UART10_Busy = false;
    UART10_Output.clear();
    Debug_UART10_Print_Upper_Data();
    assert(UART10_Output.find("online=0 valid=0") != std::string::npos);
    assert(UART10_Output.find("raw_xyw=-32768,32767,-1000") != std::string::npos);
}

void Test_Upper_Print_DMA_Busy_Keeps_Buffer()
{
    Reset_Test();
    Feed_Upper_Frame();
    Debug_UART10_Print_Upper_Data();
    const std::string in_flight(reinterpret_cast<const char *>(UART10_DMA_Buffer), UART10_DMA_Length);
    Test_Tick = 600U;
    Upper.TIM_10ms_PeriodElapsedCallback();
    const int attempts = UART10_Attempts;
    Debug_UART10_Print_Upper_Data();
    Debug_UART10_Print_Upper_Status_Data();
    Debug_UART10_Print_Upper_Data();
    assert(UART10_Attempts == attempts + 3);
    assert(std::string(reinterpret_cast<const char *>(UART10_DMA_Buffer), UART10_DMA_Length) == in_flight);
    UART10_Busy = false;
    Debug_UART10_Print_Upper_Status_Data();
    const std::string status_in_flight(reinterpret_cast<const char *>(UART10_DMA_Buffer), UART10_DMA_Length);
    Debug_UART10_Print_Upper_Status_Data();
    Debug_UART10_Print_Upper_Data();
    assert(std::string(reinterpret_cast<const char *>(UART10_DMA_Buffer), UART10_DMA_Length) == status_in_flight);
    UART10_Busy = false;
    Debug_UART10_Print_Upper_Data();
    assert(UART10_Output.find("online=0 valid=0") != std::string::npos);
}
}

extern "C" std::uint32_t HAL_GetTick() { return Test_Tick; }

bool Chassis_Swerve_Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P)
{
    if (K_P < 0.0f || K_P > 50.0f)
    {
        return false;
    }
    Last_Set_Wheel = Wheel;
    Last_Set_Value = K_P;
    ++Set_K_P_Count;
    return true;
}

bool Chassis_Swerve_Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D)
{
    if (K_D < 0.0f || K_D > 5.0f)
    {
        return false;
    }
    Last_Set_Wheel = Wheel;
    Last_Set_Value = K_D;
    ++Set_K_D_Count;
    return true;
}

bool Chassis_Swerve_Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque)
{
    if (Torque < -2.0f || Torque > 2.0f)
    {
        return false;
    }
    Last_Set_Wheel = Wheel;
    Last_Set_Value = Torque;
    ++Set_Torque_Count;
    return true;
}

std::uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, std::uint8_t *data, const std::uint16_t length)
{
    if (huart == &huart10)
    {
        ++UART10_Attempts;
        if (UART10_Busy) { return HAL_BUSY; }
        UART10_DMA_Buffer = data;
        UART10_DMA_Length = length;
        UART10_Output.append(reinterpret_cast<const char *>(data), length);
        UART10_Busy = true;
        return HAL_OK;
    }
    assert(huart == &huart9);
    assert(length <= sizeof(Transmit_Data));
    std::memcpy(Transmit_Data, data, length);
    Transmit_Length = length;
    ++Transmit_Count;
    return HAL_OK;
}

static void Test_Swerve_Print_Identifies_Missing_Rear_Feedback()
{
    Reset_Test();
    DM_Chassis_LF.Get_Fake_State().rx_update_count = 120U;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_LF.Get_Fake_State().control_status = Motor_DM_Control_Status_ENABLE;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Steer_Homing_Status = Chassis_Swerve_Homing_Status_READY;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_LB].Steer_Homing_Status = Chassis_Swerve_Homing_Status_WAIT_FEEDBACK;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_RB].Steer_Homing_Status = Chassis_Swerve_Homing_Status_WAIT_FEEDBACK;
    Debug_UART10_Print_Swerve_Data();
    assert(UART10_Output.find("home=3,1,1 dm_rx=120,0,0 dm_on=1,0,0 dm_ctl=1,0,0 drive=0,0,0 wait_ms=0 stop=0\r\n") != std::string::npos);
    assert(DM_Chassis_LF.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == 0);
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega == 0.0f);

    UART10_Busy = false; UART10_Output.clear();
    DM_Chassis_RF.Get_Fake_State().rx_update_count = 0xffffffffU;
    DM_Chassis_RF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    Debug_UART10_Print_Swerve_Data();
    assert(UART10_Output.find("dm_rx=120,4294967295,0 dm_on=1,1,0 dm_ctl=1,0,0 drive=0,0,0 wait_ms=0 stop=0\r\n") != std::string::npos);
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Drive_Status = Chassis_Swerve_Drive_Status_WAIT_STOP;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Drive_Wait_Elapsed_Time = 1230U;
    Chassis_Swerve_Diagnostic[Swerve_Wheel_F].Drive_Stop_Stable_Cycles = 7U;
    UART10_Busy = false;
    UART10_Output.clear();
    Debug_UART10_Print_Swerve_Data();
    assert(UART10_Output.find("drive=2,0,0 wait_ms=1230 stop=7\r\n") != std::string::npos);
}

int main()
{
    Reset_Test();
    Test_U8_Result.Status = U8_Test_Status_RUNNING;
    Test_U8_Result.Motor_Mask = 6U;
    auto &test_vesc = VESC_Chassis_RF.Get_Fake_State();
    test_vesc.status_1_valid = true;
    test_vesc.control_omega = 6.28318530718f;
    test_vesc.now_omega = 3.14159265359f;
    Debug_UART10_Print_U8_Test_Data();
    assert(UART10_Output.find("U8 mode=SPEED state=1 mask=6") != std::string::npos);
    assert(UART10_Output.find("id=2 enable=0 target_rpm=60.00 now_rpm=30.00 error_rpm=30.00") != std::string::npos);
    assert(UART10_Output.find("id=3") != std::string::npos);
    assert(UART10_Output.find("now_rpm=nan") != std::string::npos);
    std::string u8_in_flight(reinterpret_cast<char *>(UART10_DMA_Buffer), UART10_DMA_Length);
    test_vesc.control_omega = 12.56637061436f;
    Debug_UART10_Print_U8_Test_Data(); Debug_UART10_Print_U8_Test_Data();
    assert(std::string(reinterpret_cast<char *>(UART10_DMA_Buffer), UART10_DMA_Length) == u8_in_flight);
    Test_Swerve_Print_Identifies_Missing_Rear_Feedback();
    Reset_Test();
    Test_Upper_Control.Configured = true;
    Test_Upper_Control.Status = Upper_Control_Status_RUNNING;
    Test_Upper_Control.Command = {1.0f, -2.0f, 0.5f, true, Chassis_Command_Source_UPPER};
    Debug_UART10_Print_Upper_Control_Data();
    assert(UART10_Output.find("UPPER_CTRL configured=1 source=2 status=3 fault=0 enable=1 speed=1.00,-2.00,0.50") != std::string::npos);
    std::string in_flight(reinterpret_cast<char *>(UART10_DMA_Buffer), UART10_DMA_Length);
    Test_Upper_Control.Fault = Upper_Control_Fault_TIMEOUT;
    Debug_UART10_Print_Upper_Control_Data(); Debug_UART10_Print_Upper_Control_Data();
    assert(std::string(reinterpret_cast<char *>(UART10_DMA_Buffer), UART10_DMA_Length) == in_flight);
    // 类入口同样使用原有轮组绑定和27通道协议。
    Reset_Test();
    static Class_Debug_Control debug;
    debug.Init();
    Configure_Wheel(VESC_Chassis_RF.Get_Fake_State(), DM_Chassis_RF.Get_Fake_State(), Chassis_Swerve_Diagnostic[Swerve_Wheel_LB], 2.0f);
    debug.Set_Swerve_Wheel(Swerve_Wheel_LB);
    debug.Vofa_TIM_10ms_PeriodElapsedCallback();
    float class_channels[27]{};
    Read_Frame(class_channels);
    Assert_Near(class_channels[0], 120.0f);
    Assert_Near(class_channels[8], 0.2f);
    Configure_Wheel(VESC_Chassis_LB.Get_Fake_State(), DM_Chassis_LB.Get_Fake_State(), Chassis_Swerve_Diagnostic[Swerve_Wheel_RB], 3.0f);
    debug.Set_Swerve_Wheel(Swerve_Wheel_RB);
    debug.Vofa_TIM_10ms_PeriodElapsedCallback();
    Read_Frame(class_channels);
    Assert_Near(class_channels[0], 180.0f);
    Assert_Near(class_channels[8], 0.3f);
    Test_Debug_Emits_27_Channel_Frame_Every_10ms();
    Test_Debug_Uses_NaN_For_Invalid_Continuous_Feedback();
    Test_Debug_Stages_And_Applies_MIT_Commands_Once();
    Test_Debug_Rejects_Invalid_And_Malformed_MIT_Commands();
    Test_Upper_Print_Raw_Data();
    Test_Upper_Print_Invalid_And_CRC_State();
    Test_Upper_Print_DMA_Busy_Keeps_Buffer();
    std::puts("Debug: VOFA/MIT and upper print tests passed");
    return 0;
}
