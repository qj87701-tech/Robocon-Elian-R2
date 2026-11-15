#include <cmath>
#include <cstdio>
#include <cstdint>
#include <initializer_list>

#include "test_assert.h"
#include "2_Device/NRF24L01/dvc_nrf24l01.h"
#include "4_Task/tsk_remote/tsk_remote.h"

namespace
{
void Assert_Near(const float value, const float expected)
{
    if (std::fabs(value - expected) >= 1.0e-5f)
    {
        std::fprintf(stderr, "expected %.8f, got %.8f\n", expected, value);
        assert(false);
    }
}

void Pack_Remote_Data(const std::uint16_t channel_0, const std::uint16_t channel_1, const std::uint16_t channel_3, const std::uint8_t mode, const std::uint8_t motor_enable)
{
    std::uint16_t channel[16]{};
    channel[0] = channel_0;
    channel[1] = channel_1;
    channel[2] = 1000U;
    channel[3] = channel_3;
    for (std::uint8_t i = 4U; i < 16U; i++)
    {
        channel[i] = 1000U;
    }

    Fake_NRF24L01 = {};
    Fake_NRF24L01.Rx_Packet[0] = 0x0FU;
    Fake_NRF24L01.Rx_Packet[23] = 0xFFU;
    Fake_NRF24L01.Rx_Packet[24] = 0xAFU;
    Fake_NRF24L01.Rx_Packet[25] = static_cast<std::uint8_t>((mode & 0x03U) | ((motor_enable & 0x01U) << 2U));
    Fake_NRF24L01.Rx_Packet[28] = 0x88U;

    for (std::uint8_t channel_index = 0U; channel_index < 16U; channel_index++)
    {
        for (std::uint8_t bit_index = 0U; bit_index < 11U; bit_index++)
        {
            if ((channel[channel_index] & (1U << bit_index)) != 0U)
            {
                std::uint16_t packet_bit = static_cast<std::uint16_t>(channel_index * 11U + bit_index);
                Fake_NRF24L01.Rx_Packet[1U + packet_bit / 8U] |= static_cast<std::uint8_t>(1U << (packet_bit % 8U));
            }
        }
    }

    Fake_NRF24L01.Receive_Result = 1U;
}

void Receive_One_Packet()
{
    Remote_10ms_PeriodElapsedCallback();
    Fake_NRF24L01.Receive_Result = 0U;
}

void Test_Remote_Init_Loads_Safe_Command()
{
    Fake_NRF24L01 = {};
    Remote_Init();

    assert(Fake_NRF24L01.Init_Count == 1);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
    assert(Remote_Chassis_Command.Online == 0U);
    assert(Remote_Chassis_Command.Mode == 0U);
    assert(Remote_Chassis_Command.Motor_Enable == 0U);
}

// 挡位已在遥控端缩放; 主控不得再次乘挡位比例, 正负方向和F1/F2均须匹配。
void Test_Four_Gears_Use_Requested_Linear_And_Angular_Ceilings()
{
    const uint16_t span[] = {200U, 400U, 600U, 800U};
    const float speed[] = {4.0f, 8.0f, 12.0f, 16.0f};
    const float omega[] = {15.39600718f, 30.79201436f, 46.18802154f, 61.58402872f};
    Remote_Init();
    for (int gear = 0; gear < 4; gear++)
    {
        for (int direction : {-1, 1})
        {
            const uint16_t endpoint = static_cast<uint16_t>(1000 + direction * span[gear]);
            Pack_Remote_Data(endpoint, 1000U, endpoint, 2U, 1U);
            Receive_One_Packet();
            Assert_Near(Remote_Chassis_Command.Speed_Y, direction * speed[gear]);
            Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
            Assert_Near(Remote_Chassis_Command.Speed_W, direction * omega[gear]);
            Pack_Remote_Data(1000U, endpoint, 1000U, 2U, 1U);
            Receive_One_Packet();
            Assert_Near(Remote_Chassis_Command.Speed_X, direction * speed[gear]);
            Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
            Pack_Remote_Data(1000U, static_cast<uint16_t>(1000 + direction * span[gear] / 2), 1000U, 2U, 1U);
            Receive_One_Packet();
            Assert_Near(Remote_Chassis_Command.Speed_X, direction * speed[gear] / 2.0f);
        }
    }
    Pack_Remote_Data(1200U, 1000U, 1800U, 2U, 1U); // F2低挡与F1极速可同时使用。
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_Y, 4.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 61.58402872f);
}

// 直接接收旧遥控器的方形X/Y通道, 不依赖发送端预处理或新增挡位字段。
void Test_R2_Maps_All_Gears_And_Directions_Without_Remote_Changes()
{
    const int span[] = {200, 400, 600, 800};
    const float speed[] = {4.0f, 8.0f, 12.0f, 16.0f};
    Remote_Init();
    for (int gear = 0; gear < 4; gear++)
    {
        for (int x = -4; x <= 4; x++)
        {
            for (int y = -4; y <= 4; y++)
            {
                Pack_Remote_Data(static_cast<uint16_t>(1000 + y * span[gear] / 4), static_cast<uint16_t>(1000 + x * span[gear] / 4), 1800U, 2U, 1U);
                Receive_One_Packet();
                const float vx = Remote_Chassis_Command.Speed_X;
                const float vy = Remote_Chassis_Command.Speed_Y;
                const float expected = speed[gear] * fmaxf(std::abs(x), std::abs(y)) / 4.0f;
                Assert_Near(std::hypot(vx, vy), expected);
                assert(std::fabs(vx * y - vy * x) < 0.0001f); // 保持方向, 不是只识别45°。
                assert(vx * x >= 0.0f && vy * y >= 0.0f);
                Assert_Near(Remote_Chassis_Command.Speed_W, 61.58402872f); // F1独立。
            }
        }
    }
    Pack_Remote_Data(1200U, 1003U, 1000U, 2U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Speed_Y > 3.99f); // 轻微侧偏不能使主方向骤降到约0.707倍。
    Assert_Near(std::hypot(Remote_Chassis_Command.Speed_X, Remote_Chassis_Command.Speed_Y), 4.0f);
}

void Test_Channel_Endpoints_Map_To_Minus_Max_Zero_And_Plus_Max()
{
    Remote_Init();
    Pack_Remote_Data(200U, 1000U, 1800U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, -16.0f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 61.58402872f);
    assert(Remote_Chassis_Command.Online == 1U);
    assert(Remote_Chassis_Command.Mode == 2U);
    assert(Remote_Chassis_Command.Motor_Enable == 1U);
}

void Test_Out_Of_Range_Channels_Clamp_And_Center_Dead_Zone_Clears()
{
    Remote_Init();
    Pack_Remote_Data(0U, 2047U, 1001U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, -11.31370850f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 11.31370850f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
}

void Test_Channel_Zero_One_And_Three_Map_To_Y_X_And_W()
{
    Remote_Init();
    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, 3.57770876f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 7.15541753f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 46.18802154f);
}

void Test_Remote_Gear_Range_Is_Mapped_Only_Once()
{
    Remote_Init();

    Pack_Remote_Data(1000U, 1000U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);

    Pack_Remote_Data(1000U, 1200U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 4.0f);

    Pack_Remote_Data(1000U, 1400U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 8.0f);

    Pack_Remote_Data(1000U, 1600U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 12.0f);

    Pack_Remote_Data(1000U, 1800U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 16.0f);

    Pack_Remote_Data(1000U, 1000U, 1200U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 15.39600718f);

    Pack_Remote_Data(1000U, 1000U, 1400U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 30.79201436f);

    Pack_Remote_Data(1000U, 1000U, 1600U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 46.18802154f);

    Pack_Remote_Data(1000U, 1000U, 1800U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 61.58402872f);
}

void Test_Non_Manual_Disable_And_Offline_Clear_All_Chassis_Speeds()
{
    Remote_Init();
    Pack_Remote_Data(1200U, 1400U, 1600U, 0U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);

    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 0U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);

    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 1U);
    Receive_One_Packet();
    for (std::uint8_t i = 0U; i < 100U; i++)
    {
        Remote_10ms_PeriodElapsedCallback();
    }
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
    assert(Remote_Chassis_Command.Online == 0U);
}

void Test_Pages_Select_Manual_Or_Autonomous_Control()
{
    Remote_Init();
    Pack_Remote_Data(1200U, 1400U, 1000U, 3U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_AUTO);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    const uint32_t auto_request = Remote_Chassis_Command.Control_Request_Count;
    for (uint8_t keys : {0x01U, 0x02U, 0x03U, 0x00U})
    {
        Pack_Remote_Data(1200U, 1400U, 1000U, 3U, 1U);
        Fake_NRF24L01.Rx_Packet[26] = keys;
        Receive_One_Packet();
        assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_AUTO);
        assert(Remote_Chassis_Command.Control_Request_Count == auto_request);
        Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    }
    Pack_Remote_Data(1200U, 1400U, 1000U, 2U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_MANUAL);
    assert(Remote_Chassis_Command.Control_Request_Count == auto_request + 1U);
    Assert_Near(Remote_Chassis_Command.Speed_X, 7.15541753f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 3.57770876f);
    Pack_Remote_Data(1200U, 1400U, 1000U, 2U, 1U);
    Fake_NRF24L01.Rx_Packet[26] = 0x03U;
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Request_Count == auto_request + 1U);
    Assert_Near(Remote_Chassis_Command.Speed_X, 7.15541753f);
    Pack_Remote_Data(1200U, 1400U, 1000U, 3U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_AUTO);
    assert(Remote_Chassis_Command.Control_Request_Count == auto_request + 2U);
    for (int i = 0; i < 100; i++) { Remote_10ms_PeriodElapsedCallback(); }
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_NONE);
    Pack_Remote_Data(1200U, 1400U, 1000U, 3U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_AUTO);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f); // 重连只恢复许可, 不提交旧目标
    Pack_Remote_Data(1200U, 1400U, 1000U, 1U, 1U);
    Receive_One_Packet();
    assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_DEBUG_U8);
}

void Test_Debug_Right_Y_Uses_Normal_Gears_And_Ignores_Other_Axes()
{
    Remote_Init();
    const uint16_t amplitudes[] = {200U, 400U, 600U, 800U};
    for (uint16_t amplitude : amplitudes)
    {
        for (int direction : {-1, 1})
        {
            const uint16_t channel = static_cast<uint16_t>(1000 + direction * amplitude);
            Pack_Remote_Data(channel, 1800U, 200U, 1U, 1U);
            Receive_One_Packet();
            Assert_Near(Remote_Chassis_Command.Speed_Y, direction * amplitude * 0.02f);
            Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
            Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
            assert(Remote_Chassis_Command.Control_Source == Remote_Control_Source_DEBUG_U8);
        }
    }
    Pack_Remote_Data(1000U, 1800U, 200U, 1U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Pack_Remote_Data(1800U, 1800U, 200U, 1U, 0U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    for (int i = 0; i < 100; i++) { Remote_10ms_PeriodElapsedCallback(); }
    assert(Remote_Chassis_Command.Online == 0U);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
}

void Test_NRF_Polling_And_One_Second_Timeout_Are_Preserved()
{
    Remote_Init();
    Pack_Remote_Data(1000U, 1000U, 1000U, 2U, 1U);
    Receive_One_Packet();
    assert(Fake_NRF24L01.Receive_Count == 1);

    for (std::uint8_t i = 0U; i < 99U; i++)
    {
        Remote_10ms_PeriodElapsedCallback();
    }
    assert(Remote_Chassis_Command.Online == 1U);

    Remote_10ms_PeriodElapsedCallback();
    assert(Remote_Chassis_Command.Online == 0U);
    assert(Fake_NRF24L01.Receive_Count == 101);
}
}

int main()
{
    Test_Debug_Right_Y_Uses_Normal_Gears_And_Ignores_Other_Axes();
    Test_R2_Maps_All_Gears_And_Directions_Without_Remote_Changes();
    Test_Four_Gears_Use_Requested_Linear_And_Angular_Ceilings();
    Test_Pages_Select_Manual_Or_Autonomous_Control();
    // 类入口仍向既有底盘指令快照输出, 不创建第二个接收器。
    Class_Remote_Control remote;
    remote.Init();
    Pack_Remote_Data(200U, 1000U, 1800U, 2U, 1U);
    remote.TIM_10ms_PeriodElapsedCallback();
    Assert_Near(Remote_Chassis_Command.Speed_Y, -16.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 61.58402872f);
    Test_Remote_Init_Loads_Safe_Command();
    Test_Channel_Endpoints_Map_To_Minus_Max_Zero_And_Plus_Max();
    Test_Out_Of_Range_Channels_Clamp_And_Center_Dead_Zone_Clears();
    Test_Channel_Zero_One_And_Three_Map_To_Y_X_And_W();
    Test_Remote_Gear_Range_Is_Mapped_Only_Once();
    Test_Non_Manual_Disable_And_Offline_Clear_All_Chassis_Speeds();
    Test_NRF_Polling_And_One_Second_Timeout_Are_Preserved();
    return 0;
}
