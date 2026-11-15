/**
 * @file tsk_chassis.h
 * @brief 底盘控制任务
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 *
 * @note 三舵轮底盘任务：回零、解算调用、DM控制及U8速度斜坡和反转保护。
 * @note Task_Init中初始化，前台每10ms更新控制并发送，100ms检查在线状态。
 * @note 常改参数在本头文件：活动轮组、DEBUG测试ID、U8加减速及回零/MIT参数。
 * @note 运动学和暂定几何见tsk_chassis_resolve；正常与DEBUG共用同一U8保护状态。
 */

#ifndef TSK_CHASSIS_H
#define TSK_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "3_Module/3swerve_chassis/tsk_chassis_resolve.h"
#include "4_Task/tsk_motor_init/motor_init.h"

/* Exported macros -----------------------------------------------------------*/

// 当前只接入前中ID1轮组; 三组接齐后改为((1U << Swerve_Wheel_Number) - 1U)。
#ifndef CHASSIS_SWERVE_ACTIVE_WHEEL_MASK
#define CHASSIS_SWERVE_ACTIVE_WHEEL_MASK (1U << Swerve_Wheel_F)
#endif
#define CHASSIS_SWERVE_CONTROL_PERIOD_MS 10U

// DEBUG测试的实际CAN节点ID列表, 与正常底盘活动掩码独立; 修改后重新编译烧录。
// {1U}、{2U}、{1U,2U}、{1U,2U,3U}; 空列表、非法或重复ID均不启动测试。
#ifndef U8_TEST_MOTOR_IDS
#define U8_TEST_MOTOR_IDS {1U}
#endif
#define VESC_ACCEL_RPM_PER_S 1400.0f // U8加速斜坡, 机械rpm/s
#define VESC_DECEL_RPM_PER_S 1400.0f // U8减速斜坡, 机械rpm/s
#define VESC_STOP_ENTER_RPM 10.0f // 新反馈连续不超过此绝对机械rpm才确认停稳
#define VESC_STOP_EXIT_RPM 20.0f // 已停稳、零目标时超过此绝对机械rpm才退出停稳
#define VESC_REVERSE_STABLE_CYCLES 10U // 停稳所需的新低速反馈次数
#define VESC_REVERSE_TIMEOUT_CYCLES 500U // 停车/反转等待超时, 10ms周期数

// 低速主动制动需确认电调与电源承受能力后启用，默认只发送零电流滑行。
#define VESC_LOW_SPEED_BRAKE_ENABLE 0U
#define VESC_LOW_SPEED_BRAKE_RPM 100.0f // 只允许低于此机械rpm时主动制动
#define VESC_LOW_SPEED_BRAKE_CURRENT_A 0.2f // 正制动电流幅值，A

/* Exported types ------------------------------------------------------------*/

enum Enum_U8_Test_Status : uint8_t
{
    U8_Test_Status_DISABLED = 0,
    U8_Test_Status_RUNNING,
    U8_Test_Status_FAULT
};

/**
 * @brief U8行进保护状态, 故障锁定至在线遥控关闭总使能
 */
enum Enum_Chassis_Swerve_Drive_Status : uint8_t
{
    Chassis_Swerve_Drive_Status_NORMAL = 0,
    Chassis_Swerve_Drive_Status_DECELERATING,
    Chassis_Swerve_Drive_Status_WAIT_STOP,
    Chassis_Swerve_Drive_Status_FEEDBACK_LOST,
    Chassis_Swerve_Drive_Status_TIMEOUT,
    Chassis_Swerve_Drive_Status_STOPPED, // 新反馈连续满足低速条件，当前零电流释放
};
struct Struct_Chassis_Swerve_Drive_Control
{
    Enum_Chassis_Swerve_Drive_Status Status{};
    // 最近下发的非零方向, +1正转/-1反转/0尚未建立
    int8_t Last_Direction = 0;
    // 已消费的STATUS_1更新计数, 不重复累计旧反馈
    uint32_t Feedback_Update_Count = 0U;
    // 本次停车/反转减速及停稳等待耗时, 单位10ms周期
    uint16_t Elapsed_Cycles = 0U;
    // 已收到的新低速反馈次数
    uint8_t Stable_Cycles = 0U;
};

/**
 * @brief 正常底盘和DEBUG共用的U8参数, 加减速单位为机械rpm/s
 */
struct Struct_U8_Control_Parameter
{
    bool Low_Speed_Brake_Enable = VESC_LOW_SPEED_BRAKE_ENABLE != 0U;
    float Low_Speed_Brake_RPM = VESC_LOW_SPEED_BRAKE_RPM;
    float Low_Speed_Brake_Current_A = VESC_LOW_SPEED_BRAKE_CURRENT_A;
    float Accel_RPM_Per_S = VESC_ACCEL_RPM_PER_S;
    float Decel_RPM_Per_S = VESC_DECEL_RPM_PER_S;
    // 轮端角速度上限, rad/s; 底盘初始化时传入实际共用配置
    float Wheel_Omega_Max = 150.79644737f;
    // 速度斜坡调用周期, ms
    uint32_t Control_Period_MS = 10U;
    // 初始发送轮组掩码, 位0/1/2对应前中/左后/右后
    uint8_t Send_Wheel_Mask = 1U;
    // 三轮行进安装方向, 由底盘绑定参数传入
    float Drive_Direction[3] = {1.0f, 1.0f, 1.0f};
};

/**
 * @brief 测试快照; 读取不使能电机或刷新反馈年龄
 */
struct Struct_U8_Test_Result
{
    Enum_U8_Test_Status Status = U8_Test_Status_DISABLED;
    uint8_t Motor_Mask = 0U;
};

/**
 * @brief 单一U8控制对象, 复用原速度斜坡/反转状态, 避免正常与测试各留一套保护
 */
class Class_U8_Control
{
public:
    // 初始化相关函数

    void Init(const Struct_U8_Control_Parameter &Parameter);

    // 行进速度控制, 输入为轮端rad/s

    void Set_Speed(Enum_Swerve_Wheel Wheel, float Target_Omega);

    // 失能与保护状态

    void Disable(uint8_t Mask = 7U, bool Reset_Drive_State = false);

    void Reset_Ramp(bool Reset_Drive_State = false);

    // 读变量

    bool Get_Fault() const;

    const Struct_Chassis_Swerve_Drive_Control &Get_Drive(uint8_t Wheel) const;

    const Struct_U8_Control_Parameter &Get_Parameter() const { return Parameter; }

    // 电机绑定与发送轮组

    Class_Motor_VESC *Get_Motor(Enum_Swerve_Wheel Wheel) const;

    void Mark_Used(uint8_t Mask) { Send_Wheel_Mask |= Mask & 7U; }

    // 周期回调函数

    void Send_10ms();

    void Alive_100ms();

private:
    // 初始化相关常量

    Struct_U8_Control_Parameter Parameter{};
    static constexpr float RPM_TO_RAD_S = 0.10471975511965977f;
    // 内部变量: 每轮独立保存斜坡和反转状态

    struct Struct_U8_Wheel_Control
    {
        // 本周期斜坡后的轮端角速度, rad/s
        float Drive_Control_Omega = 0.0f;
        Struct_Chassis_Swerve_Drive_Control Drive{};
    } Wheel[3]{};
    Class_Motor_VESC *const Motor[3] = {&VESC_Chassis_LF, &VESC_Chassis_RF, &VESC_Chassis_LB};
    uint8_t Send_Wheel_Mask = 0U;

    // 内部函数

    void Set_Stop_Output(Class_Motor_VESC *Motor, bool Stopped, float Now_Omega);

    float Update_Drive_Omega(Enum_Swerve_Wheel Wheel, float Target_Omega);
};

/**
 * @brief DEBUG输入策略, 硬件输出只经过同一个U8_Control对象
 */
class Class_U8_Test
{
public:
    // 初始化相关函数

    void Init();

    // DEBUG速度测试入口, 右Y输入为m/s

    void Update(float Speed_Y, bool Enable, bool Acknowledge = false);

    void Exit();

    // 读变量

    const Struct_U8_Test_Result &Get_Result() const { return Result; }

private:
    // 初始化相关常量: DEBUG所选CAN节点

    static constexpr uint8_t Motor_ID[] = U8_TEST_MOTOR_IDS;
    // 内部变量

    Struct_U8_Test_Result Result{};
    // DEBUG速度测试是否已接管所选U8
    bool Active = false;

    // 内部函数

    uint8_t Get_Motor_Mask() const;
};

/**
 * @brief 最终底盘速度来源, 仅执行车体速度, 不包含定位或跑点状态
 */
enum Enum_Chassis_Command_Source : uint8_t
{
    Chassis_Command_Source_NONE = 0,
    Chassis_Command_Source_MANUAL,
    Chassis_Command_Source_UPPER,
    Chassis_Command_Source_DEBUG_U8,
};

/**
 * @brief 原三舵轮任务的统一车体速度输入
 * @note X向右、Y向前, 单位m/s; W逆时针为正, 单位rad/s; 默认失能
 */
struct Struct_Chassis_Command
{
    float Speed_X = 0.0f, Speed_Y = 0.0f, Speed_W = 0.0f;
    bool Motor_Enable = false;
    Enum_Chassis_Command_Source Source = Chassis_Command_Source_NONE;
};

/**
 * @brief 舵轮回零判定参数, 计数单位为10ms控制周期
 */
struct Struct_Chassis_Swerve_Homing_Parameter
{
    float Position_Limit = 0.05f;                      // 电机轴位置误差上限, rad
    float Omega_Limit = 0.5f;                          // 电机轴速度上限, rad/s
    uint8_t Stable_Cycles = 10U;                       // 连续稳定周期数
    uint16_t Timeout_Cycles = 300U;                    // 回零超时周期数
};

/**
 * @brief 串口MIT调参的允许范围, 不是驱动器电流或转矩保护配置
 */
struct Struct_Chassis_Swerve_MIT_Limit
{
    float K_P_Min = 0.0f;
    float K_P_Max = 50.0f;
    float K_D_Min = 0.0f;
    float K_D_Max = 5.0f;
    float Torque_Min = -2.0f;                          // 前馈转矩下限, N·m
    float Torque_Max = 2.0f;                           // 前馈转矩上限, N·m
};

/**
 * @brief 舵轮任务固定配置, 默认值集中在此处, 运行中不修改
 * @note CAN ID、协议量程及MIT初值仍由motor_init模块负责
 */
struct Struct_Chassis_Swerve_Config
{
    float Wheel_Radius = SWERVE_WHEEL_RADIUS_M;        // 暂定轮半径, m; 实测后在tsk_chassis_resolve.h修改直径
    float Chassis_Radius = SWERVE_CHASSIS_RADIUS_M;     // 暂定轮组中心半径=正三角形边长/√3, 不是底盘外壳尺寸
    float Wheel_Omega_Max = 150.79644737f;             // 最大轮速, rad/s
    float Steer_Angle_Limit = 245.0f;                  // 舵向目标软限位, rad
    float Steer_Full_Speed_Angle = 0.08726646f;        // 全速允许舵向误差, rad(5°)
    float Steer_Stop_Angle = 0.78539816f;              // 停止驱动舵向误差, rad(45°)
    float Accel_RPM_Per_S = VESC_ACCEL_RPM_PER_S;      // 加速度, 机械rpm/s
    float Decel_RPM_Per_S = VESC_DECEL_RPM_PER_S;      // 减速度, 机械rpm/s
    uint32_t Control_Period_MS = CHASSIS_SWERVE_CONTROL_PERIOD_MS;
    uint8_t Active_Wheel_Mask = CHASSIS_SWERVE_ACTIVE_WHEEL_MASK;
    Struct_Chassis_Swerve_Homing_Parameter Homing;
    Struct_Chassis_Swerve_MIT_Limit MIT_Limit;
};

/**
 * @brief 单轮电机绑定与安装配置, 仅引用motor_init中的既有对象
 */
struct Struct_Chassis_Swerve_Wheel_Config
{
    Class_Motor_DM_Normal *Steer_Motor;
    Class_Motor_VESC *Drive_Motor;
    float Steer_Direction;                             // 舵向安装方向, ±1
    float Steer_Offset;                                // 舵向机构角度偏移, rad
    float Drive_Direction;                             // 行进安装方向, ±1
};

/**
 * @brief 单轮MIT可调参数, 保存在RAM, 通过既有校验函数修改
 */
struct Struct_Chassis_Swerve_MIT_Parameter
{
    float K_P;
    float K_D;
    float Feedforward_Torque;                          // 前馈转矩, N·m
};

/**
 * @brief 单个舵轮遥控使能后的回零状态
 */
enum Enum_Chassis_Swerve_Homing_Status : uint8_t
{
    Chassis_Swerve_Homing_Status_DISABLED = 0,
    Chassis_Swerve_Homing_Status_WAIT_FEEDBACK,
    Chassis_Swerve_Homing_Status_HOMING,
    Chassis_Swerve_Homing_Status_READY,
    Chassis_Swerve_Homing_Status_TIMEOUT,
};

/**
 * @brief 单轮回零运行状态, 失能后重新开始回零
 */
struct Struct_Chassis_Swerve_Homing_Control
{
    Enum_Chassis_Swerve_Homing_Status Status;
    uint32_t Feedback_Update_Count;
    uint16_t Elapsed_Cycles;
    uint8_t Stable_Cycles;
};

/**
 * @brief 单轮任务运行状态, 与固定配置及MIT可调参数分开保存
 */
struct Struct_Chassis_Swerve_Wheel_Control
{
    Struct_Chassis_Swerve_Homing_Control Homing;
};

/**
 * @brief 单个舵轮控制诊断数据
 * @note 由10ms底盘任务更新，供低频串口诊断读取
 */
struct Struct_Chassis_Swerve_Diagnostic
{
    // 解算后的目标舵角, 机构侧rad
    float Steer_Target_Angle;
    // 实际舵角, 机构侧rad
    float Steer_Now_Angle;
    // 最短舵向角差, rad
    float Steer_Angle_Error;
    // 实际舵向角速度, 机构侧rad/s
    float Steer_Now_Omega;
    // 预留舵向速度限制, rad/s; 当前诊断固定为0
    float Steer_Target_Omega_Limit;
    // 舵向误差对应的行进轮速度系数, 0~1
    float Steer_Speed_Ratio;
    // DM连续角度是否已收到有效初始反馈
    bool Total_Angle_Initialized;
    // 回零状态: 0失能/1等反馈/2回零/3就绪/4超时
    Enum_Chassis_Swerve_Homing_Status Steer_Homing_Status;
    // 当前回零耗时, ms
    uint32_t Steer_Homing_Elapsed_Time;
    // 当前是否向该轮提交舵向目标
    bool Steer_Target_Enable;
    // U8保护: 0运行/1减零/2等停稳/3反馈失效/4超时/5已停稳
    Enum_Chassis_Swerve_Drive_Status Drive_Status;
    uint32_t Drive_Wait_Elapsed_Time;                   // 反转减速及停稳等待时间, ms
    uint8_t Drive_Stop_Stable_Cycles;                    // 已接受的新低速反馈数
};

/**
 * @brief 三舵轮底盘控制类, 负责回零、解算调用和电机控制
 * @note 内部解算类仅负责运动学, 本类负责回零、斜坡、安全和电机控制
 * @note 绑定既有电机对象, 生产调度只使用cpp中的一个实例; 不拥有电机驱动
 */
class Class_Chassis_Control
{
public:
    // 初始化相关函数, 需先Motor_Init

    void Init();

    // 写变量: 本周期唯一车体速度与使能

    void Set_Command(const Struct_Chassis_Command &Command);

    // 读变量: 回零就绪及故障

    bool Get_Swerve_Ready() const;

    bool Get_Swerve_Fault() const;

    // 周期回调函数, 控制更新后再发送VESC

    void TIM_10ms_Swerve_PeriodElapsedCallback();

    void TIM_10ms_VESC_PeriodElapsedCallback();

    void TIM_100ms_PeriodElapsedCallback();

    // 写变量: 单轮MIT调参, 仅保存至RAM

    bool Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P);

    bool Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D);

    bool Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque);

private:
    // 固定配置: 默认值见上方结构体, 不通过串口在运行中修改
    const Struct_Chassis_Swerve_Config Chassis_Swerve_Config = {};

    static_assert(VESC_ACCEL_RPM_PER_S > 0.0f && VESC_DECEL_RPM_PER_S > 0.0f, "VESC ramp limits must be positive");
    static_assert(Swerve_Wheel_F == 0 && Swerve_Wheel_LB == 1 && Swerve_Wheel_RB == 2 && Swerve_Wheel_Number == 3, "Wheel configuration order must match Enum_Swerve_Wheel");

    // 前中、左后、右后对应ID1、2、3; 保留motor_init原对象名及CAN协议配置。
    // 每项依次为舵向电机、行进电机、舵向方向、角度偏移(rad)、行进方向
    const Struct_Chassis_Swerve_Wheel_Config Swerve_Wheel_Config[Swerve_Wheel_Number] = {
        {&DM_Chassis_LF, &VESC_Chassis_LF, 1.0f, 0.0f, 1.0f}, // 前中, ID1
        {&DM_Chassis_RF, &VESC_Chassis_RF, 1.0f, 0.0f, 1.0f}, // 左后, ID2
        {&DM_Chassis_LB, &VESC_Chassis_LB, 1.0f, 0.0f, 1.0f}, // 右后, ID3
    };

    // 解算对象: 不负责CAN通信或电机驱动
    Class_Chassis_Swerve Chassis_Swerve;

    // 内部变量: 最终命令和MIT参数, 禁止外部直接修改
    Struct_Chassis_Command Final_Command{};

    Struct_Chassis_Swerve_Wheel_Control Swerve_Wheel_Control[Swerve_Wheel_Number] = {};
    Struct_Chassis_Swerve_MIT_Parameter Swerve_MIT_Parameter[Swerve_Wheel_Number] = {};

    // 内部函数: 电机控制与发送

    void Swerve_Motor_Send();

    void Swerve_Steer_Motor_Control();

    void Swerve_Drive_Motor_Control();

    void Swerve_Motor_Disable(bool Reset_Drive_State = true);

    void Swerve_Control();

    void Swerve_Steer_Motor_Disable();

    // 内部函数: 安装角度换算、回零及诊断
    Class_Motor_DM_Normal *Swerve_Get_Steer_Motor(Enum_Swerve_Wheel Wheel);

    Class_Motor_VESC *Swerve_Get_Drive_Motor(Enum_Swerve_Wheel Wheel);

    bool Swerve_Is_Wheel_Active(Enum_Swerve_Wheel Wheel);

    float Swerve_Motor_To_Mechanism_Angle(Enum_Swerve_Wheel Wheel, float Motor_Angle);

    float Swerve_Mechanism_To_Motor_Angle(Enum_Swerve_Wheel Wheel, float Mechanism_Angle);

    float Swerve_Motor_To_Mechanism_Omega(Enum_Swerve_Wheel Wheel, float Motor_Omega);

    void Swerve_Hold_Homing_Target(Enum_Swerve_Wheel Wheel);

    bool Swerve_Update_Homing(Enum_Swerve_Wheel Wheel);

    void Swerve_Update_Diagnostic(Enum_Swerve_Wheel Wheel);

    float Swerve_Get_Steer_Speed_Ratio(Enum_Swerve_Wheel Wheel);

};

/* Exported variables --------------------------------------------------------*/

// 保留原诊断快照接口供VOFA和UART10读取, 不参与控制状态计算
extern Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number];

/* Exported function declarations --------------------------------------------*/

// 保留现有任务与调参入口, 内部统一转调同一个Class_Chassis_Control对象

void Chassis_Init();

/**
 * @brief 在前台提交最终车体速度命令, 不直接发送电机报文
 * @param Command 车体速度m/s、rad/s及使能
 */
void Chassis_Set_Command(const Struct_Chassis_Command &Command);

/**
 * @brief 读取底盘内部回零就绪状态
 * @return bool 启用轮组全部就绪时返回true
 */
bool Chassis_Get_Swerve_Ready();

/**
 * @brief 读取底盘内部回零超时、行进反馈失效及反转超时状态
 * @return bool 任一启用轮组存在上述故障时返回true
 */
bool Chassis_Get_Swerve_Fault();

void Chassis_Swerve_10ms_PeriodElapsedCallback();

void Chassis_VESC_10ms_PeriodElapsedCallback();

void Chassis_100ms_PeriodElapsedCallback();

bool Chassis_Swerve_Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P);

bool Chassis_Swerve_Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D);

bool Chassis_Swerve_Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque);

/* U8行进控制与DEBUG速度测试接口 ----------------------------------------------*/
extern Class_U8_Control U8_Control;
/* Exported function declarations --------------------------------------------*/
/**
 * @brief 初始化唯一测试对象, 在底盘初始化后调用
 */
void U8_Test_Init();
/**
 * @brief 前台10ms测试入口, 不在此入口发送CAN
 */
void U8_Test_Update(float Speed_Y, bool Enable, bool Acknowledge = false);
/**
 * @brief 离开DEBUG时撤销测试输出
 */
void U8_Test_Exit();
/**
 * @brief 读取串口诊断快照, 不参与控制权选择
 */
const Struct_U8_Test_Result &U8_Test_Get_Result();

#endif
