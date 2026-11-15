# Robocon-Elian-R2

STM32H723ZGT6裸机三舵轮工程，使用CubeMX、CMake及VS Code STM32Cube工具链。

- 当前仅启用前中ID1轮组：DM6220舵向电机 ×1、U8 KV100行进电机 ×1、VESC电调 ×1；IMU为超核HI06 ×1。ID2/3绑定保留。
- 遥控总使能开启后，MANUAL使用摇杆驾驶，AUTOMATIC使用上位机车体速度，功能键不切换控制权。F2平移四挡上限4/8/12/16m/s，F1独立控制旋转挡位，速度适配均在R2完成。
- DEBUG使用右摇杆Y独立测试U8，DM保持失能，无需舵向回零；其余摇杆及功能键不控制U8。在[底盘与U8测试参数](D:/27Robocon/Robocon-Elian-R2/User_File/3_Module/3swerve_chassis/tsk_chassis.h)修改U8_TEST_MOTOR_IDS列表，例如{1U}、{2U}、{1U, 2U}或{1U, 2U, 3U}，重新编译烧录生效，默认ID1。速度测试沿用正常限速、F2挡位、斜坡与反转保护。
- 上位机负责定位、跑点及比赛流程；STM32保留速度接收、三舵轮解算、回零和电机保护。UART7保留32字节接收布局，生产入口启用CRC；速度缩放、限速和超时默认未配置，因此AUTOMATIC保持失能。
- 接入上位机时，在总使能关闭、电机失能的前台调用Upper_Configure_Speed，显式配置缩放、限速和超时。车体系X向右、Y向前，单位m/s；W逆时针为正，单位rad/s。参数见[速度接口](D:/27Robocon/Robocon-Elian-R2/User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.h)，字段见[UART7协议](D:/27Robocon/Robocon-Elian-R2/User_File/1_Middleware/Protocol/pro_upper/pro_upper.h)。
- 轮径和底盘尺寸尚未实测，暂用轮径0.200m、轮组中心正三角形边长0.450m。实测后修改[几何参数](D:/27Robocon/Robocon-Elian-R2/User_File/3_Module/3swerve_chassis/tsk_chassis_resolve.h)。当前活动轮组掩码为0x01，三组接齐后在[底盘任务参数](D:/27Robocon/Robocon-Elian-R2/User_File/3_Module/3swerve_chassis/tsk_chassis.h)改为0x07。
- UART9输出VOFA共27通道，周期10ms；DEBUG自动观察所选ID列表中的第一台，I0～I7仍为转速和电气量，I24为U8状态（0运行/1减零/2等待/3反馈丢失/4超时/5停稳），I25为新低速反馈数，I26为控制模式（0RPM/1零电流释放/2制动电流）。USART10每100ms正常打印SW诊断，DEBUG改为U8诊断，包含全部所选ID的目标/实际机械rpm、转速误差、反馈有效性及驱动保护码；无效反馈显示nan。当前正常底盘ID1行进须home=3,0,0且转速反馈有效。
- 遥控离线或撤销许可时清零失能。上位机坏帧、超速、断流以及行进反转保护故障按各自路径锁定；在线遥控关闭总使能确认故障后才可重新使能。保留VESC回充上限0A，反转等待零电流不代表主动刹停。

在工程根目录运行：

```powershell
cmake --build build/Debug
./tests/host/run_tests.ps1 -Suite all
```

固件产物为build/Debug/Robocon-Elian-R2.elf。首次烧录后先架空低速核对回零、方向和反转保护；软件验证不代表实车验收。

- 机械臂模块位于User_File/3_Module/Arm，当前仅保留模块，默认任务不调用。后续接入时显式调用Motor_Arm_Init与Arm.Init，接入10ms/100ms回调，并在CAN1分发三个机械臂节点的8字节标准反馈帧。mod_arm_control.h集中配置零偏、方向、实际关节/相对角限位和Point[9]。配置中的角度填°，角速度填°/s，角加速度填°/s²。初始化时统一换算为弧度制。未配置时保持失能。未来遥控在前台显式申请Arm.Set_Enable(true)，再按按下沿调用Arm.Set_Target_Point(1~9)。运动中返回BUSY且不缓存；Arm.Set_Enable(false)撤销许可。DM2325使用独立Set_Rotate_Target，单位为电机输出轴rad和rad/s。到位仅表示关节反馈满足阈值，尚未验证实体碰撞或实机精度。

- U8普通停车与反转共用实际停稳确认，先按1400机械RPM/s减到目标零，再以零电流滑行，连续10帧有效STATUS_1的绝对转速不超过10RPM才确认停稳。已停稳且零目标时，超过20RPM才退出停稳；灰区内收到起步目标时重新确认10RPM停稳条件。反馈异常或等待5秒超时锁定失能，需在线关闭总使能复位。低速主动制动在tsk_chassis.h默认关闭，显式开启后仅在100RPM以下使用0.2A正制动电流，停稳后仍释放。禁止回充的当前供电条件下保持关闭。
