# Application source migration

Each migrated module keeps its headers in `inc/` and its implementation in `src/`.
The root build includes `User/CMakeLists.txt`; CubeMX still configures peripherals
and starts the existing tasks.

## 第一阶段：硬件层

| 模块 | 入口 | 边界 |
| --- | --- | --- |
| `Hardware/Motor` | `M2006_Init()`、`moto_chassis`、`set_moto_current()` | 复用原 CAN 初始化、反馈解析和电流帧；轮速 PID 和周期任务不在这里。 |
| `Hardware/Imu` | `IMU_Init()`、`IMU_Service()`、`IMU_GetYaw()` | UART4 的 HWT101/JY901S 探测和接收；调用者仍读取百分之一度、序号和时间戳。日志经运行时适配，驱动不引用调试队列。 |
| `Hardware/Servo` | `ServoPwm_WritePulseUs()`、`ServoPwm_IsEnabled()`、`ServoPwm_Start()`、`ServoPwm_Stop()` | PA5/CH1 摄像头、PA2/CH3 挡板、PA3/CH4 夹爪。只处理脉宽和输出启停，不处理角度、姿态或延时。 |
| `Hardware/Input` | `SwitchInput_ReadStart()`、`ReadTeam()`、`ReadZoneMsb()`、`ReadZoneLsb()`（均带 `SwitchInput_` 前缀） | 只读 PE12～PE15 原始电平；低电平启动、消抖、红蓝和出发区映射留在调用方。 |
| `Hardware/Indicator` | `BoardLed_Toggle()` | 只翻转 PC13；启动故障闪烁和任务周期由上层管理。 |
| `Hardware/Serial` | `UartPort_ReceiveIT()`、`UartPort_AbortReceive()`、`UartPort_Transmit()` | 直接沿用 HAL 的状态、超时和接收缓冲区。启动日志单独走 DWT 限时轮询；不新增 UART 所有者，不解析 MaixCAM 帧。 |
| `Hardware/Display` | `OLED_Init()`、`OLED_DisplayOff()`、`OLED_Clear()`、`OLED_WriteString()` | 复用原 I2C 屏幕驱动；显示内容和刷新周期留在 OLED 任务。 |

硬件阶段保留原上层 API、动作时序、协议和任务调度。OLED 任务、串口回调
路由和旧蓝牙代码不属于硬件驱动。`robot_config.h` 仍是现有配置入口。

硬件回归入口：`tests/verify_board_io.ps1`、`tests/verify_servo_test_outputs.ps1`、
`tests/verify_imu_selection.ps1`、`tests/verify_hwt101.ps1`。

## 第二阶段：单一动作和控制环

| 模块 | 入口 | 边界 |
| --- | --- | --- |
| `Motion/Chassis` | `ChassisMotion_MixRpm()`、`ChassisMotion_SetWheelRpm()`、`Stop()`、`EmergencyStop()` | 左右轮使用车体坐标系，正值都表示前进；这里转换成现有电机方向和轮号。正常刹车和急停保持不同入口。 |
| `Motion/Chassis` | `HeadingControl_Update()`、`CheckTurn()`、`LimitStraight()` | 计算航向差速 RPM、判断转向稳定、限制偏航时的纵向速度。调用方持有状态，提供参数并决定超时和下一步。 |
| `Motion/Chassis` | `ChassisMotion_TurnToYawStep()`、`ChassisMotion_TimedDriveStep()` | 单次绝对航向转向、定时等速前后移动。返回轮速、刹车请求和完成/超时状态，不直接发轮速、不阻塞；取消时调用方停止输出。 |
| `Motion/Servo` | `ServoMotion_SetBaffleAngle()`、`StopBafflePwm()`、`SetClawAngle()`、`SetCameraAngle()` | 接受角度，单次只操作一个输出。闭爪延时和多通道启动仍由现有 `Actuator_*` 适配层管理；旧流程不能绕过适配层直接关爪。 |
| `Motion/Vision` | `VisionMotion_CalculateTurn()`、`CalculateForward()`、`GuardForward()`、`AgeScale()` | 视觉 X/Y PID、分段速度、反向新帧确认、旧帧降速和安全区限速。调用方选择控制配置、提供有效帧，不在这里解析协议或决定目标。 |
| `Motion/WheelSpeed` | 原 `SpeedLoop_*` 和 PID | 代码原样迁移。原 2 ms 电机任务仍是唯一 CAN 电流下发者。 |

`26` 的顺序、推进计时、到位参数与超时策略仍在原状态机；下一阶段再迁入
`Flows/Delivery`。ROI、目标选择、坐标有效期策略和握手仍在原层，也没有
新增 FreeRTOS 任务。独立动作回归入口：`tests/verify_motion_layer.ps1`。

## 第三阶段：动作流程

| 模块 | 改动入口 |
| --- | --- |
| `Flows/Delivery` | `26` 对正、顶边、第一次退让、推进、抬板和最终退车。推进只累计实际发出纵向前进的时间；强制退车和正常送达是不同结果。 |
| `Flows/BarShift` | `46` 对正、后退、侧转、前进、再对正。模块完成时不回包。 |
| `Flows/Obstacle` | `36` 向外转、定时前进、反向转、回到安全区方向。 |
| `Flows/Reposition` | 侧移的等速行驶、角落撤离的两段航向直行，以及各自回正。保留原始侧移起点，`26` 暂存只调整总时长。 |
| `Flows/Arrange` | `02/32/42` 固定推散、绿色首轮定向旋转、`E5` 推散、旧弧线路径与反向回放。 |
| `Flows/Capture` | 开爪与视角动作、`24` 追踪窗口、`34` 停车闭爪与等待保护、`54` 夹爪调整后的闭爪。 |
| `Flows/Departure` | 发车转向、开爪等待、抬挡板、越减速带、放板等待和闭爪。 |
| `Flows/Recovery` | 红色扫视、`6E` 首次/后续恢复、`E2/EE` 退车与扫视、定向旋转的角度累计。 |

每个流程以原比赛任务的 tick 执行；`Flows/Common` 只执行单个转向或驱动段，
没有新任务或脚本解释器。`Task/mission_task.c` 暂保留旧状态编号和适配入口，
把传感器、时钟和已有的轮速/舵机动作交给流程。急停和故障优先，流程退出后
不再调用其动作；投放挡板 PWM 的释放仍由任务层在取消时触发。

串口事件优先级、坐标帧清理、回包、送达计数、`05` 清点、补抓预算和 `07`
仍由比赛状态机负责。独立流程回归入口：`tests/verify_flows_layer.ps1`；现有
投放、避杆、整理、抓取和恢复回归同时覆盖这些适配入口。
