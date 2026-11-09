# Application source migration

自写固件统一由 `User/CMakeLists.txt` 登记。模块头文件放在 `inc/`，实现放在
`src/`；没有对外接口的模块不建空头文件。CubeMX 外设初始化、内核启动、
向量表、链接脚本和第三方库仍在原位，生成入口只保留必要的调用接线。

| 目录 | 改什么时看这里 |
| --- | --- |
| `Config/inc/robot_config.h` | 角度、速度、时长、PID、队伍映射和调试开关。 |
| `Hardware` | 电机、陀螺仪、串口、PWM 和板上 IO 的访问。 |
| `Motion` | 轮速、角度环、视觉控制和挡板/夹爪姿态接口。 |
| `Flows` | 单个流程内部动作的顺序与完成条件。 |
| `Mission` | 比赛状态切换、视觉握手、恢复和目标策略。 |
| `Runtime` | 上电、任务创建、周期服务、中断、MaixCAM 串口和调试输出。 |
| `Legacy/Bluetooth` | 旧蓝牙和模式控制，留档，不参与当前构建。 |

仓库根目录的 `tests/` 是主机回归，不是车载固件。旧版目录和参数的历史
说明存档在 [legacy_task_architecture.md](docs/legacy_task_architecture.md)。

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

`26` 的动作顺序和推进计时已在第三阶段迁入 `Flows/Delivery`。ROI、目标选择、
坐标有效期策略和握手仍由 Mission 决定。没有新增 FreeRTOS 任务。
独立动作回归入口：`tests/verify_motion_layer.ps1`。

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
没有新任务或脚本解释器。Mission 保留旧状态编号和适配入口，把传感器、
时钟和已有的轮速/舵机动作交给流程。急停和故障优先，流程退出后不再调用
其动作；投放挡板 PWM 的释放仍由任务层在取消时触发。

串口事件优先级、坐标帧清理、回包、送达计数、`05` 清点、补抓预算和 `07`
仍由比赛状态机负责。独立流程回归入口：`tests/verify_flows_layer.ps1`；现有
投放、避杆、整理、抓取和恢复回归同时覆盖这些适配入口。

## 第四阶段：任务编排

| 模块 | 手动改动入口 |
| --- | --- |
| `Mission/State` | `mission_state.c`：PE13～15 的队伍/出发区锁定、发车与越减速带衔接、中央状态切换和每轮处理顺序；`mission_task.h` 是其他任务使用的公开接口。 |
| `Mission/Safety` | `mission_safety.c`：启动检查、运行中的电机/IMU 健康检查，以及启动、复位、停止、急停事件的优先级。 |
| `Mission/Vision` | `mission_vision.c`：坐标类别、有效期、参考点、`03` 追踪、`E3` 搜索恢复和 `AE` 避让。MaixCAM 字节解析仍在原串口模块。 |
| `Mission/Capture` | `mission_arrange.c`：`04/14`、推散和 `E4/22`；`mission_capture.c`：`24/34/54` 抓取的进入、结束和超时；`mission_load.c`：`05/45` 清点、调整和撤离复检；`mission_supplement.c`：补抓计数、预算、握手和恢复。 |
| `Mission/SafeZone` | `mission_safe_zone.c`：`06/16` 找区、横移/撤离判断和 `36/46/6E`；`mission_delivery.c`：`26` 投放的进入、取消和退车后切换 `07`。 |
| `Mission/Strategy` | `mission_strategy.c`：`07` 选目标、伤员优先切换、送达次数限制和下一轮搜索。 |

只改舵机角度、速度、时长或 PID 参数，先查 `Config/inc/robot_config.h`。改一段
动作怎样执行、动作的先后顺序，先查对应的 `Flows` 模块；改收到什么消息才
开始、做完进入哪个状态，查上述 Mission 文件。动作顺序改动涉及握手或故障
时，仍应同时核对 Mission 的入口和结束条件。

Mission 仍由一个 FreeRTOS 任务顺序执行。各源文件独用的计数器、计时器保留
为文件私有；跨模块的快照和任务状态集中在私有 `MissionContext`，不会向
其他任务公开。其他任务继续通过 `MissionTask_GetSnapshot()` 读快照，通过
原有请求 API 提交事件。

## 运行时接线：FreeRTOS

- `Runtime/Rtos/src/app_rtos.c`：应用任务创建顺序、比赛和调试模式分支、
  UART2 日志接线。任务创建失败仍走原来的启动故障处理，不启动调度器。
- `Runtime/Rtos/src/app_rtos_task_config.c`：应用线程的名字、优先级和栈大小，
  栈单位为字节。Mission 的互斥锁和事件标志、日志的队列仍由各自模块创建。
- `Core/Src/main.c` 和 `Core/Src/freertos.c`：保留 CubeMX 的内核初始化、启动
  和 `defaultTask` 创建，应用入口放在 `USER CODE` 区。心跳的 `defaultTask`
  优先级和栈大小仍在 `gx_version1.ioc` 配置（Normal、128 字）。
- 调试开关在 `Config/inc/robot_config.h`，任务周期仍在各任务实现处，未修改。
  `tests/verify_straight_test.ps1` 验证实际启动代码在比赛、直行、舵机、
  挡板和直行加舵机模式下的创建顺序、线程配置与创建失败保护。

其余接线按职责归档：`Runtime/Boot` 管上电和故障处理，
`Runtime/Interrupts` 管手写的 UART4／USART3／CAN1_RX0 向量和 UART 回调，
`Runtime/Comms` 管视觉串口协议，`Runtime/Control` 管电机周期任务和 PE12，
`Runtime/Diagnostics` 管 OLED、心跳和 UART2 日志，`Runtime/Bench` 管单独调试。
如果将来在 CubeMX 开启上述三个中断向量的代码生成，要先处理手写向量的归属，
避免同名 `IRQHandler` 重复定义。
