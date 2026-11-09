# Application source migration

Each migrated module keeps its headers in `inc/` and its implementation in `src/`.
The root build includes `User/CMakeLists.txt`; CubeMX still configures peripherals
and starts the existing tasks.

## 第一阶段：硬件层

| 模块 | 入口 | 边界 |
| --- | --- | --- |
| `Hardware/Motor` | `M2006_Init()`、`moto_chassis`、`set_moto_current()` | 复用原 CAN 初始化、反馈解析和电流帧；轮速 PID 和周期任务不在这里。 |
| `Hardware/Imu` | `IMU_Init()`、`IMU_Service()`、`IMU_GetYaw()` | UART4 的 HWT101/JY901S 探测和接收；调用者仍读取百分之一度、序号和时间戳。日志经运行时适配，驱动不引用调试队列。 |
| `Hardware/Servo` | `ServoPwm_WritePulseUs()`、`ServoPwm_IsEnabled()`、`ServoPwm_Start()`、`ServoPwm_Stop()` | PA5/CH1 摄像头、PA2/CH3 挡板、PA3/CH4 夹爪。写脉宽不启停 PWM；角度换算、复合姿态与延迟闭爪仍在 `Task/actuator_task.c`。 |
| `Hardware/Input` | `SwitchInput_ReadStart()`、`ReadTeam()`、`ReadZoneMsb()`、`ReadZoneLsb()`（均带 `SwitchInput_` 前缀） | 只读 PE12～PE15 原始电平；低电平启动、消抖、红蓝和出发区映射留在调用方。 |
| `Hardware/Indicator` | `BoardLed_Toggle()` | 只翻转 PC13；启动故障闪烁和任务周期由上层管理。 |
| `Hardware/Serial` | `UartPort_ReceiveIT()`、`UartPort_AbortReceive()`、`UartPort_Transmit()` | 直接沿用 HAL 的状态、超时和接收缓冲区。启动日志单独走 DWT 限时轮询；不新增 UART 所有者，不解析 MaixCAM 帧。 |
| `Hardware/Display` | `OLED_Init()`、`OLED_DisplayOff()`、`OLED_Clear()`、`OLED_WriteString()` | 复用原 I2C 屏幕驱动；显示内容和刷新周期留在 OLED 任务。 |

这一步保留原上层 API、动作时序、协议和任务调度。CAN 速度环、OLED 任务、
串口回调路由和旧蓝牙代码尚不迁移。`robot_config.h` 仍是现有配置入口。

硬件回归入口：`tests/verify_board_io.ps1`、`tests/verify_servo_test_outputs.ps1`、
`tests/verify_imu_selection.ps1`、`tests/verify_hwt101.ps1`。
