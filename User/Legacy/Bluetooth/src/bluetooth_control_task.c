#include "bluetooth_control_task.h"

#include "M2006_Speed.h"
#include "bluetooth.h"
#include "bluetooth_frame.h"
#include "cmsis_os.h"
#include "control_mode_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

#define BLUETOOTH_CONTROL_TASK_PERIOD_TICKS 10U
#define BLUETOOTH_CONTROL_TASK_STACK_BYTES  (256U * 4U)
#define BLUETOOTH_READ_CHUNK_SIZE           32U

static osThreadId_t bluetooth_control_task_handle;
static volatile BluetoothControlSnapshot bluetooth_control_snapshot;

static const osThreadAttr_t bluetooth_control_task_attributes = {
  .name = "bluetoothControlTask",
  .stack_size = BLUETOOTH_CONTROL_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

static void BluetoothControlTask_Publish(void)
{
  /* The snapshot fields are copied under the same short interrupt critical
     section used by its reader. */
  __DMB();
}

static void BluetoothControlTask_SetWheelTargets(float left_rpm,
                                                  float right_rpm)
{
  SpeedLoop_SetMotorTarget(ROBOT_LEFT_WHEEL_MOTOR_ID,
      left_rpm * ROBOT_LEFT_WHEEL_FORWARD_SIGN,
      ROBOT_BLUETOOTH_CURRENT_LIMIT);
  SpeedLoop_SetMotorTarget(ROBOT_RIGHT_WHEEL_MOTOR_ID,
      right_rpm * ROBOT_RIGHT_WHEEL_FORWARD_SIGN,
      ROBOT_BLUETOOTH_CURRENT_LIMIT);
}

static void BluetoothControlTask_SetMotion(BluetoothMotion motion)
{
  if (bluetooth_control_snapshot.motion == motion)
  {
    return;
  }

  switch (motion)
  {
    case BLUETOOTH_MOTION_FORWARD:
      BluetoothControlTask_SetWheelTargets(ROBOT_BLUETOOTH_DRIVE_SPEED_RPM,
                                           ROBOT_BLUETOOTH_DRIVE_SPEED_RPM);
      break;

    case BLUETOOTH_MOTION_BACKWARD:
      BluetoothControlTask_SetWheelTargets(-ROBOT_BLUETOOTH_DRIVE_SPEED_RPM,
                                           -ROBOT_BLUETOOTH_DRIVE_SPEED_RPM);
      break;

    case BLUETOOTH_MOTION_LEFT:
      BluetoothControlTask_SetWheelTargets(-ROBOT_BLUETOOTH_DRIVE_SPEED_RPM,
                                           ROBOT_BLUETOOTH_DRIVE_SPEED_RPM);
      break;

    case BLUETOOTH_MOTION_RIGHT:
      BluetoothControlTask_SetWheelTargets(ROBOT_BLUETOOTH_DRIVE_SPEED_RPM,
                                           -ROBOT_BLUETOOTH_DRIVE_SPEED_RPM);
      break;

    case BLUETOOTH_MOTION_STOP:
    default:
      brake();
      motion = BLUETOOTH_MOTION_STOP;
      break;
  }

  bluetooth_control_snapshot.motion = motion;
  BluetoothControlTask_Publish();
}

static bool BluetoothControlTask_HandleCommand(uint8_t command)
{
  switch (command)
  {
    case BLUETOOTH_COMMAND_FORWARD_PRESS:
      BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_FORWARD);
      break;

    case BLUETOOTH_COMMAND_FORWARD_RELEASE:
      if (bluetooth_control_snapshot.motion == BLUETOOTH_MOTION_FORWARD)
      {
        BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
      }
      break;

    case BLUETOOTH_COMMAND_BACKWARD_PRESS:
      BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_BACKWARD);
      break;

    case BLUETOOTH_COMMAND_BACKWARD_RELEASE:
      if (bluetooth_control_snapshot.motion == BLUETOOTH_MOTION_BACKWARD)
      {
        BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
      }
      break;

    case BLUETOOTH_COMMAND_LEFT_PRESS:
      BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_LEFT);
      break;

    case BLUETOOTH_COMMAND_LEFT_RELEASE:
      if (bluetooth_control_snapshot.motion == BLUETOOTH_MOTION_LEFT)
      {
        BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
      }
      break;

    case BLUETOOTH_COMMAND_RIGHT_PRESS:
      BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_RIGHT);
      break;

    case BLUETOOTH_COMMAND_RIGHT_RELEASE:
      if (bluetooth_control_snapshot.motion == BLUETOOTH_MOTION_RIGHT)
      {
        BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
      }
      break;

    case BLUETOOTH_COMMAND_STOP_PRESS:
      BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
      break;

    default:
      return false;
  }

  bluetooth_control_snapshot.last_command = command;
  bluetooth_control_snapshot.last_command_tick = HAL_GetTick();
  ++bluetooth_control_snapshot.valid_frames;
  BluetoothControlTask_Publish();
  return true;
}

static void BluetoothControlTask_DiscardInput(BluetoothFrameParser *parser)
{
  uint8_t bytes[BLUETOOTH_READ_CHUNK_SIZE];

  while (Bluetooth_ReadRaw(bytes, sizeof(bytes)) != 0U)
  {
  }
  BluetoothFrameParser_Reset(parser);
}

static void BluetoothControlTask_ProcessInput(BluetoothFrameParser *parser)
{
  uint8_t bytes[BLUETOOTH_READ_CHUNK_SIZE];
  uint8_t command;
  uint16_t count;

  do
  {
    count = Bluetooth_ReadRaw(bytes, sizeof(bytes));
    for (uint16_t i = 0U; i < count; ++i)
    {
      BluetoothParseResult result =
          BluetoothFrameParser_PushByte(parser, bytes[i], &command);
      if (result == BLUETOOTH_PARSE_FRAME)
      {
        if (!BluetoothControlTask_HandleCommand(command))
        {
          ++bluetooth_control_snapshot.invalid_frames;
        }
      }
      else if (result == BLUETOOTH_PARSE_ERROR)
      {
        ++bluetooth_control_snapshot.invalid_frames;
      }
    }
  } while (count == sizeof(bytes));
}

bool BluetoothControlTask_Create(void)
{
  if (bluetooth_control_task_handle != NULL)
  {
    return true;
  }

  bluetooth_control_task_handle = osThreadNew(StartBluetoothControlTask, NULL,
                                              &bluetooth_control_task_attributes);
  return bluetooth_control_task_handle != NULL;
}

bool BluetoothControlTask_GetSnapshot(BluetoothControlSnapshot *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = bluetooth_control_snapshot;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

void StartBluetoothControlTask(void *argument)
{
  BluetoothFrameParser parser;
  ControlModeSnapshot mode_snapshot;
  ControlMode previous_mode = CONTROL_MODE_AUTONOMOUS;
  bool mode_initialized = false;
  uint32_t next_wake;

  (void)argument;
  BluetoothFrameParser_Reset(&parser);
  bluetooth_control_snapshot.motion = BLUETOOTH_MOTION_STOP;
  brake();
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    if (ControlModeTask_GetSnapshot(&mode_snapshot))
    {
      if (!mode_initialized || (mode_snapshot.mode != previous_mode))
      {
        BluetoothControlTask_SetMotion(BLUETOOTH_MOTION_STOP);
        BluetoothControlTask_DiscardInput(&parser);
        previous_mode = mode_snapshot.mode;
        mode_initialized = true;
      }

      bluetooth_control_snapshot.control_enabled =
          mode_snapshot.mode == CONTROL_MODE_BLUETOOTH;
      if (bluetooth_control_snapshot.control_enabled)
      {
        BluetoothControlTask_ProcessInput(&parser);
      }
      else
      {
        BluetoothControlTask_DiscardInput(&parser);
      }
      BluetoothControlTask_Publish();
    }

    next_wake += BLUETOOTH_CONTROL_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
