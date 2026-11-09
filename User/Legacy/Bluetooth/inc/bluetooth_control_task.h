#ifndef BLUETOOTH_CONTROL_TASK_H
#define BLUETOOTH_CONTROL_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  BLUETOOTH_MOTION_STOP = 0,
  BLUETOOTH_MOTION_FORWARD,
  BLUETOOTH_MOTION_BACKWARD,
  BLUETOOTH_MOTION_LEFT,
  BLUETOOTH_MOTION_RIGHT
} BluetoothMotion;

typedef struct
{
  BluetoothMotion motion;
  uint32_t valid_frames;
  uint32_t invalid_frames;
  uint32_t last_command_tick;
  uint8_t last_command;
  bool control_enabled;
} BluetoothControlSnapshot;

bool BluetoothControlTask_Create(void);
bool BluetoothControlTask_GetSnapshot(BluetoothControlSnapshot *snapshot);
void StartBluetoothControlTask(void *argument);

#endif /* BLUETOOTH_CONTROL_TASK_H */
