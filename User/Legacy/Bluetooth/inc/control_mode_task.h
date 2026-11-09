#ifndef CONTROL_MODE_TASK_H
#define CONTROL_MODE_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  CONTROL_MODE_AUTONOMOUS = 0,
  CONTROL_MODE_BLUETOOTH
} ControlMode;

typedef struct
{
  ControlMode mode;
  uint32_t sequence;
  uint32_t update_tick;
} ControlModeSnapshot;

/* Creates the PE15 debounced mode selector task. */
bool ControlModeTask_Create(void);
bool ControlModeTask_GetSnapshot(ControlModeSnapshot *snapshot);
void StartControlModeTask(void *argument);

#endif /* CONTROL_MODE_TASK_H */
