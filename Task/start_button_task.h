#ifndef START_BUTTON_TASK_H
#define START_BUTTON_TASK_H

#include <stdbool.h>

/* PE12 uses its CubeMX pull-up; a debounced low level is one button press. */
bool StartButtonTask_Create(void);
void StartStartButtonTask(void *argument);

#endif /* START_BUTTON_TASK_H */
