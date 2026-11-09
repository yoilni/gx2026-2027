#ifndef IMU_DIAGNOSTICS_H
#define IMU_DIAGNOSTICS_H

#include <stdbool.h>

/* Bound by the application at link time. Sensor drivers never depend on
   FreeRTOS or the UART2 debug queue. Called only from task context. */
typedef struct
{
  bool (*log)(const char *text);
  bool (*logf)(const char *format, ...);
} ImuDiagnosticsSink;

extern const ImuDiagnosticsSink imu_diagnostics;

#endif
