#include "imu_diagnostics.h"
#include "debug_uart_task.h"

const ImuDiagnosticsSink imu_diagnostics = {
  .log = DebugUart_Log,
  .logf = DebugUart_Logf,
};
