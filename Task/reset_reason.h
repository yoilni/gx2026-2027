#ifndef RESET_REASON_H
#define RESET_REASON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  RESET_REASON_UNKNOWN = 0,
  RESET_REASON_POWER_ON,
  RESET_REASON_BROWNOUT,
  RESET_REASON_EXTERNAL_PIN,
  RESET_REASON_SOFTWARE,
  RESET_REASON_IWDG,
  RESET_REASON_WWDG,
  RESET_REASON_LOW_POWER
} ResetReason;

typedef struct
{
  ResetReason reason;
  uint32_t raw_flags;
  bool captured;
} ResetReasonSnapshot;

/* Call once at the beginning of main(), before peripheral initialization. */
void ResetReason_Capture(void);
bool ResetReason_GetSnapshot(ResetReasonSnapshot *snapshot);

#endif /* RESET_REASON_H */
