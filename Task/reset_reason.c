#include "reset_reason.h"

#include "stm32f4xx_hal.h"

#define RESET_REASON_FLAG_MASK (RCC_CSR_BORRSTF  | RCC_CSR_PINRSTF | \
                                RCC_CSR_PORRSTF  | RCC_CSR_SFTRSTF | \
                                RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
                                RCC_CSR_LPWRRSTF)

static ResetReasonSnapshot reset_reason_snapshot;

static ResetReason ResetReason_Decode(uint32_t flags)
{
  /* Watchdog and software flags take priority because some reset sources can
     set more than one RCC flag. A normal cold start commonly sets both POR and
     BOR, so POR must be checked before classifying a standalone brownout. */
  if ((flags & RCC_CSR_IWDGRSTF) != 0U)
  {
    return RESET_REASON_IWDG;
  }
  if ((flags & RCC_CSR_WWDGRSTF) != 0U)
  {
    return RESET_REASON_WWDG;
  }
  if ((flags & RCC_CSR_SFTRSTF) != 0U)
  {
    return RESET_REASON_SOFTWARE;
  }
  if ((flags & RCC_CSR_LPWRRSTF) != 0U)
  {
    return RESET_REASON_LOW_POWER;
  }
  if ((flags & RCC_CSR_PORRSTF) != 0U)
  {
    return RESET_REASON_POWER_ON;
  }
  if ((flags & RCC_CSR_BORRSTF) != 0U)
  {
    return RESET_REASON_BROWNOUT;
  }
  if ((flags & RCC_CSR_PINRSTF) != 0U)
  {
    return RESET_REASON_EXTERNAL_PIN;
  }
  return RESET_REASON_UNKNOWN;
}

void ResetReason_Capture(void)
{
  uint32_t flags = RCC->CSR & RESET_REASON_FLAG_MASK;

  reset_reason_snapshot.raw_flags = flags;
  reset_reason_snapshot.reason = ResetReason_Decode(flags);
  reset_reason_snapshot.captured = true;

  /* Reset flags are sticky. Clear them only after saving this boot's cause so
     a future reset cannot be confused with an older one. */
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

bool ResetReason_GetSnapshot(ResetReasonSnapshot *snapshot)
{
  if ((snapshot == NULL) || !reset_reason_snapshot.captured)
  {
    return false;
  }

  *snapshot = reset_reason_snapshot;
  return true;
}
