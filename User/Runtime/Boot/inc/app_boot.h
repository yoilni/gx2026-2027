#ifndef APP_BOOT_H
#define APP_BOOT_H

/* Called from the CubeMX entry point at the corresponding boot phases. */
void AppBoot_BeforeHal(void);
void AppBoot_AfterHal(void);
void AppBoot_AfterClock(void);
void AppBoot_AfterPeripherals(void);
void AppBoot_BeforeScheduler(void);
void AppBoot_Fault(void);

#endif
