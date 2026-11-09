#ifndef RECOVERY_FLOW_H
#define RECOVERY_FLOW_H
#include "flow_motion.h"

/* Integrate directed yaw for spins that must not take a shortest-path turn. */
void RecoveryFlow_ObserveSpin(int32_t *previous_yaw, int32_t *progress, int32_t current_yaw,
    int32_t yaw_direction);
int16_t RecoveryFlow_TaperSpin(int32_t target, int32_t progress, int32_t slow_threshold,
    int16_t fast_rpm, int16_t slow_rpm);

typedef enum { RECOVERY_RED_FORWARD, RECOVERY_RED_LEFT, RECOVERY_RED_HOLD_LEFT,
               RECOVERY_RED_RIGHT, RECOVERY_RED_HOLD_RIGHT, RECOVERY_RED_HOME,
               RECOVERY_RED_DONE } RedSweep_Phase;

RedSweep_Phase RecoveryFlow_StepRed(const FlowMotion *io, RedSweep_Phase phase,
    bool (*expired)(uint32_t now, int32_t target), uint32_t now, uint32_t started_at,
    uint32_t forward_ms, int16_t forward_rpm, int32_t yaw, int32_t *origin, int32_t *target,
    int16_t *turn_command, int32_t *yaw_error);

typedef enum { ZONE_RECOVERY_FIRST_DRIVE, ZONE_RECOVERY_FACE, ZONE_RECOVERY_LONG_DRIVE,
               ZONE_RECOVERY_LEFT, ZONE_RECOVERY_RIGHT, ZONE_RECOVERY_HOME,
               ZONE_RECOVERY_DONE } ZoneRecovery_Phase;
ZoneRecovery_Phase RecoveryFlow_StepZone(const FlowMotion *io, ZoneRecovery_Phase phase,
    uint32_t now, uint32_t started_at, int32_t left_yaw, int32_t right_yaw, int32_t safe_yaw);

typedef enum { RECOVERY_POST22, RECOVERY_ARRANGE_LOSS, RECOVERY_CAPTURE_LOSS } RecoveryScan_Profile;
typedef enum { RECOVERY_SCAN_BACK, RECOVERY_SCAN_FIRST, RECOVERY_SCAN_SECOND,
               RECOVERY_SCAN_DONE } RecoveryScan_Phase;
/* For forced-side recovery, the caller supplies the directed turn primitive. */
RecoveryScan_Phase RecoveryFlow_StepScan(const FlowMotion *io, RecoveryScan_Profile profile,
    RecoveryScan_Phase phase, uint32_t now, uint32_t started_at, int32_t heading);
#endif
