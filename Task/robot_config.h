#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Bounded cold-start retries; never auto-reboot or run on a failed clock/CAN. */
#define ROBOT_BOOT_INIT_MAX_ATTEMPTS      5U
#define ROBOT_BOOT_INIT_RETRY_DELAY_MS    200U
#define ROBOT_BOOT_UART_TIMEOUT_MS       50U

/* 1: initialize OLED and refresh status/yaw; 0: blank it without a task. */
#define ROBOT_OLED_ENABLED 1U

/* Temporary MG90 position-only test; set to 0 to restore the mission. */
#define ROBOT_MG90_SPEED_TEST_ENABLED 0U
#define ROBOT_MG90_TEST_ANGLE_DEG ROBOT_CAMERA_NEAR_ANGLE_DEG
#define ROBOT_MG90_TEST_PULSE_US \
    (ROBOT_CAMERA_SERVO_MIN_PULSE_US + \
     (ROBOT_MG90_TEST_ANGLE_DEG * \
      (ROBOT_CAMERA_SERVO_MAX_PULSE_US - ROBOT_CAMERA_SERVO_MIN_PULSE_US)) / 180U)

/* Temporary frame-position test (legacy DOWN switch): no motor, start-button,
   or mission task. Lift is measured from the fully lowered position. */
#define ROBOT_FRAME_DOWN_TEST_ENABLED 0U
#define ROBOT_FRAME_TEST_LIFT_DEG 20U
#define ROBOT_FRAME_TEST_CAMERA_ANGLE_DEG ROBOT_CAMERA_WIDE_ANGLE_DEG

/* One-shot wheel test; set to 0 to restore the mission. */
#define ROBOT_STRAIGHT_TEST_ENABLED 0U
#define ROBOT_STRAIGHT_TEST_RPM 100
#define ROBOT_STRAIGHT_TEST_MS 1000U

/* UART2 periodic diagnostics only. State transitions, faults and protocol
   handshakes remain immediate; this does not throttle vision UART traffic. */
#define ROBOT_DEBUG_PERIOD_MS 750U
/* Angle snapshots, plus an immediate snapshot on every state entry. */
#define ROBOT_ANGLE_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS
/* Continuous UART2 yaw monitoring, including WAIT_START/STOPPED/FAULT. */
#define ROBOT_YAW_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS
#define ROBOT_S7_SILENT_TIMEOUT_MS 2000U
#define ROBOT_S7_SILENT_LEFT_CDEG  4500L
#define ROBOT_S7_SILENT_RIGHT_CDEG 9000L

/* M2006 CAN IDs configured on the two wheel ESCs. */
#define ROBOT_LEFT_WHEEL_MOTOR_ID       2U
#define ROBOT_RIGHT_WHEEL_MOTOR_ID      4U

/* Mirrored wheel installation: change either sign after an off-ground test if
   the corresponding wheel turns opposite to the intended vehicle direction. */
#define ROBOT_LEFT_WHEEL_FORWARD_SIGN   1.0f
#define ROBOT_RIGHT_WHEEL_FORWARD_SIGN (-1.0f)

/* Wheel-speed target slew for normal starts, stops, and reversals.
   Safety faults bypass this ramp and stop immediately. */
#define ROBOT_WHEEL_ACCEL_RPM_PER_S      700.0f
#define ROBOT_WHEEL_DECEL_RPM_PER_S      900.0f

/* PE13 team selector: high selects red and low selects blue. The mission
   latches the selected team when PE12 starts a match. */
#define ROBOT_TEAM_RED                     1U
#define ROBOT_TEAM_BLUE                    2U
#define ROBOT_PE13_LOW_TEAM    ROBOT_TEAM_BLUE
#define ROBOT_PE13_HIGH_TEAM    ROBOT_TEAM_RED

/* PE14 is the high bit and PE15 is the low bit of the start-area selector.
   There is no area zero, so 00 represents area 4. */
#define ROBOT_START_ZONE_BITS_00             4U
#define ROBOT_START_ZONE_BITS_01             1U
#define ROBOT_START_ZONE_BITS_10             2U
#define ROBOT_START_ZONE_BITS_11             3U

/* Safe-zone entry headings are relative to the yaw captured when PE12 starts
   the match. For areas 1/2, blue is forward and red is reverse; for areas
   3/4, red is forward and blue is reverse. */
#define ROBOT_SAFE_ZONE_FORWARD_OFFSET_CDEG      0L
#define ROBOT_SAFE_ZONE_REVERSE_OFFSET_CDEG  18000L

/* Legacy manual-drive calibration. Bluetooth control is currently disabled. */
#define ROBOT_BLUETOOTH_DRIVE_SPEED_RPM 100.0f
#define ROBOT_BLUETOOTH_CURRENT_LIMIT   3000

/* S1 departure calibration. Speeds are gearbox output-shaft rpm. Start areas
   1/4 turn left 45 degrees and areas 2/3 turn right 45 degrees. Change
   ROBOT_YAW_LEFT_SIGN after the first off-ground test if a physical left turn
   makes JY901S yaw decrease instead of increase. */
#define ROBOT_YAW_LEFT_SIGN                  1
/* Fixed-heading turns advance after2s even if not aligned; do not alter yaw.
   Full360 searches and timed straight motion keep their separate deadlines. */
#define ROBOT_HEADING_TURN_TIMEOUT_MS     2000U
#define ROBOT_S1_TURN_ANGLE_CDEG          4500L
#define ROBOT_S1_TURN_TOLERANCE_CDEG       400L
#define ROBOT_S1_TURN_SLOW_THRESHOLD_CDEG 1000L
#define ROBOT_S1_WRONG_DIRECTION_MARGIN_CDEG 500L
#define ROBOT_S1_TURN_FAST_RPM               35
#define ROBOT_S1_TURN_SLOW_RPM               18
#define ROBOT_S1_TURN_STABLE_MS             300U
#define ROBOT_S1_TURN_TIMEOUT_MS ROBOT_HEADING_TURN_TIMEOUT_MS
#define ROBOT_S1_CURRENT_LIMIT              3000
#define ROBOT_S1_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS

/* After the departure turn, drive straight across the speed bump before
   enabling vision control. The stop interval lets chassis vibration settle so
   the first camera coordinates do not immediately disturb the PID loops. */
#define ROBOT_S2_CROSS_BUMP_RPM               110
#define ROBOT_S2_CROSS_BUMP_DRIVE_MS         3000U
#define ROBOT_S2_CROSS_BUMP_SETTLE_MS           0U
#define ROBOT_S2_YAW_HOLD_TOLERANCE_CDEG      300L
#define ROBOT_S2_YAW_HOLD_KP                  0.025f
#define ROBOT_S2_YAW_HOLD_MAX_CORRECTION_RPM   35.0f
#define ROBOT_S2_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS

/* S3 green-target horizontal alignment. MaixCam sends signed pixel errors;
   zero is aligned and a positive x error means the target is to the right.
   Change the direction sign only if the first test turns away from it. */
#define ROBOT_VISION_X_TOLERANCE_PX           15
#define ROBOT_VISION_X_KP                    0.12f
#define ROBOT_VISION_X_KI                    0.00f
#define ROBOT_VISION_X_KD                    0.00f
#define ROBOT_VISION_X_INTEGRAL_LIMIT      500.0f
#define ROBOT_VISION_X_MIN_TURN_RPM            5.0f
#define ROBOT_VISION_X_MAX_TURN_RPM           35.0f
#define ROBOT_VISION_X_TURN_SIGN                1
#define ROBOT_VISION_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS
/* 5-6 fps vision: normal speed through 250 ms, fade to zero by 500 ms. */
#define ROBOT_VISION_COORD_DECEL_START_MS      250U
#define ROBOT_VISION_COORD_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS

/* If no fresh target error is received in command 03 or 04, drive forward
   once, then search with the JY901S: turn 90 degrees clockwise from the saved
   heading and 180 degrees counter-clockwise to the opposite side. Any target
   coordinate received during recovery immediately resumes vision PID. */
#define ROBOT_S3_GREEN_WAIT_MS                2000U
#define ROBOT_VISION_SEARCH_FORWARD_RPM          60
#define ROBOT_VISION_SEARCH_FORWARD_MS         500U
#define ROBOT_S3_SEARCH_TURN_ANGLE_CDEG       9000L
#define ROBOT_E3_SEARCH_REVERSE_RPM             80
#define ROBOT_E3_SEARCH_REVERSE_MS             800U
/* Second accepted E3 in an ordinary after07 task51/03 (not supplement):
   face away from the mapped safety heading, advance, then search CCW360. */
#define ROBOT_S51_SECOND_E3_FORWARD_RPM         100
#define ROBOT_S51_SECOND_E3_FORWARD_MS         1200U
/* First red E3 recovery after07: advance instead of reversing, then CCW360.
   Other03/04 and initial-green recovery paths retain their existing actions. */
#define ROBOT_S7_E3_SEARCH_FORWARD_RPM          80
#define ROBOT_S7_E3_SEARCH_FORWARD_MS          800U
#define ROBOT_E3_SEARCH_CCW_CDEG             36000L
#define ROBOT_E3_SEARCH_SPIN_TIMEOUT_MS       20000U
/* Initial green E3: drive forward, then sweep left/right and face the
   heading recorded when E3 arrived. Other E3 paths keep their own recovery. */
#define ROBOT_S3_GREEN_E3_FORWARD_RPM            80
#define ROBOT_S3_GREEN_E3_FORWARD_MS            800U
#define ROBOT_S3_GREEN_E3_TURN_CDEG            4500L
#define ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG    400L
#define ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG 1000L
#define ROBOT_S3_SEARCH_TURN_FAST_RPM            25
#define ROBOT_S3_SEARCH_TURN_SLOW_RPM            12
#define ROBOT_S3_SEARCH_TURN_STABLE_MS          200U
#define ROBOT_S3_SEARCH_TURN_TIMEOUT_MS ROBOT_HEADING_TURN_TIMEOUT_MS

/* S3 approach control. With the agreed image convention, a negative y error
   means the selected target is above the collection-frame reference point;
   the -1 sign converts that error into positive (forward) wheel speed. */
#define ROBOT_VISION_Y_TOLERANCE_PX           12
#define ROBOT_VISION_Y_STABLE_MS             300U
#define ROBOT_VISION_Y_KP                    0.40f
#define ROBOT_VISION_Y_KI                    0.00f
#define ROBOT_VISION_Y_KD                    0.00f
#define ROBOT_VISION_Y_INTEGRAL_LIMIT      500.0f
#define ROBOT_VISION_Y_MIN_FORWARD_RPM        12.0f
/* Far-target03 and supplement03 retain120..150. Near targets use the shared
   object-approach profile below instead of a fixed high minimum. */
#define ROBOT_VISION_03_MIN_FORWARD_RPM       120.0f
#define ROBOT_VISION_03_MAX_FORWARD_RPM       150.0f
#define ROBOT_VISION_03_WHEEL_MAX_RPM           150
#define ROBOT_VISION_04_MIN_FORWARD_RPM        70.0f
#define ROBOT_VISION_Y_MAX_FORWARD_RPM       130.0f
#define ROBOT_VISION_Y_FORWARD_SIGN             -1
#define ROBOT_VISION_TRACK_WHEEL_MAX_RPM       130
#define ROBOT_VISION_TRACK_TIMEOUT_MS        10000U
/* Object tracking only:03/04/24 and their supplement references, not06. */
#define ROBOT_VISION_OBJECT_NEAR_END_PX           80
#define ROBOT_VISION_OBJECT_FAR_BEGIN_PX         200
#define ROBOT_VISION_OBJECT_NEAR_MIN_RPM       20.0f
#define ROBOT_VISION_OBJECT_NEAR_MAX_RPM       40.0f
#define ROBOT_VISION_OBJECT_REVERSE_MAX_RPM    30.0f
#define ROBOT_VISION_OBJECT_REVERSE_FRAMES        2U
#define ROBOT_VISION_Y_JUMP_LOG_PX               120

/* Initial frame-lowering delay after MaixCam reports event 04. */
#define ROBOT_S3_FRAME_LOWER_SETTLE_MS            0U

/* S4 multi-object arrangement protocol: 04 -> 14/14 -> 02 -> 12/12 ->
   24 -> 34. MaixCam coordinates are signed errors to the reference point
   selected by its current mode. All timings are initial tuning values. */
#define ROBOT_S4_TRACK_FORWARD_MAX_RPM             70
#define ROBOT_S4_TRACK_TURN_MAX_RPM                35
#define ROBOT_S4_TRACK_WHEEL_MAX_RPM               90
#define ROBOT_S4_FINAL_X_KP                       0.09f
#define ROBOT_S4_FINAL_Y_KP                       0.30f
#define ROBOT_S4_FINAL_MIN_TURN_RPM                3.0f
#define ROBOT_S4_FINAL_MIN_FORWARD_RPM            50.0f
#define ROBOT_S4_FINAL_TURN_MAX_RPM                  25
#define ROBOT_S4_FINAL_FORWARD_MAX_RPM              120
#define ROBOT_S4_FINAL_WHEEL_MAX_RPM                120
#define ROBOT_S4_TRACK_TIMEOUT_MS                15000U
#define ROBOT_S4_HANDSHAKE_TIMEOUT_MS             5000U
#define ROBOT_S4_E4_FORWARD_RPM                     80
#define ROBOT_S4_E4_FORWARD_MS                     600U
#define ROBOT_S4_E4_FORWARD_WHEEL_MAX_RPM           100
#define ROBOT_S4_E4_LEFT_CDEG                     4500L
#define ROBOT_S4_E4_RIGHT_CDEG                    9000L
#define ROBOT_S4_E4_TURN_MAX_RPM                  20.0f
/* After07 black/green E4: first sweep max50rpm; later requests CW360. */
#define ROBOT_S4_E4_MATERIAL_TURN_MAX_RPM         50.0f
#define ROBOT_S4_E4_MATERIAL_SPIN_CDEG            36000L
#define ROBOT_S4_E4_MATERIAL_SPIN_TIMEOUT_MS      20000U
#define ROBOT_S4_E4_MATERIAL_SPIN_FAST_RPM           40
#define ROBOT_S4_E4_MATERIAL_SPIN_SLOW_RPM           12
#define ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES             2U
#define ROBOT_S4_E4_GREEN_SPIN_CDEG               36000L
#define ROBOT_S4_E4_GREEN_SPIN_TIMEOUT_MS         20000U
#define ROBOT_S4_E4_GREEN_SPIN_FAST_RPM              20
#define ROBOT_S4_E4_GREEN_SPIN_SLOW_RPM              12
#define ROBOT_S4_E4_RED_SPIN_CDEG                 36000L
#define ROBOT_S4_E4_RED_SPIN_TIMEOUT_MS            20000U
#define ROBOT_S4_E4_RED_SPIN_FAST_RPM                 20
#define ROBOT_S4_E4_RED_SPIN_SLOW_RPM                 12
#define ROBOT_S4_POST22_REVERSE_RPM                 70
#define ROBOT_S4_POST22_REVERSE_MS                 400U
#define ROBOT_S4_POST22_LEFT_CDEG                 4500L
#define ROBOT_S4_POST22_RIGHT_CDEG                9000L
/* After TX51/03, two unsuccessful short sweeps escalate to ordinary360
   search. Re-entering an individual reverse/turn must not reset this count. */
#define ROBOT_S4_POST22_MAX_SWEEPS                   2U
#define ROBOT_S4_E2_TURN_MAX_RPM                  20.0f
#define ROBOT_S4_RIGHT_CORNER_FOLLOW_MS            486U
#define ROBOT_S4_LEFT_CORNER_FOLLOW_MS             929U
/* FOLLOW_MS still describes the calibrated 70rpm/60rpm distance profile.
   Runtime durations are solved from its area, not stretched by higher speed. */
#define ROBOT_S4_CORNER_REFERENCE_RPM                70
#define ROBOT_S4_LEFT_CORNER_REFERENCE_INITIAL_RPM   60
#define ROBOT_S4_LEFT_CORNER_INITIAL_RPM            80
#define ROBOT_S4_LEFT_CORNER_INITIAL_MS            300U
#define ROBOT_S4_CORNER_PUSH_RPM                      90
/* Small curvature adjustment:02/right -3rpm,12/left +2rpm per segment.
   Center speeds, travel durations and smooth ramps remain unchanged. */
#define ROBOT_S4_CORNER_DIFF_EARLY_RPM               30
#define ROBOT_S4_CORNER_DIFF_MIDDLE_RPM              38
#define ROBOT_S4_CORNER_DIFF_LATE_RPM                45
/*12/left retains its flatter curve; reverse retraces the same profile. */
#define ROBOT_S4_LEFT_DIFF_EARLY_RPM                 13
#define ROBOT_S4_LEFT_DIFF_MIDDLE_RPM                14
#define ROBOT_S4_LEFT_DIFF_LATE_RPM                  16
#define ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN            0.25f
#define ROBOT_S4_CORNER_DIFF_MIDDLE_END              0.35f
#define ROBOT_S4_CORNER_DIFF_LATE_BEGIN              0.55f
#define ROBOT_S4_CORNER_DIFF_LATE_END                0.65f
#define ROBOT_S4_PUSH_RAMP_MS                       150U
#define ROBOT_S4_PUSH_DISTANCE_PERCENT              145U
/* Additional distance multiplier for02/right push, including E4/E5 reuse. */
#define ROBOT_S4_RIGHT_PUSH_DISTANCE_PERCENT        130U
/* Restore both02/12 forward and matching reverse travel to the length before
   the30% reduction, preserving differential profiles and smooth ramps. */
#define ROBOT_S4_CORNER_TRAVEL_PERCENT              100U
#define ROBOT_S4_LEFT_SPEED_BLEND_MS                150U
/* Backward paths replay the complete forward profiles in reverse order;
   durations and ramp times are derived from these cruise speeds. */
#define ROBOT_S4_RIGHT_REVERSE_RPM                 120
#define ROBOT_S4_LEFT_REVERSE_RPM                  120
#define ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT      80U
#define ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM        60
#define ROBOT_S4_ARRANGE_RECOVERY_REVERSE_MS        400U
#define ROBOT_S4_ARRANGE_RECOVERY_LEFT_CDEG         4500L
#define ROBOT_S4_ARRANGE_RECOVERY_RIGHT_CDEG        9000L
#define ROBOT_S4_FRAME_RAISE_SETTLE_MS              0U
/* 24 capture raises from DOWN by20deg, not to the full raised endpoint. */
#define ROBOT_S4_FINAL_FRAME_LIFT_DEG               20U
#define ROBOT_S4_FINAL_MISSING_TIMEOUT_MS         2000U
#define ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM          70
#define ROBOT_S4_FINAL_RECOVERY_REVERSE_MS          500U
/* Search both sides of the mapped safe heading; never use raw fixed yaw. */
#define ROBOT_S4_FINAL_RECOVERY_FIRST_SAFE_OFFSET_CDEG   9000L
#define ROBOT_S4_FINAL_RECOVERY_SECOND_SAFE_OFFSET_CDEG (-9000L)
#define ROBOT_S4_FINAL_RECOVERY_TURN_FAST_RPM          25
#define ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_RPM          12
#define ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_CDEG       1000L
#define ROBOT_S4_FINAL_RECOVERY_TURN_TIMEOUT_MS ROBOT_HEADING_TURN_TIMEOUT_MS
/* Original34 capture: no partial20deg lift or pre-capture backoff.
   BlackID6 advances78rpm/250ms before lowering; other colours unchanged. */
#define ROBOT_S4_FINAL_CENTER_RPM                    60
#define ROBOT_S4_FINAL_CENTER_MS                    350U
#define ROBOT_S4_BLACK_FINAL_CENTER_RPM              78
#define ROBOT_S4_BLACK_FINAL_CENTER_MS               250U
#define ROBOT_S4_RED_FINAL_CENTER_MS                500U
#define ROBOT_S4_RED_FRAME_LOWER_DELAY_MS            200U
#define ROBOT_S4_FRAME_LOWER_SETTLE_MS               300U
#define ROBOT_S4_POST_LOWER_REVERSE_RPM             100
#define ROBOT_S4_POST_LOWER_REVERSE_MS              280U
#define ROBOT_S4_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS

/* S5 waits for MaixCam event 06 after sending team-colour command 15/25. */
#define ROBOT_S5_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS
/* Nonblocking near-camera settling after the corner-return RX05 handshake. */
#define ROBOT_S6_RECHECK_CAMERA_SETTLE_MS         500U
/* Vision pauses output for800ms after recheck06 while the camera rises. */
#define ROBOT_S6_RECHECK_WIDE_SETTLE_MS           800U
/* Only the05 ->02 preparation turn; other shared yaw limits stay unchanged. */
#define ROBOT_S5_ARRANGE_TURN_MIN_RPM            25.0f
#define ROBOT_S5_ARRANGE_TURN_MAX_RPM            40.0f
/* Taper over20..10 degrees; below10 degrees allow8..15rpm corrections. */
#define ROBOT_S5_ARRANGE_TURN_SLOW_BEGIN_CDEG     2000L
#define ROBOT_S5_ARRANGE_TURN_SLOW_END_CDEG       1000L
#define ROBOT_S5_ARRANGE_TURN_SLOW_MIN_RPM         8.0f
#define ROBOT_S5_ARRANGE_TURN_SLOW_MAX_RPM        15.0f
/* Widen stop/release bands by2 degrees: stop within4, resume beyond6. */
#define ROBOT_S5_ARRANGE_TURN_RELEASE_CDEG         600L
#define ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS     0U
#define ROBOT_S5_ARRANGE_REVERSE_RPM               50
#define ROBOT_S5_ARRANGE_REVERSE_MS               700U
#define ROBOT_S5_ARRANGE_REVERSE_WHEEL_MAX_RPM      80
#define ROBOT_S5_ARRANGE_FRAME_LOWER_SETTLE_MS     300U

/* After event 06, turn by the shortest JY901S path toward the known field
   heading of the selected safe zone while waiting for MaixCam event 16. */
/* Soft warning only: keep searching/receiving16 after this interval. */
#define ROBOT_S6_SEARCH_TIMEOUT_MS              20000U
#define ROBOT_S6_DEBUG_PERIOD_MS ROBOT_DEBUG_PERIOD_MS
/* Faster initial safe-zone turn; keep the shared delivery/side-move limit. */
#define ROBOT_S6_SEARCH_TURN_MAX_RPM                 85
/* Once inside4 degrees, hold still until drift exceeds6 degrees. */
#define ROBOT_S6_SEARCH_HOLD_TOLERANCE_CDEG         600L
/* No post-turn wait: if 16 is absent, begin recovery upon reaching the heading. */
#define ROBOT_S6_SEARCH_WAIT16_MS                    0U

/* S6 tracking recovery after 1000 ms without valid coordinates.
   Initial search uses ROBOT_S6_SEARCH_WAIT16_MS while waiting for event 16.
   First attempt sweeps about the safe heading without driving forward;
   the second and later attempts use the former long-forward sweep. */
#define ROBOT_S6_RECOVERY_WAIT_MS                  1000U
#define ROBOT_S6_RECOVERY_LONG_START_COUNT             2U
#define ROBOT_S6_RECOVERY_LONG_FORWARD_RPM             70
#define ROBOT_S6_RECOVERY_LONG_FORWARD_MS            2000U
#define ROBOT_S6_RECOVERY_LONG_WHEEL_MAX_RPM           80
/* Complete the long forward segment before reacquiring from coordinates/16.
   UART parsing and explicit05/26/36 commands or safety stops remain active. */
/* Moving yaw hold must not inherit the 30 rpm minimum for stationary turns.
   +/-10 rpm keeps a 70 rpm cruise within the existing 80 rpm wheel cap. */
#define ROBOT_S6_RECOVERY_FORWARD_MAX_CORRECTION_RPM 10.0f
#define ROBOT_S6_RECOVERY_LEFT_ANGLE_CDEG           4500L
#define ROBOT_S6_RECOVERY_RIGHT_ANGLE_CDEG          9000L
/* Recovery scans and safe-heading turns have independent speed limits. */
#define ROBOT_S6_RECOVERY_SCAN_MAX_RPM             60.0f
#define ROBOT_S6_RECOVERY_SCAN_MIN_RPM             40.0f
#define ROBOT_S6_RECOVERY_SAFE_MAX_RPM             50.0f
#define ROBOT_S6_RECOVERY_SAFE_MIN_RPM             30.0f
#define ROBOT_S6_RECOVERY_TURN_STABLE_MS              0U
/* Advance to the next action after1s instead of remaining in the turn. */
#define ROBOT_S6_TURN_TIMEOUT_MS ROBOT_HEADING_TURN_TIMEOUT_MS
#define ROBOT_S6_TURN_TOLERANCE_CDEG                400L
#define ROBOT_S6_TURN_STABLE_MS                     200U

/* JY901S yaw inner loop shared by all S6 turns and straight segments.
   Straight yaw deadband widened from1 to3 degrees; turn arrival is4 above. */
#define ROBOT_S6_YAW_TOLERANCE_CDEG               300L
#define ROBOT_S6_YAW_FORWARD_ENABLE_CDEG          1000L
#define ROBOT_S6_YAW_KP                          0.012f
#define ROBOT_S6_YAW_KI                          0.000f
#define ROBOT_S6_YAW_KD                          0.000f
#define ROBOT_S6_YAW_INTEGRAL_LIMIT            5000.0f
#define ROBOT_S6_YAW_MIN_TURN_RPM                 10.0f
#define ROBOT_S6_YAW_MAX_TURN_RPM                 30.0f

/* After event 16, use image XY error to approach the safe-zone point until
   MaixCam reports event 26 (safe-zone area >= 50%). Turn to the field-mapped
   safe-zone heading, then perform the timed yaw-held push without vision X. */
/* Both supplies and casualties use a reduced Y limit until X is confirmed.
   Count distinct consecutive coordinate frames, not repeated task ticks. */
#define ROBOT_S6_X_ALIGN_MIN_FRAMES                   3U
#define ROBOT_S6_X_ALIGN_STABLE_MS                  300U
#define ROBOT_S6_X_ALIGN_MAX_GAP_MS                 500U
/* Accumulate valid visual tracking before any approach side-angle decision.
   Each acquisition, including every side-return, starts a new interval.
   Side decisions use yaw sectors only; X error is not an entry condition. */
#define ROBOT_S6_PRE_REPOSITION_TRACK_MS             500U
/* Stable X within this range enables the full Y approach speed;
   unaligned tracking uses ROBOT_S6_APPROACH_UNALIGNED_RPM instead.
   This threshold does not gate side reposition. */
#define ROBOT_S6_SIDE_X_TOLERANCE_PX                 30
#define ROBOT_S6_REPOSITION_MIN_OFFSET_CDEG         2000L
#define ROBOT_S6_REPOSITION_SECTOR_CDEG            9000L
/* Side-reposition calibration entry points; a target may have multiple sectors.
   Fields: zone, team, kind (0=supply, 1=casualty), enabled,
   sector start, sector end, side heading, include start, include end.
   Angles are centidegrees relative to the yaw latched at PE12 start (not the
   45-degree departure heading). Left turns increase yaw; right turns decrease
   yaw. A clockwise-positive angle A maps to (360 - A) modulo 360 here.
   Sectors may cross 0 degrees.
   Casualty sectors are safe_heading - 90 through safe_heading - 15,
   inclusive: 90..165 for safe=180; 270..345 for safe=0. Other zone/team
   casualty rows are symmetry-derived from zone 3 / blue, not raw-yaw rules.
   Shared corner evacuation below takes priority in overlapping casualty
   sectors: effective ordinary casualty side motion is [135,165] for safe=180
   and [315,345] for safe=0.
   Material side sectors: [135,157] and (190,224] for safe=180;
   [315,337] and (10,44] for safe=0. Adjacent corner sectors are below.
   Material rows outside zone 3 / blue are geometry-derived and need field checks. */
#define ROBOT_S6_SIDE_RULE_ROWS \
  {1U, ROBOT_TEAM_RED,  0U, 1U, 13500L, 15700L,  9000L, 1U, 1U}, \
  {1U, ROBOT_TEAM_RED,  0U, 1U, 19000L, 22400L, 27000L, 0U, 1U}, \
  {1U, ROBOT_TEAM_RED,  1U, 1U,  9000L, 16500L,  9000L, 1U, 1U}, \
  {1U, ROBOT_TEAM_BLUE, 0U, 1U,  1000L,  4400L,  9000L, 0U, 1U}, \
  {1U, ROBOT_TEAM_BLUE, 0U, 1U, 31500L, 33700L, 27000L, 1U, 1U}, \
  {1U, ROBOT_TEAM_BLUE, 1U, 1U, 27000L, 34500L, 27000L, 1U, 1U}, \
  {2U, ROBOT_TEAM_RED,  0U, 1U, 13500L, 15700L,  9000L, 1U, 1U}, \
  {2U, ROBOT_TEAM_RED,  0U, 1U, 19000L, 22400L, 27000L, 0U, 1U}, \
  {2U, ROBOT_TEAM_RED,  1U, 1U,  9000L, 16500L,  9000L, 1U, 1U}, \
  {2U, ROBOT_TEAM_BLUE, 0U, 1U,  1000L,  4400L,  9000L, 0U, 1U}, \
  {2U, ROBOT_TEAM_BLUE, 0U, 1U, 31500L, 33700L, 27000L, 1U, 1U}, \
  {2U, ROBOT_TEAM_BLUE, 1U, 1U, 27000L, 34500L, 27000L, 1U, 1U}, \
  {3U, ROBOT_TEAM_RED,  0U, 1U,  1000L,  4400L,  9000L, 0U, 1U}, \
  {3U, ROBOT_TEAM_RED,  0U, 1U, 31500L, 33700L, 27000L, 1U, 1U}, \
  {3U, ROBOT_TEAM_RED,  1U, 1U, 27000L, 34500L, 27000L, 1U, 1U}, \
  {3U, ROBOT_TEAM_BLUE, 0U, 1U, 13500L, 15700L,  9000L, 1U, 1U}, \
  {3U, ROBOT_TEAM_BLUE, 0U, 1U, 19000L, 22400L, 27000L, 0U, 1U}, \
  {3U, ROBOT_TEAM_BLUE, 1U, 1U,  9000L, 16500L,  9000L, 1U, 1U}, \
  {4U, ROBOT_TEAM_RED,  0U, 1U,  1000L,  4400L,  9000L, 0U, 1U}, \
  {4U, ROBOT_TEAM_RED,  0U, 1U, 31500L, 33700L, 27000L, 1U, 1U}, \
  {4U, ROBOT_TEAM_RED,  1U, 1U, 27000L, 34500L, 27000L, 1U, 1U}, \
  {4U, ROBOT_TEAM_BLUE, 0U, 1U, 13500L, 15700L,  9000L, 1U, 1U}, \
  {4U, ROBOT_TEAM_BLUE, 0U, 1U, 19000L, 22400L, 27000L, 0U, 1U}, \
  {4U, ROBOT_TEAM_BLUE, 1U, 1U,  9000L, 16500L,  9000L, 1U, 1U}
/* Supplies and casualties share corner evacuation, start-zero mapping and
   the 500ms visual tracking gate. Corner decisions precede ordinary side motion.
   Shared boundaries are excluded from evacuation:
   safe=180: [90,135) ->90; (224,270] ->270.
   safe=0: [270,315) ->270; (44,90] ->90.
   target_kind is unused in this shared table. No new trigger command;
   the existing post-evacuation05 load-review handshake is unchanged. */
#define ROBOT_S6_CORNER_RULE_ROWS \
  {1U, ROBOT_TEAM_RED,  0U, 1U,  9000L, 13500L,  9000L, 1U, 0U}, \
  {1U, ROBOT_TEAM_RED,  0U, 1U, 22400L, 27000L, 27000L, 0U, 1U}, \
  {1U, ROBOT_TEAM_BLUE, 0U, 1U, 27000L, 31500L, 27000L, 1U, 0U}, \
  {1U, ROBOT_TEAM_BLUE, 0U, 1U,  4400L,  9000L,  9000L, 0U, 1U}, \
  {2U, ROBOT_TEAM_RED,  0U, 1U,  9000L, 13500L,  9000L, 1U, 0U}, \
  {2U, ROBOT_TEAM_RED,  0U, 1U, 22400L, 27000L, 27000L, 0U, 1U}, \
  {2U, ROBOT_TEAM_BLUE, 0U, 1U, 27000L, 31500L, 27000L, 1U, 0U}, \
  {2U, ROBOT_TEAM_BLUE, 0U, 1U,  4400L,  9000L,  9000L, 0U, 1U}, \
  {3U, ROBOT_TEAM_RED,  0U, 1U, 27000L, 31500L, 27000L, 1U, 0U}, \
  {3U, ROBOT_TEAM_RED,  0U, 1U,  4400L,  9000L,  9000L, 0U, 1U}, \
  {3U, ROBOT_TEAM_BLUE, 0U, 1U,  9000L, 13500L,  9000L, 1U, 0U}, \
  {3U, ROBOT_TEAM_BLUE, 0U, 1U, 22400L, 27000L, 27000L, 0U, 1U}, \
  {4U, ROBOT_TEAM_RED,  0U, 1U, 27000L, 31500L, 27000L, 1U, 0U}, \
  {4U, ROBOT_TEAM_RED,  0U, 1U,  4400L,  9000L,  9000L, 0U, 1U}, \
  {4U, ROBOT_TEAM_BLUE, 0U, 1U,  9000L, 13500L,  9000L, 1U, 0U}, \
  {4U, ROBOT_TEAM_BLUE, 0U, 1U, 22400L, 27000L, 27000L, 0U, 1U}
#define ROBOT_S6_CORNER_FORWARD_RPM                 100
#define ROBOT_S6_CORNER_AWAY_FORWARD_MS             2000U
#define ROBOT_S6_CORNER_SIDE_FORWARD_MS             1200U
#define ROBOT_S6_CORNER_WHEEL_MAX_RPM                110
/* Ordinary side travel varies linearly from1x near safe to3x at the far edge.
   Reference nominal distance is100rpm *490ms; actual time compensates120rpm.
   Corner evacuation above retains its independent100rpm/2000ms/1200ms. */
#define ROBOT_S6_REPOSITION_TURN_MAX_RPM             55.0f
#define ROBOT_S6_REPOSITION_FORWARD_RPM              120
#define ROBOT_S6_REPOSITION_REFERENCE_RPM            100
#define ROBOT_S6_REPOSITION_FORWARD_MS               490U
#define ROBOT_S6_REPOSITION_DISTANCE_MIN_SCALE       1.0f
#define ROBOT_S6_REPOSITION_DISTANCE_MAX_SCALE       3.0f
#define ROBOT_S6_REPOSITION_WHEEL_MAX_RPM             130
#define ROBOT_S6_APPROACH_UNALIGNED_RPM              30
#define ROBOT_S6_APPROACH_RPM                      150
/* The unaligned cap and stale-coordinate fading take priority over this floor. */
#define ROBOT_S6_APPROACH_MIN_FORWARD_RPM         100.0f
#define ROBOT_S6_APPROACH_TURN_MAX_RPM              35
#define ROBOT_S6_APPROACH_WHEEL_MAX_RPM            150
#define ROBOT_S6_APPROACH_TIMEOUT_MS              15000U
#define ROBOT_S6_TRACK_REVERSE_INTERVAL_MS         8000U
#define ROBOT_S6_TRACK_REVERSE_RPM                  100
#define ROBOT_S6_TRACK_REVERSE_MS                   700U
/* Event36: supplies turn left, casualties right; mirror the counter-turn,
   then face the absolute safe heading. Use the cargo class locked at06. */
#define ROBOT_S6_OBSTACLE_TURN_MAX_RPM             85.0f
/* Raising the cap alone would not speed up45deg legs with the existing KP. */
#define ROBOT_S6_OBSTACLE_TURN_GAIN_SCALE       (85.0f / 55.0f)
#define ROBOT_S6_OBSTACLE_TURN_STABLE_MS              0U
#define ROBOT_S6_OBSTACLE_LEFT_ANGLE_CDEG          4500L
/* Preserve nominal travel:120rpm *417ms ~=140rpm *357ms. */
#define ROBOT_S6_OBSTACLE_DRIVE_RPM                 140
#define ROBOT_S6_OBSTACLE_FORWARD_MS               357U
#define ROBOT_S6_OBSTACLE_RIGHT_ANGLE_CDEG        4500L
#define ROBOT_S6_FRAME_RAISE_SETTLE_MS              300U
#define ROBOT_S6_PRE_PUSH_REVERSE_RPM               80
#define ROBOT_S6_PRE_PUSH_REVERSE_WHEEL_MAX_RPM     90
#define ROBOT_S6_PRE_PUSH_REVERSE_MS               500U
#define ROBOT_S6_PRE_PUSH_BRAKE_MS                 200U
#define ROBOT_S6_PRE_PUSH_TIMEOUT_MS              1500U
/* After 26: locate against the border with the frame down, back off, raise
   only 30 degrees, then deliver. Motion distances are timed, not sensed. */
#define ROBOT_S6_BORDER_LOCATE_RPM                  40
#define ROBOT_S6_BORDER_LOCATE_WHEEL_MAX_RPM       100
#define ROBOT_S6_BORDER_LOCATE_MS                  1000U
#define ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS          3000U
#define ROBOT_S6_BORDER_BACKOFF_RPM                 40
#define ROBOT_S6_BORDER_BACKOFF_WHEEL_MAX_RPM      100
#define ROBOT_S6_BORDER_BACKOFF_MS                  300U
#define ROBOT_S6_BORDER_BACKOFF_TIMEOUT_MS         1500U
#define ROBOT_S6_PARTIAL_FRAME_RAISE_DEG             30U
#define ROBOT_S6_PARTIAL_FRAME_SETTLE_MS            300U
#define ROBOT_S6_FINAL_PUSH_RPM                     40
#define ROBOT_S6_FINAL_PUSH_WHEEL_MAX_RPM          100
#define ROBOT_S6_FINAL_PUSH_MS                      750U
#define ROBOT_S6_FINAL_PUSH_TIMEOUT_MS             3000U
#define ROBOT_S6_REVERSE_RPM                       100
#define ROBOT_S6_REVERSE_WHEEL_MAX_RPM             110
#define ROBOT_S6_REVERSE_BRAKE_MS                    0U
#define ROBOT_S6_REVERSE_DRIVE_MS                  300U
#define ROBOT_S6_REVERSE_TIMEOUT_MS               3000U
#define ROBOT_S6_EXIT_TURN_ANGLE_CDEG            18000L
#define ROBOT_S6_EXIT_TURN_MAX_RPM                  50
#define ROBOT_S6_EXIT_TURN_TIMEOUT_MS ROBOT_HEADING_TURN_TIMEOUT_MS

/* Mirrored collection-frame servo endpoints (0..280 degree command scale). */
#define ROBOT_LEFT_FRAME_DOWN_DEG       204U
#define ROBOT_LEFT_FRAME_UP_DEG         116U
#define ROBOT_RIGHT_FRAME_DOWN_DEG      23U
#define ROBOT_RIGHT_FRAME_UP_DEG        103U
#define ROBOT_FRAME_SERVO_MAX_ANGLE_DEG 280U
#define ROBOT_FRAME_SERVO_MIN_PULSE_US  500U
#define ROBOT_FRAME_SERVO_MAX_PULSE_US 2500U

/* PA5/TIM2 CH1 MG90 camera-servo calibration. The tested 88-degree position
   gives the wide search view; 53 degrees is the calibrated near view. */
#define ROBOT_CAMERA_WIDE_ANGLE_DEG       88U
#define ROBOT_CAMERA_NEAR_ANGLE_DEG       53U
#define ROBOT_CAMERA_POWERUP_SETTLE_MS    500U
/* After07 task51 only. Budget includes view changes, handshakes and recovery. */
#define ROBOT_SUPPLEMENT_MAX_CAPTURES       2U
#define ROBOT_SUPPLEMENT_TOTAL_MS       15000U
#define ROBOT_SUPPLEMENT_WIDE_SETTLE_MS    500U
#define ROBOT_SUPPLEMENT_LOST_MS         1000U
#define ROBOT_SUPPLEMENT_TURN_MAX_RPM    20.0f
/* Once04/24 arrives, reuse the ordinary S4 capture/servo parameters above. */
#define ROBOT_CAMERA_SERVO_MIN_PULSE_US  500U
#define ROBOT_CAMERA_SERVO_MAX_PULSE_US 2500U

#if (ROBOT_YAW_LEFT_SIGN != 1) && (ROBOT_YAW_LEFT_SIGN != -1)
#error "ROBOT_YAW_LEFT_SIGN must be 1 or -1"
#endif

#if ROBOT_S5_ARRANGE_TURN_SLOW_BEGIN_CDEG <= ROBOT_S5_ARRANGE_TURN_SLOW_END_CDEG
#error "Arrange turn slow-begin angle must exceed slow-end angle"
#endif

#if ROBOT_S5_ARRANGE_TURN_RELEASE_CDEG < ROBOT_S6_TURN_TOLERANCE_CDEG
#error "Arrange turn release band must not be narrower than its stop band"
#endif

#if (ROBOT_VISION_X_TURN_SIGN != 1) && (ROBOT_VISION_X_TURN_SIGN != -1)
#error "ROBOT_VISION_X_TURN_SIGN must be 1 or -1"
#endif

#if (ROBOT_VISION_Y_FORWARD_SIGN != 1) && \
    (ROBOT_VISION_Y_FORWARD_SIGN != -1)
#error "ROBOT_VISION_Y_FORWARD_SIGN must be 1 or -1"
#endif

#if (ROBOT_PE13_LOW_TEAM == ROBOT_PE13_HIGH_TEAM) || \
    ((ROBOT_PE13_LOW_TEAM != ROBOT_TEAM_RED) && \
     (ROBOT_PE13_LOW_TEAM != ROBOT_TEAM_BLUE)) || \
    ((ROBOT_PE13_HIGH_TEAM != ROBOT_TEAM_RED) && \
     (ROBOT_PE13_HIGH_TEAM != ROBOT_TEAM_BLUE))
#error "PE13 low/high team mapping must contain one red and one blue"
#endif

#if (ROBOT_START_ZONE_BITS_00 < 1U) || \
    (ROBOT_START_ZONE_BITS_00 > 4U) || \
    (ROBOT_START_ZONE_BITS_01 < 1U) || \
    (ROBOT_START_ZONE_BITS_01 > 4U) || \
    (ROBOT_START_ZONE_BITS_10 < 1U) || \
    (ROBOT_START_ZONE_BITS_10 > 4U) || \
    (ROBOT_START_ZONE_BITS_11 < 1U) || \
    (ROBOT_START_ZONE_BITS_11 > 4U)
#error "Start-zone selector mappings must be in the range 1..4"
#endif

#if (ROBOT_LEFT_WHEEL_MOTOR_ID < 1U) || \
    (ROBOT_LEFT_WHEEL_MOTOR_ID > 4U) || \
    (ROBOT_RIGHT_WHEEL_MOTOR_ID < 1U) || \
    (ROBOT_RIGHT_WHEEL_MOTOR_ID > 4U) || \
    (ROBOT_LEFT_WHEEL_MOTOR_ID == ROBOT_RIGHT_WHEEL_MOTOR_ID)
#error "Invalid M2006 wheel motor IDs"
#endif

#endif /* ROBOT_CONFIG_H */
