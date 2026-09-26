#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* M2006 CAN IDs configured on the two wheel ESCs. */
#define ROBOT_LEFT_WHEEL_MOTOR_ID       2U
#define ROBOT_RIGHT_WHEEL_MOTOR_ID      4U

/* Mirrored wheel installation: change either sign after an off-ground test if
   the corresponding wheel turns opposite to the intended vehicle direction. */
#define ROBOT_LEFT_WHEEL_FORWARD_SIGN   1.0f
#define ROBOT_RIGHT_WHEEL_FORWARD_SIGN (-1.0f)

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
#define ROBOT_S1_TURN_ANGLE_CDEG          4500L
#define ROBOT_S1_TURN_TOLERANCE_CDEG       200L
#define ROBOT_S1_TURN_SLOW_THRESHOLD_CDEG 1000L
#define ROBOT_S1_WRONG_DIRECTION_MARGIN_CDEG 500L
#define ROBOT_S1_TURN_FAST_RPM               35
#define ROBOT_S1_TURN_SLOW_RPM               18
#define ROBOT_S1_TURN_STABLE_MS             300U
#define ROBOT_S1_TURN_TIMEOUT_MS           4000U
#define ROBOT_S1_CURRENT_LIMIT              3000
#define ROBOT_S1_DEBUG_PERIOD_MS             100U

/* After the departure turn, drive straight across the speed bump before
   enabling vision control. The stop interval lets chassis vibration settle so
   the first camera coordinates do not immediately disturb the PID loops. */
#define ROBOT_S2_CROSS_BUMP_RPM               110
#define ROBOT_S2_CROSS_BUMP_DRIVE_MS         3000U
#define ROBOT_S2_CROSS_BUMP_SETTLE_MS         300U
#define ROBOT_S2_YAW_HOLD_TOLERANCE_CDEG      100L
#define ROBOT_S2_YAW_HOLD_KP                  0.025f
#define ROBOT_S2_YAW_HOLD_MAX_CORRECTION_RPM   35.0f
#define ROBOT_S2_DEBUG_PERIOD_MS               100U

/* S3 green-target horizontal alignment. MaixCam sends signed pixel errors;
   zero is aligned and a positive x error means the target is to the right.
   Change the direction sign only if the first test turns away from it. */
#define ROBOT_VISION_X_TOLERANCE_PX           15
#define ROBOT_VISION_X_STABLE_MS             300U
#define ROBOT_VISION_X_KP                    0.12f
#define ROBOT_VISION_X_KI                    0.00f
#define ROBOT_VISION_X_KD                    0.00f
#define ROBOT_VISION_X_INTEGRAL_LIMIT      500.0f
#define ROBOT_VISION_X_MIN_TURN_RPM            5.0f
#define ROBOT_VISION_X_MAX_TURN_RPM           35.0f
#define ROBOT_VISION_X_TURN_SIGN                1
#define ROBOT_VISION_DEBUG_PERIOD_MS          100U

/* If no fresh target error is received in command 03 or 04, drive forward
   once, then search with the JY901S: turn 90 degrees clockwise from the saved
   heading and 180 degrees counter-clockwise to the opposite side. Any target
   coordinate received during recovery immediately resumes vision PID. */
#define ROBOT_S3_GREEN_WAIT_MS                2000U
#define ROBOT_VISION_SEARCH_FORWARD_RPM          60
#define ROBOT_VISION_SEARCH_FORWARD_MS         500U
#define ROBOT_S3_SEARCH_TURN_ANGLE_CDEG       9000L
#define ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG    200L
#define ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG 1000L
#define ROBOT_S3_SEARCH_TURN_FAST_RPM            25
#define ROBOT_S3_SEARCH_TURN_SLOW_RPM            12
#define ROBOT_S3_SEARCH_TURN_STABLE_MS          200U
#define ROBOT_S3_SEARCH_TURN_TIMEOUT_MS        6000U

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
#define ROBOT_VISION_Y_MAX_FORWARD_RPM       110.0f
#define ROBOT_VISION_Y_FORWARD_SIGN             -1
#define ROBOT_VISION_TRACK_WHEEL_MAX_RPM       110
#define ROBOT_VISION_TRACK_TIMEOUT_MS        10000U

/* Initial frame-lowering delay after MaixCam reports event 04. */
#define ROBOT_S3_FRAME_LOWER_SETTLE_MS         1000U

/* S4 multi-object arrangement protocol: 04 -> 14/14 -> 02 -> 12/12 ->
   24 -> 34. MaixCam coordinates are signed errors to the reference point
   selected by its current mode. All timings are initial tuning values. */
#define ROBOT_S4_TRACK_FORWARD_MAX_RPM             70
#define ROBOT_S4_TRACK_TURN_MAX_RPM                35
#define ROBOT_S4_TRACK_WHEEL_MAX_RPM               90
#define ROBOT_S4_TRACK_TIMEOUT_MS                15000U
#define ROBOT_S4_HANDSHAKE_TIMEOUT_MS             5000U
#define ROBOT_S4_CENTER_FOLLOW_RPM                   60
#define ROBOT_S4_CENTER_FOLLOW_TURN_MAX_RPM          20
#define ROBOT_S4_CENTER_FOLLOW_MS                 1000U
#define ROBOT_S4_RIGHT_CORNER_FOLLOW_MS            800U
#define ROBOT_S4_LEFT_CORNER_FOLLOW_MS            1050U
#define ROBOT_S4_CORNER_PUSH_RPM                      80
/* Fixed wheel differences for the two timed corner pushes. */
#define ROBOT_S4_RIGHT_CORNER_WHEEL_DIFF_RPM          21
#define ROBOT_S4_LEFT_CORNER_WHEEL_DIFF_RPM           19
#define ROBOT_S4_RIGHT_REVERSE_RPM                  80
#define ROBOT_S4_RIGHT_REVERSE_MS                  900U
#define ROBOT_S4_LEFT_REVERSE_RPM                   80
#define ROBOT_S4_LEFT_REVERSE_MS                   900U
#define ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM        60
#define ROBOT_S4_ARRANGE_RECOVERY_REVERSE_MS        500U
#define ROBOT_S4_ARRANGE_RECOVERY_LEFT_CDEG         4500L
#define ROBOT_S4_ARRANGE_RECOVERY_RIGHT_CDEG        9000L
#define ROBOT_S4_FRAME_RAISE_SETTLE_MS           1000U
#define ROBOT_S4_FINAL_MISSING_TIMEOUT_MS         2000U
#define ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM          50
#define ROBOT_S4_FINAL_RECOVERY_REVERSE_MS          700U
#define ROBOT_S4_FINAL_RECOVERY_TURN_270_CDEG      27000L
#define ROBOT_S4_FINAL_RECOVERY_TURN_90_CDEG        9000L
#define ROBOT_S4_FINAL_RECOVERY_TURN_FAST_RPM          25
#define ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_RPM          12
#define ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_CDEG       1000L
#define ROBOT_S4_FINAL_RECOVERY_TURN_TIMEOUT_MS     10000U
#define ROBOT_S4_FINAL_CENTER_RPM                    60
#define ROBOT_S4_FINAL_CENTER_MS                    150U
#define ROBOT_S4_FRAME_LOWER_SETTLE_MS           1000U
#define ROBOT_S4_POST_LOWER_REVERSE_RPM              70
#define ROBOT_S4_POST_LOWER_REVERSE_MS              400U
#define ROBOT_S4_DEBUG_PERIOD_MS                   100U

/* S5 waits for MaixCam event 06 after sending team-colour command 15/25. */
#define ROBOT_S5_DEBUG_PERIOD_MS                 500U
#define ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS  1000U
#define ROBOT_S5_ARRANGE_REVERSE_RPM               50
#define ROBOT_S5_ARRANGE_REVERSE_MS               700U
#define ROBOT_S5_ARRANGE_REVERSE_WHEEL_MAX_RPM      80
#define ROBOT_S5_ARRANGE_FRAME_LOWER_SETTLE_MS    1000U

/* After event 06, turn by the shortest JY901S path toward the known field
   heading of the selected safe zone while waiting for MaixCam event 16. */
#define ROBOT_S6_SEARCH_TIMEOUT_MS              20000U
#define ROBOT_S6_DEBUG_PERIOD_MS                  100U

/* S6 visual recovery. Once pointed toward the safe zone, two seconds without
   event 16 (or without fresh coordinates during tracking) starts this move. */
#define ROBOT_S6_RECOVERY_WAIT_MS                   700U
#define ROBOT_S6_RECOVERY_FORWARD_RPM                40
#define ROBOT_S6_RECOVERY_FORWARD_MS                500U
#define ROBOT_S6_RECOVERY_WHEEL_MAX_RPM              60
#define ROBOT_S6_RECOVERY_LONG_START_COUNT             3U
#define ROBOT_S6_RECOVERY_LONG_FORWARD_RPM             70
#define ROBOT_S6_RECOVERY_LONG_FORWARD_MS            2000U
#define ROBOT_S6_RECOVERY_LONG_WHEEL_MAX_RPM           80
#define ROBOT_S6_RECOVERY_LEFT_ANGLE_CDEG           4500L
#define ROBOT_S6_RECOVERY_RIGHT_ANGLE_CDEG          9000L
#define ROBOT_S6_TURN_TIMEOUT_MS                   6000U
#define ROBOT_S6_TURN_TOLERANCE_CDEG                200L
#define ROBOT_S6_TURN_STABLE_MS                     200U

/* JY901S yaw inner loop shared by all S6 turns and straight segments. */
#define ROBOT_S6_YAW_TOLERANCE_CDEG               100L
#define ROBOT_S6_YAW_FORWARD_ENABLE_CDEG          1000L
#define ROBOT_S6_YAW_KP                          0.012f
#define ROBOT_S6_YAW_KI                          0.000f
#define ROBOT_S6_YAW_KD                          0.000f
#define ROBOT_S6_YAW_INTEGRAL_LIMIT            5000.0f
#define ROBOT_S6_YAW_MIN_TURN_RPM                 10.0f
#define ROBOT_S6_YAW_MAX_TURN_RPM                 30.0f

/* After event 16, use image XY error to approach the safe-zone point until
   MaixCam reports event 26 (safe-zone area >= 50%). Then turn to the field-
   mapped safe-zone normal heading and perform a short yaw-held final push. */
#define ROBOT_S6_PRE_REPOSITION_TRACK_MS           1000U
#define ROBOT_S6_REPOSITION_MIN_OFFSET_CDEG         2000L
#define ROBOT_S6_REPOSITION_SECTOR_CDEG            9000L
#define ROBOT_S6_REPOSITION_FORWARD_RPM               50
#define ROBOT_S6_REPOSITION_FORWARD_MS               700U
#define ROBOT_S6_REPOSITION_WHEEL_MAX_RPM              80
#define ROBOT_S6_REPOSITION_X_STABLE_MS               300U
#define ROBOT_S6_APPROACH_RPM                       70
#define ROBOT_S6_APPROACH_TURN_MAX_RPM              35
#define ROBOT_S6_APPROACH_WHEEL_MAX_RPM             90
#define ROBOT_S6_APPROACH_TIMEOUT_MS              15000U
#define ROBOT_S6_FRAME_RAISE_SETTLE_MS             1000U
#define ROBOT_S6_PRE_PUSH_REVERSE_RPM               80
#define ROBOT_S6_PRE_PUSH_REVERSE_WHEEL_MAX_RPM     90
#define ROBOT_S6_PRE_PUSH_REVERSE_MS               500U
#define ROBOT_S6_PRE_PUSH_BRAKE_MS                 200U
#define ROBOT_S6_PRE_PUSH_TIMEOUT_MS              1500U
#define ROBOT_S6_FINAL_PUSH_RPM                     70
#define ROBOT_S6_FINAL_PUSH_WHEEL_MAX_RPM           80
#define ROBOT_S6_FINAL_PUSH_MS                     1300U
#define ROBOT_S6_FINAL_PUSH_TIMEOUT_MS             3000U
#define ROBOT_S6_REVERSE_RPM                        70
#define ROBOT_S6_REVERSE_WHEEL_MAX_RPM              80
#define ROBOT_S6_REVERSE_BRAKE_MS                  300U
#define ROBOT_S6_REVERSE_DRIVE_MS                 2000U
#define ROBOT_S6_REVERSE_TIMEOUT_MS               3000U
#define ROBOT_S6_EXIT_TURN_ANGLE_CDEG            18000L

/* Mirrored collection-frame servo endpoints (0..280 degree command scale). */
#define ROBOT_LEFT_FRAME_DOWN_DEG       222U
#define ROBOT_LEFT_FRAME_UP_DEG         130U
#define ROBOT_RIGHT_FRAME_DOWN_DEG      2U
#define ROBOT_RIGHT_FRAME_UP_DEG        90U
#define ROBOT_FRAME_SERVO_MAX_ANGLE_DEG 280U
#define ROBOT_FRAME_SERVO_MIN_PULSE_US  500U
#define ROBOT_FRAME_SERVO_MAX_PULSE_US 2500U

/* PA5/TIM2 CH1 MG90 camera-servo calibration. The tested 80-degree position
   gives the wide search view; 45 degrees sees the complete collection frame. */
#define ROBOT_CAMERA_WIDE_ANGLE_DEG       80U
#define ROBOT_CAMERA_NEAR_ANGLE_DEG       45U
#define ROBOT_CAMERA_SERVO_MIN_PULSE_US  500U
#define ROBOT_CAMERA_SERVO_MAX_PULSE_US 2500U

#if (ROBOT_YAW_LEFT_SIGN != 1) && (ROBOT_YAW_LEFT_SIGN != -1)
#error "ROBOT_YAW_LEFT_SIGN must be 1 or -1"
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
