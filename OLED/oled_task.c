#include "oled_task.h"

#include "cmsis_os.h"
#include "main.h"
#include "mission_task.h"
#include "motor_test_task.h"
#include "reset_reason.h"
#include "robot_config.h"
#include "servo_test_task.h"

#include <stdio.h>

#define OLED_TASK_PERIOD_TICKS 200U

static uint8_t oled_ready;

static void OLED_TaskShowStartButton(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);

  (void)OLED_WriteString(0U, 0U,
      pin_state == GPIO_PIN_SET ? "PE12:H READY    " : "PE12:L PRESSED  ");
}

static void OLED_TaskShowMission(void)
{
  MissionSnapshot snapshot;
  const char *text = "MISSION:UNKNOWN ";

  if (!MissionTask_GetSnapshot(&snapshot))
  {
    (void)OLED_WriteString(0U, 2U, text);
    return;
  }

  switch (snapshot.state)
  {
    case MISSION_STATE_BOOT:
      text = "MISSION:BOOT    ";
      break;
    case MISSION_STATE_WAIT_START:
      text = "MISSION:WAIT    ";
      break;
    case MISSION_STATE_S1_DEPART:
      switch (snapshot.s1_phase)
      {
        case MISSION_S1_TURN:        text = "S1:TURN         "; break;
        case MISSION_S1_TURN_STABLE: text = "S1:TURN STABLE  "; break;
        default:                     text = "S1:STARTING     "; break;
      }
      break;
    case MISSION_STATE_S2_CROSS_BUMP:
      text = "S2:CROSS BUMP   ";
      break;
    case MISSION_STATE_S3_ALIGN_GREEN:
      text = snapshot.vision_target_valid
                 ? "S3:X ALIGN      "
                 : "S3:SEARCH GREEN ";
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CW:
      text = "S3:SEARCH CW90  ";
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
      text = "S3:SEARCH CCW180";
      break;
    case MISSION_STATE_S3_TRACK_GREEN:
      text = snapshot.vision_target_valid
                 ? "S3:XY TRACK     "
                 : "S3:TARGET LOST  ";
      break;
    case MISSION_STATE_S3_LOWER_FRAME:
      text = "S3:LOWER FRAME   ";
      break;
    case MISSION_STATE_S4_TRACK_CENTER:
      text = "S4:TRACK CENTER ";
      break;
    case MISSION_STATE_S4_CENTER_FOLLOW_THROUGH:
      text = "S4:ACK14        ";
      break;
    case MISSION_STATE_S4_WAIT_ARRANGE_READY:
      text = "S4:WAIT CAM 02  ";
      break;
    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      text = "S4:CURVE RIGHT ";
      break;
    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:
      text = "S4:BACK RIGHT   ";
      break;
    case MISSION_STATE_S4_WAIT_LEFT_TARGET:
      text = "S4:WAIT CAM 12  ";
      break;
    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:
      text = "S4:CURVE LEFT  ";
      break;
    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:
      text = "S4:BACK LEFT    ";
      break;
    case MISSION_STATE_S4_WAIT_FINAL_READY:
      text = "S4:WAIT CAM 24  ";
      break;
    case MISSION_STATE_S4_POST22_REVERSE:
      text = "22E2:REVERSE   ";
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
      text = "22E2:LEFT45    ";
      break;
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      text = "22E2:RIGHT90   ";
      break;
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
      text = "E4:PUSH RIGHT  ";
      break;
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
      text = "E4:BACK RIGHT  ";
      break;
    case MISSION_STATE_S4_E4_PUSH_LEFT:
      text = "E4:PUSH LEFT   ";
      break;
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
      text = "E4:BACK LEFT   ";
      break;
    case MISSION_STATE_S4_E4_FORWARD:
      text = "E4:FORWARD     ";
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
      text = "E4:AWAY SAFE   ";
      break;
    case MISSION_STATE_S4_E4_TURN_LEFT:
      text = "E4:LEFT45      ";
      break;
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      text = "E4:RIGHT90     ";
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      text = "E4:RED CCW360  ";
      break;
    case MISSION_STATE_S4_RAISE_FRAME:
      text = "S4:FRAME UP     ";
      break;
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
      text = "S4:TRACK FINAL  ";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      text = "S4:FINAL BACK   ";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      text = "S4:TURN SIDE1   ";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      text = "S4:TURN SIDE2   ";
      break;
    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      text = "S4:FINAL CENTER ";
      break;
    case MISSION_STATE_S4_FINAL_LOWER_FRAME:
      text = "S4:FRAME DOWN   ";
      break;
    case MISSION_STATE_S5_WAIT_SINGLE_GREEN:
      text = "S5:LOAD CHECK   ";
      break;
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
      text = "S5:ALIGN SAFE   ";
      break;
    case MISSION_STATE_S5_RAISE_FOR_ARRANGE:
      text = "S5:FRAME UP     ";
      break;
    case MISSION_STATE_S5_REVERSE_FOR_ARRANGE:
      text = "S5:REVERSE      ";
      break;
    case MISSION_STATE_S5_LOWER_FOR_ARRANGE:
      text = "S5:FRAME DOWN   ";
      break;
    case MISSION_STATE_S6_SEARCH_SAFE_ZONE:
      text = "S6:SCAN SAFE    ";
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
      text = "S6:SIDE TURN    ";
      break;
    case MISSION_STATE_S6_REPOSITION_FORWARD:
      text = "S6:SIDE FORWARD ";
      break;
    case MISSION_STATE_S6_REPOSITION_FACE_SAFE:
      text = "S6:FACE SAFE    ";
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      text = "S6:SIDE TO SAFE ";
      break;
    case MISSION_STATE_S6_RECOVERY_START:
      text = "S6:RECOVER START";
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      text = "S6:RECOVERY LEFT";
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      text = "S6:RECOVERY BACK";
      break;
    case MISSION_STATE_S6_TRACK_SAFE_ZONE:
      text = snapshot.vision_target_valid
                 ? "S6:VISION TRACK "
                 : "S6:TARGET LOST  ";
      break;
    case MISSION_STATE_S6_FINAL_ALIGN:
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      text = "S6:FINAL ALIGN   ";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_LEFT:
      text = "S6:AVOID LEFT   ";
      break;
    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      text = "S6:AVOID FWD    ";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_RIGHT:
      text = "S6:AVOID RIGHT ";
      break;
    case MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE:
      text = "S6:8S REVERSE   ";
      break;
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
      text = "S7:SILENT LEFT  ";
      break;
    case MISSION_STATE_S7_SILENT_TURN_BACK:
      text = "S7:SILENT RIGHT ";
      break;
    case MISSION_STATE_S6_RAISE_FRAME:
      text = "S6:FRAME UP      ";
      break;
    case MISSION_STATE_S6_PRE_PUSH_REVERSE:
      text = "S6:PRE REVERSE  ";
      break;
    case MISSION_STATE_S6_FINAL_PUSH:
      text = "S6:FINAL PUSH    ";
      break;
    case MISSION_STATE_S6_FINAL_REVERSE:
      text = "S6:FINAL REVERSE";
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      text = "S6:EXIT TURN180 ";
      break;
    case MISSION_STATE_S6_SAFE_ZONE_REACHED:
      text = "S6:PUSH DONE     ";
      break;
    case MISSION_STATE_STOPPED:
      text = "MISSION:STOPPED ";
      break;
    case MISSION_STATE_FAULT:
      text = "MISSION:FAULT   ";
      break;
    default:
      text = "MISSION:UNKNOWN ";
      break;
  }

  (void)OLED_WriteString(0U, 2U, text);
}

static uint8_t OLED_TaskReadStartZone(void)
{
  uint8_t selector = 0U;

  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_SET)
  {
    selector |= 2U;
  }
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET)
  {
    selector |= 1U;
  }

  switch (selector)
  {
    case 0U: return ROBOT_START_ZONE_BITS_00;
    case 1U: return ROBOT_START_ZONE_BITS_01;
    case 2U: return ROBOT_START_ZONE_BITS_10;
    case 3U: return ROBOT_START_ZONE_BITS_11;
    default: return ROBOT_START_ZONE_BITS_00;
  }
}

static void OLED_TaskShowConfig(void)
{
  MissionSnapshot snapshot;
  bool red;
  uint8_t zone;
  char text[17] = "CFG:RED Z0 READY";

  if (MissionTask_GetSnapshot(&snapshot) && snapshot.team_locked &&
      snapshot.start_zone_locked)
  {
    red = snapshot.team == ROBOT_TEAM_RED;
    zone = snapshot.start_zone;
    text[11] = 'L';
    text[12] = 'O';
    text[13] = 'C';
    text[14] = 'K';
    text[15] = ' ';
    text[16] = '\0';
    if (!red)
    {
      text[4] = 'B';
      text[5] = 'L';
      text[6] = 'U';
    }
    text[9] = (char)('0' + zone);
    (void)OLED_WriteString(0U, 4U, text);
    return;
  }

  red = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET
             ? ROBOT_PE13_HIGH_TEAM
             : ROBOT_PE13_LOW_TEAM) == ROBOT_TEAM_RED;
  zone = OLED_TaskReadStartZone();
  if (!red)
  {
    text[4] = 'B';
    text[5] = 'L';
    text[6] = 'U';
  }
  text[9] = (char)('0' + zone);
  (void)OLED_WriteString(0U, 4U, text);
}

static void OLED_TaskShowResetReason(void)
{
  ResetReasonSnapshot snapshot;
  const char *text;

  if (!ResetReason_GetSnapshot(&snapshot))
  {
    (void)OLED_WriteString(0U, 6U, "RST:NOT CAPTURED");
    return;
  }

  switch (snapshot.reason)
  {
    case RESET_REASON_POWER_ON:     text = "RST:POWER ON    "; break;
    case RESET_REASON_BROWNOUT:     text = "RST:BROWNOUT    "; break;
    case RESET_REASON_EXTERNAL_PIN: text = "RST:EXT PIN     "; break;
    case RESET_REASON_SOFTWARE:     text = "RST:SOFTWARE    "; break;
    case RESET_REASON_IWDG:         text = "RST:IWDG        "; break;
    case RESET_REASON_WWDG:         text = "RST:WWDG        "; break;
    case RESET_REASON_LOW_POWER:    text = "RST:LOW POWER   "; break;
    case RESET_REASON_UNKNOWN:
    default:                        text = "RST:UNKNOWN     "; break;
  }
  (void)OLED_WriteString(0U, 6U, text);
}

static void OLED_TaskUpdate(void)
{
  if (oled_ready == 0U) return;

  if (ROBOT_MG90_SPEED_TEST_ENABLED != 0U)
  {
    uint16_t pulse_us = ServoTest_GetPulseUs();
    char angle_text[17];
    char pulse_text[18];
    (void)snprintf(angle_text, sizeof(angle_text), "MG90:%3uD HOLD  ",
                   (unsigned)ROBOT_MG90_TEST_ANGLE_DEG);
    (void)snprintf(pulse_text, sizeof(pulse_text), "PWM:%4uus HOLD ",
                   (unsigned)pulse_us);
    (void)OLED_WriteString(0U, 0U,
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == GPIO_PIN_SET
            ? "PE12:H UNUSED   " : "PE12:L UNUSED   ");
    (void)OLED_WriteString(0U, 2U, angle_text);
    (void)OLED_WriteString(0U, 4U, pulse_text);
    OLED_TaskShowResetReason();
    return;
  }

  if (ROBOT_FRAME_DOWN_TEST_ENABLED != 0U)
  {
    (void)OLED_WriteString(0U, 0U, "FRAME:DOWN TEST ");
    (void)OLED_WriteString(0U, 2U, "MOTORS:OFF      ");
    (void)OLED_WriteString(0U, 4U, "CAM:45D PE12:-- ");
    OLED_TaskShowResetReason();
    return;
  }

  if (ROBOT_STRAIGHT_TEST_ENABLED != 0U)
  {
    const char *status = "WAIT CAN FEEDBK ";
    switch (MotorTest_GetStatus())
    {
      case MOTOR_TEST_RUNNING:        status = "FORWARD 1S      "; break;
      case MOTOR_TEST_DONE:           status = "STOPPED         "; break;
      case MOTOR_TEST_FEEDBACK_FAULT: status = "FAULT:CAN LOST  "; break;
      case MOTOR_TEST_WAIT_FEEDBACK:
      default:                        break;
    }
    (void)OLED_WriteString(0U, 0U, "MOTOR TEST100RPM");
    (void)OLED_WriteString(0U, 2U, status);
    (void)OLED_WriteString(0U, 4U, "PE12:UNUSED     ");
    OLED_TaskShowResetReason();
    return;
  }

  OLED_TaskShowStartButton();
  OLED_TaskShowMission();
  OLED_TaskShowConfig();
  OLED_TaskShowResetReason();
}

HAL_StatusTypeDef OLED_TaskInit(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status = OLED_Init(hi2c);

  if (status != HAL_OK) return status;

  oled_ready = 1U;
  status = OLED_WriteString(0U, 0U, "PE12:CHECKING   ");
  if (status == HAL_OK) status = OLED_WriteString(0U, 2U, "MISSION:BOOT    ");
  if (status == HAL_OK) status = OLED_WriteString(0U, 4U, "CFG:CHECKING    ");
  if (status == HAL_OK) OLED_TaskShowResetReason();
  return status;
}

void StartOledTask(void *argument)
{
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    OLED_TaskUpdate();
    next_wake += OLED_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
