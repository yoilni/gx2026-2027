#include "oled_task.h"

#include "cmsis_os.h"
#include "jy901s.h"

#define OLED_TASK_PERIOD_TICKS 200U
#define OLED_LINE_CHARACTERS 16U
#define JY901S_DISPLAY_UNKNOWN 0U
#define JY901S_DISPLAY_ONLINE  1U
#define JY901S_DISPLAY_OFFLINE 2U

static uint8_t oled_ready;
static uint8_t jy901s_display_state;

static void OLED_TaskFormatAngle(char *text, const char *label, int32_t centi_degrees)
{
  uint8_t index = 0U;
  uint32_t absolute_value;
  uint32_t whole_degrees;

  while (*label != '\0')
  {
    text[index++] = *label++;
  }

  text[index++] = (centi_degrees < 0) ? '-' : '+';
  absolute_value = (uint32_t)((centi_degrees < 0) ? -centi_degrees : centi_degrees);
  whole_degrees = absolute_value / 100U;

  text[index++] = (whole_degrees >= 100U) ? (char)('0' + (whole_degrees / 100U)) : ' ';
  text[index++] = (whole_degrees >= 10U) ? (char)('0' + ((whole_degrees / 10U) % 10U)) : ' ';
  text[index++] = (char)('0' + (whole_degrees % 10U));
  text[index++] = '.';
  text[index++] = (char)('0' + ((absolute_value / 10U) % 10U));
  text[index++] = (char)('0' + (absolute_value % 10U));
  while (index < OLED_LINE_CHARACTERS)
  {
    text[index++] = ' ';
  }
  text[index] = '\0';
}

static void OLED_TaskUpdate(void)
{
  JY901S_Attitude attitude;
  char oled_string[OLED_LINE_CHARACTERS + 1U];

  if (oled_ready == 0U) return;

  if (!JY901S_GetAttitude(&attitude))
  {
    if (jy901s_display_state != JY901S_DISPLAY_OFFLINE)
    {
      HAL_StatusTypeDef status = OLED_WriteString(0U, 0U, "JY901S OFFLINE  ");
      if (status == HAL_OK) status = OLED_WriteString(0U, 2U, "ROLL:   OFFLINE ");
      if (status == HAL_OK) status = OLED_WriteString(0U, 4U, "PITCH:  OFFLINE ");
      if (status == HAL_OK) status = OLED_WriteString(0U, 6U, "YAW:    OFFLINE ");
      if (status == HAL_OK) jy901s_display_state = JY901S_DISPLAY_OFFLINE;
    }
    return;
  }

  if (jy901s_display_state != JY901S_DISPLAY_ONLINE)
  {
    if (OLED_WriteString(0U, 0U, "JY901S ONLINE   ") == HAL_OK)
    {
      jy901s_display_state = JY901S_DISPLAY_ONLINE;
    }
  }

  OLED_TaskFormatAngle(oled_string, "Roll:", attitude.roll_cdeg);
  (void)OLED_WriteString(0U, 2U, oled_string);
  OLED_TaskFormatAngle(oled_string, "Pitch:", attitude.pitch_cdeg);
  (void)OLED_WriteString(0U, 4U, oled_string);
  OLED_TaskFormatAngle(oled_string, "Yaw:", attitude.yaw_cdeg);
  (void)OLED_WriteString(0U, 6U, oled_string);
}

HAL_StatusTypeDef OLED_TaskInit(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status = OLED_Init(hi2c);

  if (status != HAL_OK) return status;

  oled_ready = 1U;
  jy901s_display_state = JY901S_DISPLAY_UNKNOWN;
  status = OLED_WriteString(0U, 0U, "JY901S UART4    ");
  if (status == HAL_OK) status = OLED_WriteString(0U, 2U, "ROLL:           ");
  if (status == HAL_OK) status = OLED_WriteString(0U, 4U, "PITCH:          ");
  if (status == HAL_OK) status = OLED_WriteString(0U, 6U, "YAW:            ");
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
