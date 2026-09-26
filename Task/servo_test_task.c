#include "servo_test_task.h"

#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

#include <stdint.h>

#define SERVO_TEST_TASK_PERIOD_TICKS  10U
#define SERVO_TEST_DEBOUNCE_SAMPLES   5U
#define SERVO_TEST_TASK_STACK_BYTES   (256U * 4U)

#define SERVO_TEST_TIMER_PSC          83U
#define SERVO_TEST_TIMER_ARR          19999U

#define MG90_TEST_ANGLE_DEG           80U

static osThreadId_t servo_test_task_handle;

static const osThreadAttr_t servo_test_task_attributes = {
  .name = "servoTestTask",
  .stack_size = SERVO_TEST_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityLow,
};

static bool ServoTest_TimerConfigurationIsSafe(void)
{
  return (htim2.Instance == TIM2) &&
         (htim2.Init.Prescaler == SERVO_TEST_TIMER_PSC) &&
         (htim2.Init.Period == SERVO_TEST_TIMER_ARR);
}

static uint16_t ServoTest_Mg90AngleToPulse(uint16_t angle_deg)
{
  if (angle_deg > 180U)
  {
    angle_deg = 180U;
  }

  return (uint16_t)(((uint32_t)angle_deg * 2000U) / 180U + 500U);
}

static void ServoTest_SetMg90Pulse(uint16_t pulse_us)
{
  /* PA5 is TIM2 CH1 and TIM2 runs at one count per microsecond. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

static bool ServoTest_StartOutput(uint16_t pulse_us)
{
  ServoTest_SetMg90Pulse(pulse_us);
  return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) == HAL_OK;
}

static void ServoTest_StopMg90Output(void)
{
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
}

static void ServoTest_StopOutputs(void)
{
  ServoTest_StopMg90Output();
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
}

bool ServoTestTask_Create(void)
{
  if (servo_test_task_handle != NULL)
  {
    return true;
  }

  servo_test_task_handle = osThreadNew(StartServoTestTask, NULL,
                                       &servo_test_task_attributes);
  return servo_test_task_handle != NULL;
}

void StartServoTestTask(void *argument)
{
  GPIO_PinState candidate;
  GPIO_PinState last_sample;
  GPIO_PinState stable_state;
  uint8_t stable_samples = 1U;
  bool armed;
  uint32_t next_wake;

  (void)argument;
  ServoTest_StopOutputs();
  last_sample = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
  stable_state = last_sample;

  /* PE12 has an internal pull-up. Requiring a stable low level once after
     startup prevents an unconnected input or a high switch position from
     moving the servos immediately at power-on. */
  armed = stable_state == GPIO_PIN_RESET;
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    candidate = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
    if (candidate == last_sample)
    {
      if (stable_samples < SERVO_TEST_DEBOUNCE_SAMPLES)
      {
        ++stable_samples;
      }
    }
    else
    {
      last_sample = candidate;
      stable_samples = 1U;
    }

    if ((stable_samples >= SERVO_TEST_DEBOUNCE_SAMPLES) &&
        (candidate != stable_state))
    {
      stable_state = candidate;
      if (stable_state == GPIO_PIN_RESET)
      {
        ServoTest_StopOutputs();
        armed = true;
      }
      else if (armed && ServoTest_TimerConfigurationIsSafe())
      {
        (void)ServoTest_StartOutput(
            ServoTest_Mg90AngleToPulse(MG90_TEST_ANGLE_DEG));
      }
    }

    next_wake += SERVO_TEST_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
