#include "maixcam_task.h"

typedef enum
{
  MAIXCAM_WAIT_HEADER_1 = 0,
  MAIXCAM_WAIT_HEADER_2,
  MAIXCAM_READ_ID,
  MAIXCAM_READ_X_HIGH,
  MAIXCAM_READ_X_LOW,
  MAIXCAM_READ_X_SIGN,
  MAIXCAM_READ_Y_HIGH,
  MAIXCAM_READ_Y_LOW,
  MAIXCAM_READ_Y_SIGN,
  MAIXCAM_WAIT_TAIL_1,
  MAIXCAM_WAIT_TAIL_2,
  MAIXCAM_WAIT_EVENT_TAIL_2
} MaixCam_ParserState;

static UART_HandleTypeDef *maixcam_uart;
static uint8_t maixcam_rx_byte;
static uint8_t maixcam_frame_buffer[MAIXCAM_FRAME_SIZE];
static uint8_t maixcam_frame_index;
static MaixCam_ParserState maixcam_parser_state;
static volatile MaixCam_Object maixcam_object;
static volatile uint8_t maixcam_has_object;
static volatile uint8_t maixcam_event_code;
static volatile uint8_t maixcam_has_event;
static volatile uint8_t maixcam_red_priority_pending;

static void MaixCam_ResetParser(uint8_t byte)
{
  if (byte == MAIXCAM_FRAME_HEADER_1)
  {
    maixcam_frame_buffer[0] = byte;
    maixcam_frame_index = 1U;
    maixcam_parser_state = MAIXCAM_WAIT_HEADER_2;
  }
  else
  {
    maixcam_frame_index = 0U;
    maixcam_parser_state = MAIXCAM_WAIT_HEADER_1;
  }
}

static void MaixCam_PublishFrame(void)
{
  uint16_t x_magnitude =
      (uint16_t)(((uint16_t)maixcam_frame_buffer[3] << 8U) |
                 maixcam_frame_buffer[4]);
  uint16_t y_magnitude =
      (uint16_t)(((uint16_t)maixcam_frame_buffer[6] << 8U) |
                 maixcam_frame_buffer[7]);
  uint8_t x_sign = maixcam_frame_buffer[5];
  uint8_t y_sign = maixcam_frame_buffer[8];

  if (((x_sign != MAIXCAM_SIGN_POSITIVE) &&
       (x_sign != MAIXCAM_SIGN_NEGATIVE)) ||
      ((y_sign != MAIXCAM_SIGN_POSITIVE) &&
       (y_sign != MAIXCAM_SIGN_NEGATIVE)) ||
      (x_magnitude > MAIXCAM_X_ERROR_MAX) ||
      (y_magnitude > MAIXCAM_Y_ERROR_MAX))
  {
    return;
  }

  maixcam_object.object_id = maixcam_frame_buffer[2];
  maixcam_object.x_error_px =
      x_sign == MAIXCAM_SIGN_POSITIVE
          ? (int16_t)x_magnitude
          : (int16_t)-(int16_t)x_magnitude;
  maixcam_object.y_error_px =
      y_sign == MAIXCAM_SIGN_POSITIVE
          ? (int16_t)y_magnitude
          : (int16_t)-(int16_t)y_magnitude;
  maixcam_object.update_tick = HAL_GetTick();
  ++maixcam_object.sequence;
  maixcam_has_object = 1U;
}

static void MaixCam_ProcessByte(uint8_t byte)
{
  switch (maixcam_parser_state)
  {
    case MAIXCAM_WAIT_HEADER_1:
      if (byte == MAIXCAM_FRAME_HEADER_1)
      {
        maixcam_frame_buffer[0] = byte;
        maixcam_frame_index = 1U;
        maixcam_parser_state = MAIXCAM_WAIT_HEADER_2;
      }
      break;

    case MAIXCAM_WAIT_HEADER_2:
      if (byte == MAIXCAM_FRAME_HEADER_2)
      {
        maixcam_frame_buffer[maixcam_frame_index++] = byte;
        maixcam_parser_state = MAIXCAM_READ_ID;
      }
      else
      {
        MaixCam_ResetParser(byte);
      }
      break;

    case MAIXCAM_READ_ID:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_READ_X_HIGH;
      break;

    case MAIXCAM_READ_X_HIGH:
      /* A 5-byte event and an 11-byte coordinate frame share F1 F2 and the
         third byte. XH can only be 0..2 with the configured image bounds, so
         1F here unambiguously identifies the event-frame tail. */
      if (byte == MAIXCAM_FRAME_TAIL_1)
      {
        maixcam_parser_state = MAIXCAM_WAIT_EVENT_TAIL_2;
      }
      else
      {
        maixcam_frame_buffer[maixcam_frame_index++] = byte;
        maixcam_parser_state = MAIXCAM_READ_X_LOW;
      }
      break;

    case MAIXCAM_READ_X_LOW:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_READ_X_SIGN;
      break;

    case MAIXCAM_READ_X_SIGN:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_READ_Y_HIGH;
      break;

    case MAIXCAM_READ_Y_HIGH:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_READ_Y_LOW;
      break;

    case MAIXCAM_READ_Y_LOW:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_READ_Y_SIGN;
      break;

    case MAIXCAM_READ_Y_SIGN:
      maixcam_frame_buffer[maixcam_frame_index++] = byte;
      maixcam_parser_state = MAIXCAM_WAIT_TAIL_1;
      break;

    case MAIXCAM_WAIT_TAIL_1:
      if (byte == MAIXCAM_FRAME_TAIL_1)
      {
        maixcam_frame_buffer[maixcam_frame_index++] = byte;
        maixcam_parser_state = MAIXCAM_WAIT_TAIL_2;
      }
      else
      {
        MaixCam_ResetParser(byte);
      }
      break;

    case MAIXCAM_WAIT_TAIL_2:
      if (byte == MAIXCAM_FRAME_TAIL_2)
      {
        maixcam_frame_buffer[maixcam_frame_index] = byte;
        MaixCam_PublishFrame();
      }
      MaixCam_ResetParser(byte);
      break;

    case MAIXCAM_WAIT_EVENT_TAIL_2:
      if (byte == MAIXCAM_FRAME_TAIL_2)
      {
        if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_NO_TARGET)
        {
          maixcam_has_object = 0U;
          maixcam_object.update_tick = 0U;
        }
        else if (maixcam_frame_buffer[2] ==
                 MAIXCAM_EVENT_SEARCH_TARGET_LOST ||
                 maixcam_frame_buffer[2] == MAIXCAM_EVENT_CENTER_TARGET_LOST)
        {
          maixcam_has_object = 0U;
          maixcam_object.update_tick = 0U;
          maixcam_event_code = maixcam_frame_buffer[2];
          maixcam_has_event = 1U;
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_RED_PRIORITY_REQUEST)
        {
          maixcam_red_priority_pending = 1U;
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_OBJECT_IN_FRAME)
        {
          maixcam_event_code = MAIXCAM_EVENT_OBJECT_IN_FRAME;
          maixcam_has_event = 1U;
        }
        else if ((maixcam_frame_buffer[2] ==
                  MAIXCAM_EVENT_LOAD_CHECK_REQUEST) ||
                 (maixcam_frame_buffer[2] ==
                  MAIXCAM_EVENT_NO_ARRANGE_REQUIRED))
        {
          maixcam_event_code = maixcam_frame_buffer[2];
          maixcam_has_event = 1U;
        }
        else if (maixcam_frame_buffer[2] ==
                 MAIXCAM_EVENT_SAFE_ZONE_FOUND)
        {
          maixcam_event_code = MAIXCAM_EVENT_SAFE_ZONE_FOUND;
          maixcam_has_event = 1U;
        }
        else if ((maixcam_frame_buffer[2] ==
                  MAIXCAM_EVENT_SAFE_ZONE_ALIGN_READY) ||
                 (maixcam_frame_buffer[2] ==
                  MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE))
        {
          maixcam_event_code = maixcam_frame_buffer[2];
          maixcam_has_event = 1U;
        }
        else if ((maixcam_frame_buffer[2] == MAIXCAM_EVENT_ARRANGE_READY) ||
                 (maixcam_frame_buffer[2] == MAIXCAM_EVENT_CENTER_REACHED) ||
                 (maixcam_frame_buffer[2] ==
                  MAIXCAM_EVENT_LEFT_TARGET_READY) ||
                  (maixcam_frame_buffer[2] ==
                   MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK) ||
                  (maixcam_frame_buffer[2] ==
                   MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME) ||
                  (maixcam_frame_buffer[2] ==
                   MAIXCAM_EVENT_ARRANGE_TARGET_LOST))
        {
          maixcam_event_code = maixcam_frame_buffer[2];
          maixcam_has_event = 1U;
        }
      }
      MaixCam_ResetParser(byte);
      break;

    default:
      MaixCam_ResetParser(byte);
      break;
  }
}

HAL_StatusTypeDef MaixCam_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  maixcam_uart = huart;
  maixcam_frame_index = 0U;
  maixcam_parser_state = MAIXCAM_WAIT_HEADER_1;
  maixcam_has_object = 0U;
  maixcam_event_code = 0U;
  maixcam_has_event = 0U;
  maixcam_object.sequence = 0U;
  maixcam_red_priority_pending = 0U;
  maixcam_object.update_tick = 0U;
  return HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
}

HAL_StatusTypeDef MaixCam_SendCommand(uint8_t command)
{
  uint8_t frame[MAIXCAM_COMMAND_SIZE] = {
    MAIXCAM_COMMAND_HEADER_1,
    MAIXCAM_COMMAND_HEADER_2,
    command,
    MAIXCAM_COMMAND_TAIL_1,
    MAIXCAM_COMMAND_TAIL_2
  };

  if (maixcam_uart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(maixcam_uart, frame, sizeof(frame), 20U);
}

void MaixCam_ClearObject(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  maixcam_has_object = 0U;
  maixcam_object.update_tick = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void MaixCam_ClearEvent(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  maixcam_event_code = 0U;
  maixcam_has_event = 0U;
  maixcam_red_priority_pending = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void MaixCam_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != maixcam_uart))
  {
    return;
  }

  MaixCam_ProcessByte(maixcam_rx_byte);
  (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
}

void MaixCam_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart == maixcam_uart))
  {
    maixcam_frame_index = 0U;
    maixcam_parser_state = MAIXCAM_WAIT_HEADER_1;
    (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
  }
}

bool MaixCam_GetObject(MaixCam_Object *object)
{
  uint32_t primask;

  if (object == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (maixcam_has_object == 0U)
  {
    if (primask == 0U) __enable_irq();
    return false;
  }
  *object = maixcam_object;
  if (primask == 0U) __enable_irq();

  return ((uint32_t)(HAL_GetTick() - object->update_tick) <= MAIXCAM_DATA_TIMEOUT_MS);
}

bool MaixCam_TakeRedPriorityRequest(void)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_red_priority_pending != 0U;
  maixcam_red_priority_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}

bool MaixCam_TakeEvent(uint8_t *event_code)
{
  uint32_t primask;

  if (event_code == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (maixcam_has_event == 0U)
  {
    if (primask == 0U) __enable_irq();
    return false;
  }
  *event_code = maixcam_event_code;
  maixcam_event_code = 0U;
  maixcam_has_event = 0U;
  if (primask == 0U) __enable_irq();
  return true;
}
