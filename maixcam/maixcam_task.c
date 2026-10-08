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
/* Only the three valid E4 successors are retained. De-duplication bounds
   storage even if the camera repeats a phase notification indefinitely. */
#define MAIXCAM_E4_STAGE_CAPACITY 3U
static volatile uint8_t maixcam_e4_stage_events[MAIXCAM_E4_STAGE_CAPACITY];
static volatile uint8_t maixcam_e4_stage_count;
static volatile uint8_t maixcam_e4_stage_buffering;
static volatile uint8_t maixcam_discard_object_frame;
static volatile uint8_t maixcam_red_priority_pending;
static volatile uint8_t maixcam_safe_align_pending;
static volatile uint8_t maixcam_corner_evacuation_active;
static volatile uint16_t maixcam_ignored_corner_align_count;
static volatile uint8_t maixcam_recovery_loss_filter;
static volatile uint8_t maixcam_load_empty_recovery_enabled;
static volatile uint16_t maixcam_ignored_load_empty_recovery_count;
static volatile uint8_t maixcam_discard_current_frame;
static volatile uint8_t maixcam_supplement_window_enabled;
static volatile uint8_t maixcam_supplement_load_check_enabled;
static volatile uint8_t maixcam_supplement_request_pending;
static volatile uint8_t maixcam_supplement_load_check_pending;
static volatile uint32_t maixcam_supplement_request_tick;
static volatile uint8_t maixcam_supplement_finish_pending;
static volatile MaixCam_Object maixcam_supplement_cargo_report;
static volatile MaixCam_Object maixcam_supplement_finish_report;
static volatile uint8_t maixcam_load_recheck_enabled;
static volatile uint8_t maixcam_load_check_interrupt_enabled;
static volatile uint8_t maixcam_load_recheck_acked;
static volatile uint8_t maixcam_load_recheck_ack_pending;
static volatile uint8_t maixcam_load_recheck_finish_pending;
static volatile MaixCam_Object maixcam_load_recheck_report;
static volatile MaixCam_Object maixcam_load_recheck_finish_report;

static void MaixCam_ResetE4StageEvents(void)
{
  maixcam_e4_stage_count = 0U;
}

/* Called inside UART ISR or a task critical section. A phase already queued
   is not overwritten by duplicate E4 or by a later phase in the same burst. */
static bool MaixCam_BufferE4StageEvent(uint8_t command)
{
  if ((command != MAIXCAM_EVENT_CENTER_REACHED &&
       command != MAIXCAM_EVENT_ARRANGE_READY &&
       command != MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK) ||
      (maixcam_recovery_loss_filter != MAIXCAM_EVENT_CENTER_TARGET_LOST &&
       maixcam_e4_stage_buffering == 0U &&
       maixcam_e4_stage_count == 0U))
  {
    return false;
  }
  for (uint8_t i = 0U; i < maixcam_e4_stage_count; ++i)
  {
    if (maixcam_e4_stage_events[i] == command) return true;
  }
  /* Three possible distinct commands and three slots: no overwrite/overflow. */
  maixcam_e4_stage_events[maixcam_e4_stage_count++] = command;
  return true;
}

static void MaixCam_ResetParser(uint8_t byte)
{
  maixcam_discard_current_frame = 0U;
  maixcam_discard_object_frame = 0U;
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
  if (maixcam_discard_current_frame != 0U || maixcam_discard_object_frame != 0U)
  {
    return;
  }
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

  if (maixcam_load_recheck_enabled != 0U)
  {
    if (maixcam_load_recheck_acked != 0U && x_magnitude == 0U && y_magnitude == 0U &&
        maixcam_frame_buffer[2] >= 3U && maixcam_frame_buffer[2] <= 6U)
    {
      maixcam_load_recheck_report.object_id = maixcam_frame_buffer[2];
      maixcam_load_recheck_report.x_error_px = 0;
      maixcam_load_recheck_report.y_error_px = 0;
      maixcam_load_recheck_report.update_tick = HAL_GetTick();
      ++maixcam_load_recheck_report.sequence;
    }
    /* Before05 there is no class confirmation; during05 no frame drives. */
    return;
  }

  if (maixcam_supplement_window_enabled != 0U &&
      ((maixcam_supplement_load_check_enabled != 0U &&
        maixcam_frame_buffer[2] >= 3U && maixcam_frame_buffer[2] <= 6U) ||
       (maixcam_frame_buffer[2] == 5U && x_magnitude == 0U && y_magnitude == 0U)))
  {
    /* Reserved report, not an arrival coordinate. Freeze it only when06
       follows; normal safe-zone/target frames cannot overwrite this mailbox. */
    maixcam_supplement_cargo_report.object_id = maixcam_frame_buffer[2];
    maixcam_supplement_cargo_report.x_error_px = 0;
    maixcam_supplement_cargo_report.y_error_px = 0;
    maixcam_supplement_cargo_report.update_tick = HAL_GetTick();
    ++maixcam_supplement_cargo_report.sequence;
    /* While counting, every class frame is a report, never a motion target.
       Outside counting only zero-error ID5 is reserved as before. */
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
      if ((byte == MAIXCAM_FRAME_TAIL_2) &&
          ((maixcam_discard_current_frame == 0U) ||
           (maixcam_load_check_interrupt_enabled != 0U &&
            maixcam_frame_buffer[2] == MAIXCAM_EVENT_LOAD_CHECK_REQUEST) ||
           ((maixcam_supplement_window_enabled != 0U ||
             (maixcam_load_recheck_enabled != 0U && maixcam_load_recheck_acked != 0U)) &&
            maixcam_frame_buffer[2] == MAIXCAM_EVENT_NO_ARRANGE_REQUIRED)))
      {
        if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_LOAD_CHECK_REQUEST &&
            maixcam_load_check_interrupt_enabled != 0U)
        {
          if (maixcam_load_recheck_finish_pending == 0U)
          {
            maixcam_load_recheck_enabled = 1U;
            maixcam_load_recheck_ack_pending = 1U;
            if (maixcam_load_recheck_acked == 0U)
            {
              maixcam_load_recheck_acked = 1U;
              maixcam_load_recheck_report.object_id = 0U;
              maixcam_has_event = 0U;
              MaixCam_ResetE4StageEvents();
              maixcam_has_object = 0U;
              maixcam_red_priority_pending = 0U;
              maixcam_safe_align_pending = 0U;
              maixcam_supplement_request_pending = 0U;
              maixcam_supplement_load_check_pending = 0U;
            }
          }
          /* A duplicate05 still reaches the task; it cannot erase this
             window's report/result or restart its camera-settle timer. */
        }
        else if (maixcam_load_recheck_enabled != 0U)
        {
          uint8_t command = maixcam_frame_buffer[2];
          if (command == MAIXCAM_EVENT_LOAD_CHECK_REQUEST)
          {
            if (maixcam_load_recheck_acked == 0U)
            {
              maixcam_load_recheck_acked = 1U;
              maixcam_load_recheck_ack_pending = 1U;
              maixcam_load_recheck_report.object_id = 0U;
              maixcam_has_event = 0U;
              MaixCam_ResetE4StageEvents();
              maixcam_has_object = 0U;
              maixcam_safe_align_pending = 0U;
            }
          }
          else if (maixcam_load_recheck_acked != 0U &&
                   command == MAIXCAM_EVENT_NO_ARRANGE_REQUIRED)
          {
            if (maixcam_load_recheck_finish_pending == 0U)
            {
              maixcam_load_recheck_finish_report = maixcam_load_recheck_report;
              if ((uint32_t)(HAL_GetTick() - maixcam_load_recheck_finish_report.update_tick) >
                  MAIXCAM_DATA_TIMEOUT_MS)
                maixcam_load_recheck_finish_report.object_id = 0U;
              maixcam_has_event = 0U;
              MaixCam_ResetE4StageEvents();
              maixcam_has_object = 0U;
              maixcam_safe_align_pending = 0U;
            }
            maixcam_load_recheck_finish_pending = 1U;
            maixcam_supplement_request_pending = 0U;
          }
          else if (maixcam_load_recheck_finish_pending != 0U &&
                   command == MAIXCAM_EVENT_SAFE_ZONE_FOUND)
          {
            /* A16 immediately following06 is new safe-zone evidence.
               Preserve this one-shot event until the mission accepts06. */
            maixcam_event_code = command;
            maixcam_has_event = 1U;
          }
          else if (maixcam_load_recheck_acked != 0U &&
                   maixcam_load_recheck_finish_pending == 0U &&
                   command == MAIXCAM_EVENT_SUPPLEMENT_CAPTURE_REQUEST &&
                   maixcam_supplement_window_enabled != 0U)
          {
            if (maixcam_supplement_request_pending == 0U)
              maixcam_supplement_request_tick = HAL_GetTick();
            maixcam_supplement_request_pending = 1U;
          }
          else if (maixcam_load_recheck_acked != 0U &&
                   maixcam_load_recheck_finish_pending == 0U &&
                   (command == MAIXCAM_EVENT_ARRANGE_READY ||
                    command == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK ||
                    (command == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY &&
                     maixcam_load_empty_recovery_enabled != 0U)))
          {
            maixcam_event_code = command;
            maixcam_has_event = 1U;
          }
          /* Ignore old safe-zone events/26, duplicate05 and03 supplement
             requests. Keep receiving;06 wins over02/24 in this count window. */
          else if (command == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY &&
                   maixcam_ignored_load_empty_recovery_count < UINT16_MAX)
          {
            ++maixcam_ignored_load_empty_recovery_count;
          }
        }
        else if (maixcam_supplement_window_enabled != 0U &&
            maixcam_frame_buffer[2] == MAIXCAM_EVENT_NO_ARRANGE_REQUIRED)
        {
          if (maixcam_supplement_finish_pending == 0U)
          {
            maixcam_supplement_finish_report = maixcam_supplement_cargo_report;
            if ((uint32_t)(HAL_GetTick() -
                maixcam_supplement_finish_report.update_tick) > MAIXCAM_DATA_TIMEOUT_MS)
            {
              maixcam_supplement_finish_report.object_id = 0U;
            }
          }
          maixcam_supplement_finish_pending = 1U;
          /* Cancel only pre06 motion inputs. A subsequent16/26 must remain
             available when the mission task consumes this priority latch. */
          maixcam_has_object = 0U;
          maixcam_has_event = 0U;
          MaixCam_ResetE4StageEvents();
          maixcam_red_priority_pending = 0U;
          maixcam_safe_align_pending = 0U;
          maixcam_supplement_request_pending = 0U;
          maixcam_supplement_load_check_pending = 0U;
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_LOAD_CHECK_REQUEST &&
                 maixcam_supplement_window_enabled != 0U)
        {
          if (maixcam_supplement_finish_pending == 0U)
          {
            maixcam_supplement_load_check_pending = 1U;
            if (maixcam_supplement_load_check_enabled == 0U)
            {
              /* Fence pre05 motion at receipt, preserving later03/02/24.
                 Duplicate05 in the same count window must not erase reports. */
              maixcam_has_object = 0U;
              maixcam_has_event = 0U;
              MaixCam_ResetE4StageEvents();
              maixcam_red_priority_pending = 0U;
              maixcam_safe_align_pending = 0U;
              maixcam_supplement_request_pending = 0U;
              maixcam_supplement_cargo_report.object_id = 0U;
              maixcam_supplement_load_check_enabled = 1U;
            }
          }
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_SUPPLEMENT_CAPTURE_REQUEST)
        {
          if (maixcam_supplement_window_enabled != 0U)
          {
            if (maixcam_supplement_request_pending == 0U)
            {
              maixcam_supplement_request_tick = HAL_GetTick();
            }
            maixcam_supplement_request_pending = 1U;
          }
          else
          {
            maixcam_event_code = MAIXCAM_EVENT_SUPPLEMENT_CAPTURE_REQUEST;
            maixcam_has_event = 1U;
          }
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY)
        {
          if (maixcam_load_empty_recovery_enabled != 0U)
          {
            maixcam_event_code = MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY;
            maixcam_has_event = 1U;
          }
          else if (maixcam_ignored_load_empty_recovery_count < UINT16_MAX)
          {
            ++maixcam_ignored_load_empty_recovery_count;
          }
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_NO_TARGET)
        {
          if (maixcam_recovery_loss_filter != MAIXCAM_EVENT_NO_TARGET)
          {
            maixcam_has_object = 0U;
          }
          maixcam_event_code = MAIXCAM_EVENT_NO_TARGET;
          maixcam_has_event = 1U;
        }
        else if (maixcam_frame_buffer[2] ==
                 MAIXCAM_EVENT_SEARCH_TARGET_LOST ||
                 maixcam_frame_buffer[2] == MAIXCAM_EVENT_CENTER_TARGET_LOST)
        {
          if (maixcam_recovery_loss_filter != maixcam_frame_buffer[2])
          {
            maixcam_has_object = 0U;
          }
          /* Repeated E4 during recovery must not overwrite a one-shot14/02/24.
             E3 retains its existing task-side duplicate handling. */
          if (maixcam_frame_buffer[2] != MAIXCAM_EVENT_CENTER_TARGET_LOST ||
              maixcam_recovery_loss_filter != MAIXCAM_EVENT_CENTER_TARGET_LOST)
          {
            maixcam_event_code = maixcam_frame_buffer[2];
            maixcam_has_event = 1U;
          }
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
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_SAFE_ZONE_ALIGN_READY)
        {
          if (maixcam_corner_evacuation_active != 0U)
          {
            if (maixcam_ignored_corner_align_count < UINT16_MAX)
              ++maixcam_ignored_corner_align_count;
          }
          else
          {
            maixcam_safe_align_pending = 1U;
          }
        }
        else if (maixcam_frame_buffer[2] == MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE)
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
          if (!MaixCam_BufferE4StageEvent(maixcam_frame_buffer[2]))
          {
            maixcam_event_code = maixcam_frame_buffer[2];
            maixcam_has_event = 1U;
          }
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
  MaixCam_ResetE4StageEvents();
  maixcam_e4_stage_buffering = 0U;
  maixcam_discard_object_frame = 0U;
  maixcam_object.sequence = 0U;
  maixcam_red_priority_pending = 0U;
  maixcam_safe_align_pending = 0U;
  maixcam_corner_evacuation_active = 0U;
  maixcam_ignored_corner_align_count = 0U;
  maixcam_recovery_loss_filter = 0U;
  maixcam_load_empty_recovery_enabled = 0U;
  maixcam_ignored_load_empty_recovery_count = 0U;
  maixcam_discard_current_frame = 0U;
  maixcam_object.update_tick = 0U;
  maixcam_supplement_window_enabled = 0U;
  maixcam_supplement_load_check_enabled = 0U;
  maixcam_supplement_request_pending = 0U;
  maixcam_supplement_load_check_pending = 0U;
  maixcam_supplement_finish_pending = 0U;
  maixcam_supplement_cargo_report.object_id = 0U;
  maixcam_supplement_finish_report.object_id = 0U;
  maixcam_load_recheck_enabled = 0U;
  maixcam_load_check_interrupt_enabled = 0U;
  maixcam_load_recheck_acked = 0U;
  maixcam_load_recheck_ack_pending = 0U;
  maixcam_load_recheck_finish_pending = 0U;
  maixcam_load_recheck_report.object_id = 0U;
  maixcam_load_recheck_report.sequence = 0U;
  maixcam_load_recheck_finish_report.object_id = 0U;
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
  /* Retain the last receipt time for diagnostics; presence controls validity. */
  if (primask == 0U)
  {
    __enable_irq();
  }
}

uint32_t MaixCam_FenceObjectReference(void)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t sequence;
  __disable_irq();
  sequence = maixcam_object.sequence;
  maixcam_has_object = 0U;
  /* A partially received old-reference coordinate cannot become a fresh ID4
     after TX11/03. Short events use a different publish path and survive. */
  maixcam_discard_object_frame =
      maixcam_parser_state != MAIXCAM_WAIT_HEADER_1 ? 1U : 0U;
  if (primask == 0U) __enable_irq();
  return sequence;
}

void MaixCam_ClearEvent(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  maixcam_event_code = 0U;
  maixcam_has_event = 0U;
  MaixCam_ResetE4StageEvents();
  maixcam_red_priority_pending = 0U;
  /* 26 has an independent lifecycle; clearing maneuver events must not lose it. */
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void MaixCam_ClearPendingInput(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  maixcam_has_object = 0U;
  maixcam_object.object_id = 0U;
  maixcam_object.x_error_px = 0;
  maixcam_object.y_error_px = 0;
  /* Preserve sequence/time for diagnostics and freshness fences. */
  maixcam_event_code = 0U;
  maixcam_has_event = 0U;
  MaixCam_ResetE4StageEvents();
  maixcam_red_priority_pending = 0U;
  maixcam_safe_align_pending = 0U;
  maixcam_supplement_request_pending = 0U;
  maixcam_supplement_load_check_pending = 0U;
  /* Supplement06 and recheck05/06 with their frozen reports survive ordinary
     clearing. Only closing their respective windows cancels these latches. */
  /* Keep the parser and RX interrupt armed. Do not let the remainder of a
     pre44 frame republish stale data after this clearing boundary. */
  maixcam_discard_current_frame =
      maixcam_parser_state != MAIXCAM_WAIT_HEADER_1 ? 1U : 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void MaixCam_SetLoadEmptyRecoveryEnabled(bool enabled)
{
  maixcam_load_empty_recovery_enabled = enabled ? 1U : 0U;
}

uint16_t MaixCam_TakeIgnoredLoadEmptyRecoveryCount(void)
{
  uint32_t primask = __get_PRIMASK();
  uint16_t count;

  __disable_irq();
  count = maixcam_ignored_load_empty_recovery_count;
  maixcam_ignored_load_empty_recovery_count = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return count;
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
    maixcam_discard_current_frame = 0U;
    maixcam_discard_object_frame = 0U;
    (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
  }
}

bool MaixCam_GetObject(MaixCam_Object *object)
{
  return MaixCam_GetObjectSnapshot(object) &&
      ((uint32_t)(HAL_GetTick() - object->update_tick) <= MAIXCAM_DATA_TIMEOUT_MS);
}

bool MaixCam_GetObjectSnapshot(MaixCam_Object *object)
{
  uint32_t primask;
  bool present;

  if (object == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  present = maixcam_has_object != 0U;
  *object = maixcam_object;
  if (primask == 0U) __enable_irq();

  return present;
}

void MaixCam_SetRecoveryLossFilter(uint8_t event_code)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  maixcam_recovery_loss_filter = event_code;
  if (event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST && maixcam_has_event != 0U &&
      MaixCam_BufferE4StageEvent(maixcam_event_code))
  {
    /* Preserve a valid phase already received while entering recovery. */
    maixcam_has_event = 0U;
  }
  if (primask == 0U) __enable_irq();
}

void MaixCam_SetE4StageBuffering(bool enabled)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  maixcam_e4_stage_buffering = enabled ? 1U : 0U;
  if (enabled && maixcam_has_event != 0U &&
      MaixCam_BufferE4StageEvent(maixcam_event_code))
    maixcam_has_event = 0U;
  if (primask == 0U) __enable_irq();
}

bool MaixCam_DropEventIf(uint8_t event_code)
{
  uint32_t primask = __get_PRIMASK();
  bool dropped;
  __disable_irq();
  dropped = maixcam_has_event != 0U && maixcam_event_code == event_code;
  if (dropped) maixcam_has_event = 0U;
  for (uint8_t i = 0U; i < maixcam_e4_stage_count; ++i)
  {
    if (maixcam_e4_stage_events[i] != event_code) continue;
    --maixcam_e4_stage_count;
    for (uint8_t j = i; j < maixcam_e4_stage_count; ++j)
      maixcam_e4_stage_events[j] = maixcam_e4_stage_events[j + 1U];
    dropped = true;
    break;
  }
  if (primask == 0U) __enable_irq();
  return dropped;
}

void MaixCam_SetCornerEvacuationActive(bool active)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!active && maixcam_corner_evacuation_active != 0U &&
      maixcam_parser_state != MAIXCAM_WAIT_HEADER_1)
  {
    /* A frame that began during evacuation is not a new post-return request.
       Let it finish parsing, then discard it without aborting UART reception. */
    maixcam_discard_current_frame = 1U;
  }
  maixcam_corner_evacuation_active = active ? 1U : 0U;
  if (primask == 0U) __enable_irq();
}

uint16_t MaixCam_TakeIgnoredCornerAlignCount(void)
{
  uint32_t primask = __get_PRIMASK();
  uint16_t count;
  __disable_irq();
  count = maixcam_ignored_corner_align_count;
  maixcam_ignored_corner_align_count = 0U;
  if (primask == 0U) __enable_irq();
  return count;
}

bool MaixCam_TakeSafeZoneAlignRequest(void)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_safe_align_pending != 0U;
  maixcam_safe_align_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
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
  if (maixcam_e4_stage_count != 0U)
  {
    *event_code = maixcam_e4_stage_events[0];
    --maixcam_e4_stage_count;
    for (uint8_t i = 0U; i < maixcam_e4_stage_count; ++i)
      maixcam_e4_stage_events[i] = maixcam_e4_stage_events[i + 1U];
    if (primask == 0U) __enable_irq();
    return true;
  }
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

bool MaixCam_PeekEvent(uint8_t *event_code)
{
  uint32_t primask;
  bool pending;
  if (event_code == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  pending = maixcam_e4_stage_count != 0U || maixcam_has_event != 0U;
  if (pending)
    *event_code = maixcam_e4_stage_count != 0U
        ? maixcam_e4_stage_events[0] : maixcam_event_code;
  if (primask == 0U) __enable_irq();
  return pending;
}

void MaixCam_SetSupplementWindowEnabled(bool enabled)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  maixcam_supplement_window_enabled = enabled ? 1U : 0U;
  if (!enabled)
  {
    maixcam_supplement_load_check_enabled = 0U;
    maixcam_supplement_request_pending = 0U;
    maixcam_supplement_load_check_pending = 0U;
    maixcam_supplement_finish_pending = 0U;
    maixcam_supplement_cargo_report.object_id = 0U;
    maixcam_supplement_finish_report.object_id = 0U;
  }
  if (primask == 0U) __enable_irq();
}

void MaixCam_SetSupplementLoadCheckEnabled(bool enabled)
{
  maixcam_supplement_load_check_enabled = enabled ? 1U : 0U;
}

bool MaixCam_TakeSupplementRequest(uint32_t *request_tick)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_supplement_request_pending != 0U;
  if (pending && request_tick != NULL) *request_tick = maixcam_supplement_request_tick;
  maixcam_supplement_request_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}

bool MaixCam_TakeSupplementLoadCheckRequest(void)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_supplement_load_check_pending != 0U;
  maixcam_supplement_load_check_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}

bool MaixCam_TakeSupplementFinishRequest(MaixCam_Object *report)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_supplement_finish_pending != 0U;
  if (pending && report != NULL) *report = maixcam_supplement_finish_report;
  maixcam_supplement_finish_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}

void MaixCam_SetLoadCheckInterruptEnabled(bool enabled)
{
  maixcam_load_check_interrupt_enabled = enabled ? 1U : 0U;
  if (!enabled) MaixCam_SetLoadRecheckEnabled(false);
}

bool MaixCam_LoadRecheckResultPending(void)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_load_recheck_finish_pending != 0U ||
      (maixcam_has_event != 0U &&
       (maixcam_event_code == MAIXCAM_EVENT_ARRANGE_READY ||
        maixcam_event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK ||
        maixcam_event_code == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY));
  if (primask == 0U) __enable_irq();
  return pending;
}

void MaixCam_SetLoadRecheckEnabled(bool enabled)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t value = enabled ? 1U : 0U;
  __disable_irq();
  if (maixcam_load_recheck_enabled != value)
  {
    if (!enabled && maixcam_parser_state != MAIXCAM_WAIT_HEADER_1 &&
        !(maixcam_frame_index >= 3U &&
          maixcam_frame_buffer[2] == MAIXCAM_EVENT_SAFE_ZONE_FOUND))
      maixcam_discard_current_frame = 1U;
    maixcam_load_recheck_acked = 0U;
    maixcam_load_recheck_ack_pending = 0U;
    maixcam_load_recheck_finish_pending = 0U;
    maixcam_load_recheck_report.object_id = 0U;
    maixcam_load_recheck_finish_report.object_id = 0U;
  }
  maixcam_load_recheck_enabled = value;
  if (primask == 0U) __enable_irq();
}

bool MaixCam_TakeLoadRecheckAck(void)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_load_recheck_ack_pending != 0U;
  maixcam_load_recheck_ack_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}

bool MaixCam_TakeLoadRecheckFinish(MaixCam_Object *report)
{
  uint32_t primask = __get_PRIMASK();
  bool pending;
  __disable_irq();
  pending = maixcam_load_recheck_finish_pending != 0U;
  if (pending && report != NULL) *report = maixcam_load_recheck_finish_report;
  maixcam_load_recheck_finish_pending = 0U;
  if (primask == 0U) __enable_irq();
  return pending;
}
