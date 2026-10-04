#ifndef MAIXCAM_TASK_H
#define MAIXCAM_TASK_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* MaixCam -> MCU coordinate frame:
   F1 F2 CLASS XH XL XS YH YL YS 1F 2F.
   X/Y are unsigned magnitudes; sign 01 is positive and 02 is negative. */
#define MAIXCAM_FRAME_HEADER_1 0xF1U
#define MAIXCAM_FRAME_HEADER_2 0xF2U
#define MAIXCAM_FRAME_TAIL_1   0x1FU
#define MAIXCAM_FRAME_TAIL_2   0x2FU
#define MAIXCAM_FRAME_SIZE     11U

#define MAIXCAM_SIGN_POSITIVE  0x01U
#define MAIXCAM_SIGN_NEGATIVE  0x02U

#define MAIXCAM_X_ERROR_MAX    576U
#define MAIXCAM_Y_ERROR_MAX    448U
#define MAIXCAM_DATA_TIMEOUT_MS 500U

/* MCU -> MaixCam short command frame: E1 E2 CMD 1E 2E. */
#define MAIXCAM_COMMAND_HEADER_1 0xE1U
#define MAIXCAM_COMMAND_HEADER_2 0xE2U
#define MAIXCAM_COMMAND_TAIL_1   0x1EU
#define MAIXCAM_COMMAND_TAIL_2   0x2EU
#define MAIXCAM_COMMAND_SIZE     5U

#define MAIXCAM_COMMAND_SEARCH_TARGET 0x03U
#define MAIXCAM_COMMAND_TRACK_CENTER_ACK 0x04U
#define MAIXCAM_COMMAND_CENTER_RECOVERY_DONE 0x44U
#define MAIXCAM_COMMAND_FINAL_RECOVERY_DONE 0xF1U
#define MAIXCAM_COMMAND_SELECT_BLACK_GREEN 0x51U
#define MAIXCAM_COMMAND_RED_SEARCH_DONE 0x13U
#define MAIXCAM_COMMAND_CENTER_ACK    0x14U
#define MAIXCAM_COMMAND_RIGHT_DONE_ACK 0x12U
#define MAIXCAM_COMMAND_LEFT_DONE_ACK 0x22U
#define MAIXCAM_COMMAND_FINAL_CAPTURE 0x24U
#define MAIXCAM_COMMAND_OBSTACLE_DONE_ACK 0x36U
#define MAIXCAM_COMMAND_ANALYZE_LOAD_RED  0x15U
#define MAIXCAM_COMMAND_ANALYZE_LOAD_BLUE 0x25U
#define MAIXCAM_COMMAND_SELECT_BLUE   0x01U
#define MAIXCAM_COMMAND_SELECT_RED    0x11U
#define MAIXCAM_COMMAND_SELECT_GREEN  0x21U
#define MAIXCAM_COMMAND_SELECT_BLACK  0x31U
#define MAIXCAM_EVENT_OBJECT_IN_FRAME 0x04U
#define MAIXCAM_EVENT_ARRANGE_READY   0x02U
#define MAIXCAM_EVENT_CENTER_REACHED  0x14U
#define MAIXCAM_EVENT_LEFT_TARGET_READY 0x12U
#define MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK 0x24U
#define MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME 0x34U
#define MAIXCAM_EVENT_LOAD_CHECK_REQUEST 0x05U
#define MAIXCAM_EVENT_NO_ARRANGE_REQUIRED 0x06U
#define MAIXCAM_EVENT_SAFE_ZONE_FOUND 0x16U
#define MAIXCAM_EVENT_SAFE_ZONE_ALIGN_READY 0x26U
#define MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE 0x36U
#define MAIXCAM_EVENT_RED_PRIORITY_REQUEST 0x11U
#define MAIXCAM_EVENT_ARRANGE_TARGET_LOST 0xE2U
#define MAIXCAM_EVENT_SEARCH_TARGET_LOST 0xE3U
#define MAIXCAM_EVENT_CENTER_TARGET_LOST 0xE4U
/* First green only: repeated empty05 capture requests the E4 push path. */
#define MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY 0xE5U
#define MAIXCAM_EVENT_NO_TARGET       0xEEU

typedef struct
{
  uint8_t object_id;
  int16_t x_error_px;
  int16_t y_error_px;
  uint32_t sequence;
  uint32_t update_tick;
} MaixCam_Object;

/* USART3 is configured as 115200 baud, 8 data bits, no parity, 1 stop bit. */
HAL_StatusTypeDef MaixCam_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t command);
void MaixCam_ClearObject(void);
void MaixCam_ClearEvent(void);
/* Clear recovery-era inputs, including11/26, without stopping UART reception.
   A frame already being parsed is discarded on completion, not aborted. */
void MaixCam_ClearPendingInput(void);
/* E5 is accepted only while the mission explicitly opens the first05 window. */
void MaixCam_SetLoadEmptyRecoveryEnabled(bool enabled);
uint16_t MaixCam_TakeIgnoredLoadEmptyRecoveryCount(void);
void MaixCam_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void MaixCam_UART_ErrorCallback(UART_HandleTypeDef *huart);
bool MaixCam_GetObject(MaixCam_Object *object);
/* Copies last receipt even when invalidated; true means present, not fresh. */
bool MaixCam_GetObjectSnapshot(MaixCam_Object *object);
/* Ignore duplicate loss invalidation while the matching recovery is active. */
void MaixCam_SetRecoveryLossFilter(uint8_t event_code);
bool MaixCam_DropEventIf(uint8_t event_code);
/* Separate one-shot latch: 26 must survive other events during side moves. */
bool MaixCam_TakeSafeZoneAlignRequest(void);
/* Atomically reads and clears one pending 5-byte event frame. */
bool MaixCam_TakeEvent(uint8_t *event_code);
/* Separate latch so a subsequent E3/04 cannot overwrite the red-priority request. */
bool MaixCam_TakeRedPriorityRequest(void);

#endif /* MAIXCAM_TASK_H */
