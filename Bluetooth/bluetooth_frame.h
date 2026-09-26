#ifndef BLUETOOTH_FRAME_H
#define BLUETOOTH_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define BLUETOOTH_FRAME_HEADER_1 0xA1U
#define BLUETOOTH_FRAME_HEADER_2 0xA2U
#define BLUETOOTH_FRAME_TAIL_1   0xB1U
#define BLUETOOTH_FRAME_TAIL_2   0xB2U
#define BLUETOOTH_FRAME_SIZE     5U

typedef enum
{
  BLUETOOTH_COMMAND_FORWARD_PRESS   = 0x11U,
  BLUETOOTH_COMMAND_FORWARD_RELEASE = 0x12U,
  BLUETOOTH_COMMAND_BACKWARD_PRESS  = 0x21U,
  BLUETOOTH_COMMAND_BACKWARD_RELEASE = 0x22U,
  BLUETOOTH_COMMAND_LEFT_PRESS      = 0x31U,
  BLUETOOTH_COMMAND_LEFT_RELEASE    = 0x32U,
  BLUETOOTH_COMMAND_RIGHT_PRESS     = 0x41U,
  BLUETOOTH_COMMAND_RIGHT_RELEASE   = 0x42U,
  BLUETOOTH_COMMAND_STOP_PRESS      = 0x55U
} BluetoothCommand;

typedef enum
{
  BLUETOOTH_PARSER_WAIT_HEADER_1 = 0,
  BLUETOOTH_PARSER_WAIT_HEADER_2,
  BLUETOOTH_PARSER_WAIT_COMMAND,
  BLUETOOTH_PARSER_WAIT_TAIL_1,
  BLUETOOTH_PARSER_WAIT_TAIL_2
} BluetoothParserState;

typedef struct
{
  BluetoothParserState state;
  uint8_t command;
} BluetoothFrameParser;

typedef enum
{
  BLUETOOTH_PARSE_NONE = 0,
  BLUETOOTH_PARSE_FRAME,
  BLUETOOTH_PARSE_ERROR
} BluetoothParseResult;

void BluetoothFrameParser_Reset(BluetoothFrameParser *parser);
BluetoothParseResult BluetoothFrameParser_PushByte(
    BluetoothFrameParser *parser, uint8_t byte, uint8_t *command);

#endif /* BLUETOOTH_FRAME_H */
