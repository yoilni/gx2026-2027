#include "bluetooth_frame.h"

#include <stddef.h>

void BluetoothFrameParser_Reset(BluetoothFrameParser *parser)
{
  if (parser != NULL)
  {
    parser->state = BLUETOOTH_PARSER_WAIT_HEADER_1;
    parser->command = 0U;
  }
}

BluetoothParseResult BluetoothFrameParser_PushByte(
    BluetoothFrameParser *parser, uint8_t byte, uint8_t *command)
{
  if ((parser == NULL) || (command == NULL))
  {
    return BLUETOOTH_PARSE_ERROR;
  }

  switch (parser->state)
  {
    case BLUETOOTH_PARSER_WAIT_HEADER_1:
      if (byte == BLUETOOTH_FRAME_HEADER_1)
      {
        parser->state = BLUETOOTH_PARSER_WAIT_HEADER_2;
      }
      return BLUETOOTH_PARSE_NONE;

    case BLUETOOTH_PARSER_WAIT_HEADER_2:
      if (byte == BLUETOOTH_FRAME_HEADER_2)
      {
        parser->state = BLUETOOTH_PARSER_WAIT_COMMAND;
        return BLUETOOTH_PARSE_NONE;
      }
      parser->state = (byte == BLUETOOTH_FRAME_HEADER_1)
                          ? BLUETOOTH_PARSER_WAIT_HEADER_2
                          : BLUETOOTH_PARSER_WAIT_HEADER_1;
      return BLUETOOTH_PARSE_ERROR;

    case BLUETOOTH_PARSER_WAIT_COMMAND:
      parser->command = byte;
      parser->state = BLUETOOTH_PARSER_WAIT_TAIL_1;
      return BLUETOOTH_PARSE_NONE;

    case BLUETOOTH_PARSER_WAIT_TAIL_1:
      if (byte == BLUETOOTH_FRAME_TAIL_1)
      {
        parser->state = BLUETOOTH_PARSER_WAIT_TAIL_2;
        return BLUETOOTH_PARSE_NONE;
      }
      parser->state = (byte == BLUETOOTH_FRAME_HEADER_1)
                          ? BLUETOOTH_PARSER_WAIT_HEADER_2
                          : BLUETOOTH_PARSER_WAIT_HEADER_1;
      return BLUETOOTH_PARSE_ERROR;

    case BLUETOOTH_PARSER_WAIT_TAIL_2:
      if (byte == BLUETOOTH_FRAME_TAIL_2)
      {
        *command = parser->command;
        BluetoothFrameParser_Reset(parser);
        return BLUETOOTH_PARSE_FRAME;
      }
      parser->state = (byte == BLUETOOTH_FRAME_HEADER_1)
                          ? BLUETOOTH_PARSER_WAIT_HEADER_2
                          : BLUETOOTH_PARSER_WAIT_HEADER_1;
      return BLUETOOTH_PARSE_ERROR;

    default:
      BluetoothFrameParser_Reset(parser);
      return BLUETOOTH_PARSE_ERROR;
  }
}
