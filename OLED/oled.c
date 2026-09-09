#include "oled.h"

#include <stddef.h>

#define OLED_WIDTH 128U
#define OLED_PAGES 8U
#define OLED_I2C_TIMEOUT_MS 20U

static I2C_HandleTypeDef *oled_i2c;

static const uint8_t oled_digits[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}};

static const uint8_t oled_uppercase[26][5] = {
    {0x7CU, 0x12U, 0x11U, 0x12U, 0x7CU}, {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}, {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}};

static const uint8_t *OLED_Glyph(char character)
{
  static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
  static const uint8_t minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t plus[5] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
  static const uint8_t dot[5] = {0U, 0x60U, 0x60U, 0U, 0U};
  static const uint8_t unknown[5] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};

  if ((character >= '0') && (character <= '9'))
  {
    return oled_digits[character - '0'];
  }
  if ((character >= 'A') && (character <= 'Z'))
  {
    return oled_uppercase[character - 'A'];
  }
  if ((character >= 'a') && (character <= 'z'))
  {
    return oled_uppercase[character - 'a'];
  }
  if (character == ':') return colon;
  if (character == '-') return minus;
  if (character == '+') return plus;
  if (character == '.') return dot;
  if (character == ' ') return blank;
  return unknown;
}

static HAL_StatusTypeDef OLED_SendCommand(uint8_t command)
{
  uint8_t packet[2] = {0x00U, command};
  return HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDRESS, packet, sizeof(packet), OLED_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef OLED_SetPosition(uint8_t column, uint8_t page)
{
  HAL_StatusTypeDef status;
  status = OLED_SendCommand((uint8_t)(0xB0U + page));
  if (status != HAL_OK) return status;
  status = OLED_SendCommand((uint8_t)(0x00U + (column & 0x0FU)));
  if (status != HAL_OK) return status;
  return OLED_SendCommand((uint8_t)(0x10U + ((column >> 4U) & 0x0FU)));
}

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c)
{
  static const uint8_t init_commands[] = {0xAEU, 0x20U, 0x02U, 0xB0U, 0xC8U, 0x00U,
      0x10U, 0x40U, 0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
      0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU, 0x40U, 0x8DU,
      0x14U, 0xAFU};
  uint32_t index;

  if (hi2c == NULL) return HAL_ERROR;
  oled_i2c = hi2c;
  HAL_Delay(100U);
  for (index = 0U; index < (sizeof(init_commands) / sizeof(init_commands[0])); ++index)
  {
    if (OLED_SendCommand(init_commands[index]) != HAL_OK) return HAL_ERROR;
  }
  return OLED_Clear();
}

HAL_StatusTypeDef OLED_Clear(void)
{
  uint8_t packet[OLED_WIDTH + 1U] = {0U};
  uint8_t page;

  if (oled_i2c == NULL) return HAL_ERROR;
  packet[0] = 0x40U;
  for (page = 0U; page < OLED_PAGES; ++page)
  {
    if (OLED_SetPosition(0U, page) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDRESS, packet, sizeof(packet), OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef OLED_WriteString(uint8_t column, uint8_t page, const char *text)
{
  uint8_t packet[OLED_WIDTH + 1U];
  uint8_t length = 1U;

  if ((oled_i2c == NULL) || (text == NULL) || (page >= OLED_PAGES) || (column >= OLED_WIDTH)) return HAL_ERROR;
  packet[0] = 0x40U;
  while ((*text != '\0') && ((uint16_t)column + length + 5U <= OLED_WIDTH))
  {
    const uint8_t *glyph = OLED_Glyph(*text++);
    uint8_t glyph_index;
    for (glyph_index = 0U; glyph_index < 5U; ++glyph_index) packet[length++] = glyph[glyph_index];
    packet[length++] = 0U;
  }
  if (OLED_SetPosition(column, page) != HAL_OK) return HAL_ERROR;
  return HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDRESS, packet, length, OLED_I2C_TIMEOUT_MS);
}
