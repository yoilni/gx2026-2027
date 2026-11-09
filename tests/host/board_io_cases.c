#include "board_led.h"
#include "switch_input.h"
#include "uart_port.h"

#include <assert.h>
#include <stdio.h>

GPIO_TypeDef test_gpio_c, test_gpio_e;
static DWT_Type test_counter;
static unsigned cycle_reads;

DWT_Type *BoardIoTest_ReadCounter(void)
{
  ++test_counter.CYCCNT;
  ++cycle_reads;
  return &test_counter;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  assert(port == GPIOE);
  return (port->pins & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
  assert(port == GPIOC && pin == GPIO_PIN_13);
  port->pins ^= pin;
}

static void check_inputs(void)
{
  GPIO_PinState (*const readers[])(void) = {
    SwitchInput_ReadStart, SwitchInput_ReadTeam,
    SwitchInput_ReadZoneMsb, SwitchInput_ReadZoneLsb,
  };
  for (unsigned i = 0U; i < 4U; ++i)
  {
    test_gpio_e.pins = 1UL << (12U + i);
    for (unsigned j = 0U; j < 4U; ++j)
    {
      assert(readers[j]() == (i == j ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
  }
  BoardLed_Toggle();
  assert(test_gpio_c.pins == GPIO_PIN_13);
  BoardLed_Toggle();
  assert(test_gpio_c.pins == 0U);
}

static void check_boot_uart(void)
{
  USART_TypeDef registers = { .SR = USART_SR_TXE | USART_SR_TC };
  UART_HandleTypeDef port = { .Instance = &registers };
  UartPort_WriteBootText(&port, "AB", 0U, 4U);
  assert(registers.DR == 'B' && cycle_reads == 0U);

  registers.SR = 0U;
  registers.DR = 0U;
  test_counter.CYCCNT = UINT32_MAX - 1U;
  UartPort_WriteBootText(&port, "A", test_counter.CYCCNT, 4U);
  assert(registers.DR == 0U && cycle_reads == 4U && test_counter.CYCCNT == 2U);

  cycle_reads = 0U;
  test_counter.CYCCNT = 50U;
  registers.SR = USART_SR_TXE;
  UartPort_WriteBootText(&port, "A", 47U, 4U);
  assert(registers.DR == 'A' && cycle_reads == 1U); /* Same budget at TC. */
}

int main(void)
{
  check_inputs();
  check_boot_uart();
  puts("PASS: PE12..15 mapping, PC13 LED, early UART TXE/TC deadlines and cycle wrap");
  return 0;
}
