#ifndef TEST_BOARD_IO_HAL_H
#define TEST_BOARD_IO_HAL_H

#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t pins; } GPIO_TypeDef;
typedef struct { volatile uint32_t SR, DR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef struct { uint32_t CYCCNT; } DWT_Type;

extern GPIO_TypeDef test_gpio_c, test_gpio_e;
#define GPIOC (&test_gpio_c)
#define GPIOE (&test_gpio_e)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define USART_SR_TXE (1U << 7)
#define USART_SR_TC (1U << 6)
#define DWT BoardIoTest_ReadCounter()

DWT_Type *BoardIoTest_ReadCounter(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);

#endif
