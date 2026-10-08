#ifndef TEST_SUPPLEMENT_HAL_H
#define TEST_SUPPLEMENT_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
extern uint32_t fake_now;
extern unsigned fake_rx_arms;
static inline uint32_t HAL_GetTick(void) { return fake_now; }
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ (void)u; (void)b; (void)n; ++fake_rx_arms; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t t)
{ (void)u; (void)b; (void)n; (void)t; return HAL_OK; }
#endif
