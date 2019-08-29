#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void USART1_Init(uint32_t BaudRate);
void UART4_Init(uint32_t BaudRate);
void USART3_Init(uint32_t BaudRate);
void UART5_Init(uint32_t BaudRate);
void USART6_Init(uint32_t BaudRate);
void USART2_Init(uint32_t BaudRate);

void USART_OUT(USART_TypeDef *USARTx, const uint8_t *Data, ...);
char *itoa(int value, char *string, int radix);
void UART5_OUT(const uint8_t *Data, ...);

#endif
