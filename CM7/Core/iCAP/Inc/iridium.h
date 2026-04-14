#ifndef IRIDIUM_H_
#define IRIDIUM_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

void Iridium_Init(UART_HandleTypeDef *huart);
void Iridium_Poll(void);
bool Iridium_PollNewData(const uint8_t **out_data);
void Iridium_HandleRxCplt(UART_HandleTypeDef *huart);
void Iridium_HandleUartError(UART_HandleTypeDef *huart);

bool Iridium_IsResponseReady(void);
const uint8_t *Iridium_GetRxBuffer(void);
uint16_t Iridium_GetRxLength(void);
void Iridium_ClearResponseReady(void);

#endif /* IRIDIUM_H_ */
