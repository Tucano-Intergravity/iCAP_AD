#include "iridium.h"
#include <string.h>

static UART_HandleTypeDef *s_iridium_uart = NULL;
static uint8_t iridium_tx_cmd[] = "AT\r";
static uint8_t iridium_rx_byte = 0U;
static uint8_t iridium_rx_buffer[128];
static volatile uint16_t iridium_rx_index = 0U;
static volatile uint8_t iridium_rx_line_ready = 0U;
static volatile uint8_t iridium_rx_resp_ready = 0U;
static uint32_t iridium_last_tx_tick = 0U;
static volatile uint8_t iridium_rx_restart_request = 0U;
static volatile uint32_t iridium_rx_start_fail_count = 0U;
static volatile uint32_t iridium_tx_fail_count = 0U;

static void Iridium_StartReceiveIT(void)
{
  HAL_StatusTypeDef rx_status;

  if (s_iridium_uart == NULL)
  {
    return;
  }

  rx_status = HAL_UART_Receive_IT(s_iridium_uart, &iridium_rx_byte, 1U);
  if ((rx_status != HAL_OK) && (rx_status != HAL_BUSY))
  {
    iridium_rx_start_fail_count++;
    iridium_rx_restart_request = 1U;
  }
}

static void Iridium_SendAtCommandIT(void)
{
  HAL_StatusTypeDef tx_status;

  if (s_iridium_uart == NULL)
  {
    return;
  }

  tx_status = HAL_UART_Transmit_IT(s_iridium_uart, iridium_tx_cmd, sizeof(iridium_tx_cmd) - 1U);
  if ((tx_status != HAL_OK) && (tx_status != HAL_BUSY))
  {
    iridium_tx_fail_count++;
  }
}

void Iridium_Init(UART_HandleTypeDef *huart)
{
  s_iridium_uart = huart;
  iridium_rx_index = 0U;
  memset(iridium_rx_buffer, 0, sizeof(iridium_rx_buffer));
  iridium_rx_line_ready = 0U;
  iridium_rx_resp_ready = 0U;
  iridium_last_tx_tick = HAL_GetTick();
  iridium_rx_restart_request = 0U;
  Iridium_StartReceiveIT();
  Iridium_SendAtCommandIT();
}

void Iridium_Poll(void)
{
  if (s_iridium_uart == NULL)
  {
    return;
  }

  if ((HAL_GetTick() - iridium_last_tx_tick) >= 1000U)
  {
    iridium_last_tx_tick = HAL_GetTick();
    __disable_irq();
    iridium_rx_index = 0U;
    memset(iridium_rx_buffer, 0, sizeof(iridium_rx_buffer));
    iridium_rx_line_ready = 0U;
    iridium_rx_resp_ready = 0U;
    __enable_irq();
    Iridium_SendAtCommandIT();
  }

  if (iridium_rx_restart_request != 0U)
  {
    iridium_rx_restart_request = 0U;
    Iridium_StartReceiveIT();
  }
}

bool Iridium_PollNewData(const uint8_t **out_data)
{
  bool has_new_data = false;

  Iridium_Poll();
  if (iridium_rx_resp_ready != 0U)
  {
    if (out_data != NULL)
    {
      *out_data = iridium_rx_buffer;
    }
    iridium_rx_resp_ready = 0U;
    has_new_data = true;
  }

  return has_new_data;
}

void Iridium_HandleRxCplt(UART_HandleTypeDef *huart)
{
  if ((s_iridium_uart == NULL) || (huart != s_iridium_uart))
  {
    return;
  }

  if (iridium_rx_index < (sizeof(iridium_rx_buffer) - 1U))
  {
    iridium_rx_buffer[iridium_rx_index++] = iridium_rx_byte;
    iridium_rx_buffer[iridium_rx_index] = '\0';
  }

  if (iridium_rx_byte == '\n')
  {
    iridium_rx_line_ready = 1U;
  }

  if ((strstr((char *)iridium_rx_buffer, "\r\nOK\r\n") != NULL) ||
      (strstr((char *)iridium_rx_buffer, "\r\nERROR\r\n") != NULL))
  {
    iridium_rx_resp_ready = 1U;
  }

  Iridium_StartReceiveIT();
}

void Iridium_HandleUartError(UART_HandleTypeDef *huart)
{
  if ((s_iridium_uart == NULL) || (huart != s_iridium_uart))
  {
    return;
  }

  iridium_rx_index = 0U;
  memset(iridium_rx_buffer, 0, sizeof(iridium_rx_buffer));
  iridium_rx_restart_request = 1U;
}

bool Iridium_IsResponseReady(void)
{
  return (iridium_rx_resp_ready != 0U);
}

const uint8_t *Iridium_GetRxBuffer(void)
{
  return iridium_rx_buffer;
}

uint16_t Iridium_GetRxLength(void)
{
  return iridium_rx_index;
}

void Iridium_ClearResponseReady(void)
{
  iridium_rx_resp_ready = 0U;
}
