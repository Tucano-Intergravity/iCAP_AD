#include "imu.h"
#include <stddef.h>
#include <string.h>

#define IMU_DMA_BUFFER_SIZE 32768U

static UART_HandleTypeDef *s_imu_uart = NULL;
static uint8_t imu_dma_buffer[IMU_DMA_BUFFER_SIZE] __attribute__((aligned(32)));
static uint16_t imu_dma_last_pos = 0U;
static uint8_t imu_packet_buf[IMU_PACKET_SIZE];
static volatile uint8_t imu_rx_restart_request = 0U;
static volatile uint32_t imu_rx_start_fail_count = 0U;
static volatile uint32_t imu_packet_count = 0U;
static volatile uint32_t imu_packet_error_count = 0U;
static IMU_Data_t imu_last_data;
static volatile uint16_t imu_dma_current_pos_dbg = 0U;
static volatile uint32_t imu_dma_bytes_total = 0U;
static volatile uint32_t imu_dma_restart_count = 0U;
static volatile uint32_t imu_uart4_error_count = 0U;
static volatile uint32_t imu_uart4_last_error = 0U;
static volatile uint32_t imu_last_rx_tick = 0U;
static volatile uint32_t imu_uart4_idle_count = 0U;
static volatile uint32_t imu_stall_restart_count = 0U;
static volatile uint32_t imu_non_nav_header_count = 0U;
static volatile uint32_t imu_last_progress_tick = 0U;
static volatile uint16_t imu_dma_prev_pos_dbg = 0U;
static volatile uint32_t imu_msg_id_ctrl_count = 0U;

static void IMU_ResetPacketState(void);
static void IMU_InvalidateDCacheRange(uint16_t start, uint16_t length);
static void IMU_ProcessDmaBuffer(void);
static void IMU_StartReceiveDMA(void);
static void IMU_ServiceRecovery(void);

static void IMU_ResetPacketState(void)
{
  (void)0;
}

static void IMU_InvalidateDCacheRange(uint16_t start, uint16_t length)
{
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uintptr_t addr;
  uintptr_t aligned_addr;
  uintptr_t end_addr;
  int32_t invalidate_len;

  if ((length == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  addr = (uintptr_t)&imu_dma_buffer[start];
  aligned_addr = addr & ~(uintptr_t)31U;
  end_addr = (addr + (uintptr_t)length + (uintptr_t)31U) & ~(uintptr_t)31U;
  invalidate_len = (int32_t)(end_addr - aligned_addr);

  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_addr, invalidate_len);
#else
  (void)start;
  (void)length;
#endif
}

static void IMU_ProcessDmaBuffer(void)
{
  uint16_t current_pos;
  uint16_t available;
  uint16_t pos;
  uint16_t next_pos;
  uint8_t msg_id;
  uint16_t remaining;

  if ((s_imu_uart == NULL) || (s_imu_uart->hdmarx == NULL))
  {
    return;
  }

  for (;;)
  {
    current_pos = (uint16_t)(IMU_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(s_imu_uart->hdmarx));
    imu_dma_current_pos_dbg = current_pos;
    if (current_pos != imu_dma_prev_pos_dbg)
    {
      imu_dma_prev_pos_dbg = current_pos;
      imu_last_progress_tick = HAL_GetTick();
    }

    if (current_pos >= imu_dma_last_pos)
    {
      available = (uint16_t)(current_pos - imu_dma_last_pos);
      IMU_InvalidateDCacheRange(imu_dma_last_pos, available);
    }
    else
    {
      available = (uint16_t)((IMU_DMA_BUFFER_SIZE - imu_dma_last_pos) + current_pos);
      IMU_InvalidateDCacheRange(imu_dma_last_pos, (uint16_t)(IMU_DMA_BUFFER_SIZE - imu_dma_last_pos));
      IMU_InvalidateDCacheRange(0U, current_pos);
    }

    if (available == 0U)
    {
      return;
    }

    while (available >= IMU_PACKET_SIZE)
    {
      pos = imu_dma_last_pos;
      next_pos = (uint16_t)((pos + 1U) % IMU_DMA_BUFFER_SIZE);
      msg_id = imu_dma_buffer[next_pos];

      if (imu_dma_buffer[pos] == IMU_SYNC_BYTE)
      {
        if (msg_id == IMU_MSG_ID_NAV)
        {
          for (uint16_t i = 0U; i < IMU_PACKET_SIZE; i++)
          {
            imu_packet_buf[i] = imu_dma_buffer[(pos + i) % IMU_DMA_BUFFER_SIZE];
          }

          if (IMU_ParsePacket(imu_packet_buf, &imu_last_data))
          {
            imu_last_data.timestamp = HAL_GetTick();
            imu_packet_count++;
            imu_last_rx_tick = HAL_GetTick();
            imu_dma_bytes_total += IMU_PACKET_SIZE;
            imu_dma_last_pos = (uint16_t)((pos + IMU_PACKET_SIZE) % IMU_DMA_BUFFER_SIZE);
            available = (uint16_t)(available - IMU_PACKET_SIZE);
            continue;
          }

          imu_packet_error_count++;
        }
        else
        {
          imu_non_nav_header_count++;
          if (msg_id == IMU_MSG_ID_CTRL)
          {
            imu_msg_id_ctrl_count++;
          }
        }
      }

      imu_dma_bytes_total++;
      imu_dma_last_pos = (uint16_t)((imu_dma_last_pos + 1U) % IMU_DMA_BUFFER_SIZE);
      available--;
    }

    while (available > 0U)
    {
      if (imu_dma_buffer[imu_dma_last_pos] == IMU_SYNC_BYTE)
      {
        break;
      }

      imu_dma_bytes_total++;
      imu_dma_last_pos = (uint16_t)((imu_dma_last_pos + 1U) % IMU_DMA_BUFFER_SIZE);
      available--;
    }

    current_pos = (uint16_t)(IMU_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(s_imu_uart->hdmarx));
    if (current_pos >= imu_dma_last_pos)
    {
      remaining = (uint16_t)(current_pos - imu_dma_last_pos);
    }
    else
    {
      remaining = (uint16_t)((IMU_DMA_BUFFER_SIZE - imu_dma_last_pos) + current_pos);
    }

    if (remaining < IMU_PACKET_SIZE)
    {
      return;
    }
  }
}

static void IMU_StartReceiveDMA(void)
{
  HAL_StatusTypeDef status;

  if (s_imu_uart == NULL)
  {
    return;
  }

#ifdef UART_RXDATA_FLUSH_REQUEST
  __HAL_UART_SEND_REQ(s_imu_uart, UART_RXDATA_FLUSH_REQUEST);
#endif
  __HAL_UART_CLEAR_IDLEFLAG(s_imu_uart);
  status = HAL_UART_Receive_DMA(s_imu_uart, imu_dma_buffer, IMU_DMA_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    imu_rx_start_fail_count++;
    imu_rx_restart_request = 1U;
  }
  else
  {
    imu_dma_restart_count++;
    imu_dma_prev_pos_dbg = 0U;
    imu_last_progress_tick = HAL_GetTick();
    __HAL_UART_CLEAR_IDLEFLAG(s_imu_uart);
    __HAL_UART_ENABLE_IT(s_imu_uart, UART_IT_IDLE);
  }
}

static void IMU_ServiceRecovery(void)
{
  if (imu_rx_restart_request != 0U)
  {
    imu_rx_restart_request = 0U;
    IMU_StartReceiveDMA();
  }

  if ((s_imu_uart != NULL) &&
      (s_imu_uart->hdmarx != NULL) &&
      ((HAL_GetTick() - imu_last_progress_tick) > 100U) &&
      (imu_rx_restart_request == 0U))
  {
    HAL_UART_DMAStop(s_imu_uart);
    imu_stall_restart_count++;
    IMU_StartReceiveDMA();
  }
}

uint16_t IMU_CalculateChecksum(const uint8_t *data, uint16_t length)
{
  uint16_t sum = 0U;

  if ((data == NULL) || (length < IMU_PACKET_SIZE))
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < (IMU_PACKET_SIZE - 2U); i += 2U)
  {
    uint16_t word = (uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8);
    sum = (uint16_t)(sum + word);
  }

  return sum;
}

bool IMU_ParsePacket(const uint8_t *packet_data, IMU_Data_t *imu_data)
{
  const imu_raw_packet_t *packet = (const imu_raw_packet_t *)packet_data;
  const float angular_rate_lsb = 1.0f / 2048.0f;
  const float linear_accel_lsb = (1.0f / 32.0f) * 0.3048f;
  const float delta_angle_lsb = 1.0f / 8589934592.0f;
  const float delta_velocity_lsb = (1.0f / 134217728.0f) * 0.3048f;
  uint16_t calculated_checksum;
  uint16_t received_checksum;

  if ((packet_data == NULL) || (imu_data == NULL))
  {
    return false;
  }

  if ((packet->sync != IMU_SYNC_BYTE) || (packet->msg_id != IMU_MSG_ID_NAV))
  {
    return false;
  }

  calculated_checksum = IMU_CalculateChecksum(packet_data, IMU_PACKET_SIZE);
  received_checksum = (uint16_t)packet_data[IMU_PACKET_SIZE - 2U] |
                      ((uint16_t)packet_data[IMU_PACKET_SIZE - 1U] << 8);

  if (calculated_checksum != received_checksum)
  {
    return false;
  }

  imu_data->angular_rate_x = (float)packet->angular_rate_x * angular_rate_lsb;
  imu_data->angular_rate_y = (float)packet->angular_rate_y * angular_rate_lsb;
  imu_data->angular_rate_z = (float)packet->angular_rate_z * angular_rate_lsb;
  imu_data->linear_accel_x = (float)packet->linear_accel_x * linear_accel_lsb;
  imu_data->linear_accel_y = (float)packet->linear_accel_y * linear_accel_lsb;
  imu_data->linear_accel_z = (float)packet->linear_accel_z * linear_accel_lsb;
  imu_data->status_word = packet->status_word;
  imu_data->delta_angle_x = (float)packet->delta_angle_x * delta_angle_lsb;
  imu_data->delta_angle_y = (float)packet->delta_angle_y * delta_angle_lsb;
  imu_data->delta_angle_z = (float)packet->delta_angle_z * delta_angle_lsb;
  imu_data->delta_velocity_x = (float)packet->delta_velocity_x * delta_velocity_lsb;
  imu_data->delta_velocity_y = (float)packet->delta_velocity_y * delta_velocity_lsb;
  imu_data->delta_velocity_z = (float)packet->delta_velocity_z * delta_velocity_lsb;
  imu_data->timestamp = 0U;

  return true;
}

bool IMU_IsValidMessageID(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_CTRL) || (msg_id == IMU_MSG_ID_NAV);
}

bool IMU_IsControlPacket(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_CTRL);
}

bool IMU_IsNavigationPacket(uint8_t msg_id)
{
  return (msg_id == IMU_MSG_ID_NAV);
}

void IMU_Init(UART_HandleTypeDef *huart)
{
  s_imu_uart = huart;
  IMU_ResetPacketState();
  imu_dma_last_pos = 0U;
  imu_last_rx_tick = HAL_GetTick();
  imu_last_progress_tick = imu_last_rx_tick;
  imu_dma_prev_pos_dbg = 0U;
  if (s_imu_uart != NULL)
  {
    CLEAR_BIT(s_imu_uart->Instance->CR3, USART_CR3_OVRDIS);
  }
  IMU_StartReceiveDMA();
}

bool IMU_PollNewData(IMU_Data_t *out_data)
{
  static uint32_t last_consumed_packet_count = 0U;
  bool has_new_data = false;

  IMU_ProcessDmaBuffer();
  IMU_ServiceRecovery();

  __disable_irq();
  if (imu_packet_count != last_consumed_packet_count)
  {
    last_consumed_packet_count = imu_packet_count;
    if (out_data != NULL)
    {
      *out_data = imu_last_data;
    }
    has_new_data = true;
  }
  __enable_irq();

  return has_new_data;
}

void IMU_HandleUartError(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef restart_status;

  if ((s_imu_uart == NULL) || (huart != s_imu_uart))
  {
    return;
  }

  imu_uart4_error_count++;
  imu_uart4_last_error = huart->ErrorCode;
  HAL_UART_DMAStop(s_imu_uart);

  if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(s_imu_uart, UART_CLEAR_OREF);
  }
  if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(s_imu_uart, UART_CLEAR_NEF);
  }
  if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(s_imu_uart, UART_CLEAR_FEF);
  }
  if ((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(s_imu_uart, UART_CLEAR_PEF);
  }

#ifdef UART_RXDATA_FLUSH_REQUEST
  __HAL_UART_SEND_REQ(s_imu_uart, UART_RXDATA_FLUSH_REQUEST);
#endif

  IMU_ResetPacketState();
  imu_dma_last_pos = 0U;
  __HAL_UART_CLEAR_IDLEFLAG(s_imu_uart);
  restart_status = HAL_UART_Receive_DMA(s_imu_uart, imu_dma_buffer, IMU_DMA_BUFFER_SIZE);
  if (restart_status == HAL_OK)
  {
    imu_dma_restart_count++;
    imu_rx_restart_request = 0U;
    imu_dma_prev_pos_dbg = 0U;
    imu_last_progress_tick = HAL_GetTick();
    __HAL_UART_CLEAR_IDLEFLAG(s_imu_uart);
    __HAL_UART_ENABLE_IT(s_imu_uart, UART_IT_IDLE);
  }
  else
  {
    imu_rx_restart_request = 1U;
  }
  imu_last_rx_tick = HAL_GetTick();
}

void IMU_UART4_IdleIrqHandler(void)
{
  if (s_imu_uart == NULL)
  {
    return;
  }
  __HAL_UART_CLEAR_IDLEFLAG(s_imu_uart);
  imu_uart4_idle_count++;
  imu_last_rx_tick = HAL_GetTick();
}
