/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
#define IRIDIUM_TEST_ENABLE 1
#define IMU_DMA_BUFFER_SIZE 32768U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */

static uint8_t uart7_tx_cmd[] = "AT\r";
static uint8_t uart7_rx_byte = 0;
static uint8_t uart7_rx_buffer[128];
static volatile uint16_t uart7_rx_index = 0;
static volatile uint8_t uart7_rx_line_ready = 0;
static volatile uint8_t uart7_rx_resp_ready = 0;
static uint32_t uart7_last_tx_tick = 0U;
static volatile uint8_t uart7_rx_restart_request = 0U;
static volatile uint32_t uart7_rx_start_fail_count = 0U;
static volatile uint32_t uart7_tx_fail_count = 0U;
static uint8_t imu_dma_buffer[IMU_DMA_BUFFER_SIZE] __attribute__((aligned(32)));
static uint16_t imu_dma_last_pos = 0U;
static uint8_t imu_packet_buf[IMU_PACKET_SIZE];
static volatile uint8_t imu_rx_restart_request = 0U;
static volatile uint32_t imu_rx_start_fail_count = 0U;
static volatile uint32_t imu_packet_count = 0U;
static volatile uint32_t imu_packet_error_count = 0U;
static volatile uint8_t imu_nav_ready = 0U;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART7_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

static void UART7_StartReceiveIT(void);
static void UART7_SendAtCommandIT(void);
static void IMU_ResetPacketState(void);
static void IMU_ProcessDmaBuffer(void);
static void IMU_StartReceiveDMA(void);
static void IMU_InvalidateDCacheRange(uint16_t start, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART7_StartReceiveIT(void)
{
  HAL_StatusTypeDef rx_status = HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);

  if ((rx_status != HAL_OK) && (rx_status != HAL_BUSY))
  {
    uart7_rx_start_fail_count++;
    uart7_rx_restart_request = 1U;
  }
}

static void UART7_SendAtCommandIT(void)
{
  HAL_StatusTypeDef tx_status = HAL_UART_Transmit_IT(&huart7, uart7_tx_cmd, sizeof(uart7_tx_cmd) - 1U);

  if ((tx_status != HAL_OK) && (tx_status != HAL_BUSY))
  {
    uart7_tx_fail_count++;
  }
}

static void IMU_ResetPacketState(void)
{
  /* Reserved for future parser state reset. */
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

  if (huart4.hdmarx == NULL)
  {
    return;
  }

  /* Parse only from the main loop (not from UART/DMA ISRs) to avoid re-entrancy
   * and missed work when a second IRQ returns early. Re-snapshot DMA position
   * in an outer loop so bytes that arrive during parsing are still drained. */
  for (;;)
  {
    current_pos = (uint16_t)(IMU_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx));
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
            imu_nav_ready = 1U;
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

      /* Skip one byte and keep scanning for NAV header (0x0E 0xA2). */
      imu_dma_bytes_total++;
      imu_dma_last_pos = (uint16_t)((imu_dma_last_pos + 1U) % IMU_DMA_BUFFER_SIZE);
      available--;
    }

    /* Optional: drop remaining non-header bytes without counting as packet errors. */
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

    current_pos = (uint16_t)(IMU_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx));
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

#ifdef UART_RXDATA_FLUSH_REQUEST
  __HAL_UART_SEND_REQ(&huart4, UART_RXDATA_FLUSH_REQUEST);
#endif
  __HAL_UART_CLEAR_IDLEFLAG(&huart4);
  status = HAL_UART_Receive_DMA(&huart4, imu_dma_buffer, IMU_DMA_BUFFER_SIZE);
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
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  //Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
//Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  IMU_ResetPacketState();
  imu_dma_last_pos = 0U;
  imu_last_rx_tick = HAL_GetTick();
  imu_last_progress_tick = imu_last_rx_tick;
  imu_dma_prev_pos_dbg = 0U;
  /* Match reference project behavior: keep overrun detection enabled. */
  CLEAR_BIT(huart4.Instance->CR3, USART_CR3_OVRDIS);
  IMU_StartReceiveDMA();

  /* IRIDIUM 9603 power control: PF10 = HIGH */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1000);

#if IRIDIUM_TEST_ENABLE
  UART7_StartReceiveIT();
  UART7_SendAtCommandIT();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IMU_ProcessDmaBuffer();

#if IRIDIUM_TEST_ENABLE
    if ((HAL_GetTick() - uart7_last_tx_tick) >= 1000U)
    {
      uart7_last_tx_tick = HAL_GetTick();
      __disable_irq();
      uart7_rx_index = 0U;
      memset(uart7_rx_buffer, 0, sizeof(uart7_rx_buffer));
      uart7_rx_line_ready = 0U;
      uart7_rx_resp_ready = 0U;
      __enable_irq();
      UART7_SendAtCommandIT();
    }

    if (uart7_rx_restart_request != 0U)
    {
      uart7_rx_restart_request = 0U;
      UART7_StartReceiveIT();
    }
#endif

    if (imu_rx_restart_request != 0U)
    {
      imu_rx_restart_request = 0U;
      IMU_StartReceiveDMA();
    }

    if ((huart4.hdmarx != NULL) &&
        ((HAL_GetTick() - imu_last_progress_tick) > 100U) &&
        (imu_rx_restart_request == 0U))
    {
      /* Recover from silent UART4 DMA stalls with no HAL error callback. */
      HAL_UART_DMAStop(&huart4);
      imu_stall_restart_count++;
      IMU_StartReceiveDMA();
    }
#if IRIDIUM_TEST_ENABLE
    if (uart7_rx_resp_ready != 0U)
    {
      uart7_rx_resp_ready = 0U;
      /* Inspect uart7_rx_buffer in debugger (example: "AT\r\r\nOK\r\n"). */
    }
#endif

    if (imu_nav_ready != 0U)
    {
      /* Avoid toggling shared control pins in fast IMU path. */
      imu_nav_ready = 0U;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  /* Keep default ORE behavior to expose real UART4 overrun conditions. */
  CLEAR_BIT(huart4.Instance->CR3, USART_CR3_OVRDIS);

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 19200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF8 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    /* Parsing runs in the main loop (IMU_ProcessDmaBuffer). */
    return;
  }

  if (huart->Instance == UART7)
  {
    if (uart7_rx_index < (sizeof(uart7_rx_buffer) - 1U))
    {
      uart7_rx_buffer[uart7_rx_index++] = uart7_rx_byte;
      uart7_rx_buffer[uart7_rx_index] = '\0';
    }

    if (uart7_rx_byte == '\n')
    {
      uart7_rx_line_ready = 1U;
    }

    if ((strstr((char *)uart7_rx_buffer, "\r\nOK\r\n") != NULL) ||
        (strstr((char *)uart7_rx_buffer, "\r\nERROR\r\n") != NULL))
    {
      uart7_rx_resp_ready = 1U;
    }

    UART7_StartReceiveIT();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    /* Parsing runs in the main loop (IMU_ProcessDmaBuffer). */
    return;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    HAL_StatusTypeDef restart_status;

    imu_uart4_error_count++;
    imu_uart4_last_error = huart->ErrorCode;
    HAL_UART_DMAStop(&huart4);

    /* Clear UART error flags (especially ORE) before restarting DMA RX */
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
    {
      __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U)
    {
      __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_NEF);
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U)
    {
      __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_FEF);
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U)
    {
      __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_PEF);
    }

#ifdef UART_RXDATA_FLUSH_REQUEST
    __HAL_UART_SEND_REQ(&huart4, UART_RXDATA_FLUSH_REQUEST);
#endif

    IMU_ResetPacketState();
    imu_dma_last_pos = 0U;
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    restart_status = HAL_UART_Receive_DMA(&huart4, imu_dma_buffer, IMU_DMA_BUFFER_SIZE);
    if (restart_status == HAL_OK)
    {
      imu_dma_restart_count++;
      imu_rx_restart_request = 0U;
      imu_dma_prev_pos_dbg = 0U;
      imu_last_progress_tick = HAL_GetTick();
      __HAL_UART_CLEAR_IDLEFLAG(&huart4);
      __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    }
    else
    {
      imu_rx_restart_request = 1U;
    }
    imu_last_rx_tick = HAL_GetTick();
    return;
  }

  if (huart->Instance == UART7)
  {
    uart7_rx_index = 0U;
    memset(uart7_rx_buffer, 0, sizeof(uart7_rx_buffer));
    uart7_rx_restart_request = 1U;
  }
}

void IMU_UART4_IdleIrqHandler(void)
{
  __HAL_UART_CLEAR_IDLEFLAG(&huart4);
  imu_uart4_idle_count++;
  imu_last_rx_tick = HAL_GetTick();
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
