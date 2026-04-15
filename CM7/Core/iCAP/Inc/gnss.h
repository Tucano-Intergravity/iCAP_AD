#ifndef GNSS_H_
#define GNSS_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef struct
{
  char solution_status[24];
  char position_type[24];
  double latitude_deg;
  double longitude_deg;
  double height_msl_m;
  float undulation_m;
  char datum[16];
  float latitude_sigma_m;
  float longitude_sigma_m;
  float height_sigma_m;
  char base_station_id[8];
  float diff_age_s;
  float sol_age_s;
  uint8_t num_svs_tracked;
  uint8_t num_svs_in_solution;
  uint8_t num_l1_svs_in_solution;
  uint8_t num_multi_svs_in_solution;
  uint8_t reserved_hex;
  uint8_t ext_solution_status_hex;
  uint8_t galileo_beidou_mask_hex;
  uint8_t gps_glonass_mask_hex;
  uint32_t rx_tick_ms;
} GNSS_BestPosA_t;

void GNSS_Init(UART_HandleTypeDef *huart);
bool GNSS_ParseBestPosA(const uint8_t *line, GNSS_BestPosA_t *out_data);
bool GNSS_Process(GNSS_BestPosA_t *out_data);
HAL_StatusTypeDef GNSS_SendCommand(const char *command);
void GNSS_ConfigureBestPosA2Hz(void);
void GNSS_HandleRxCplt(UART_HandleTypeDef *huart);
void GNSS_HandleUartError(UART_HandleTypeDef *huart);

#endif /* GNSS_H_ */
