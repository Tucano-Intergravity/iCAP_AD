#include "gnss.h"
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *s_gnss_uart = NULL;
static uint8_t gnss_rx_byte = 0U;
static uint8_t gnss_line_buffer[256];
static volatile uint16_t gnss_line_index = 0U;
static uint8_t gnss_bestposa_buffer[256];
static volatile uint16_t gnss_bestposa_length = 0U;
static volatile uint8_t gnss_bestposa_ready = 0U;
static volatile uint8_t gnss_rx_restart_request = 0U;
static volatile uint32_t gnss_rx_start_fail_count = 0U;
static volatile uint32_t gnss_rx_overflow_count = 0U;
static volatile uint32_t gnss_command_tx_fail_count = 0U;
static volatile uint32_t gnss_bestposa_count = 0U;
static volatile uint32_t gnss_other_line_count = 0U;
static volatile uint8_t gnss_config_command_sent = 0U;
static volatile uint32_t gnss_parse_fail_count = 0U;

static size_t GNSS_StrnlenSafe(const char *str, size_t max_len)
{
  size_t i;

  if (str == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < max_len; i++)
  {
    if (str[i] == '\0')
    {
      break;
    }
  }

  return i;
}

static void GNSS_CopyToken(char *dest, size_t dest_size, const char *src)
{
  size_t copy_len;

  if ((dest == NULL) || (dest_size == 0U))
  {
    return;
  }

  if (src == NULL)
  {
    dest[0] = '\0';
    return;
  }

  copy_len = GNSS_StrnlenSafe(src, dest_size - 1U);
  memcpy(dest, src, copy_len);
  dest[copy_len] = '\0';

  if ((copy_len >= 2U) && (dest[0] == '"') && (dest[copy_len - 1U] == '"'))
  {
    memmove(dest, &dest[1], copy_len - 2U);
    dest[copy_len - 2U] = '\0';
  }
}

static uint16_t GNSS_SplitCsv(char *text, char *tokens[], uint16_t max_tokens)
{
  uint16_t count = 0U;
  char *p = text;

  if ((text == NULL) || (tokens == NULL) || (max_tokens == 0U))
  {
    return 0U;
  }

  tokens[count++] = p;
  while ((*p != '\0') && (count < max_tokens))
  {
    if (*p == ',')
    {
      *p = '\0';
      tokens[count++] = p + 1;
    }
    p++;
  }

  return count;
}

static bool GNSS_ParseDouble(const char *text, double *out_value)
{
  char *end_ptr;
  double value;

  if ((text == NULL) || (out_value == NULL))
  {
    return false;
  }

  value = strtod(text, &end_ptr);
  if ((end_ptr == text) || (*end_ptr != '\0'))
  {
    return false;
  }

  *out_value = value;
  return true;
}

static bool GNSS_ParseFloat(const char *text, float *out_value)
{
  double tmp_value;

  if ((text == NULL) || (out_value == NULL))
  {
    return false;
  }

  if (!GNSS_ParseDouble(text, &tmp_value))
  {
    return false;
  }

  *out_value = (float)tmp_value;
  return true;
}

static bool GNSS_ParseU8(const char *text, uint8_t *out_value, int base)
{
  unsigned long value;
  char *end_ptr;

  if ((text == NULL) || (out_value == NULL))
  {
    return false;
  }

  value = strtoul(text, &end_ptr, base);
  if ((end_ptr == text) || (*end_ptr != '\0') || (value > 255UL))
  {
    return false;
  }

  *out_value = (uint8_t)value;
  return true;
}

static bool GNSS_IsBestPosALine(const uint8_t *line)
{
  if (line == NULL)
  {
    return false;
  }

  return (strstr((const char *)line, "BESTPOSA,") != NULL);
}

static void GNSS_ProcessCompletedLine(void)
{
  uint16_t copy_len;

  if (!GNSS_IsBestPosALine(gnss_line_buffer))
  {
    gnss_other_line_count++;
    return;
  }

  copy_len = gnss_line_index;
  if (copy_len >= sizeof(gnss_bestposa_buffer))
  {
    copy_len = (uint16_t)(sizeof(gnss_bestposa_buffer) - 1U);
  }

  memcpy(gnss_bestposa_buffer, gnss_line_buffer, copy_len);
  gnss_bestposa_buffer[copy_len] = '\0';
  gnss_bestposa_length = copy_len;
  gnss_bestposa_ready = 1U;
  gnss_bestposa_count++;
}

static void GNSS_StartReceiveIT(void)
{
  HAL_StatusTypeDef status;

  if (s_gnss_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(s_gnss_uart, &gnss_rx_byte, 1U);
  if ((status != HAL_OK) && (status != HAL_BUSY))
  {
    gnss_rx_start_fail_count++;
    gnss_rx_restart_request = 1U;
  }
}

void GNSS_Init(UART_HandleTypeDef *huart)
{
  s_gnss_uart = huart;
  gnss_line_index = 0U;
  memset(gnss_line_buffer, 0, sizeof(gnss_line_buffer));
  memset(gnss_bestposa_buffer, 0, sizeof(gnss_bestposa_buffer));
  gnss_bestposa_length = 0U;
  gnss_bestposa_ready = 0U;
  gnss_rx_restart_request = 0U;
  GNSS_StartReceiveIT();
}

static void GNSS_Poll(void)
{
  if (s_gnss_uart == NULL)
  {
    return;
  }

  if (gnss_rx_restart_request != 0U)
  {
    gnss_rx_restart_request = 0U;
    GNSS_StartReceiveIT();
  }
}

static bool GNSS_PollNewData(const uint8_t **out_data)
{
  bool has_new_data = false;

  GNSS_Poll();

  if (gnss_bestposa_ready != 0U)
  {
    if (out_data != NULL)
    {
      *out_data = gnss_bestposa_buffer;
    }
    gnss_bestposa_ready = 0U;
    has_new_data = true;
  }

  return has_new_data;
}

bool GNSS_ParseBestPosA(const uint8_t *line, GNSS_BestPosA_t *out_data)
{
  char local_line[256];
  char *payload;
  char *crc_sep;
  char *tokens[24];
  uint16_t token_count;
  size_t copy_len;

  if ((line == NULL) || (out_data == NULL))
  {
    return false;
  }

  copy_len = GNSS_StrnlenSafe((const char *)line, sizeof(local_line) - 1U);
  memcpy(local_line, line, copy_len);
  local_line[copy_len] = '\0';

  payload = strchr(local_line, ';');
  if (payload == NULL)
  {
    return false;
  }
  payload++;

  crc_sep = strchr(payload, '*');
  if (crc_sep != NULL)
  {
    *crc_sep = '\0';
  }

  token_count = GNSS_SplitCsv(payload, tokens, (uint16_t)(sizeof(tokens) / sizeof(tokens[0])));
  if (token_count < 21U)
  {
    return false;
  }

  memset(out_data, 0, sizeof(*out_data));
  GNSS_CopyToken(out_data->solution_status, sizeof(out_data->solution_status), tokens[0]);
  GNSS_CopyToken(out_data->position_type, sizeof(out_data->position_type), tokens[1]);
  GNSS_CopyToken(out_data->datum, sizeof(out_data->datum), tokens[6]);
  GNSS_CopyToken(out_data->base_station_id, sizeof(out_data->base_station_id), tokens[10]);

  if (!GNSS_ParseDouble(tokens[2], &out_data->latitude_deg) ||
      !GNSS_ParseDouble(tokens[3], &out_data->longitude_deg) ||
      !GNSS_ParseDouble(tokens[4], &out_data->height_msl_m) ||
      !GNSS_ParseFloat(tokens[5], &out_data->undulation_m) ||
      !GNSS_ParseFloat(tokens[7], &out_data->latitude_sigma_m) ||
      !GNSS_ParseFloat(tokens[8], &out_data->longitude_sigma_m) ||
      !GNSS_ParseFloat(tokens[9], &out_data->height_sigma_m) ||
      !GNSS_ParseFloat(tokens[11], &out_data->diff_age_s) ||
      !GNSS_ParseFloat(tokens[12], &out_data->sol_age_s) ||
      !GNSS_ParseU8(tokens[13], &out_data->num_svs_tracked, 10) ||
      !GNSS_ParseU8(tokens[14], &out_data->num_svs_in_solution, 10) ||
      !GNSS_ParseU8(tokens[15], &out_data->num_l1_svs_in_solution, 10) ||
      !GNSS_ParseU8(tokens[16], &out_data->num_multi_svs_in_solution, 10) ||
      !GNSS_ParseU8(tokens[17], &out_data->reserved_hex, 16) ||
      !GNSS_ParseU8(tokens[18], &out_data->ext_solution_status_hex, 16) ||
      !GNSS_ParseU8(tokens[19], &out_data->galileo_beidou_mask_hex, 16) ||
      !GNSS_ParseU8(tokens[20], &out_data->gps_glonass_mask_hex, 16))
  {
    return false;
  }

  out_data->rx_tick_ms = HAL_GetTick();
  return true;
}

bool GNSS_Process(GNSS_BestPosA_t *out_data)
{
  const uint8_t *line_data = NULL;

  if (!GNSS_PollNewData(&line_data))
  {
    return false;
  }

  if ((out_data == NULL) || !GNSS_ParseBestPosA(line_data, out_data))
  {
    gnss_parse_fail_count++;
    return false;
  }

  return true;
}

HAL_StatusTypeDef GNSS_SendCommand(const char *command)
{
  uint8_t tx_buffer[96];
  size_t cmd_len;
  size_t tx_len;
  HAL_StatusTypeDef tx_status;

  if ((s_gnss_uart == NULL) || (command == NULL))
  {
    return HAL_ERROR;
  }

  cmd_len = strlen(command);
  if (cmd_len == 0U)
  {
    return HAL_ERROR;
  }

  if (cmd_len > (sizeof(tx_buffer) - 3U))
  {
    return HAL_ERROR;
  }

  memcpy(tx_buffer, command, cmd_len);
  tx_len = cmd_len;

  if ((tx_len == 0U) || (tx_buffer[tx_len - 1U] != '\n'))
  {
    tx_buffer[tx_len++] = '\r';
    tx_buffer[tx_len++] = '\n';
  }

  tx_status = HAL_UART_Transmit(s_gnss_uart, tx_buffer, (uint16_t)tx_len, 200U);
  if (tx_status != HAL_OK)
  {
    gnss_command_tx_fail_count++;
  }

  return tx_status;
}

void GNSS_ConfigureBestPosA2Hz(void)
{
  (void)GNSS_SendCommand("UNLOGALL THISPORT");
  HAL_Delay(50);
  (void)GNSS_SendCommand("LOG THISPORT BESTPOSA ONTIME 0.5");
  gnss_config_command_sent = 1U;
}

void GNSS_HandleRxCplt(UART_HandleTypeDef *huart)
{
  if ((s_gnss_uart == NULL) || (huart != s_gnss_uart))
  {
    return;
  }

  if (gnss_rx_byte == '\r')
  {
    GNSS_StartReceiveIT();
    return;
  }

  if (gnss_line_index < (sizeof(gnss_line_buffer) - 1U))
  {
    gnss_line_buffer[gnss_line_index++] = gnss_rx_byte;
    gnss_line_buffer[gnss_line_index] = '\0';
  }
  else
  {
    gnss_rx_overflow_count++;
    gnss_line_index = 0U;
    gnss_line_buffer[0] = '\0';
  }

  if (gnss_rx_byte == '\n')
  {
    if (gnss_line_index > 0U)
    {
      GNSS_ProcessCompletedLine();
    }
    gnss_line_index = 0U;
    gnss_line_buffer[0] = '\0';
  }

  GNSS_StartReceiveIT();
}

void GNSS_HandleUartError(UART_HandleTypeDef *huart)
{
  if ((s_gnss_uart == NULL) || (huart != s_gnss_uart))
  {
    return;
  }

  gnss_line_index = 0U;
  memset(gnss_line_buffer, 0, sizeof(gnss_line_buffer));
  gnss_rx_restart_request = 1U;
}
