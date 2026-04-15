#include "ssr.h"
#include "stm32h7xx_hal.h"

typedef struct
{
  SSR_Mode_t mode;
  uint8_t output_on;
  uint32_t last_toggle_tick;
} SSR_State_t;

static const uint16_t s_ssr_pins[SSR_CH_COUNT] =
{
  GPIO_PIN_0,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_3
};

static SSR_State_t s_ssr_state[SSR_CH_COUNT];

static bool SSR_IsValidChannel(SSR_Channel_t channel)
{
  return (channel >= SSR_CH0) && (channel < SSR_CH_COUNT);
}

static uint32_t SSR_GetBlinkIntervalMs(SSR_Mode_t mode)
{
  if (mode == SSR_MODE_BLINK_500MS)
  {
    return 500U;
  }

  if (mode == SSR_MODE_BLINK_100MS)
  {
    return 100U;
  }

  return 0U;
}

static void SSR_WriteChannel(SSR_Channel_t channel, uint8_t on)
{
  HAL_GPIO_WritePin(GPIOD, s_ssr_pins[channel], (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SSR_Init(void)
{
  GPIO_InitTypeDef gpio_init;
  uint32_t tick;
  uint8_t i;

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &gpio_init);
  HAL_GPIO_WritePin(GPIOD, gpio_init.Pin, GPIO_PIN_RESET);

  tick = HAL_GetTick();
  for (i = 0U; i < (uint8_t)SSR_CH_COUNT; i++)
  {
    s_ssr_state[i].mode = SSR_MODE_OFF;
    s_ssr_state[i].output_on = 0U;
    s_ssr_state[i].last_toggle_tick = tick;
  }
}

void SSR_Process(void)
{
  uint8_t i;
  uint32_t now;

  now = HAL_GetTick();
  for (i = 0U; i < (uint8_t)SSR_CH_COUNT; i++)
  {
    uint32_t interval = SSR_GetBlinkIntervalMs(s_ssr_state[i].mode);

    if (s_ssr_state[i].mode == SSR_MODE_OFF)
    {
      if (s_ssr_state[i].output_on != 0U)
      {
        s_ssr_state[i].output_on = 0U;
        SSR_WriteChannel((SSR_Channel_t)i, 0U);
      }
      continue;
    }

    if (s_ssr_state[i].mode == SSR_MODE_ON)
    {
      if (s_ssr_state[i].output_on == 0U)
      {
        s_ssr_state[i].output_on = 1U;
        SSR_WriteChannel((SSR_Channel_t)i, 1U);
      }
      continue;
    }

    if ((now - s_ssr_state[i].last_toggle_tick) >= interval)
    {
      s_ssr_state[i].last_toggle_tick = now;
      s_ssr_state[i].output_on = (uint8_t)(s_ssr_state[i].output_on == 0U ? 1U : 0U);
      SSR_WriteChannel((SSR_Channel_t)i, s_ssr_state[i].output_on);
    }
  }
}

bool SSR_SetMode(SSR_Channel_t channel, SSR_Mode_t mode)
{
  if (!SSR_IsValidChannel(channel))
  {
    return false;
  }

  if ((mode < SSR_MODE_OFF) || (mode > SSR_MODE_BLINK_100MS))
  {
    return false;
  }

  s_ssr_state[channel].mode = mode;
  s_ssr_state[channel].last_toggle_tick = HAL_GetTick();

  if (mode == SSR_MODE_OFF)
  {
    s_ssr_state[channel].output_on = 0U;
    SSR_WriteChannel(channel, 0U);
  }
  else if (mode == SSR_MODE_ON)
  {
    s_ssr_state[channel].output_on = 1U;
    SSR_WriteChannel(channel, 1U);
  }
  else
  {
    s_ssr_state[channel].output_on = 0U;
    SSR_WriteChannel(channel, 0U);
  }

  return true;
}

SSR_Mode_t SSR_GetMode(SSR_Channel_t channel)
{
  if (!SSR_IsValidChannel(channel))
  {
    return SSR_MODE_OFF;
  }

  return s_ssr_state[channel].mode;
}
