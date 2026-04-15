#include "powercontrol.h"
#include "stm32h7xx_hal.h"

static GPIO_PinState powercontrol_bool_to_pinstate(bool enable)
{
  return enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void PowerControl_Init(void)
{
  /* Default all external module power rails to OFF. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); /* GNSS */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); /* IRIDIUM */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); /* IMU */
}

void PowerControl_SetIMU(bool enable)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, powercontrol_bool_to_pinstate(enable));
}

void PowerControl_SetIridium(bool enable)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, powercontrol_bool_to_pinstate(enable));
}

void PowerControl_SetGNSS(bool enable)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, powercontrol_bool_to_pinstate(enable));
}
