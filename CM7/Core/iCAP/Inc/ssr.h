#ifndef SSR_H_
#define SSR_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SSR_MODE_OFF = 0,
  SSR_MODE_ON,
  SSR_MODE_BLINK_500MS,
  SSR_MODE_BLINK_100MS
} SSR_Mode_t;

typedef enum
{
  SSR_CH0 = 0, /* PD0 */
  SSR_CH1,     /* PD1 */
  SSR_CH2,     /* PD2 */
  SSR_CH3,     /* PD3 */
  SSR_CH_COUNT
} SSR_Channel_t;

void SSR_Init(void);
void SSR_Process(void);
bool SSR_SetMode(SSR_Channel_t channel, SSR_Mode_t mode);
SSR_Mode_t SSR_GetMode(SSR_Channel_t channel);

#endif /* SSR_H_ */
