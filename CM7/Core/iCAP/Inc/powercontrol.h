#ifndef POWERCONTROL_H_
#define POWERCONTROL_H_

#include <stdbool.h>

void PowerControl_Init(void);
void PowerControl_SetIMU(bool enable);
void PowerControl_SetIridium(bool enable);
void PowerControl_SetGNSS(bool enable);

#endif /* POWERCONTROL_H_ */
