#ifndef APP_TM1637_H
#define APP_TM1637_H

#include <stdint.h>

#include "app_messages.h"

#ifndef APP_HOST_TEST

void Tm1637_Init(void);
void Tm1637_BuildSegments(const DisplayMsg *message, uint32_t now_ms,
                          uint8_t segments[4]);
void Tm1637_SetSegments(const uint8_t segments[4]);
uint8_t Tm1637_GetAckCount(void);

#endif /* APP_HOST_TEST */

#endif /* APP_TM1637_H */
