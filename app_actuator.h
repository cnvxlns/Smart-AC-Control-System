#ifndef APP_ACTUATOR_H
#define APP_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "app_messages.h"

#ifndef APP_HOST_TEST

void Actuator_SetOutputs(const ActuatorMsg *message, uint32_t now_ms,
                         bool buzzer_click_active);

#endif /* APP_HOST_TEST */

#endif /* APP_ACTUATOR_H */
