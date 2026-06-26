#include "app_actuator.h"

#ifndef APP_HOST_TEST

#include "app_board.h"
#include "app_config.h"

void Actuator_SetOutputs(const ActuatorMsg *message, uint32_t now_ms,
                         bool buzzer_click_active)
{
    bool ld1 = false;
    bool ld2 = false;
    bool ld3 = false;
    bool buzzer = buzzer_click_active;

    switch (message->state) {
    case AC_STATE_COOLING:
        ld1 = true;
        break;
    case AC_STATE_MANUAL_OVERRIDE:
        ld2 = true;
        break;
    case AC_STATE_WINDOW_SUSPECT:
        ld3 = (now_ms % 1000U) < 500U;
        break;
    case AC_STATE_AC_PAUSED:
        ld3 = (now_ms % 250U) < 125U;
        break;
    default:
        break;
    }

    Board_WriteActive(APP_LD1_PORT, APP_LD1_PIN, ld1, GPIO_PIN_SET);
    Board_WriteActive(APP_LD2_PORT, APP_LD2_PIN, ld2, GPIO_PIN_SET);
    Board_WriteActive(APP_LD3_PORT, APP_LD3_PIN, ld3, GPIO_PIN_SET);
    Board_WriteActive(APP_BUZZER_PORT, APP_BUZZER_PIN, buzzer,
                      APP_BUZZER_ON_LEVEL);
}

#endif /* APP_HOST_TEST */
