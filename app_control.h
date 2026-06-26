#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "app_messages.h"

typedef struct {
    bool powered;
    bool manual;
    bool have_sensor;
    bool window_was_open;
    bool pause_latched;
    int8_t setpoint_c;
    float temperature_c;
    float humidity_pct;
    float humidity_before_window_pct;
    uint32_t window_open_since_ms;
    uint32_t recovery_since_ms;
    AcState state;
} ControlContext;

void Control_Init(ControlContext *ctx);
ControlOutput Control_Step(ControlContext *ctx,
                           const SensorMsg *sensor,
                           uint32_t user_events,
                           uint32_t now_ms);

#endif /* APP_CONTROL_H */
