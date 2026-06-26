#include "app_control.h"

#include <string.h>

#include "app_config.h"

static int8_t RoundTemperature(float value)
{
    return (int8_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t CoolingLevel(float temperature_c, int8_t setpoint_c, bool manual)
{
    float delta = temperature_c - (float)setpoint_c;

    if ((!manual && delta <= 1.0f) || (manual && delta <= 0.0f)) {
        return 0U;
    }
    if (delta <= 2.0f) {
        return 1U;
    }
    if (delta <= 4.0f) {
        return 2U;
    }
    return 3U;
}

void Control_Init(ControlContext *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->setpoint_c = APP_SETPOINT_DEFAULT_C;
    ctx->state = AC_STATE_OFF;
}

ControlOutput Control_Step(ControlContext *ctx,
                           const SensorMsg *sensor,
                           uint32_t user_events,
                           uint32_t now_ms)
{
    ControlOutput output;
    bool opening_edge = false;

    if ((user_events & USER_EVT_POWER) != 0U) {
        ctx->powered = !ctx->powered;
        if (!ctx->powered) {
            ctx->manual = false;
            ctx->pause_latched = false;
            ctx->window_was_open = false;
        }
    }
    if ((user_events & USER_EVT_SETPOINT_UP) != 0U &&
        ctx->setpoint_c < APP_SETPOINT_MAX_C) {
        ++ctx->setpoint_c;
    }
    if ((user_events & USER_EVT_SETPOINT_DOWN) != 0U &&
        ctx->setpoint_c > APP_SETPOINT_MIN_C) {
        --ctx->setpoint_c;
    }
    if ((user_events & USER_EVT_MANUAL) != 0U && ctx->powered) {
        ctx->manual = !ctx->manual;
        if (ctx->manual) {
            ctx->pause_latched = false;
        }
    }

    if (sensor != NULL) {
        opening_edge = sensor->window_open && !ctx->window_was_open;
        if (opening_edge) {
            ctx->window_open_since_ms = now_ms;
            ctx->humidity_before_window_pct =
                ctx->have_sensor ? ctx->humidity_pct : sensor->humidity_pct;
        }
    }
    if (sensor != NULL && sensor->valid) {
        ctx->temperature_c = sensor->temperature_c;
        ctx->humidity_pct = sensor->humidity_pct;
        ctx->have_sensor = true;
    }

    if (!ctx->powered) {
        ctx->state = AC_STATE_OFF;
    } else if (ctx->manual) {
        ctx->state = AC_STATE_MANUAL_OVERRIDE;
    } else if (sensor != NULL && sensor->window_open) {
        uint32_t open_ms = now_ms - ctx->window_open_since_ms;
        bool humidity_rise = sensor->valid &&
            ((sensor->humidity_pct - ctx->humidity_before_window_pct) >=
             APP_HUMIDITY_RISE_PCT);

        if (open_ms >= APP_WINDOW_PAUSE_MS || humidity_rise ||
            ctx->pause_latched) {
            ctx->pause_latched = true;
            ctx->state = AC_STATE_AC_PAUSED;
        } else {
            ctx->state = AC_STATE_WINDOW_SUSPECT;
        }
    } else if (ctx->window_was_open) {
        ctx->recovery_since_ms = now_ms;
        ctx->pause_latched = false;
        ctx->state = AC_STATE_RECOVERY_WAIT;
    } else if (ctx->state == AC_STATE_RECOVERY_WAIT &&
               (now_ms - ctx->recovery_since_ms) < APP_RECOVERY_WAIT_MS) {
        ctx->state = AC_STATE_RECOVERY_WAIT;
    } else if (ctx->have_sensor &&
               CoolingLevel(ctx->temperature_c, ctx->setpoint_c, false) > 0U) {
        ctx->state = AC_STATE_COOLING;
    } else {
        ctx->state = AC_STATE_IDLE;
    }

    if (sensor != NULL) {
        ctx->window_was_open = sensor->window_open;
    }

    output.state = ctx->state;
    output.current_temperature_c =
        ctx->have_sensor ? RoundTemperature(ctx->temperature_c) : 0;
    output.setpoint_c = ctx->setpoint_c;
    output.temperature_valid = ctx->have_sensor;
    output.cooling_level =
        (ctx->state == AC_STATE_COOLING)
            ? CoolingLevel(ctx->temperature_c, ctx->setpoint_c, false)
            : ((ctx->state == AC_STATE_MANUAL_OVERRIDE)
                   ? CoolingLevel(ctx->temperature_c, ctx->setpoint_c, true)
                   : 0U);
    return output;
}
