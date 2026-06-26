#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#define USER_EVT_SETPOINT_UP         (1UL << 0)
#define USER_EVT_SETPOINT_DOWN       (1UL << 1)
#define USER_EVT_MANUAL              (1UL << 2)
#define USER_EVT_POWER               (1UL << 3)
#define USER_EVT_BUZZER_CLICK        (1UL << 4)

typedef enum {
    AC_STATE_OFF = 0,
    AC_STATE_IDLE,
    AC_STATE_COOLING,
    AC_STATE_WINDOW_SUSPECT,
    AC_STATE_AC_PAUSED,
    AC_STATE_RECOVERY_WAIT,
    AC_STATE_MANUAL_OVERRIDE
} AcState;

typedef struct {
    float temperature_c;
    float humidity_pct;
    bool window_open;
    bool valid;
} SensorMsg;

typedef struct {
    AcState state;
    uint8_t cooling_level;
    int8_t current_temperature_c;
    int8_t setpoint_c;
    bool temperature_valid;
} ControlOutput;

typedef struct {
    AcState state;
    uint8_t cooling_level;
    int8_t current_temperature_c;
    int8_t setpoint_c;
    bool temperature_valid;
} DisplayMsg;

typedef struct {
    AcState state;
    uint8_t cooling_level;
    bool buzzer_click;
} ActuatorMsg;

#endif /* APP_MESSAGES_H */
