/*
 * Smart AC Control System - single-file application
 *
 * Target: NUCLEO-F439ZI, STM32 HAL, uC/OS-III, TrueSTUDIO
 *
 * CubeMX integration:
 *   1. Configure the GPIOs/peripherals listed in BOARD CONFIGURATION below.
 *   2. Configure APP_US_TIMER_HANDLE as a free-running 1 MHz timer.
 *   3. Configure APP_UART_HANDLE for ST-LINK VCP at 115200 baud.
 *   4. Call App_Start() after OSInit() and before OSStart().
 *   5. Configure reed/up/down/B1 GPIOs for both-edge EXTI.
 *
 * All project-specific source is intentionally contained in this file.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef APP_HOST_TEST
#include "main.h"
#include "os.h"
#include <stdarg.h>
#include <stdio.h>
#endif

/* -------------------------------------------------------------------------- */
/* Application constants                                                       */
/* -------------------------------------------------------------------------- */

#define APP_SETPOINT_DEFAULT_C       24
#define APP_SETPOINT_MIN_C           16
#define APP_SETPOINT_MAX_C           30
#define APP_WINDOW_PAUSE_MS          5000U
#define APP_RECOVERY_WAIT_MS         5000U
#define APP_HUMIDITY_RISE_PCT        10.0f
#define APP_DEBOUNCE_MS              50U
#define APP_LONG_PRESS_MS            1500U

#define USER_EVT_SETPOINT_UP         (1UL << 0)
#define USER_EVT_SETPOINT_DOWN       (1UL << 1)
#define USER_EVT_MANUAL              (1UL << 2)
#define USER_EVT_POWER               (1UL << 3)

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
} ControlOutput;

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
    output.cooling_level =
        (ctx->state == AC_STATE_COOLING)
            ? CoolingLevel(ctx->temperature_c, ctx->setpoint_c, false)
            : ((ctx->state == AC_STATE_MANUAL_OVERRIDE)
                   ? CoolingLevel(ctx->temperature_c, ctx->setpoint_c, true)
                   : 0U);
    return output;
}

#ifndef APP_HOST_TEST

/* -------------------------------------------------------------------------- */
/* Board configuration - change only this section to match CubeMX              */
/* -------------------------------------------------------------------------- */

#ifndef APP_UART_HANDLE
#define APP_UART_HANDLE              huart3
#endif
#ifndef APP_US_TIMER_HANDLE
#define APP_US_TIMER_HANDLE          htim2
#endif

extern UART_HandleTypeDef APP_UART_HANDLE;
extern TIM_HandleTypeDef APP_US_TIMER_HANDLE;

#define APP_DHT_PORT                 GPIOD
#define APP_DHT_PIN                  GPIO_PIN_1
#define APP_REED_PORT                GPIOC
#define APP_REED_PIN                 GPIO_PIN_2
#define APP_UP_PORT                  GPIOC
#define APP_UP_PIN                   GPIO_PIN_0
#define APP_DOWN_PORT                GPIOC
#define APP_DOWN_PIN                 GPIO_PIN_1
#define APP_B1_PORT                  GPIOC
#define APP_B1_PIN                   GPIO_PIN_13
#define APP_BUZZER_PORT              GPIOD
#define APP_BUZZER_PIN               GPIO_PIN_0

#define APP_LD1_PORT                 GPIOB
#define APP_LD1_PIN                  GPIO_PIN_0
#define APP_LD2_PORT                 GPIOB
#define APP_LD2_PIN                  GPIO_PIN_7
#define APP_LD3_PORT                 GPIOB
#define APP_LD3_PIN                  GPIO_PIN_14

#define APP_SEG_PORT                 GPIOE
#define APP_SEG_A_PIN                GPIO_PIN_0
#define APP_SEG_B_PIN                GPIO_PIN_1
#define APP_SEG_C_PIN                GPIO_PIN_2
#define APP_SEG_D_PIN                GPIO_PIN_3
#define APP_SEG_E_PIN                GPIO_PIN_4
#define APP_SEG_F_PIN                GPIO_PIN_5
#define APP_SEG_G_PIN                GPIO_PIN_6
#define APP_SEG_DP_PIN               GPIO_PIN_7
#define APP_DIGIT_PORT               GPIOF
#define APP_DIGIT_LEFT_PIN           GPIO_PIN_0
#define APP_DIGIT_RIGHT_PIN          GPIO_PIN_1

#define APP_REED_OPEN_LEVEL          GPIO_PIN_SET
#define APP_BUTTON_PRESSED_LEVEL     GPIO_PIN_RESET
#define APP_SEGMENT_ON_LEVEL         GPIO_PIN_SET
#define APP_DIGIT_ON_LEVEL           GPIO_PIN_SET
#define APP_BUZZER_ON_LEVEL          GPIO_PIN_SET

#define APP_PRIO_SENSOR              5U
#define APP_PRIO_USER                6U
#define APP_PRIO_CONTROL             7U
#define APP_PRIO_DISPLAY             8U
#define APP_PRIO_ACTUATOR            9U
#define APP_PRIO_MONITOR             12U
#define APP_TASK_STACK_SIZE          384U
#define APP_QUEUE_DEPTH              8U

#define RAW_EDGE_UP                  (1UL << 0)
#define RAW_EDGE_DOWN                (1UL << 1)
#define RAW_EDGE_B1                  (1UL << 2)

typedef struct {
    AcState state;
    uint8_t cooling_level;
    int8_t current_temperature_c;
    int8_t setpoint_c;
} DisplayMsg;

typedef struct {
    AcState state;
    uint8_t cooling_level;
} ActuatorMsg;

static OS_TCB SensorTaskTCB;
static OS_TCB UserTaskTCB;
static OS_TCB ControlTaskTCB;
static OS_TCB DisplayTaskTCB;
static OS_TCB ActuatorTaskTCB;
static OS_TCB MonitorTaskTCB;

static CPU_STK SensorTaskStk[APP_TASK_STACK_SIZE];
static CPU_STK UserTaskStk[APP_TASK_STACK_SIZE];
static CPU_STK ControlTaskStk[APP_TASK_STACK_SIZE];
static CPU_STK DisplayTaskStk[APP_TASK_STACK_SIZE];
static CPU_STK ActuatorTaskStk[APP_TASK_STACK_SIZE];
static CPU_STK MonitorTaskStk[APP_TASK_STACK_SIZE];

static OS_Q SensorQ;
static OS_Q DisplayQ;
static OS_Q ActuatorQ;
static OS_FLAG_GRP UserEventFlags;
static OS_MUTEX UartMutex;

static SensorMsg SensorSlots[APP_QUEUE_DEPTH];
static DisplayMsg DisplaySlots[APP_QUEUE_DEPTH];
static ActuatorMsg ActuatorSlots[APP_QUEUE_DEPTH];
static uint8_t SensorSlotIndex;
static uint8_t DisplaySlotIndex;
static uint8_t ActuatorSlotIndex;
static volatile uint32_t RawEdgeBits;
static volatile bool ReedEdgePending;
static volatile AcState MonitorState;
static volatile uint8_t MonitorCoolingLevel;
static volatile uint32_t SensorPostCount;
static volatile uint32_t SensorDropCount;
static volatile uint32_t DisplayPostCount;
static volatile uint32_t DisplayDropCount;
static volatile uint32_t ActuatorPostCount;
static volatile uint32_t ActuatorDropCount;

static void SensorTask(void *argument);
static void UserInputTask(void *argument);
static void ControlTask(void *argument);
static void DisplayTask(void *argument);
static void ActuatorTask(void *argument);
static void MonitorTask(void *argument);

static uint32_t App_Millis(void)
{
    OS_ERR err;
    OS_TICK ticks = OSTimeGet(&err);
    (void)err;
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / OS_CFG_TICK_RATE_HZ);
}

static void App_DelayMs(uint32_t milliseconds)
{
    OS_ERR err;
    OSTimeDlyHMSM(0U, 0U, (CPU_INT16U)(milliseconds / 1000U),
                  (CPU_INT32U)(milliseconds % 1000U),
                  OS_OPT_TIME_HMSM_STRICT, &err);
    (void)err;
}

static GPIO_PinState InactiveLevel(GPIO_PinState active)
{
    return (active == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void WriteActive(GPIO_TypeDef *port, uint16_t pin, bool active,
                        GPIO_PinState active_level)
{
    HAL_GPIO_WritePin(port, pin,
                      active ? active_level : InactiveLevel(active_level));
}

static bool PinIsPressed(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == APP_BUTTON_PRESSED_LEVEL;
}

static uint32_t TakeRawEdges(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t edges;
    __disable_irq();
    edges = RawEdgeBits;
    RawEdgeBits = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return edges;
}

static bool TakeReedEdge(void)
{
    uint32_t primask = __get_PRIMASK();
    bool pending;
    __disable_irq();
    pending = ReedEdgePending;
    ReedEdgePending = false;
    if (primask == 0U) {
        __enable_irq();
    }
    return pending;
}

static void DelayUs(uint16_t microseconds)
{
    __HAL_TIM_SET_COUNTER(&APP_US_TIMER_HANDLE, 0U);
    while (__HAL_TIM_GET_COUNTER(&APP_US_TIMER_HANDLE) < microseconds) {
    }
}

static void DhtSetOutput(void)
{
    GPIO_InitTypeDef config;
    memset(&config, 0, sizeof(config));
    config.Pin = APP_DHT_PIN;
    config.Mode = GPIO_MODE_OUTPUT_OD;
    config.Pull = GPIO_PULLUP;
    config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(APP_DHT_PORT, &config);
}

static void DhtSetInput(void)
{
    GPIO_InitTypeDef config;
    memset(&config, 0, sizeof(config));
    config.Pin = APP_DHT_PIN;
    config.Mode = GPIO_MODE_INPUT;
    config.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(APP_DHT_PORT, &config);
}

static bool DhtWaitFor(GPIO_PinState level, uint16_t timeout_us)
{
    __HAL_TIM_SET_COUNTER(&APP_US_TIMER_HANDLE, 0U);
    while (HAL_GPIO_ReadPin(APP_DHT_PORT, APP_DHT_PIN) != level) {
        if (__HAL_TIM_GET_COUNTER(&APP_US_TIMER_HANDLE) >= timeout_us) {
            return false;
        }
    }
    return true;
}

static bool DhtRead(float *temperature_c, float *humidity_pct)
{
    uint8_t data[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t byte_index;
    uint8_t bit_index;

    DhtSetOutput();
    HAL_GPIO_WritePin(APP_DHT_PORT, APP_DHT_PIN, GPIO_PIN_RESET);
    App_DelayMs(20U);
    HAL_GPIO_WritePin(APP_DHT_PORT, APP_DHT_PIN, GPIO_PIN_SET);
    DelayUs(30U);
    DhtSetInput();

    if (!DhtWaitFor(GPIO_PIN_RESET, 100U) ||
        !DhtWaitFor(GPIO_PIN_SET, 100U) ||
        !DhtWaitFor(GPIO_PIN_RESET, 100U)) {
        return false;
    }

    for (byte_index = 0U; byte_index < 5U; ++byte_index) {
        for (bit_index = 0U; bit_index < 8U; ++bit_index) {
            if (!DhtWaitFor(GPIO_PIN_SET, 80U)) {
                return false;
            }
            DelayUs(40U);
            data[byte_index] <<= 1U;
            if (HAL_GPIO_ReadPin(APP_DHT_PORT, APP_DHT_PIN) == GPIO_PIN_SET) {
                data[byte_index] |= 1U;
            }
            if (!DhtWaitFor(GPIO_PIN_RESET, 100U)) {
                return false;
            }
        }
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return false;
    }
    *humidity_pct = (float)data[0] + ((float)data[1] * 0.1f);
    *temperature_c = (float)data[2] + ((float)data[3] * 0.1f);
    return true;
}

static void UartLog(const char *format, ...)
{
    char buffer[192];
    va_list args;
    int length;
    OS_ERR err;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length <= 0) {
        return;
    }
    if ((size_t)length >= sizeof(buffer)) {
        length = (int)(sizeof(buffer) - 1U);
    }
    OSMutexPend(&UartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        (void)HAL_UART_Transmit(&APP_UART_HANDLE, (uint8_t *)buffer,
                                (uint16_t)length, 100U);
        OSMutexPost(&UartMutex, OS_OPT_POST_NONE, &err);
    }
}

static void PostSensor(const SensorMsg *message)
{
    OS_ERR err;
    SensorMsg *slot = &SensorSlots[SensorSlotIndex];
    *slot = *message;
    OSQPost(&SensorQ, slot, sizeof(*slot), OS_OPT_POST_FIFO, &err);
    if (err == OS_ERR_NONE) {
        ++SensorPostCount;
        SensorSlotIndex = (uint8_t)((SensorSlotIndex + 1U) % APP_QUEUE_DEPTH);
    } else {
        ++SensorDropCount;
    }
}

static void PostDisplay(const ControlOutput *output)
{
    OS_ERR err;
    DisplayMsg *slot = &DisplaySlots[DisplaySlotIndex];
    slot->state = output->state;
    slot->cooling_level = output->cooling_level;
    slot->current_temperature_c = output->current_temperature_c;
    slot->setpoint_c = output->setpoint_c;
    OSQPost(&DisplayQ, slot, sizeof(*slot), OS_OPT_POST_FIFO, &err);
    if (err == OS_ERR_NONE) {
        ++DisplayPostCount;
        DisplaySlotIndex = (uint8_t)((DisplaySlotIndex + 1U) % APP_QUEUE_DEPTH);
    } else {
        ++DisplayDropCount;
    }
}

static void PostActuator(const ControlOutput *output)
{
    OS_ERR err;
    ActuatorMsg *slot = &ActuatorSlots[ActuatorSlotIndex];
    slot->state = output->state;
    slot->cooling_level = output->cooling_level;
    OSQPost(&ActuatorQ, slot, sizeof(*slot), OS_OPT_POST_FIFO, &err);
    if (err == OS_ERR_NONE) {
        ++ActuatorPostCount;
        ActuatorSlotIndex = (uint8_t)((ActuatorSlotIndex + 1U) % APP_QUEUE_DEPTH);
    } else {
        ++ActuatorDropCount;
    }
}

static const uint8_t SegmentDigits[10] = {
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
    0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
};

static void SegmentWritePattern(uint8_t pattern)
{
    const uint16_t pins[8] = {
        APP_SEG_A_PIN, APP_SEG_B_PIN, APP_SEG_C_PIN, APP_SEG_D_PIN,
        APP_SEG_E_PIN, APP_SEG_F_PIN, APP_SEG_G_PIN, APP_SEG_DP_PIN
    };
    uint8_t index;
    for (index = 0U; index < 8U; ++index) {
        WriteActive(APP_SEG_PORT, pins[index],
                    (pattern & (uint8_t)(1U << index)) != 0U,
                    APP_SEGMENT_ON_LEVEL);
    }
}

static uint8_t SegmentLetter(char value)
{
    switch (value) {
    case 'E': return 0x79U;
    case 'F': return 0x71U;
    case 'L': return 0x38U;
    case 'O': return 0x3FU;
    case 'P': return 0x73U;
    case 'S': return 0x6DU;
    default: return 0x00U;
    }
}

static void DisplaySelectDigit(bool left)
{
    WriteActive(APP_DIGIT_PORT, APP_DIGIT_LEFT_PIN, left,
                APP_DIGIT_ON_LEVEL);
    WriteActive(APP_DIGIT_PORT, APP_DIGIT_RIGHT_PIN, !left,
                APP_DIGIT_ON_LEVEL);
}

static void DisplayDisableDigits(void)
{
    WriteActive(APP_DIGIT_PORT, APP_DIGIT_LEFT_PIN, false,
                APP_DIGIT_ON_LEVEL);
    WriteActive(APP_DIGIT_PORT, APP_DIGIT_RIGHT_PIN, false,
                APP_DIGIT_ON_LEVEL);
}

static void DisplayPairForMessage(const DisplayMsg *message, uint32_t now_ms,
                                  uint8_t *left, uint8_t *right)
{
    uint32_t phase = now_ms % 5000U;
    int value = message->current_temperature_c;
    bool blank = false;
    bool dp = false;
    char code_left = '\0';
    char code_right = '\0';

    switch (message->state) {
    case AC_STATE_OFF:
        code_left = 'O'; code_right = 'F'; blank = (now_ms % 1000U) >= 500U;
        break;
    case AC_STATE_WINDOW_SUSPECT:
        if ((now_ms % 2000U) >= 1000U) { code_left = 'O'; code_right = 'P'; }
        break;
    case AC_STATE_AC_PAUSED:
        code_left = 'O'; code_right = 'F'; blank = (now_ms % 1000U) >= 500U;
        break;
    case AC_STATE_RECOVERY_WAIT:
        if ((now_ms % 2000U) >= 1000U) { code_left = 'E'; code_right = 'S'; }
        break;
    case AC_STATE_COOLING:
    case AC_STATE_MANUAL_OVERRIDE:
        dp = true;
        if (phase >= 4000U) { code_left = 'L'; value = message->cooling_level; }
        else if (phase >= 3000U) { value = message->setpoint_c; dp = false; }
        break;
    case AC_STATE_IDLE:
    default:
        if (phase >= 3000U) { value = message->setpoint_c; }
        break;
    }

    if (blank) {
        *left = 0U; *right = 0U;
    } else if (code_left != '\0') {
        *left = SegmentLetter(code_left);
        *right = (code_left == 'L') ? SegmentDigits[value % 10]
                                    : SegmentLetter(code_right);
    } else {
        if (value < 0) { value = 0; }
        if (value > 99) { value = 99; }
        *left = SegmentDigits[(uint8_t)value / 10U];
        *right = SegmentDigits[(uint8_t)value % 10U] | (dp ? 0x80U : 0U);
    }
}

static void SetActuators(const ActuatorMsg *message, uint32_t now_ms)
{
    bool ld1 = false;
    bool ld2 = false;
    bool ld3 = false;
    bool buzzer = false;

    switch (message->state) {
    case AC_STATE_COOLING:
        ld1 = true;
        break;
    case AC_STATE_MANUAL_OVERRIDE:
        ld2 = true;
        break;
    case AC_STATE_WINDOW_SUSPECT:
        ld3 = (now_ms % 1000U) < 500U;
        buzzer = (now_ms % 2000U) < 120U;
        break;
    case AC_STATE_AC_PAUSED:
        ld3 = (now_ms % 250U) < 125U;
        buzzer = true;
        break;
    default:
        break;
    }
    WriteActive(APP_LD1_PORT, APP_LD1_PIN, ld1, GPIO_PIN_SET);
    WriteActive(APP_LD2_PORT, APP_LD2_PIN, ld2, GPIO_PIN_SET);
    WriteActive(APP_LD3_PORT, APP_LD3_PIN, ld3, GPIO_PIN_SET);
    WriteActive(APP_BUZZER_PORT, APP_BUZZER_PIN, buzzer, APP_BUZZER_ON_LEVEL);
}

static void SensorTask(void *argument)
{
    SensorMsg message;
    bool last_window = false;
    uint32_t last_dht_ms = 0U;
    (void)argument;
    memset(&message, 0, sizeof(message));

    for (;;) {
        uint32_t now = App_Millis();
        bool window_open =
            HAL_GPIO_ReadPin(APP_REED_PORT, APP_REED_PIN) == APP_REED_OPEN_LEVEL;
        bool publish = (window_open != last_window) || TakeReedEdge();

        if ((now - last_dht_ms) >= 2000U || last_dht_ms == 0U) {
            message.valid = DhtRead(&message.temperature_c,
                                    &message.humidity_pct);
            if (!message.valid) {
                UartLog("DHT11 read failed\r\n");
            }
            message.window_open = window_open;
            publish = true;
            last_dht_ms = now;
        } else if (publish) {
            message.window_open = window_open;
            message.valid = true;
        }
        if (publish) {
            PostSensor(&message);
            last_window = window_open;
        }
        App_DelayMs(100U);
    }
}

static void UserInputTask(void *argument)
{
    bool b1_was_pressed = false;
    uint32_t b1_pressed_since = 0U;
    (void)argument;

    for (;;) {
        uint32_t edges = TakeRawEdges();
        uint32_t events = 0U;
        bool b1_pressed;
        OS_ERR err;

        if (edges != 0U) {
            App_DelayMs(APP_DEBOUNCE_MS);
        }
        if ((edges & RAW_EDGE_UP) != 0U &&
            PinIsPressed(APP_UP_PORT, APP_UP_PIN)) {
            events |= USER_EVT_SETPOINT_UP;
        }
        if ((edges & RAW_EDGE_DOWN) != 0U &&
            PinIsPressed(APP_DOWN_PORT, APP_DOWN_PIN)) {
            events |= USER_EVT_SETPOINT_DOWN;
        }

        b1_pressed = PinIsPressed(APP_B1_PORT, APP_B1_PIN);
        if (b1_pressed && !b1_was_pressed) {
            b1_pressed_since = App_Millis();
        } else if (!b1_pressed && b1_was_pressed) {
            events |= ((App_Millis() - b1_pressed_since) >= APP_LONG_PRESS_MS)
                          ? USER_EVT_POWER : USER_EVT_MANUAL;
        }
        b1_was_pressed = b1_pressed;

        if (events != 0U) {
            OSFlagPost(&UserEventFlags, events, OS_OPT_POST_FLAG_SET, &err);
        }
        App_DelayMs(10U);
    }
}

static void ControlTask(void *argument)
{
    ControlContext context;
    SensorMsg latest;
    bool have_latest = false;
    bool have_last_output = false;
    ControlOutput last_output;
    (void)argument;
    Control_Init(&context);
    memset(&latest, 0, sizeof(latest));

    for (;;) {
        OS_ERR err;
        OS_MSG_SIZE size;
        CPU_TS timestamp;
        SensorMsg *received =
            (SensorMsg *)OSQPend(&SensorQ, 10U, OS_OPT_PEND_BLOCKING,
                                 &size, &timestamp, &err);
        uint32_t events = (uint32_t)OSFlagPend(
            &UserEventFlags, USER_EVT_SETPOINT_UP | USER_EVT_SETPOINT_DOWN |
                                 USER_EVT_MANUAL | USER_EVT_POWER,
            0U, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME |
                    OS_OPT_PEND_NON_BLOCKING,
            &timestamp, &err);
        ControlOutput output;

        if (received != NULL && size == sizeof(SensorMsg)) {
            latest = *received;
            have_latest = true;
        }
        output = Control_Step(&context, have_latest ? &latest : NULL,
                              events, App_Millis());
        if (!have_last_output || output.state != last_output.state ||
            output.cooling_level != last_output.cooling_level ||
            output.current_temperature_c != last_output.current_temperature_c ||
            output.setpoint_c != last_output.setpoint_c) {
            PostDisplay(&output);
            PostActuator(&output);
            last_output = output;
            have_last_output = true;
        }
        MonitorState = output.state;
        MonitorCoolingLevel = output.cooling_level;
    }
}

static void DisplayTask(void *argument)
{
    DisplayMsg current;
    bool left_digit = false;
    (void)argument;
    memset(&current, 0, sizeof(current));
    current.state = AC_STATE_OFF;

    for (;;) {
        OS_ERR err;
        OS_MSG_SIZE size;
        CPU_TS timestamp;
        DisplayMsg *received =
            (DisplayMsg *)OSQPend(&DisplayQ, 0U, OS_OPT_PEND_NON_BLOCKING,
                                  &size, &timestamp, &err);
        uint8_t left;
        uint8_t right;
        if (received != NULL && size == sizeof(DisplayMsg)) {
            current = *received;
        }
        DisplayPairForMessage(&current, App_Millis(), &left, &right);
        DisplayDisableDigits();
        SegmentWritePattern(left_digit ? left : right);
        DisplaySelectDigit(left_digit);
        left_digit = !left_digit;
        App_DelayMs(5U);
    }
}

static void ActuatorTask(void *argument)
{
    ActuatorMsg current;
    (void)argument;
    memset(&current, 0, sizeof(current));
    current.state = AC_STATE_OFF;

    for (;;) {
        OS_ERR err;
        OS_MSG_SIZE size;
        CPU_TS timestamp;
        ActuatorMsg *received =
            (ActuatorMsg *)OSQPend(&ActuatorQ, 0U, OS_OPT_PEND_NON_BLOCKING,
                                   &size, &timestamp, &err);
        if (received != NULL && size == sizeof(ActuatorMsg)) {
            current = *received;
        }
        SetActuators(&current, App_Millis());
        App_DelayMs(25U);
    }
}

static void MonitorTask(void *argument)
{
    (void)argument;
    for (;;) {
#if OS_CFG_STAT_TASK_EN > 0U
        UartLog("SmartAC state=%u cooling=L%u cpu=%u%% "
                "q[s=%lu/%lu d=%lu/%lu a=%lu/%lu]\r\n",
                (unsigned)MonitorState, (unsigned)MonitorCoolingLevel,
                (unsigned)OSStatTaskCPUUsage,
                (unsigned long)SensorPostCount, (unsigned long)SensorDropCount,
                (unsigned long)DisplayPostCount, (unsigned long)DisplayDropCount,
                (unsigned long)ActuatorPostCount,
                (unsigned long)ActuatorDropCount);
#else
        UartLog("SmartAC state=%u cooling=L%u cpu=n/a "
                "q[s=%lu/%lu d=%lu/%lu a=%lu/%lu]\r\n",
                (unsigned)MonitorState, (unsigned)MonitorCoolingLevel,
                (unsigned long)SensorPostCount, (unsigned long)SensorDropCount,
                (unsigned long)DisplayPostCount, (unsigned long)DisplayDropCount,
                (unsigned long)ActuatorPostCount,
                (unsigned long)ActuatorDropCount);
#endif
        App_DelayMs(2000U);
    }
}

static void CreateTask(OS_TCB *tcb, CPU_CHAR *name, OS_TASK_PTR function,
                       OS_PRIO priority, CPU_STK *stack)
{
    OS_ERR err;
    OSTaskCreate(tcb, name, function, NULL, priority, stack,
                 APP_TASK_STACK_SIZE / 10U, APP_TASK_STACK_SIZE, 0U, 0U,
                 NULL, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    if (err != OS_ERR_NONE) {
        UartLog("Task create failed: %s err=%u\r\n", name, (unsigned)err);
    }
}

void App_Start(void)
{
    OS_ERR err;

    (void)HAL_TIM_Base_Start(&APP_US_TIMER_HANDLE);
    OSQCreate(&SensorQ, "SensorQ", APP_QUEUE_DEPTH, &err);
    OSQCreate(&DisplayQ, "DisplayQ", APP_QUEUE_DEPTH, &err);
    OSQCreate(&ActuatorQ, "ActuatorQ", APP_QUEUE_DEPTH, &err);
    OSFlagCreate(&UserEventFlags, "UserEventFlags", 0U, &err);
    OSMutexCreate(&UartMutex, "UartMutex", &err);

    CreateTask(&SensorTaskTCB, "Sensor Task", SensorTask, APP_PRIO_SENSOR,
               SensorTaskStk);
    CreateTask(&UserTaskTCB, "User Input Task", UserInputTask, APP_PRIO_USER,
               UserTaskStk);
    CreateTask(&ControlTaskTCB, "Control Task", ControlTask, APP_PRIO_CONTROL,
               ControlTaskStk);
    CreateTask(&DisplayTaskTCB, "Display Task", DisplayTask, APP_PRIO_DISPLAY,
               DisplayTaskStk);
    CreateTask(&ActuatorTaskTCB, "Actuator Task", ActuatorTask,
               APP_PRIO_ACTUATOR, ActuatorTaskStk);
    CreateTask(&MonitorTaskTCB, "Monitor Task", MonitorTask, APP_PRIO_MONITOR,
               MonitorTaskStk);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == APP_UP_PIN) {
        RawEdgeBits |= RAW_EDGE_UP;
    } else if (gpio_pin == APP_DOWN_PIN) {
        RawEdgeBits |= RAW_EDGE_DOWN;
    } else if (gpio_pin == APP_B1_PIN) {
        RawEdgeBits |= RAW_EDGE_B1;
    } else if (gpio_pin == APP_REED_PIN) {
        ReedEdgePending = true;
    }
}

#endif /* APP_HOST_TEST */
