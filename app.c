/*
 * Smart AC Control System - single-file application
 *
 * Target: NUCLEO-F439ZI, STM32 StdPeriph, uC/OS-III, TrueSTUDIO
 *
 * Pin map for NUCLEO-F439ZI
 *
 * External wiring pins:
 *   - DHT22 data:              A2 / PC3   (GPIO, 3.3V pull-up recommended)
 *   - TM1637 CLK/DIO:          D2 / PF15, D3 / PE13 (open-drain GPIO)
 *   - Reed switch/window:      A0 / PA3   (EXTI3, open = GPIO_PIN_SET)
 *   - Setpoint UP button:      A1 / PC0   (EXTI0, pressed = GPIO_PIN_RESET)
 *   - Setpoint DOWN button:    D8 / PF12  (EXTI12, pressed = GPIO_PIN_RESET)
 *   - External buzzer:         D9 / PD15  (GPIO output, active low)
 *
 * On-board parts, no external wiring required:
 *   - USER/B1 button:          PC13       (EXTI13, pressed = GPIO_PIN_RESET)
 *   - LD1 green cooling LED:   PB0        (onboard LD1)
 *   - LD2 blue manual LED:     PB7        (onboard LD2)
 *   - LD3 red warning LED:     PB14       (onboard LD3)
 *
 * Other board functions:
 *   - UART debug:              APP_UART (default USART3, 115200 baud)
 *   - 1 MHz microsecond timer: APP_US_TIMER (default TIM2)
 *
 * Note:
 *   - External wiring must use only A0~A5 or D0~D15.
 *   - Arduino names such as A2 and D9 are for wiring reference only.
 *   - Board code uses STM32 GPIO names such as GPIOC/GPIO_Pin_3.
 *   - Do not configure DHT22 A2/PC3 as ADC or EXTI; use it as GPIO only.
 *   - Connect DHT22 VCC to 3.3V, DATA to PC3/A2, and add a 4.7k or 10k
 *     pull-up from DATA to 3.3V. Avoid 5V DATA pull-up on STM32.
 *
 * TrueSTUDIO/uC-OS-III integration:
 *   1. Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, and GPIOF clocks.
 *   2. Configure DHT22 A2/PC3 as GPIO with pull-up, not ADC and not EXTI.
 *   3. Configure TM1637 D2/PF15 and D3/PE13 as open-drain GPIO outputs with pull-up.
 *   4. Configure reed/up/down/B1 with GPIO_PULLUP and both-edge EXTI.
 *   5. Use non-overlapping EXTI0 (A1/PC0), EXTI3 (A0/PA3),
 *      EXTI12 (D8/PF12), and EXTI13 (USER/B1 PC13).
 *   6. Configure buzzer D9/PD15 as active-low GPIO output.
 *   7. Configure APP_US_TIMER as a free-running 1 MHz timer.
 *   8. Configure APP_UART for ST-LINK VCP at 115200 baud.
 *
 * All project-specific source is intentionally contained in this file.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef APP_HOST_TEST
#include <includes.h>
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
#define APP_BUTTON_POLL_MS           10U
/* Consecutive identical polls required to accept a new button level.
 * 50 ms / 10 ms = 5 samples of stability. */
#define APP_DEBOUNCE_SAMPLES         (APP_DEBOUNCE_MS / APP_BUTTON_POLL_MS)
#define APP_LONG_PRESS_MS            1500U
#define APP_BUZZER_CLICK_MS          120U
#define APP_MONITOR_PERIOD_MS        1000U
#define APP_MONITOR_START_DELAY_MS   1000U

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
    output.temperature_valid = ctx->have_sensor;
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
/* Board configuration                                                         */
/* -------------------------------------------------------------------------- */

#ifndef APP_UART
#define APP_UART                     USART3
#endif
#ifndef APP_US_TIMER
#define APP_US_TIMER                 TIM2
#endif

#define APP_DHT_PORT                 GPIOC       /* A2 / PC3, GPIO, 3.3V pull-up */
#define APP_DHT_PIN                  GPIO_Pin_3
#define APP_REED_PORT                GPIOA       /* A0 / PA3, EXTI3, open high */
#define APP_REED_PIN                 GPIO_Pin_3
#define APP_UP_PORT                  GPIOC       /* A1 / PC0, EXTI0, pressed low */
#define APP_UP_PIN                   GPIO_Pin_0
#define APP_DOWN_PORT                GPIOF       /* D8 / PF12, EXTI12, pressed low */
#define APP_DOWN_PIN                 GPIO_Pin_12
#define APP_B1_PORT                  GPIOC       /* PC13 / onboard USER/B1, EXTI13, pressed low */
#define APP_B1_PIN                   GPIO_Pin_13
#define APP_BUZZER_PORT              GPIOD       /* D9 / PD15, active-low output */
#define APP_BUZZER_PIN               GPIO_Pin_15
#define APP_TM1637_CLK_PORT          GPIOF       /* D2 / PF15, open-drain */
#define APP_TM1637_CLK_PIN           GPIO_Pin_15
#define APP_TM1637_DIO_PORT          GPIOE       /* D3 / PE13, open-drain */
#define APP_TM1637_DIO_PIN           GPIO_Pin_13

#define APP_LD1_PORT                 GPIOB       /* PB0 / onboard LD1 green */
#define APP_LD1_PIN                  GPIO_Pin_0
#define APP_LD2_PORT                 GPIOB       /* PB7 / onboard LD2 blue */
#define APP_LD2_PIN                  GPIO_Pin_7
#define APP_LD3_PORT                 GPIOB       /* PB14 / onboard LD3 red */
#define APP_LD3_PIN                  GPIO_Pin_14

typedef BitAction GPIO_PinState;

#define GPIO_PIN_RESET               Bit_RESET
#define GPIO_PIN_SET                 Bit_SET

#define APP_REED_OPEN_LEVEL          GPIO_PIN_SET
/* External UP/DOWN buttons are user-wired active-low with a pull-up. */
#define APP_BUTTON_PRESSED_LEVEL     GPIO_PIN_RESET
/* Onboard USER/B1 (PC13) on NUCLEO-144 is active-HIGH: idle low, pressed high,
 * board has an external pull-down. (The original STM32F429II-SK BSP this code
 * came from used the opposite polarity.) So B1 needs its own pressed level and
 * an internal pull-DOWN, not the shared active-low/pull-up button config. */
#define APP_B1_PRESSED_LEVEL         GPIO_PIN_SET
#define APP_BUZZER_ON_LEVEL          GPIO_PIN_RESET

#define APP_PRIO_START               APP_CFG_TASK_START_PRIO
#define APP_PRIO_SENSOR              5U
#define APP_PRIO_USER                6U
#define APP_PRIO_CONTROL             7U
#define APP_PRIO_DISPLAY             8U
#define APP_PRIO_ACTUATOR            9U
#define APP_PRIO_MONITOR             4U
/* DIAGNOSTIC: 384 words was marginal for a task that does float math + UART
 * formatting + nested DhtRead while taking interrupts that push FPU lazy-stack
 * frames. An overflow corrupts the return address -> HardFault, which presents
 * exactly as "prints up to 'DHT before' then the whole board freezes". Raised
 * to 1024 to rule it out; tune back down once confirmed via OSTaskStkChk. */
#define APP_TASK_STACK_SIZE          1024U
#define APP_QUEUE_DEPTH              8U
#define APP_QUEUE_SLOT_COUNT         (APP_QUEUE_DEPTH + 2U)
#define APP_BUSY_WAIT_GUARD          1000000UL
#define APP_UART_TX_TIMEOUT_GUARD    1000000UL
/* Half-period of the bit-banged TM1637 clock. DIO is open-drain and a data '1'
 * is raised only by the pull-up, so the bit period must be long enough for the
 * line to reach a valid HIGH before the TM1637 samples it. The weak ~40k
 * internal pull-up rises slowly, so 5us was too short and corrupted '1' bits
 * (garbage digits even though every byte still ACKed). 50us is well within
 * TM1637 limits and adds large margin. If still marginal, add an external
 * 4.7k-10k pull-up from DIO to 3.3V, which is the robust hardware fix. */
#define APP_TM1637_DELAY_US          50U
#define APP_TM1637_BRIGHTNESS        4U
#define APP_DHT22_START_US           1200U
#define APP_DHT22_PULL_US            30U
#define APP_DHT_BIT_ONE_THRESHOLD_US 50U
#define APP_DHT_READ_PERIOD_MS       2500U

/* --- DHT22 diagnostics switches (toggle with #define) --------------------- */
/* APP_DHT_READ_ENABLE = 0 skips DhtRead() entirely so message.valid is always
 * false. Use it to prove RTOS/UART/SensorTask keep running independent of the
 * sensor: "DHT before" / "DHT fail" / "INPUT ..." must keep repeating. */
#define APP_DHT_READ_ENABLE          1
/* APP_DHT_DEBUG_LOG = 0 silences the SensorTask "DHT before/ok/fail" lines. */
#define APP_DHT_DEBUG_LOG            1

/* Diagnostic timeouts intentionally generous so a slow/marginal sensor times
 * out cleanly instead of looking like a hard hang. */
#define APP_DHT_PULSE_TIMEOUT_US     2000U
#define APP_DHT_READ_TIMEOUT_US      20000U
/* IMPORTANT: this guard is the *fallback* exit when the microsecond-timeout is
 * not effective (e.g. TIM2 is not really 1 MHz). The whole read runs with
 * interrupts disabled, so a huge guard makes a non-responding sensor look like
 * a total freeze. Keep it small enough that the worst case (all ~82 pulse
 * measurements maxing out) still bails within tens of ms and lets the RTOS,
 * UART and MonitorTask keep running. Raise only if you confirm TIM2 = 1 MHz. */
#define APP_DHT_LOOP_GUARD           20000UL

/* TIM2 self-test: count timer ticks across this OS delay. At a true 1 MHz the
 * measured tick delta should be ~ (period_ms * 1000). Logged as
 * us_ticks_100ms in the INPUT line so the timer rate is directly observable. */
#define APP_US_TIMER_PROBE_MS        100U

/* DIAGNOSTIC: when 1 the read runs with interrupts globally disabled (correct
 * for real timing, but a fault/hang in that window freezes the whole board and
 * nothing can recover). Set to 0 to run the SAME read with interrupts ENABLED:
 * the bit timing will be jittery and the read will probably fail, but the RTOS
 * keeps running so MonitorTask can print dht_err/dht_stage. Use this purely to
 * find WHERE DhtRead dies. */
#define APP_DHT_DISABLE_IRQ          1

/* DhtRead progress breadcrumb (plain volatile writes, safe with IRQ off).
 * Printed as dht_stage= in the INPUT line. Last value reached before a freeze
 * tells you which step hung/faulted. */
typedef enum {
    DHT_STAGE_IDLE = 0,
    DHT_STAGE_ENTER = 1,          /* DhtRead entered, timer was ready */
    DHT_STAGE_START_PULSE = 2,    /* host start-low pulse driven */
    DHT_STAGE_CRITICAL_ENTER = 3, /* interrupts about to be disabled */
    DHT_STAGE_RESP_LOW = 4,       /* response LOW captured */
    DHT_STAGE_RESP_HIGH = 5,      /* response HIGH captured */
    DHT_STAGE_DATA_DONE = 6,      /* all 40 bits captured */
    DHT_STAGE_CRITICAL_EXIT = 7,  /* interrupts re-enabled */
    DHT_STAGE_DONE = 8            /* checksum/convert finished */
} DhtStage;

/* Step-by-step failure classification for serial-monitor diagnosis. */
typedef enum {
    DHT_ERR_NONE = 0,
    DHT_ERR_TIMER_NOT_READY = 1,
    DHT_ERR_RESPONSE_LOW_TIMEOUT = 2,
    DHT_ERR_RESPONSE_HIGH_TIMEOUT = 3,
    DHT_ERR_DATA_LOW_TIMEOUT = 4,
    DHT_ERR_DATA_HIGH_TIMEOUT = 5,
    DHT_ERR_CHECKSUM = 6
} DhtErrorCode;

#define RAW_EDGE_UP                  (1UL << 0)
#define RAW_EDGE_DOWN                (1UL << 1)
#define RAW_EDGE_B1                  (1UL << 2)

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

static OS_TCB AppTaskStartTCB;
static OS_TCB SensorTaskTCB;
static OS_TCB UserTaskTCB;
static OS_TCB ControlTaskTCB;
static OS_TCB DisplayTaskTCB;
static OS_TCB ActuatorTaskTCB;
static OS_TCB MonitorTaskTCB;

static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
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

/* Extra slots cover one consumer-owned message and the producer's next write. */
static SensorMsg SensorSlots[APP_QUEUE_SLOT_COUNT];
static DisplayMsg DisplaySlots[APP_QUEUE_SLOT_COUNT];
static ActuatorMsg ActuatorSlots[APP_QUEUE_SLOT_COUNT];
static uint8_t SensorSlotIndex;
static uint8_t DisplaySlotIndex;
static uint8_t ActuatorSlotIndex;
static bool UsTimerReady;
static volatile uint32_t RawEdgeBits;
static volatile bool ReedEdgePending;
static volatile AcState MonitorState;
static volatile uint8_t MonitorCoolingLevel;
static volatile bool MonitorDhtValid;
static volatile int16_t MonitorDhtTemperatureTenths;
static volatile int16_t MonitorDhtHumidityTenths;
static volatile uint32_t MonitorDhtFailCount;
static volatile uint32_t MonitorDhtLastError;
static volatile uint16_t MonitorDhtLastRespLowUs;
static volatile uint16_t MonitorDhtLastRespHighUs;
static volatile uint16_t MonitorDhtLastBitLowUs;
static volatile uint16_t MonitorDhtLastBitHighUs;
static volatile uint8_t MonitorDhtLastByte0;
static volatile uint8_t MonitorDhtLastByte1;
static volatile uint8_t MonitorDhtLastByte2;
static volatile uint8_t MonitorDhtLastByte3;
static volatile uint8_t MonitorDhtLastByte4;
static volatile uint32_t MonitorUsTicksPerProbe;
static volatile uint8_t MonitorDhtStage;
/* TM1637 diagnostic: number of bytes ACKed in the last full refresh.
 * A working module ACKs every byte, so the expected value is 7
 * (0x40 + 0xC0 + 4 segment bytes + display command). 0 means the module
 * is not responding at all -> wiring / power / logic-level problem. */
static volatile uint8_t MonitorTm1637AckCount;
static volatile uint32_t SensorPostCount;
static volatile uint32_t SensorDropCount;
static volatile uint32_t ActuatorPostCount;
static volatile uint32_t ActuatorDropCount;

static void AppTaskStart(void *argument);
static void SensorTask(void *argument);
static void UserInputTask(void *argument);
static void ControlTask(void *argument);
static void DisplayTask(void *argument);
static void ActuatorTask(void *argument);
static void MonitorTask(void *argument);

static void Board_Init(void);
static void Board_InitGpio(void);
static void Board_InitUsart(void);
static void Board_InitTimer(void);
static void Board_InitExti(void);
static void App_EXTI0_ISR(void);
static void App_EXTI3_ISR(void);
static void App_EXTI15_10_ISR(void);
static void App_HandleGpioExti(uint16_t gpio_pin);
static void UartWriteBytes(const char *buffer, uint16_t length);

static uint32_t App_Millis(void)
{
    OS_ERR err;
    OS_TICK ticks = OSTimeGet(&err);
    (void)err;
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / OSCfg_TickRate_Hz);
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
    GPIO_WriteBit(port, pin, active ? active_level : InactiveLevel(active_level));
}

static bool PinIsPressed(GPIO_TypeDef *port, uint16_t pin)
{
    return GPIO_ReadInputDataBit(port, pin) == APP_BUTTON_PRESSED_LEVEL;
}

/* Onboard B1 has the opposite polarity to the external buttons. */
static bool B1IsPressed(void)
{
    return GPIO_ReadInputDataBit(APP_B1_PORT, APP_B1_PIN) == APP_B1_PRESSED_LEVEL;
}

static int16_t AppFloatToTenths(float value)
{
    float scaled = value * 10.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)scaled;
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

static uint32_t TimerElapsedUs(uint32_t start_us)
{
    return (uint32_t)(TIM_GetCounter(APP_US_TIMER) - start_us);
}

static void DelayUs(uint16_t microseconds)
{
    uint32_t start_us = TIM_GetCounter(APP_US_TIMER);
    uint32_t guard = 0U;

    while (TimerElapsedUs(start_us) < microseconds) {
        if (++guard >= APP_BUSY_WAIT_GUARD) {
            break;
        }
    }
}

static bool UsTimerIsRunning(void)
{
    uint32_t guard = 0U;

    TIM_SetCounter(APP_US_TIMER, 0U);
    while (TIM_GetCounter(APP_US_TIMER) == 0U) {
        if (++guard >= APP_BUSY_WAIT_GUARD) {
            return false;
        }
    }
    return true;
}

static void DhtSetOutput(void)
{
    GPIO_InitTypeDef config;
    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = APP_DHT_PIN;
    config.GPIO_Mode = GPIO_Mode_OUT;
    config.GPIO_OType = GPIO_OType_OD;
    config.GPIO_PuPd = GPIO_PuPd_UP;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(APP_DHT_PORT, &config);
}

static void DhtSetInput(void)
{
    GPIO_InitTypeDef config;
    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = APP_DHT_PIN;
    config.GPIO_Mode = GPIO_Mode_IN;
    config.GPIO_OType = GPIO_OType_OD;
    config.GPIO_PuPd = GPIO_PuPd_UP;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(APP_DHT_PORT, &config);
}

static bool DhtMeasurePulse(GPIO_PinState level,
                            uint16_t *duration_us,
                            uint32_t deadline_start_us)
{
    uint32_t pulse_start_us;
    uint32_t guard = 0U;

    while (GPIO_ReadInputDataBit(APP_DHT_PORT, APP_DHT_PIN) != level) {
        if (TimerElapsedUs(deadline_start_us) >= APP_DHT_READ_TIMEOUT_US ||
            ++guard >= APP_DHT_LOOP_GUARD) {
            return false;
        }
    }
    pulse_start_us = TIM_GetCounter(APP_US_TIMER);
    guard = 0U;
    while (GPIO_ReadInputDataBit(APP_DHT_PORT, APP_DHT_PIN) == level) {
        if (TimerElapsedUs(deadline_start_us) >= APP_DHT_READ_TIMEOUT_US ||
            TimerElapsedUs(pulse_start_us) >= APP_DHT_PULSE_TIMEOUT_US ||
            ++guard >= APP_DHT_LOOP_GUARD) {
            return false;
        }
    }
    *duration_us = (uint16_t)TimerElapsedUs(pulse_start_us);
    return true;
}

static bool DhtRead(float *temperature_c, float *humidity_pct)
{
    uint8_t data[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t byte_index;
    uint8_t bit_index;
    uint16_t low_us = 0U;
    uint16_t high_us = 0U;
    uint16_t resp_low_us = 0U;
    uint16_t resp_high_us = 0U;
    uint16_t bit_low_us = 0U;
    uint16_t bit_high_us = 0U;
    uint16_t raw_humidity;
    uint16_t raw_temperature;
    bool read_ok = true;
    bool checksum_ok = false;
    bool dht_result = false;
    uint32_t deadline_start_us;
    uint32_t primask;
    /* Local error code; copied to the volatile monitor var only after the
     * critical section so UART/monitor never reads a half-updated value and we
     * keep a single write-back point. */
    uint8_t last_error = (uint8_t)DHT_ERR_NONE;

    /* No microsecond timer means every pulse measurement is meaningless.
     * Report it before touching interrupts; nothing was disabled yet. */
    if (!UsTimerReady) {
        MonitorDhtLastError = (uint32_t)DHT_ERR_TIMER_NOT_READY;
        return false;
    }

    MonitorDhtStage = (uint8_t)DHT_STAGE_ENTER;
    DhtSetInput();
    App_DelayMs(2U);
    DhtSetOutput();
    GPIO_WriteBit(APP_DHT_PORT, APP_DHT_PIN, GPIO_PIN_RESET);
    DelayUs(APP_DHT22_START_US);
    MonitorDhtStage = (uint8_t)DHT_STAGE_START_PULSE;

    primask = __get_PRIMASK();
    MonitorDhtStage = (uint8_t)DHT_STAGE_CRITICAL_ENTER;
#if APP_DHT_DISABLE_IRQ
    __disable_irq();
#endif
    /* --- timing-critical: no UART, no OS calls, no early return below ------ */

    GPIO_WriteBit(APP_DHT_PORT, APP_DHT_PIN, GPIO_PIN_SET);
    DhtSetInput();
    deadline_start_us = TIM_GetCounter(APP_US_TIMER);
    DelayUs(APP_DHT22_PULL_US);

    do {
        /* Response phase: sensor pulls the line low ~80us then high ~80us. */
        if (!DhtMeasurePulse(GPIO_PIN_RESET, &resp_low_us,
                             deadline_start_us)) {
            last_error = (uint8_t)DHT_ERR_RESPONSE_LOW_TIMEOUT;
            read_ok = false;
            break;
        }
        MonitorDhtStage = (uint8_t)DHT_STAGE_RESP_LOW;
        if (!DhtMeasurePulse(GPIO_PIN_SET, &resp_high_us,
                             deadline_start_us)) {
            last_error = (uint8_t)DHT_ERR_RESPONSE_HIGH_TIMEOUT;
            read_ok = false;
            break;
        }
        MonitorDhtStage = (uint8_t)DHT_STAGE_RESP_HIGH;
        /* Data phase: 40 bits, each a ~50us low followed by a 26-70us high. */
        for (byte_index = 0U; byte_index < 5U; ++byte_index) {
            for (bit_index = 0U; bit_index < 8U; ++bit_index) {
                if (!DhtMeasurePulse(GPIO_PIN_RESET, &low_us,
                                     deadline_start_us)) {
                    last_error = (uint8_t)DHT_ERR_DATA_LOW_TIMEOUT;
                    read_ok = false;
                    break;
                }
                if (!DhtMeasurePulse(GPIO_PIN_SET, &high_us,
                                     deadline_start_us)) {
                    last_error = (uint8_t)DHT_ERR_DATA_HIGH_TIMEOUT;
                    read_ok = false;
                    break;
                }
                bit_low_us = low_us;
                bit_high_us = high_us;
                data[byte_index] <<= 1U;
                if (high_us > APP_DHT_BIT_ONE_THRESHOLD_US) {
                    data[byte_index] |= 1U;
                }
            }
            if (!read_ok) {
                break;
            }
        }
        if (read_ok) {
            MonitorDhtStage = (uint8_t)DHT_STAGE_DATA_DONE;
        }
    } while (false);

#if APP_DHT_DISABLE_IRQ
    if (primask == 0U) {
        __enable_irq();
    }
#else
    (void)primask;
#endif
    MonitorDhtStage = (uint8_t)DHT_STAGE_CRITICAL_EXIT;
    /* --- interrupts restored: UART/monitor writes are safe again ---------- */

    if (read_ok) {
        checksum_ok =
            ((uint8_t)(data[0] + data[1] + data[2] + data[3]) == data[4]);
        if (!checksum_ok) {
            last_error = (uint8_t)DHT_ERR_CHECKSUM;
        }
    }

    if (read_ok && checksum_ok) {
        raw_humidity = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
        raw_temperature = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
        *humidity_pct = (float)raw_humidity * 0.1f;
        if ((raw_temperature & 0x8000U) != 0U) {
            raw_temperature &= 0x7FFFU;
            *temperature_c = -((float)raw_temperature * 0.1f);
        } else {
            *temperature_c = (float)raw_temperature * 0.1f;
        }
        dht_result = true;
        last_error = (uint8_t)DHT_ERR_NONE;
    }

    /* Single write-back of the diagnostic snapshot after interrupts restored.
     * On failure these hold whatever was captured up to the failing stage. */
    MonitorDhtLastRespLowUs = resp_low_us;
    MonitorDhtLastRespHighUs = resp_high_us;
    MonitorDhtLastBitLowUs = bit_low_us;
    MonitorDhtLastBitHighUs = bit_high_us;
    MonitorDhtLastByte0 = data[0];
    MonitorDhtLastByte1 = data[1];
    MonitorDhtLastByte2 = data[2];
    MonitorDhtLastByte3 = data[3];
    MonitorDhtLastByte4 = data[4];
    MonitorDhtLastError = (uint32_t)last_error;
    if (dht_result) {
        MonitorDhtStage = (uint8_t)DHT_STAGE_DONE;
    }
    return dht_result;
}

static uint16_t AppStrLen(const char *text)
{
    uint16_t length = 0U;

    while (text[length] != '\0') {
        ++length;
    }
    return length;
}

static void UartWriteBytes(const char *buffer, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        uint32_t guard = 0U;
        while (USART_GetFlagStatus(APP_UART, USART_FLAG_TXE) == RESET) {
            if (++guard >= APP_UART_TX_TIMEOUT_GUARD) {
                return;
            }
        }
        USART_SendData(APP_UART, (uint16_t)((uint8_t)buffer[index]));
    }
}

static void UartWriteStringUnlocked(const char *text)
{
    UartWriteBytes(text, AppStrLen(text));
}

static void UartWriteUIntUnlocked(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        UartWriteBytes("0", 1U);
        return;
    }

    while (value != 0U && count < sizeof(digits)) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) {
        --count;
        UartWriteBytes(&digits[count], 1U);
    }
}

static void UartWriteTenthsUnlocked(int16_t value)
{
    int32_t signed_value = (int32_t)value;
    uint32_t whole;
    uint32_t fraction;

    if (signed_value < 0) {
        UartWriteBytes("-", 1U);
        signed_value = -signed_value;
    }
    whole = (uint32_t)(signed_value / 10);
    fraction = (uint32_t)(signed_value % 10);
    UartWriteUIntUnlocked(whole);
    UartWriteBytes(".", 1U);
    UartWriteUIntUnlocked(fraction);
}

static void UartLog(const char *text)
{
    OS_ERR err;

    OSMutexPend(&UartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        UartWriteStringUnlocked(text);
        OSMutexPost(&UartMutex, OS_OPT_POST_NONE, &err);
    }
}

static void UartLogTaskCreateFailed(const CPU_CHAR *name, OS_ERR task_err)
{
    OS_ERR err;

    OSMutexPend(&UartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        UartWriteStringUnlocked("Task create failed: ");
        UartWriteStringUnlocked((const char *)name);
        UartWriteStringUnlocked(" err=");
        UartWriteUIntUnlocked((uint32_t)task_err);
        UartWriteStringUnlocked("\r\n");
        OSMutexPost(&UartMutex, OS_OPT_POST_NONE, &err);
    }
}

static void UartLogInputs(void)
{
    OS_ERR err;
    bool up_pressed = PinIsPressed(APP_UP_PORT, APP_UP_PIN);
    bool down_pressed = PinIsPressed(APP_DOWN_PORT, APP_DOWN_PIN);
    bool b1_pressed = B1IsPressed();
    bool reed_open =
        GPIO_ReadInputDataBit(APP_REED_PORT, APP_REED_PIN) ==
        APP_REED_OPEN_LEVEL;
    bool dht_valid = MonitorDhtValid;
    int16_t temperature_tenths = MonitorDhtTemperatureTenths;
    int16_t humidity_tenths = MonitorDhtHumidityTenths;
    uint32_t dht_fail_count = MonitorDhtFailCount;
    uint32_t dht_err = MonitorDhtLastError;
    uint16_t resp_low_us = MonitorDhtLastRespLowUs;
    uint16_t resp_high_us = MonitorDhtLastRespHighUs;
    uint16_t bit_low_us = MonitorDhtLastBitLowUs;
    uint16_t bit_high_us = MonitorDhtLastBitHighUs;
    uint8_t raw0 = MonitorDhtLastByte0;
    uint8_t raw1 = MonitorDhtLastByte1;
    uint8_t raw2 = MonitorDhtLastByte2;
    uint8_t raw3 = MonitorDhtLastByte3;
    uint8_t raw4 = MonitorDhtLastByte4;
    bool us_timer_ready = UsTimerReady;
    uint32_t us_ticks_probe = MonitorUsTicksPerProbe;
    uint8_t dht_stage = MonitorDhtStage;
    uint8_t tm1637_ack = MonitorTm1637AckCount;
    /* SensorTask free stack in words. If this trends toward 0 the task is about
     * to overflow and HardFault; that is the prime suspect for the freeze. */
    CPU_STK_SIZE sensor_stk_free = 0U;
    CPU_STK_SIZE sensor_stk_used = 0U;
    OS_ERR stk_err;
    OSTaskStkChk(&SensorTaskTCB, &sensor_stk_free, &sensor_stk_used, &stk_err);

    OSMutexPend(&UartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        UartWriteStringUnlocked("INPUT button_up=");
        UartWriteBytes(up_pressed ? "1" : "0", 1U);
        UartWriteStringUnlocked(" button_down=");
        UartWriteBytes(down_pressed ? "1" : "0", 1U);
        UartWriteStringUnlocked(" button_b1=");
        UartWriteBytes(b1_pressed ? "1" : "0", 1U);
        UartWriteStringUnlocked(" reed_open=");
        UartWriteBytes(reed_open ? "1" : "0", 1U);
        UartWriteStringUnlocked(" dht22=");
        UartWriteBytes(dht_valid ? "1" : "0", 1U);
        UartWriteStringUnlocked(" temp_c=");
        if (dht_valid) {
            UartWriteTenthsUnlocked(temperature_tenths);
        } else {
            UartWriteStringUnlocked("--.-");
        }
        UartWriteStringUnlocked(" humidity_pct=");
        if (dht_valid) {
            UartWriteTenthsUnlocked(humidity_tenths);
        } else {
            UartWriteStringUnlocked("--.-");
        }
        UartWriteStringUnlocked(" dht_fail_count=");
        UartWriteUIntUnlocked(dht_fail_count);
        UartWriteStringUnlocked(" dht_err=");
        UartWriteUIntUnlocked(dht_err);
        UartWriteStringUnlocked(" dht_stage=");
        UartWriteUIntUnlocked((uint32_t)dht_stage);
        UartWriteStringUnlocked(" resp_low_us=");
        UartWriteUIntUnlocked((uint32_t)resp_low_us);
        UartWriteStringUnlocked(" resp_high_us=");
        UartWriteUIntUnlocked((uint32_t)resp_high_us);
        UartWriteStringUnlocked(" bit_low_us=");
        UartWriteUIntUnlocked((uint32_t)bit_low_us);
        UartWriteStringUnlocked(" bit_high_us=");
        UartWriteUIntUnlocked((uint32_t)bit_high_us);
        UartWriteStringUnlocked(" us_timer_ready=");
        UartWriteBytes(us_timer_ready ? "1" : "0", 1U);
        UartWriteStringUnlocked(" us_ticks_100ms=");
        UartWriteUIntUnlocked(us_ticks_probe);
        UartWriteStringUnlocked(" sensor_stk_free=");
        UartWriteUIntUnlocked((uint32_t)sensor_stk_free);
        UartWriteStringUnlocked(" tm1637_ack=");
        UartWriteUIntUnlocked((uint32_t)tm1637_ack);
        UartWriteStringUnlocked(" raw=");
        UartWriteUIntUnlocked((uint32_t)raw0);
        UartWriteBytes(",", 1U);
        UartWriteUIntUnlocked((uint32_t)raw1);
        UartWriteBytes(",", 1U);
        UartWriteUIntUnlocked((uint32_t)raw2);
        UartWriteBytes(",", 1U);
        UartWriteUIntUnlocked((uint32_t)raw3);
        UartWriteBytes(",", 1U);
        UartWriteUIntUnlocked((uint32_t)raw4);
        UartWriteStringUnlocked("\r\n");
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
        SensorSlotIndex =
            (uint8_t)((SensorSlotIndex + 1U) % APP_QUEUE_SLOT_COUNT);
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
    slot->temperature_valid = output->temperature_valid;
    OSQPost(&DisplayQ, slot, sizeof(*slot), OS_OPT_POST_FIFO, &err);
    if (err == OS_ERR_NONE) {
        DisplaySlotIndex =
            (uint8_t)((DisplaySlotIndex + 1U) % APP_QUEUE_SLOT_COUNT);
    }
}

static void PostActuator(const ControlOutput *output, bool buzzer_click)
{
    OS_ERR err;
    ActuatorMsg *slot = &ActuatorSlots[ActuatorSlotIndex];
    slot->state = output->state;
    slot->cooling_level = output->cooling_level;
    slot->buzzer_click = buzzer_click;
    OSQPost(&ActuatorQ, slot, sizeof(*slot), OS_OPT_POST_FIFO, &err);
    if (err == OS_ERR_NONE) {
        ++ActuatorPostCount;
        ActuatorSlotIndex =
            (uint8_t)((ActuatorSlotIndex + 1U) % APP_QUEUE_SLOT_COUNT);
    } else {
        ++ActuatorDropCount;
    }
}

static void Tm1637Delay(void)
{
    DelayUs(APP_TM1637_DELAY_US);
}

static void Tm1637WriteClk(bool high)
{
    GPIO_WriteBit(APP_TM1637_CLK_PORT, APP_TM1637_CLK_PIN,
                  high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Tm1637WriteDio(bool high)
{
    GPIO_WriteBit(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN,
                  high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Tm1637Start(void)
{
    Tm1637WriteDio(true);
    Tm1637WriteClk(true);
    Tm1637Delay();
    Tm1637WriteDio(false);
    Tm1637Delay();
    Tm1637WriteClk(false);
}

static void Tm1637Stop(void)
{
    Tm1637WriteClk(false);
    Tm1637WriteDio(false);
    Tm1637Delay();
    Tm1637WriteClk(true);
    Tm1637Delay();
    Tm1637WriteDio(true);
    Tm1637Delay();
}

static bool Tm1637WriteByte(uint8_t value)
{
    uint8_t bit_index;
    bool ack;

    for (bit_index = 0U; bit_index < 8U; ++bit_index) {
        Tm1637WriteClk(false);
        Tm1637WriteDio((value & 0x01U) != 0U);
        Tm1637Delay();
        Tm1637WriteClk(true);
        Tm1637Delay();
        value >>= 1U;
    }

    Tm1637WriteClk(false);
    Tm1637WriteDio(true);
    Tm1637Delay();
    Tm1637WriteClk(true);
    Tm1637Delay();
    ack = GPIO_ReadInputDataBit(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN) ==
          GPIO_PIN_RESET;
    Tm1637WriteClk(false);
    return ack;
}

static void Tm1637SetSegments(const uint8_t segments[4])
{
    uint8_t index;
    uint8_t ack_count = 0U;
    uint8_t display_cmd =
        (uint8_t)(0x88U | (APP_TM1637_BRIGHTNESS & 0x07U));

    Tm1637Start();
    if (Tm1637WriteByte(0x40U)) { ++ack_count; }
    Tm1637Stop();

    Tm1637Start();
    if (Tm1637WriteByte(0xC0U)) { ++ack_count; }
    for (index = 0U; index < 4U; ++index) {
        if (Tm1637WriteByte(segments[index])) { ++ack_count; }
    }
    Tm1637Stop();

    Tm1637Start();
    if (Tm1637WriteByte(display_cmd)) { ++ack_count; }
    Tm1637Stop();

    /* 7 = module ACKed every byte (comms OK). 0 = no response. */
    MonitorTm1637AckCount = ack_count;
}

static void Tm1637ClearSegments(uint8_t segments[4])
{
    uint8_t index;

    for (index = 0U; index < 4U; ++index) {
        segments[index] = 0x00U;
    }
}

static uint8_t Tm1637DigitSegment(uint8_t digit)
{
    static const uint8_t digit_segments[10] = {
        0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
        0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
    };

    return (digit < 10U) ? digit_segments[digit] : 0x00U;
}

static uint8_t Tm1637CharSegment(char value)
{
    switch (value) {
    case '0':
    case 'O':
        return 0x3FU;
    case 'E':
        return 0x79U;
    case 'F':
        return 0x71U;
    case 'L':
        return 0x38U;
    case 'P':
        return 0x73U;
    case 'S':
        return 0x6DU;
    case 'A':
        return 0x77U;
    case 'U':
        return 0x3EU;
    case '-':
        return 0x40U;
    case ' ':
        return 0x00U;
    default:
        return 0x00U;
    }
}

/* Render a value into two adjacent cells (*hi = tens, *lo = ones).
 * value is clamped to 0..99. zero_pad selects "03" vs " 3" for values < 10.
 * A negative value renders as '-' followed by a single clamped digit ("-5"). */
static void Tm1637PutTwoDigits(int16_t value, bool zero_pad,
                               uint8_t *hi, uint8_t *lo)
{
    if (value < 0) {
        value = (int16_t)(-value);
        if (value > 9) {
            value = 9;
        }
        *hi = Tm1637CharSegment('-');
        *lo = Tm1637DigitSegment((uint8_t)value);
        return;
    }
    if (value > 99) {
        value = 99;
    }
    if (value >= 10) {
        *hi = Tm1637DigitSegment((uint8_t)(value / 10));
    } else {
        *hi = zero_pad ? Tm1637DigitSegment(0U) : Tm1637CharSegment(' ');
    }
    *lo = Tm1637DigitSegment((uint8_t)(value % 10));
}

/* Two temperature cells with validity handling: "--" when no valid reading,
 * otherwise the temperature as a blank-leading 2-digit value. */
static void Tm1637PutTemperature(const DisplayMsg *message,
                                 uint8_t *hi, uint8_t *lo)
{
    if (!message->temperature_valid) {
        *hi = Tm1637CharSegment('-');
        *lo = Tm1637CharSegment('-');
    } else {
        Tm1637PutTwoDigits((int16_t)message->current_temperature_c,
                           false, hi, lo);
    }
}

/* Four literal characters across all four digits, e.g. "OFF ", "PAUS", "----". */
static void Tm1637RenderChars(char c0, char c1, char c2, char c3,
                              uint8_t segments[4])
{
    segments[0] = Tm1637CharSegment(c0);
    segments[1] = Tm1637CharSegment(c1);
    segments[2] = Tm1637CharSegment(c2);
    segments[3] = Tm1637CharSegment(c3);
}

/* Front two digits = left_value, back two = right_value. separator_dp lights
 * the decimal point on digit 1, so 24 / 23 reads as "24.23". */
static void Tm1637RenderTwoTwoNumbers(int16_t left_value, int16_t right_value,
                                      bool separator_dp, uint8_t segments[4])
{
    Tm1637PutTwoDigits(left_value, false, &segments[0], &segments[1]);
    Tm1637PutTwoDigits(right_value, true, &segments[2], &segments[3]);
    if (separator_dp) {
        segments[1] |= 0x80U;
    }
}

/* Front two = temperature (with validity), third = level prefix ('L' or 'A'),
 * fourth = level digit. Gives "24L2"/"24A2", or "--L2"/"--A2" when invalid.
 * Takes the message (not a raw int) so the invalid-temperature case is handled
 * in one place. */
static void Tm1637RenderTempAndLevel(const DisplayMsg *message,
                                     char level_prefix, uint8_t level,
                                     uint8_t segments[4])
{
    Tm1637PutTemperature(message, &segments[0], &segments[1]);
    segments[2] = Tm1637CharSegment(level_prefix);
    segments[3] = Tm1637DigitSegment((uint8_t)(level % 10U));
}

/* Front two = a two-char code, back two = a 0..99 second count, zero padded.
 * Gives "OP03", "ES04". */
static void Tm1637RenderCodeAndSeconds(char c0, char c1, uint8_t seconds,
                                       uint8_t segments[4])
{
    segments[0] = Tm1637CharSegment(c0);
    segments[1] = Tm1637CharSegment(c1);
    Tm1637PutTwoDigits((int16_t)seconds, true, &segments[2], &segments[3]);
}

static void Tm1637BuildSegments(const DisplayMsg *message, uint32_t now_ms,
                                uint8_t segments[4])
{
    /* Method A: track the time the displayed state was entered so the
     * window-open elapsed seconds and recovery-wait remaining seconds can be
     * derived here without changing the control FSM. Safe as plain statics
     * because Tm1637BuildSegments is only ever called from DisplayTask. */
    static AcState last_display_state = AC_STATE_OFF;
    static uint32_t display_state_since_ms = 0U;
    uint32_t elapsed_ms;
    uint32_t seconds;

    if (message->state != last_display_state) {
        last_display_state = message->state;
        display_state_since_ms = now_ms;
    }
    elapsed_ms = now_ms - display_state_since_ms;

    Tm1637ClearSegments(segments);

    switch (message->state) {
    case AC_STATE_OFF:
        /* "OFF " for the first half-second, blank for the second: 1 Hz blink. */
        if ((now_ms % 1000U) < 500U) {
            Tm1637RenderChars('O', 'F', 'F', ' ', segments);
        }
        break;
    case AC_STATE_IDLE:
        if (message->temperature_valid) {
            Tm1637RenderTwoTwoNumbers((int16_t)message->current_temperature_c,
                                      (int16_t)message->setpoint_c,
                                      true, segments);
        } else {
            Tm1637RenderChars('-', '-', '-', '-', segments);
        }
        break;
    case AC_STATE_COOLING:
        Tm1637RenderTempAndLevel(message, 'L', message->cooling_level, segments);
        break;
    case AC_STATE_WINDOW_SUSPECT:
        seconds = elapsed_ms / 1000U;
        if (seconds > 99U) {
            seconds = 99U;
        }
        Tm1637RenderCodeAndSeconds('O', 'P', (uint8_t)seconds, segments);
        break;
    case AC_STATE_AC_PAUSED:
        Tm1637RenderChars('P', 'A', 'U', 'S', segments);
        break;
    case AC_STATE_RECOVERY_WAIT:
        {
            uint32_t remaining_ms = (elapsed_ms >= APP_RECOVERY_WAIT_MS)
                                        ? 0U
                                        : (APP_RECOVERY_WAIT_MS - elapsed_ms);
            /* Round up so a 5 s wait counts 05,04,...,01,00 one per second. */
            seconds = (remaining_ms + 999U) / 1000U;
            if (seconds > 99U) {
                seconds = 99U;
            }
            Tm1637RenderCodeAndSeconds('E', 'S', (uint8_t)seconds, segments);
        }
        break;
    case AC_STATE_MANUAL_OVERRIDE:
        Tm1637RenderTempAndLevel(message, 'A', message->cooling_level, segments);
        break;
    default:
        Tm1637RenderChars('-', '-', '-', '-', segments);
        break;
    }
}

static void Tm1637Init(void)
{
    uint8_t segments[4];

    Tm1637WriteClk(true);
    Tm1637WriteDio(true);
    Tm1637ClearSegments(segments);
    Tm1637SetSegments(segments);
}

static void SetActuators(const ActuatorMsg *message, uint32_t now_ms,
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
    WriteActive(APP_LD1_PORT, APP_LD1_PIN, ld1, GPIO_PIN_SET);
    WriteActive(APP_LD2_PORT, APP_LD2_PIN, ld2, GPIO_PIN_SET);
    WriteActive(APP_LD3_PORT, APP_LD3_PIN, ld3, GPIO_PIN_SET);
    WriteActive(APP_BUZZER_PORT, APP_BUZZER_PIN, buzzer, APP_BUZZER_ON_LEVEL);
}

static void SensorTask(void *argument)
{
    SensorMsg message;
    bool last_window = false;
    bool last_dht_valid = false;
    uint32_t last_dht_ms = 0U;
    (void)argument;
    memset(&message, 0, sizeof(message));

    App_DelayMs(1200U);

    if (!UsTimerReady) {
        UartLog("Microsecond timer not running\r\n");
    }

    /* One-shot TIM2 rate probe. Done in task context (no critical section) so
     * the OS delay is real time. delta ~ APP_US_TIMER_PROBE_MS*1000 means the
     * timer is ~1 MHz; a much smaller number means the prescaler/clock is wrong
     * and every microsecond-timeout in DhtRead is ineffective. */
    {
        uint32_t probe_start = TIM_GetCounter(APP_US_TIMER);
        App_DelayMs(APP_US_TIMER_PROBE_MS);
        MonitorUsTicksPerProbe =
            (uint32_t)(TIM_GetCounter(APP_US_TIMER) - probe_start);
    }
#if APP_DHT_DEBUG_LOG
    UartLog("TIM2 ticks/probe=");
    {
        OS_ERR probe_err;
        OSMutexPend(&UartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &probe_err);
        if (probe_err == OS_ERR_NONE) {
            UartWriteUIntUnlocked(MonitorUsTicksPerProbe);
            UartWriteStringUnlocked(" (expect ~");
            UartWriteUIntUnlocked((uint32_t)APP_US_TIMER_PROBE_MS * 1000U);
            UartWriteStringUnlocked(" for 1MHz)\r\n");
            OSMutexPost(&UartMutex, OS_OPT_POST_NONE, &probe_err);
        }
    }
#endif

    for (;;) {
        uint32_t now = App_Millis();
        bool window_open =
            GPIO_ReadInputDataBit(APP_REED_PORT, APP_REED_PIN) ==
            APP_REED_OPEN_LEVEL;
        bool publish = (window_open != last_window) || TakeReedEdge();

        if ((now - last_dht_ms) >= APP_DHT_READ_PERIOD_MS ||
            last_dht_ms == 0U) {
#if APP_DHT_DEBUG_LOG
            UartLog("DHT before\r\n");
#endif
#if APP_DHT_READ_ENABLE
            message.valid = DhtRead(&message.temperature_c,
                                    &message.humidity_pct);
#else
            message.valid = false;
#endif
#if APP_DHT_DEBUG_LOG
            UartLog(message.valid ? "DHT ok\r\n" : "DHT fail\r\n");
#endif
            last_dht_valid = message.valid;
            MonitorDhtValid = message.valid;
            if (message.valid) {
                MonitorDhtTemperatureTenths =
                    AppFloatToTenths(message.temperature_c);
                MonitorDhtHumidityTenths =
                    AppFloatToTenths(message.humidity_pct);
            } else {
                ++MonitorDhtFailCount;
            }
            message.window_open = window_open;
            publish = true;
            last_dht_ms = now;
        } else if (publish) {
            message.window_open = window_open;
            message.valid = last_dht_valid;
        }
        if (publish) {
            PostSensor(&message);
            last_window = window_open;
        }
        App_DelayMs(100U);
    }
}

/* Polled software debounce for a single button. A new level is accepted only
 * after it has been read on APP_DEBOUNCE_SAMPLES consecutive polls
 * (~APP_DEBOUNCE_MS of stability); a level that flips back before then is
 * rejected as bounce. DebounceButton returns true exactly once, on the
 * released->pressed transition, so one physical press yields one event. */
typedef struct {
    bool pressed;     /* debounced (stable) state */
    uint8_t count;    /* consecutive samples disagreeing with the stable state */
} ButtonDebounce;

static bool DebounceButton(ButtonDebounce *db, bool raw_pressed)
{
    if (raw_pressed == db->pressed) {
        db->count = 0U;
        return false;
    }
    if (++db->count < APP_DEBOUNCE_SAMPLES) {
        return false;
    }
    db->count = 0U;
    db->pressed = raw_pressed;
    return db->pressed;   /* true only on the confirmed press edge */
}

static void UserInputTask(void *argument)
{
    bool b1_was_pressed = false;
    uint32_t b1_pressed_since = 0U;
    ButtonDebounce up_db = {false, 0U};
    ButtonDebounce down_db = {false, 0U};
    (void)argument;

    for (;;) {
        uint32_t events = 0U;
        bool b1_pressed;
        OS_ERR err;

        /* EXTI still fires on these pins; drain the raw edge bits so they do
         * not accumulate. UP/DOWN are now handled purely by polled debounce. */
        (void)TakeRawEdges();

        if (DebounceButton(&up_db,
                           PinIsPressed(APP_UP_PORT, APP_UP_PIN))) {
            events |= USER_EVT_SETPOINT_UP;
        }
        if (DebounceButton(&down_db,
                           PinIsPressed(APP_DOWN_PORT, APP_DOWN_PIN))) {
            events |= USER_EVT_SETPOINT_DOWN;
        }

        b1_pressed = B1IsPressed();
        if (b1_pressed && !b1_was_pressed) {
            b1_pressed_since = App_Millis();
        } else if (!b1_pressed && b1_was_pressed) {
            events |= USER_EVT_BUZZER_CLICK;
            events |= ((App_Millis() - b1_pressed_since) >= APP_LONG_PRESS_MS)
                          ? USER_EVT_POWER : USER_EVT_MANUAL;
        }
        b1_was_pressed = b1_pressed;

        if (events != 0U) {
            OSFlagPost(&UserEventFlags, events, OS_OPT_POST_FLAG_SET, &err);
        }
        App_DelayMs(APP_BUTTON_POLL_MS);
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
                                 USER_EVT_MANUAL | USER_EVT_POWER |
                                 USER_EVT_BUZZER_CLICK,
            0U, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME |
                    OS_OPT_PEND_NON_BLOCKING,
            &timestamp, &err);
        ControlOutput output;
        bool buzzer_click = (events & USER_EVT_BUZZER_CLICK) != 0U;

        if (received != NULL && size == sizeof(SensorMsg)) {
            latest = *received;
            have_latest = true;
        }
        output = Control_Step(&context, have_latest ? &latest : NULL,
                              events, App_Millis());
        if (buzzer_click || !have_last_output ||
            output.state != last_output.state ||
            output.cooling_level != last_output.cooling_level ||
            output.current_temperature_c != last_output.current_temperature_c ||
            output.setpoint_c != last_output.setpoint_c ||
            output.temperature_valid != last_output.temperature_valid) {
            PostDisplay(&output);
            PostActuator(&output, buzzer_click);
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
    uint8_t segments[4];
    (void)argument;
    memset(&current, 0, sizeof(current));
    current.state = AC_STATE_OFF;
    current.setpoint_c = APP_SETPOINT_DEFAULT_C;

    Tm1637Init();

    for (;;) {
        OS_ERR err;
        OS_MSG_SIZE size;
        CPU_TS timestamp;
        DisplayMsg *received =
            (DisplayMsg *)OSQPend(&DisplayQ, 0U, OS_OPT_PEND_NON_BLOCKING,
                                  &size, &timestamp, &err);
        if (received != NULL && size == sizeof(DisplayMsg)) {
            current = *received;
        }
        Tm1637BuildSegments(&current, App_Millis(), segments);
        Tm1637SetSegments(segments);
        App_DelayMs(100U);
    }
}

static void ActuatorTask(void *argument)
{
    ActuatorMsg current;
    uint32_t buzzer_until_ms = 0U;
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
        uint32_t now = App_Millis();
        if (received != NULL && size == sizeof(ActuatorMsg)) {
            current = *received;
            if (current.buzzer_click) {
                buzzer_until_ms = now + APP_BUZZER_CLICK_MS;
            }
        }
        SetActuators(&current, now,
                     ((int32_t)(now - buzzer_until_ms) < 0));
        App_DelayMs(25U);
    }
}

static void MonitorTask(void *argument)
{
    (void)argument;
    App_DelayMs(APP_MONITOR_START_DELAY_MS);
    for (;;) {
        UartLogInputs();
        App_DelayMs(APP_MONITOR_PERIOD_MS);
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
        UartLogTaskCreateFailed(name, err);
    }
}

static void ConfigureGpioInput(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef config;

    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = pins;
    config.GPIO_Mode = GPIO_Mode_IN;
    config.GPIO_OType = GPIO_OType_PP;
    config.GPIO_PuPd = GPIO_PuPd_UP;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &config);
}

/* Active-high input with an internal pull-down so the idle level is a defined
 * LOW (used for the onboard active-high B1 button). */
static void ConfigureGpioInputPullDown(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef config;

    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = pins;
    config.GPIO_Mode = GPIO_Mode_IN;
    config.GPIO_OType = GPIO_OType_PP;
    config.GPIO_PuPd = GPIO_PuPd_DOWN;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &config);
}

static void ConfigureGpioOutput(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef config;

    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = pins;
    config.GPIO_Mode = GPIO_Mode_OUT;
    config.GPIO_OType = GPIO_OType_PP;
    config.GPIO_PuPd = GPIO_PuPd_NOPULL;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &config);
}

static void ConfigureGpioOpenDrain(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef config;

    memset(&config, 0, sizeof(config));
    config.GPIO_Pin = pins;
    config.GPIO_Mode = GPIO_Mode_OUT;
    config.GPIO_OType = GPIO_OType_OD;
    config.GPIO_PuPd = GPIO_PuPd_UP;
    config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &config);
}

static void Board_InitGpio(void)
{
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOC);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOE);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOF);

    ConfigureGpioInput(APP_DHT_PORT, APP_DHT_PIN);
    ConfigureGpioInput(APP_REED_PORT, APP_REED_PIN);
    ConfigureGpioInput(APP_UP_PORT, APP_UP_PIN);
    ConfigureGpioInput(APP_DOWN_PORT, APP_DOWN_PIN);
    /* Onboard B1/PC13 is active-high on NUCLEO-144: pull-down, not pull-up. */
    ConfigureGpioInputPullDown(APP_B1_PORT, APP_B1_PIN);

    ConfigureGpioOutput(APP_BUZZER_PORT, APP_BUZZER_PIN);
    ConfigureGpioOutput(APP_LD1_PORT, APP_LD1_PIN);
    ConfigureGpioOutput(APP_LD2_PORT, APP_LD2_PIN);
    ConfigureGpioOutput(APP_LD3_PORT, APP_LD3_PIN);
    /* CLK is driven only by the MCU (the TM1637 never drives it), so use a
     * push-pull output for clean, fast rising edges. Leaving it open-drain on
     * the weak ~40k internal pull-up gave slow edges that the TM1637 missed,
     * showing up as a fluctuating ACK count (7/3/4). DIO stays open-drain
     * because it is bidirectional (the TM1637 pulls it low to ACK). */
    ConfigureGpioOutput(APP_TM1637_CLK_PORT, APP_TM1637_CLK_PIN);
    ConfigureGpioOpenDrain(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN);
    WriteActive(APP_LD1_PORT, APP_LD1_PIN, false, GPIO_PIN_SET);
    WriteActive(APP_LD2_PORT, APP_LD2_PIN, false, GPIO_PIN_SET);
    WriteActive(APP_LD3_PORT, APP_LD3_PIN, false, GPIO_PIN_SET);
    WriteActive(APP_BUZZER_PORT, APP_BUZZER_PIN, false, APP_BUZZER_ON_LEVEL);
    Tm1637WriteClk(true);
    Tm1637WriteDio(true);
}

static void Board_InitUsart(void)
{
    GPIO_InitTypeDef gpio_config;
    USART_InitTypeDef usart_config;

    BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);
    BSP_PeriphEn(BSP_PERIPH_ID_USART3);

    USART_DeInit(APP_UART);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    memset(&gpio_config, 0, sizeof(gpio_config));
    gpio_config.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio_config.GPIO_Mode = GPIO_Mode_AF;
    gpio_config.GPIO_OType = GPIO_OType_PP;
    gpio_config.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio_config);

    memset(&usart_config, 0, sizeof(usart_config));
    usart_config.USART_BaudRate = 115200U;
    usart_config.USART_WordLength = USART_WordLength_8b;
    usart_config.USART_StopBits = USART_StopBits_1;
    usart_config.USART_Parity = USART_Parity_No;
    usart_config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(APP_UART, &usart_config);

    (void)APP_UART->SR;
    (void)APP_UART->DR;
    USART_Cmd(APP_UART, ENABLE);
}

static void Board_InitTimer(void)
{
    TIM_TimeBaseInitTypeDef timer_config;
    CPU_INT32U timer_clk_hz;
    CPU_INT32U prescaler;

    BSP_PeriphEn(BSP_PERIPH_ID_TIM2);

    timer_clk_hz = BSP_PeriphClkFreqGet(BSP_PERIPH_ID_TIM2);
    if (timer_clk_hz < BSP_CPU_ClkFreq()) {
        timer_clk_hz *= 2U;
    }
    prescaler = (timer_clk_hz / 1000000U) - 1U;

    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.TIM_Prescaler = (uint16_t)prescaler;
    timer_config.TIM_CounterMode = TIM_CounterMode_Up;
    timer_config.TIM_Period = 0xFFFFFFFFU;
    timer_config.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(APP_US_TIMER, &timer_config);
    TIM_SetCounter(APP_US_TIMER, 0U);
    TIM_Cmd(APP_US_TIMER, ENABLE);
}

static void Board_InitExti(void)
{
    EXTI_InitTypeDef exti_config;

    BSP_PeriphEn(BSP_PERIPH_ID_SYSCFG);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource12);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    memset(&exti_config, 0, sizeof(exti_config));
    exti_config.EXTI_Line = EXTI_Line0 | EXTI_Line3 |
                            EXTI_Line12 | EXTI_Line13;
    exti_config.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_config.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_config.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_config);

    EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line3 |
                           EXTI_Line12 | EXTI_Line13);

    BSP_IntVectSet(BSP_INT_ID_EXTI0, App_EXTI0_ISR);
    BSP_IntVectSet(BSP_INT_ID_EXTI3, App_EXTI3_ISR);
    BSP_IntVectSet(BSP_INT_ID_EXTI15_10, App_EXTI15_10_ISR);

    BSP_IntPrioSet(BSP_INT_ID_EXTI0, 10U);
    BSP_IntPrioSet(BSP_INT_ID_EXTI3, 10U);
    BSP_IntPrioSet(BSP_INT_ID_EXTI15_10, 10U);

    BSP_IntEn(BSP_INT_ID_EXTI0);
    BSP_IntEn(BSP_INT_ID_EXTI3);
    BSP_IntEn(BSP_INT_ID_EXTI15_10);
}

static void Board_Init(void)
{
    Board_InitGpio();
    Board_InitUsart();
    Board_InitTimer();
    Board_InitExti();
}

static void App_HandleGpioExti(uint16_t gpio_pin)
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

static void App_EXTI0_ISR(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        App_HandleGpioExti(APP_UP_PIN);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

static void App_EXTI3_ISR(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        App_HandleGpioExti(APP_REED_PIN);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

static void App_EXTI15_10_ISR(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        App_HandleGpioExti(APP_DOWN_PIN);
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
        App_HandleGpioExti(APP_B1_PIN);
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

/* DIAGNOSTIC: capture the real cause of the freeze. The TrueSTUDIO vector table
 * sends Hard Fault to App_Fault_ISR (a spin loop, now .weak in cstartup.s). We
 * override it to dump the Cortex-M fault status registers over UART (polling,
 * no OS calls) so a fault stops being a silent freeze. Disabled faults escalate
 * to HardFault but still set their CFSR sub-fields, so CFSR tells us the type. */
#define APP_FAULT_DUMP 1
#if APP_FAULT_DUMP

#define APP_SCB_CFSR   (*(volatile uint32_t *)0xE000ED28UL)
#define APP_SCB_HFSR   (*(volatile uint32_t *)0xE000ED2CUL)
#define APP_SCB_MMFAR  (*(volatile uint32_t *)0xE000ED34UL)
#define APP_SCB_BFAR   (*(volatile uint32_t *)0xE000ED38UL)

static void AppFaultWriteHex(uint32_t value)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    int8_t shift;

    UartWriteBytes("0x", 2U);
    for (shift = 28; shift >= 0; shift -= 4) {
        char nibble = hex_digits[(value >> (uint8_t)shift) & 0xFU];
        UartWriteBytes(&nibble, 1U);
    }
}

void App_Fault_ISR(void);
void App_Fault_ISR(void)
{
    uint32_t cfsr = APP_SCB_CFSR;
    uint32_t hfsr = APP_SCB_HFSR;
    uint32_t bfar = APP_SCB_BFAR;
    uint32_t mmfar = APP_SCB_MMFAR;

    UartWriteStringUnlocked("\r\n*** HARD FAULT *** dht_stage=");
    UartWriteUIntUnlocked((uint32_t)MonitorDhtStage);
    UartWriteStringUnlocked(" cfsr=");
    AppFaultWriteHex(cfsr);
    UartWriteStringUnlocked(" hfsr=");
    AppFaultWriteHex(hfsr);
    UartWriteStringUnlocked(" bfar=");
    AppFaultWriteHex(bfar);
    UartWriteStringUnlocked(" mmfar=");
    AppFaultWriteHex(mmfar);
    UartWriteStringUnlocked("\r\n");
    for (;;) {
    }
}
#endif /* APP_FAULT_DUMP */

void App_Start(void)
{
    OS_ERR err;

    UsTimerReady = UsTimerIsRunning();
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

static void AppTaskStart(void *argument)
{
    OS_ERR err;

    (void)argument;

    BSP_Init();
    Board_Init();
    BSP_Tick_Init();
    App_Start();
    UartLog("\r\nSmartAC started\r\n");

    for (;;) {
        OSTimeDlyHMSM(0U, 0U, 1U, 0U, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* Enable the Cortex-M4F floating-point unit (CP10/CP11 full access).
 *
 * This BSP's Reset_Handler has "bl SystemInit" commented out, so the CMSIS
 * FPU-enable that normally lives in SystemInit() never runs. The project is
 * built with FPU instructions (hard/softfp ABI), so the very first floating-
 * point instruction executed - the prologue of DhtRead() - takes a NOCP
 * UsageFault that escalates to HardFault. Enabling CPACR here, before any code
 * that could touch the FPU, fixes that at the root. Must run before OSInit()
 * and before any task starts. */
static void App_FpuEnable(void)
{
#if !defined(__FPU_PRESENT) || (__FPU_PRESENT == 1U)
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();
#endif
}

int main(void)
{
    OS_ERR err;

    App_FpuEnable();

    BSP_IntDisAll();

    CPU_Init();
    Mem_Init();
    Math_Init();

    OSInit(&err);
    if (err != OS_ERR_NONE) {
        while (DEF_TRUE) {
        }
    }

    OSTaskCreate(&AppTaskStartTCB,
                 "App Task Start",
                 AppTaskStart,
                 NULL,
                 APP_PRIO_START,
                 &AppTaskStartStk[0U],
                 APP_CFG_TASK_START_STK_SIZE / 10U,
                 APP_CFG_TASK_START_STK_SIZE,
                 0U,
                 0U,
                 NULL,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);
    if (err != OS_ERR_NONE) {
        while (DEF_TRUE) {
        }
    }

    OSStart(&err);

    while (DEF_TRUE) {
    }
}

#endif /* APP_HOST_TEST */
