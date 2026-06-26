#include "app_tasks.h"

#ifndef APP_HOST_TEST

#include <string.h>

#include "app_actuator.h"
#include "app_board.h"
#include "app_config.h"
#include "app_control.h"
#include "app_dht22.h"
#include "app_monitor.h"
#include "app_tm1637.h"

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

static SensorMsg SensorSlots[APP_QUEUE_SLOT_COUNT];
static DisplayMsg DisplaySlots[APP_QUEUE_SLOT_COUNT];
static ActuatorMsg ActuatorSlots[APP_QUEUE_SLOT_COUNT];
static uint8_t SensorSlotIndex;
static uint8_t DisplaySlotIndex;
static uint8_t ActuatorSlotIndex;
static volatile AcState MonitorState;
static volatile uint8_t MonitorCoolingLevel;
static volatile uint32_t MonitorUsTicksPerProbe;
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

void App_Start(void);

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

static void SensorTask(void *argument)
{
    SensorMsg message;
    bool last_window = false;
    bool last_dht_valid = false;
    uint32_t last_dht_ms = 0U;

    (void)argument;
    memset(&message, 0, sizeof(message));

    App_DelayMs(1200U);

    if (!Board_IsUsTimerReady()) {
        Monitor_Log("Microsecond timer not running\r\n");
    }

    {
        uint32_t probe_start = Board_UsTimerCounter();

        App_DelayMs(APP_US_TIMER_PROBE_MS);
        MonitorUsTicksPerProbe =
            (uint32_t)(Board_UsTimerCounter() - probe_start);
    }
#if APP_DHT_DEBUG_LOG
    Monitor_LogTimerProbe(MonitorUsTicksPerProbe);
#endif

    for (;;) {
        uint32_t now = App_Millis();
        bool window_open = Board_ReedIsOpen();
        bool publish = (window_open != last_window) || Board_TakeReedEdge();

        if ((now - last_dht_ms) >= APP_DHT_READ_PERIOD_MS ||
            last_dht_ms == 0U) {
#if APP_DHT_DEBUG_LOG
            Monitor_Log("DHT before\r\n");
#endif
#if APP_DHT_READ_ENABLE
            message.valid = DhtRead(&message.temperature_c,
                                    &message.humidity_pct);
#else
            message.valid = false;
#endif
#if APP_DHT_DEBUG_LOG
            Monitor_Log(message.valid ? "DHT ok\r\n" : "DHT fail\r\n");
#endif
            last_dht_valid = message.valid;
            Dht_RecordSample(message.valid, message.temperature_c,
                             message.humidity_pct);
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

typedef struct {
    bool pressed;
    uint8_t count;
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
    return db->pressed;
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

        (void)Board_TakeRawEdges();

        if (DebounceButton(&up_db,
                           Board_PinIsPressed(APP_UP_PORT, APP_UP_PIN))) {
            events |= USER_EVT_SETPOINT_UP;
        }
        if (DebounceButton(&down_db,
                           Board_PinIsPressed(APP_DOWN_PORT, APP_DOWN_PIN))) {
            events |= USER_EVT_SETPOINT_DOWN;
        }

        b1_pressed = Board_B1IsPressed();
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

    Tm1637_Init();

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
        Tm1637_BuildSegments(&current, App_Millis(), segments);
        Tm1637_SetSegments(segments);
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
        Actuator_SetOutputs(&current, now,
                            ((int32_t)(now - buzzer_until_ms) < 0));
        App_DelayMs(25U);
    }
}

static void GetRuntimeDiagnostics(AppRuntimeDiagnostics *runtime)
{
    CPU_STK_SIZE sensor_stk_used = 0U;
    OS_ERR stk_err;

    runtime->us_ticks_per_probe = MonitorUsTicksPerProbe;
    runtime->sensor_stk_free = 0U;
    OSTaskStkChk(&SensorTaskTCB, &runtime->sensor_stk_free,
                 &sensor_stk_used, &stk_err);
    (void)stk_err;
    (void)sensor_stk_used;
}

static void MonitorTask(void *argument)
{
    AppRuntimeDiagnostics runtime;

    (void)argument;
    App_DelayMs(APP_MONITOR_START_DELAY_MS);
    for (;;) {
        GetRuntimeDiagnostics(&runtime);
        Monitor_LogInputs(&runtime);
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
        Monitor_LogTaskCreateFailed(name, err);
    }
}

void AppTasks_CreateAll(void)
{
    OS_ERR err;

    Board_SetUsTimerReady(Board_UsTimerIsRunning());
    OSQCreate(&SensorQ, "SensorQ", APP_QUEUE_DEPTH, &err);
    OSQCreate(&DisplayQ, "DisplayQ", APP_QUEUE_DEPTH, &err);
    OSQCreate(&ActuatorQ, "ActuatorQ", APP_QUEUE_DEPTH, &err);
    OSFlagCreate(&UserEventFlags, "UserEventFlags", 0U, &err);
    OSMutexCreate(&UartMutex, "UartMutex", &err);
    Monitor_Init(&UartMutex);

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
    Monitor_Log("\r\nSmartAC started\r\n");

    for (;;) {
        OSTimeDlyHMSM(0U, 0U, 1U, 0U, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

bool AppTasks_CreateStart(void)
{
    OS_ERR err;

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
    return err == OS_ERR_NONE;
}

#endif /* APP_HOST_TEST */
