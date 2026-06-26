# Smart AC Functional Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the current TrueSTUDIO/uC/OS-III `app.c` into standard
feature-oriented `.c/.h` modules without changing runtime behavior.

**Architecture:** Keep pure control logic independent from STM32/uC-OS headers.
Move embedded hardware, drivers, diagnostics, and RTOS task ownership behind
small module headers. Leave `app.c` as the startup entry point that calls the
task subsystem.

**Tech Stack:** C99, STM32F4 StdPeriph/BSP, uC/OS-III, TrueSTUDIO/GCC

---

## File Structure

- Create `app_config.h`: all shared timing, setpoint, pin, task, queue, DHT,
  TM1637, and diagnostic macros.
- Create `app_messages.h`: `AcState`, `SensorMsg`, `ControlOutput`,
  `DisplayMsg`, `ActuatorMsg`, and user event bit definitions.
- Create `app_control.h/.c`: `Control_Init()` and `Control_Step()`.
- Create `app_board.h/.c`: `Board_Init()`, `Board_EnableFpu()`, timing helpers,
  GPIO helpers, raw EXTI edge capture, reed edge capture, UART byte write, and
  pin read/write helpers.
- Create `app_dht22.h/.c`: `DhtRead()` plus DHT diagnostic accessors.
- Create `app_tm1637.h/.c`: TM1637 initialization, segment rendering, segment
  writes, and last ACK count accessor.
- Create `app_actuator.h/.c`: actuator output application.
- Create `app_monitor.h/.c`: UART logging, monitor line output, task-create
  failure logging, and fault hex output.
- Create `app_tasks.h/.c`: RTOS object ownership, all task bodies, task
  creation, counters, and monitor state.
- Create `app_fault.h/.c`: fault dump ISR.
- Modify `app.c`: keep includes, `App_Start()`, start task creation, `main()`,
  and calls into `app_tasks`.
- Create temporary `app_test.c`: host regression tests for pure control logic.

## Task 1: Control Module and Host Regression Test

**Files:**
- Create: `app_config.h`
- Create: `app_messages.h`
- Create: `app_control.h`
- Create: `app_control.c`
- Create temporarily: `app_test.c`
- Modify: `app.c`

- [ ] **Step 1: Add shared config/message headers**

Move these definitions out of `app.c` unchanged:

```c
/* app_config.h */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_SETPOINT_DEFAULT_C       24
#define APP_SETPOINT_MIN_C           16
#define APP_SETPOINT_MAX_C           30
#define APP_WINDOW_PAUSE_MS          5000U
#define APP_RECOVERY_WAIT_MS         5000U
#define APP_HUMIDITY_RISE_PCT        10.0f
#define APP_DEBOUNCE_MS              50U
#define APP_BUTTON_POLL_MS           10U
#define APP_DEBOUNCE_SAMPLES         (APP_DEBOUNCE_MS / APP_BUTTON_POLL_MS)
#define APP_LONG_PRESS_MS            1500U
#define APP_BUZZER_CLICK_MS          120U
#define APP_MONITOR_PERIOD_MS        1000U
#define APP_MONITOR_START_DELAY_MS   1000U

#endif
```

```c
/* app_messages.h */
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

#endif
```

- [ ] **Step 2: Move pure control code**

Create `app_control.h` and `app_control.c` using the exact public API currently
provided by `app.c`:

```c
/* app_control.h */
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

#endif
```

Move `RoundTemperature()`, `CoolingLevel()`, `Control_Init()`, and
`Control_Step()` from `app.c` into `app_control.c`. Include `app_config.h`,
`app_control.h`, and `<string.h>`.

- [ ] **Step 3: Add host test for moved control logic**

Create `app_test.c`:

```c
#include <assert.h>
#include "app_config.h"
#include "app_control.h"

static SensorMsg sensor(float temp, float humidity, bool open)
{
    SensorMsg msg = { temp, humidity, open, true };
    return msg;
}

int main(void)
{
    ControlContext ctx;
    ControlOutput out;
    SensorMsg s;

    Control_Init(&ctx);
    s = sensor(27.5f, 50.0f, false);
    out = Control_Step(&ctx, &s, USER_EVT_POWER, 100U);
    assert(out.state == AC_STATE_COOLING);
    assert(out.cooling_level == 2U);

    Control_Init(&ctx);
    s = sensor(23.0f, 40.0f, true);
    out = Control_Step(&ctx, &s, USER_EVT_POWER, 1000U);
    assert(out.state == AC_STATE_WINDOW_SUSPECT);
    out = Control_Step(&ctx, &s, 0U, 7000U);
    assert(out.state == AC_STATE_AC_PAUSED);

    s = sensor(23.0f, 40.0f, false);
    out = Control_Step(&ctx, &s, 0U, 7100U);
    assert(out.state == AC_STATE_RECOVERY_WAIT);
    out = Control_Step(&ctx, &s, 0U, 13000U);
    assert(out.state == AC_STATE_IDLE);

    Control_Init(&ctx);
    s = sensor(25.5f, 50.0f, true);
    (void)Control_Step(&ctx, &s, USER_EVT_POWER, 0U);
    out = Control_Step(&ctx, &s, USER_EVT_MANUAL, 10U);
    assert(out.state == AC_STATE_MANUAL_OVERRIDE);

    for (int i = 0; i < 20; ++i) {
        out = Control_Step(&ctx, &s, USER_EVT_SETPOINT_UP, 20U + (uint32_t)i);
    }
    assert(out.setpoint_c == APP_SETPOINT_MAX_C);

    return 0;
}
```

- [ ] **Step 4: Compile and run host test**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0` and no assertion output.

- [ ] **Step 5: Commit Task 1**

```powershell
git add app_config.h app_messages.h app_control.h app_control.c app.c app_test.c
git commit -m "refactor: extract control module"
```

## Task 2: Board Module

**Files:**
- Create: `app_board.h`
- Create: `app_board.c`
- Modify: `app_config.h`
- Modify: `app.c`

- [ ] **Step 1: Move embedded configuration macros**

Move board-only macros from the embedded section of `app.c` into `app_config.h`
inside `#ifndef APP_HOST_TEST`, including `APP_UART`, `APP_US_TIMER`, all GPIO
pin macros, polarity macros, task priorities, stack/queue sizes, guard values,
DHT/TM1637 config, timer probe, IRQ-disable flag, and fault dump flag.

- [ ] **Step 2: Create board API**

```c
/* app_board.h */
#ifndef APP_BOARD_H
#define APP_BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

#ifndef APP_HOST_TEST
#include <includes.h>
typedef BitAction GPIO_PinState;

void Board_Init(void);
void Board_EnableFpu(void);
uint32_t App_Millis(void);
void App_DelayMs(uint32_t milliseconds);
void DelayUs(uint16_t microseconds);
bool UsTimerIsRunning(void);
uint32_t TimerElapsedUs(uint32_t start_us);
void Board_WriteActive(GPIO_TypeDef *port, uint16_t pin, bool active,
                       GPIO_PinState active_level);
bool Board_PinIsPressed(GPIO_TypeDef *port, uint16_t pin);
bool Board_B1IsPressed(void);
bool Board_ReedIsOpen(void);
void Board_DhtSetOutput(void);
void Board_DhtSetInput(void);
uint32_t Board_TakeRawEdges(void);
bool Board_TakeReedEdge(void);
void Board_UartWriteBytes(const char *buffer, uint16_t length);

#endif
#endif
```

- [ ] **Step 3: Move board implementation**

Move these functions from `app.c` to `app_board.c`: `App_Millis()`,
`App_DelayMs()`, `InactiveLevel()`, `WriteActive()`, `PinIsPressed()`,
`B1IsPressed()`, `TakeRawEdges()`, `TakeReedEdge()`, `TimerElapsedUs()`,
`DelayUs()`, `UsTimerIsRunning()`, `DhtSetOutput()`, `DhtSetInput()`,
`UartWriteBytes()`, GPIO configuration helpers, `Board_InitGpio()`,
`Board_InitUsart()`, `Board_InitTimer()`, `Board_InitExti()`, `Board_Init()`,
EXTI ISR handlers, `App_HandleGpioExti()`, and `App_FpuEnable()`.

Rename public functions to the `Board_` names declared in `app_board.h` and
update call sites.

- [ ] **Step 4: Compile host test again**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 2**

```powershell
git add app_config.h app_board.h app_board.c app.c app_control.c app_test.c
git commit -m "refactor: extract board support"
```

## Task 3: DHT22 Module

**Files:**
- Create: `app_dht22.h`
- Create: `app_dht22.c`
- Modify: `app_tasks.c` if already created, otherwise `app.c`
- Modify: `app_monitor.c` if already created, otherwise `app.c`

- [ ] **Step 1: Create DHT API and diagnostic snapshot**

```c
/* app_dht22.h */
#ifndef APP_DHT22_H
#define APP_DHT22_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    int16_t temperature_tenths;
    int16_t humidity_tenths;
    uint32_t fail_count;
    uint32_t last_error;
    uint16_t resp_low_us;
    uint16_t resp_high_us;
    uint16_t bit_low_us;
    uint16_t bit_high_us;
    uint8_t raw[5];
    uint8_t stage;
} DhtDiagnostics;

bool DhtRead(float *temperature_c, float *humidity_pct);
void Dht_RecordSample(bool valid, float temperature_c, float humidity_pct);
void Dht_GetDiagnostics(DhtDiagnostics *out);
uint8_t Dht_GetStage(void);

#endif
```

- [ ] **Step 2: Move DHT implementation**

Move `DhtStage`, `DhtErrorCode`, DHT volatile monitor fields,
`AppFloatToTenths()`, `DhtMeasurePulse()`, and `DhtRead()` into
`app_dht22.c`. Use `app_board.h` for timer, delay, GPIO, and DHT pin direction
helpers.

- [ ] **Step 3: Update sensor and monitor users**

In `SensorTask`, replace direct monitor field writes with:

```c
message.valid = DhtRead(&message.temperature_c, &message.humidity_pct);
Dht_RecordSample(message.valid, message.temperature_c, message.humidity_pct);
```

In monitor/fault output, use `Dht_GetDiagnostics()` and `Dht_GetStage()`.

- [ ] **Step 4: Compile host test**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 3**

```powershell
git add app_dht22.h app_dht22.c app.c app_board.h app_board.c app_test.c
git commit -m "refactor: extract dht22 driver"
```

## Task 4: Display, Actuator, and Monitor Modules

**Files:**
- Create: `app_tm1637.h`
- Create: `app_tm1637.c`
- Create: `app_actuator.h`
- Create: `app_actuator.c`
- Create: `app_monitor.h`
- Create: `app_monitor.c`
- Modify: `app.c`

- [ ] **Step 1: Create display API**

```c
/* app_tm1637.h */
#ifndef APP_TM1637_H
#define APP_TM1637_H

#include <stdint.h>
#include "app_messages.h"

void Tm1637_Init(void);
void Tm1637_BuildSegments(const DisplayMsg *message, uint32_t now_ms,
                          uint8_t segments[4]);
void Tm1637_SetSegments(const uint8_t segments[4]);
uint8_t Tm1637_GetAckCount(void);

#endif
```

Move all `Tm1637*` functions and ACK storage into `app_tm1637.c`.

- [ ] **Step 2: Create actuator API**

```c
/* app_actuator.h */
#ifndef APP_ACTUATOR_H
#define APP_ACTUATOR_H

#include <stdint.h>
#include "app_messages.h"

void Actuator_SetOutputs(const ActuatorMsg *message, uint32_t now_ms,
                         uint32_t click_started_ms);

#endif
```

Move `SetActuators()` into `app_actuator.c` and update GPIO writes to use
`Board_WriteActive()`.

- [ ] **Step 3: Create monitor API**

```c
/* app_monitor.h */
#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include <stdint.h>
#ifndef APP_HOST_TEST
#include <includes.h>
#endif
#include "app_messages.h"

typedef struct {
    AcState state;
    uint8_t cooling_level;
    uint32_t us_ticks_per_probe;
    uint32_t sensor_post_count;
    uint32_t sensor_drop_count;
    uint32_t actuator_post_count;
    uint32_t actuator_drop_count;
} AppRuntimeDiagnostics;

void Monitor_Log(const char *text);
void Monitor_LogTaskCreateFailed(const CPU_CHAR *name, OS_ERR task_err);
void Monitor_LogInputs(const AppRuntimeDiagnostics *runtime);
void Monitor_WriteHex(uint32_t value);

#endif
```

Move UART formatting helpers, task-create logging, monitor input logging, and
fault hex formatting support into `app_monitor.c`.

- [ ] **Step 4: Compile host test**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 4**

```powershell
git add app_tm1637.h app_tm1637.c app_actuator.h app_actuator.c app_monitor.h app_monitor.c app.c app_test.c
git commit -m "refactor: extract display actuator monitor"
```

## Task 5: RTOS Tasks, Fault Handler, and Entry Point

**Files:**
- Create: `app_tasks.h`
- Create: `app_tasks.c`
- Create: `app_fault.h`
- Create: `app_fault.c`
- Modify: `app.c`

- [ ] **Step 1: Create task subsystem API**

```c
/* app_tasks.h */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifndef APP_HOST_TEST
#include <includes.h>
#include "app_monitor.h"

void AppTasks_CreateAll(void);
void AppTasks_Start(void *argument);
void AppTasks_GetRuntimeDiagnostics(AppRuntimeDiagnostics *out);
OS_TCB *AppTasks_GetSensorTcb(void);

#endif
#endif
```

Include `app_monitor.h` in `app_tasks.h` so `AppRuntimeDiagnostics` is known.

- [ ] **Step 2: Move task implementation**

Move RTOS globals, queues, slots, counters, `PostSensor()`, `PostDisplay()`,
`PostActuator()`, `ButtonDebounce`, `DebounceButton()`, all six task functions,
`CreateTask()`, and the previous body of `AppTaskStart()` into `app_tasks.c`.

Expose `AppTasks_Start()` as the start-task entry function and
`AppTasks_CreateAll()` for creating Sensor/User/Control/Display/Actuator/Monitor
tasks after board initialization.

- [ ] **Step 3: Create fault module**

```c
/* app_fault.h */
#ifndef APP_FAULT_H
#define APP_FAULT_H

void App_Fault_ISR(void);

#endif
```

Move `App_Fault_ISR()` into `app_fault.c`. Use `Dht_GetStage()`,
`Monitor_Log()`, `Monitor_WriteHex()`, and direct SCB register macros kept local
to `app_fault.c`.

- [ ] **Step 4: Shrink app.c to startup only**

`app.c` should include `app_board.h`, `app_tasks.h`, and `app_fault.h`, then
keep only `App_Start()` and `main()`:

```c
void App_Start(void)
{
    OS_ERR err;
    OSTaskCreate(&AppTaskStartTCB,
                 "App Task Start",
                 AppTasks_Start,
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
    (void)err;
}
```

Keep `AppTaskStartTCB` and `AppTaskStartStk` either in `app.c` or move them to
`app_tasks.c`; choose one owner and avoid duplicate definitions.

- [ ] **Step 5: Compile host test**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0`.

- [ ] **Step 6: Commit Task 5**

```powershell
git add app_tasks.h app_tasks.c app_fault.h app_fault.c app.c app_test.c
git commit -m "refactor: extract rtos tasks and fault handler"
```

## Task 6: Verification and Documentation

**Files:**
- Modify: `README.md`
- Delete: `app_test.c`

- [ ] **Step 1: Verify no old single-file claim remains**

Run:

```powershell
rg -n "single-file|단일|contained in this file|app.c.*전체" README.md app.c docs
```

Expected: matches may appear in old design docs, but current README and `app.c`
must not claim all project-specific code is intentionally in one file.

- [ ] **Step 2: Run host regression one last time**

Run:

```powershell
gcc -std=c99 -DAPP_HOST_TEST app_test.c app_control.c -o app_test.exe
.\app_test.exe
```

Expected: exit code `0`.

- [ ] **Step 3: Remove temporary host artifacts**

Delete `app_test.c` and `app_test.exe` after the final host test. Do not delete
source modules.

- [ ] **Step 4: Update README file table**

Replace the old `app.c`-only file description with a module table listing:

```markdown
| File | Purpose |
|---|---|
| `app.c` | Application startup entry point |
| `app_config.h` | Shared timing, pin, task, and diagnostic configuration |
| `app_messages.h` | Cross-task message and event types |
| `app_control.c/.h` | Pure AC state machine |
| `app_board.c/.h` | STM32 board initialization and hardware helpers |
| `app_dht22.c/.h` | DHT22 sensor protocol and diagnostics |
| `app_tm1637.c/.h` | TM1637 display protocol and rendering |
| `app_actuator.c/.h` | LED and buzzer output behavior |
| `app_monitor.c/.h` | UART diagnostics |
| `app_tasks.c/.h` | uC/OS-III tasks, queues, flags, and runtime counters |
| `app_fault.c/.h` | HardFault diagnostic dump |
```

- [ ] **Step 5: Commit Task 6**

```powershell
git add README.md app_test.c app_config.h app_messages.h app_control.h app_control.c app_board.h app_board.c app_dht22.h app_dht22.c app_tm1637.h app_tm1637.c app_actuator.h app_actuator.c app_monitor.h app_monitor.c app_tasks.h app_tasks.c app_fault.h app_fault.c app.c
git commit -m "docs: document modular source layout"
```
