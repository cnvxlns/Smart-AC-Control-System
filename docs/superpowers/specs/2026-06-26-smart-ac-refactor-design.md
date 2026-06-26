# Smart AC app.c Functional Refactor Design

## Scope

Refactor the current TrueSTUDIO/uC/OS-III `app.c` implementation into standard
feature-oriented C modules. Preserve the current runtime behavior, pin mapping,
task priorities, diagnostics, and host-testable pure control logic. Do not add
new application features during the split.

The refactor assumes the TrueSTUDIO project will compile each new `.c` file as
a normal translation unit. `app.c` will no longer include implementation files.

## Architecture

`app.c` becomes the application entry point. It keeps `main()`, top-level
startup, and the minimal handoff into the task subsystem. Feature code moves
behind headers with explicit ownership:

- `app_config.h`: shared timing, setpoint, task, queue, pin, and diagnostic
  configuration macros.
- `app_messages.h`: cross-module message and event types.
- `app_control.h/.c`: pure AC state machine and setpoint/cooling decisions.
- `app_board.h/.c`: board initialization, GPIO helpers, UART byte writes,
  millisecond/microsecond timing, EXTI dispatch, and FPU enable.
- `app_dht22.h/.c`: DHT22 timing protocol and DHT diagnostic fields.
- `app_tm1637.h/.c`: TM1637 byte protocol, segment rendering, initialization,
  and ACK diagnostics.
- `app_actuator.h/.c`: LED and buzzer output behavior.
- `app_monitor.h/.c`: UART formatting, input diagnostics, task-create failure
  logging, and fault hex output support.
- `app_tasks.h/.c`: uC/OS-III queues, flags, mutex, stacks, task bodies, and
  task creation.
- `app_fault.h/.c`: `App_Fault_ISR()` and Cortex-M fault register dump.

## Module Boundaries

The control module has no STM32 or uC/OS-III dependency and remains compilable
with `APP_HOST_TEST`. It consumes `SensorMsg` plus user event bits and returns
`ControlOutput`.

The task module owns RTOS state and cross-task queues. Other modules do not
directly manipulate queue objects, task stacks, TCBs, or OS flags. The task
module exposes only startup and diagnostic accessors needed by monitoring.

The board module owns hardware primitives and IRQ vector registration. Drivers
such as DHT22, TM1637, actuator, and monitor call board-level helpers rather
than duplicating GPIO, UART, or timer code.

Diagnostics are kept explicit. DHT, TM1637, sensor post/drop counters, actuator
post/drop counters, current control state, and stack free data remain available
to the monitor output with the same field meaning as before.

## Data Flow

`SensorTask` reads reed and DHT22 data, updates DHT diagnostics, and posts
`SensorMsg` to `SensorQ`.

`UserInputTask` polls debounced buttons and drains raw EXTI edge bits from the
board/task boundary. It posts user event flags for setpoint changes, manual
mode, power toggle, and buzzer click feedback.

`ControlTask` owns `ControlContext`, calls `Control_Step()`, records monitor
state, and posts display and actuator messages.

`DisplayTask` renders the latest `DisplayMsg` through the TM1637 module.

`ActuatorTask` applies LED and buzzer behavior through the actuator module.

`MonitorTask` periodically prints input, sensor, timer, queue, stack, and
display ACK diagnostics over mutex-protected UART output.

## Error Handling

Task creation failures continue to be reported over UART. Queue post failures
increment drop counters. DHT22 read failures preserve detailed stage, error,
timing, and raw-byte diagnostics. TM1637 communication keeps the last ACK count.

`App_Fault_ISR()` remains available when `APP_FAULT_DUMP` is enabled and prints
CFSR, HFSR, BFAR, MMFAR, and DHT stage before stopping.

## Testing and Verification

Keep host-test compatibility for the pure control module by compiling it with
`APP_HOST_TEST` and without embedded headers. Add a small host test source if
needed to verify cooling levels, window pause, recovery wait, manual override,
and setpoint clamping after the split.

For embedded integration, verify that each new `.c` file is included in the
TrueSTUDIO project build. A successful refactor should have no behavior changes
other than file organization.
