# Smart AC Single-File Application Design

## Scope

Implement the README system as one portable TrueSTUDIO/uC/OS-III application
file, `app.c`. CubeMX-generated startup, HAL, BSP, and uC/OS-III source files
remain external dependencies; all project-specific behavior is contained in
`app.c`.

## Architecture

`app.c` is organized into configuration, data model, pure control logic,
hardware adapters, RTOS objects, six task implementations, and HAL callbacks.
The pure control function receives the latest sensor snapshot, user events,
and elapsed time, then produces display and actuator commands. Hardware access
is confined to adapter functions so board pin changes do not affect control
logic.

## RTOS Design

- Sensor Task, priority 5: samples DHT11 every two seconds and publishes
  `SensorMsg` to the control queue.
- User Input Task, priority 6: consumes EXTI edge bits, applies 50 ms debounce,
  and posts user event flags.
- Control Task, priority 7: owns the FSM and posts `DisplayMsg` and
  `ActuatorMsg`.
- Display Task, priority 8: multiplexes two common-cathode seven-segment digits
  and time-slices values/status codes.
- Actuator Task, priority 9: drives LEDs and buzzer patterns.
- Monitor Task, priority 12: prints state and queue statistics through a
  mutex-protected UART.

The tasks communicate through three uC/OS-III queues, one event flag group, and
one UART mutex. EXTI callbacks only latch edge bits and never perform blocking
work.

## State Machine

- `OFF`: outputs disabled; briefly displays `OF`.
- `IDLE`: powered, temperature at or below cooling threshold.
- `COOLING`: temperature exceeds setpoint by more than 1 C; level is selected
  from the temperature difference.
- `WINDOW_SUSPECT`: entered immediately while a window is open.
- `AC_PAUSED`: entered after five seconds open or a humidity rise of at least
  ten percentage points.
- `RECOVERY_WAIT`: holds cooling off for five seconds after the window closes.
- `MANUAL_OVERRIDE`: ignores automatic window pause behavior and cools
  according to temperature.

Power, setpoint up/down, and manual-toggle events are processed by the control
owner. Setpoint is clamped to 16-30 C. Invalid DHT11 samples preserve the last
valid reading and are reported over UART.

## Board Configuration

The top of `app.c` contains all overridable GPIO, UART, timer, polarity, and
seven-segment macros. Defaults use the NUCLEO-F439ZI onboard LEDs and B1 plus
example external pins. CubeMX must configure those pins, EXTI lines, UART3 at
115200 baud, and a free-running 1 MHz timer before calling `App_Start()`.

## Verification

Host-test mode compiles the pure control logic without STM32 or uC/OS-III
headers. Tests exercise cooling levels, window timeout, humidity-rise pause,
recovery delay, manual override, and setpoint clamping. A static audit checks
that all six tasks and required RTOS primitives exist in `app.c`.
