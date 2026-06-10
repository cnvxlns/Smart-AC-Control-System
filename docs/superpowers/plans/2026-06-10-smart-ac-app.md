# Smart AC Single-File Application Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the README smart AC controller as a portable uC/OS-III `app.c`.

**Architecture:** Keep pure state-transition logic independent from STM32 HAL
and uC/OS-III calls, while retaining all project-specific code in one file.
Six tasks communicate through queues, event flags, and a UART mutex.

**Tech Stack:** C99, STM32F4 HAL, uC/OS-III, TrueSTUDIO/GCC

---

### Task 1: Pure Control Model

**Files:**
- Create: `app.c`
- Create temporarily for test execution: `app_test.c`

- [x] Write host tests for cooling, window pause, recovery, manual mode, and
  setpoint limits.
- [x] Run `gcc -std=c99 -DAPP_HOST_TEST app_test.c -o app_test.exe` and confirm
  failure because `app.c` does not exist.
- [x] Add the data model and pure `Control_Step` implementation to `app.c`.
- [x] Rebuild and run `app_test.exe`; expect all assertions to pass.

### Task 2: Hardware Adapters

**Files:**
- Modify: `app.c`

- [x] Add overridable board pin and peripheral macros.
- [x] Add DHT11 timing/read, reed input, seven-segment multiplexing, LED,
  buzzer, and mutex-protected UART adapters.
- [x] Compile host-test mode again to ensure embedded-only code remains
  isolated.

### Task 3: uC/OS-III Runtime

**Files:**
- Modify: `app.c`

- [x] Add stacks, TCBs, queues, event flags, mutex, and six task functions.
- [x] Add `App_Start()` to create RTOS objects and tasks.
- [x] Add `HAL_GPIO_EXTI_Callback()` edge latching.
- [x] Audit names and priorities against README requirements.

### Task 4: Verification and Documentation

**Files:**
- Modify: `README.md`
- Delete temporary test artifact: `app_test.c`

- [x] Run host tests and static requirement audit.
- [x] Document integration prerequisites and configurable pins in README.
- [x] Confirm the final project-specific codebase consists of `app.c`.
