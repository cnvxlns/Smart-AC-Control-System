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
/* Consecutive identical polls required to accept a new button level.
 * 50 ms / 10 ms = 5 samples of stability. */
#define APP_DEBOUNCE_SAMPLES         (APP_DEBOUNCE_MS / APP_BUTTON_POLL_MS)
#define APP_LONG_PRESS_MS            1500U
#define APP_BUZZER_CLICK_MS          120U
#define APP_MONITOR_PERIOD_MS        1000U
#define APP_MONITOR_START_DELAY_MS   1000U

#ifndef APP_HOST_TEST

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
#define APP_TM1637_CLK_PORT          GPIOF       /* D2 / PF15, push-pull */
#define APP_TM1637_CLK_PIN           GPIO_Pin_15
#define APP_TM1637_DIO_PORT          GPIOE       /* D3 / PE13, open-drain */
#define APP_TM1637_DIO_PIN           GPIO_Pin_13

#define APP_LD1_PORT                 GPIOB       /* PB0 / onboard LD1 green */
#define APP_LD1_PIN                  GPIO_Pin_0
#define APP_LD2_PORT                 GPIOB       /* PB7 / onboard LD2 blue */
#define APP_LD2_PIN                  GPIO_Pin_7
#define APP_LD3_PORT                 GPIOB       /* PB14 / onboard LD3 red */
#define APP_LD3_PIN                  GPIO_Pin_14

#define GPIO_PIN_RESET               Bit_RESET
#define GPIO_PIN_SET                 Bit_SET

#define APP_REED_OPEN_LEVEL          GPIO_PIN_SET
/* External UP/DOWN buttons are user-wired active-low with a pull-up. */
#define APP_BUTTON_PRESSED_LEVEL     GPIO_PIN_RESET
/* Onboard USER/B1 (PC13) on NUCLEO-144 is active-HIGH: idle low, pressed high,
 * board has an external pull-down. */
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
 * frames. */
#define APP_TASK_STACK_SIZE          1024U
#define APP_QUEUE_DEPTH              8U
#define APP_QUEUE_SLOT_COUNT         (APP_QUEUE_DEPTH + 2U)
#define APP_BUSY_WAIT_GUARD          1000000UL
#define APP_UART_TX_TIMEOUT_GUARD    1000000UL

/* TM1637 DIO is open-drain and a data '1' is raised only by the pull-up. */
#define APP_TM1637_DELAY_US          50U
#define APP_TM1637_BRIGHTNESS        4U

#define APP_DHT22_START_US           1200U
#define APP_DHT22_PULL_US            30U
#define APP_DHT_BIT_ONE_THRESHOLD_US 50U
#define APP_DHT_READ_PERIOD_MS       2500U
#define APP_DHT_READ_ENABLE          1
#define APP_DHT_DEBUG_LOG            1
#define APP_DHT_PULSE_TIMEOUT_US     2000U
#define APP_DHT_READ_TIMEOUT_US      20000U
#define APP_DHT_LOOP_GUARD           20000UL
#define APP_US_TIMER_PROBE_MS        100U
#define APP_DHT_DISABLE_IRQ          1

#define APP_FAULT_DUMP               1

#endif /* APP_HOST_TEST */

#endif /* APP_CONFIG_H */
