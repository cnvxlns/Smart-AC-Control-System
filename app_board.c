#include "app_board.h"

#ifndef APP_HOST_TEST

#include <string.h>

#define RAW_EDGE_UP                  (1UL << 0)
#define RAW_EDGE_DOWN                (1UL << 1)
#define RAW_EDGE_B1                  (1UL << 2)

static bool UsTimerReady;
static volatile uint32_t RawEdgeBits;
static volatile bool ReedEdgePending;

static void Board_InitGpio(void);
static void Board_InitUsart(void);
static void Board_InitTimer(void);
static void Board_InitExti(void);
static void App_EXTI0_ISR(void);
static void App_EXTI3_ISR(void);
static void App_EXTI15_10_ISR(void);
static void App_HandleGpioExti(uint16_t gpio_pin);

uint32_t App_Millis(void)
{
    OS_ERR err;
    OS_TICK ticks = OSTimeGet(&err);
    (void)err;
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / OSCfg_TickRate_Hz);
}

void App_DelayMs(uint32_t milliseconds)
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

void Board_WriteActive(GPIO_TypeDef *port, uint16_t pin, bool active,
                       GPIO_PinState active_level)
{
    GPIO_WriteBit(port, pin, active ? active_level : InactiveLevel(active_level));
}

bool Board_PinIsPressed(GPIO_TypeDef *port, uint16_t pin)
{
    return GPIO_ReadInputDataBit(port, pin) == APP_BUTTON_PRESSED_LEVEL;
}

bool Board_B1IsPressed(void)
{
    return GPIO_ReadInputDataBit(APP_B1_PORT, APP_B1_PIN) == APP_B1_PRESSED_LEVEL;
}

bool Board_ReedIsOpen(void)
{
    return GPIO_ReadInputDataBit(APP_REED_PORT, APP_REED_PIN) ==
           APP_REED_OPEN_LEVEL;
}

uint32_t Board_TakeRawEdges(void)
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

bool Board_TakeReedEdge(void)
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

uint32_t Board_UsTimerCounter(void)
{
    return TIM_GetCounter(APP_US_TIMER);
}

uint32_t Board_TimerElapsedUs(uint32_t start_us)
{
    return (uint32_t)(Board_UsTimerCounter() - start_us);
}

void Board_DelayUs(uint16_t microseconds)
{
    uint32_t start_us = Board_UsTimerCounter();
    uint32_t guard = 0U;

    while (Board_TimerElapsedUs(start_us) < microseconds) {
        if (++guard >= APP_BUSY_WAIT_GUARD) {
            break;
        }
    }
}

bool Board_UsTimerIsRunning(void)
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

void Board_SetUsTimerReady(bool ready)
{
    UsTimerReady = ready;
}

bool Board_IsUsTimerReady(void)
{
    return UsTimerReady;
}

void Board_DhtSetOutput(void)
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

void Board_DhtSetInput(void)
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

GPIO_PinState Board_DhtRead(void)
{
    return GPIO_ReadInputDataBit(APP_DHT_PORT, APP_DHT_PIN);
}

void Board_DhtWrite(GPIO_PinState state)
{
    GPIO_WriteBit(APP_DHT_PORT, APP_DHT_PIN, state);
}

void Board_Tm1637WriteClk(bool high)
{
    GPIO_WriteBit(APP_TM1637_CLK_PORT, APP_TM1637_CLK_PIN,
                  high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Board_Tm1637WriteDio(bool high)
{
    GPIO_WriteBit(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN,
                  high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

GPIO_PinState Board_Tm1637ReadDio(void)
{
    return GPIO_ReadInputDataBit(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN);
}

void Board_UartWriteBytes(const char *buffer, uint16_t length)
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
    ConfigureGpioInputPullDown(APP_B1_PORT, APP_B1_PIN);

    ConfigureGpioOutput(APP_BUZZER_PORT, APP_BUZZER_PIN);
    ConfigureGpioOutput(APP_LD1_PORT, APP_LD1_PIN);
    ConfigureGpioOutput(APP_LD2_PORT, APP_LD2_PIN);
    ConfigureGpioOutput(APP_LD3_PORT, APP_LD3_PIN);
    ConfigureGpioOutput(APP_TM1637_CLK_PORT, APP_TM1637_CLK_PIN);
    ConfigureGpioOpenDrain(APP_TM1637_DIO_PORT, APP_TM1637_DIO_PIN);

    Board_WriteActive(APP_LD1_PORT, APP_LD1_PIN, false, GPIO_PIN_SET);
    Board_WriteActive(APP_LD2_PORT, APP_LD2_PIN, false, GPIO_PIN_SET);
    Board_WriteActive(APP_LD3_PORT, APP_LD3_PIN, false, GPIO_PIN_SET);
    Board_WriteActive(APP_BUZZER_PORT, APP_BUZZER_PIN, false,
                      APP_BUZZER_ON_LEVEL);
    Board_Tm1637WriteClk(true);
    Board_Tm1637WriteDio(true);
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

void Board_Init(void)
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

void Board_EnableFpu(void)
{
#if !defined(__FPU_PRESENT) || (__FPU_PRESENT == 1U)
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();
#endif
}

#endif /* APP_HOST_TEST */
