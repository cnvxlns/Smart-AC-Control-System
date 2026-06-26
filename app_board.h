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
uint32_t Board_UsTimerCounter(void);
uint32_t Board_TimerElapsedUs(uint32_t start_us);
void Board_DelayUs(uint16_t microseconds);
bool Board_UsTimerIsRunning(void);
void Board_SetUsTimerReady(bool ready);
bool Board_IsUsTimerReady(void);

void Board_WriteActive(GPIO_TypeDef *port, uint16_t pin, bool active,
                       GPIO_PinState active_level);
bool Board_PinIsPressed(GPIO_TypeDef *port, uint16_t pin);
bool Board_B1IsPressed(void);
bool Board_ReedIsOpen(void);

void Board_DhtSetOutput(void);
void Board_DhtSetInput(void);
GPIO_PinState Board_DhtRead(void);
void Board_DhtWrite(GPIO_PinState state);

void Board_Tm1637WriteClk(bool high);
void Board_Tm1637WriteDio(bool high);
GPIO_PinState Board_Tm1637ReadDio(void);

uint32_t Board_TakeRawEdges(void);
bool Board_TakeReedEdge(void);
void Board_UartWriteBytes(const char *buffer, uint16_t length);

#endif /* APP_HOST_TEST */

#endif /* APP_BOARD_H */
