#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include <stdint.h>

#include "app_config.h"

#ifndef APP_HOST_TEST

#include <includes.h>

typedef struct {
    uint32_t us_ticks_per_probe;
    CPU_STK_SIZE sensor_stk_free;
} AppRuntimeDiagnostics;

void Monitor_Init(OS_MUTEX *uart_mutex);
void Monitor_Log(const char *text);
void Monitor_LogTimerProbe(uint32_t ticks);
void Monitor_LogTaskCreateFailed(const CPU_CHAR *name, OS_ERR task_err);
void Monitor_LogInputs(const AppRuntimeDiagnostics *runtime);
void Monitor_WriteBytesUnlocked(const char *buffer, uint16_t length);
void Monitor_WriteStringUnlocked(const char *text);
void Monitor_WriteUIntUnlocked(uint32_t value);
void Monitor_WriteHexUnlocked(uint32_t value);

#endif /* APP_HOST_TEST */

#endif /* APP_MONITOR_H */
