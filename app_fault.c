#include "app_fault.h"

#ifndef APP_HOST_TEST

#include "app_config.h"
#include "app_dht22.h"
#include "app_monitor.h"

#if APP_FAULT_DUMP

#define APP_SCB_CFSR   (*(volatile uint32_t *)0xE000ED28UL)
#define APP_SCB_HFSR   (*(volatile uint32_t *)0xE000ED2CUL)
#define APP_SCB_MMFAR  (*(volatile uint32_t *)0xE000ED34UL)
#define APP_SCB_BFAR   (*(volatile uint32_t *)0xE000ED38UL)

void App_Fault_ISR(void)
{
    uint32_t cfsr = APP_SCB_CFSR;
    uint32_t hfsr = APP_SCB_HFSR;
    uint32_t bfar = APP_SCB_BFAR;
    uint32_t mmfar = APP_SCB_MMFAR;

    Monitor_WriteStringUnlocked("\r\n*** HARD FAULT *** dht_stage=");
    Monitor_WriteUIntUnlocked((uint32_t)Dht_GetStage());
    Monitor_WriteStringUnlocked(" cfsr=");
    Monitor_WriteHexUnlocked(cfsr);
    Monitor_WriteStringUnlocked(" hfsr=");
    Monitor_WriteHexUnlocked(hfsr);
    Monitor_WriteStringUnlocked(" bfar=");
    Monitor_WriteHexUnlocked(bfar);
    Monitor_WriteStringUnlocked(" mmfar=");
    Monitor_WriteHexUnlocked(mmfar);
    Monitor_WriteStringUnlocked("\r\n");
    for (;;) {
    }
}

#endif /* APP_FAULT_DUMP */

#endif /* APP_HOST_TEST */
