/*
 * Smart AC Control System - application entry point
 *
 * Target: NUCLEO-F439ZI, STM32 StdPeriph, uC/OS-III, TrueSTUDIO
 *
 * Feature code is split into app_* modules. This file keeps only the startup
 * path that initializes the CPU/OS and creates the application start task.
 */

#ifndef APP_HOST_TEST

#include <includes.h>

#include "app_board.h"
#include "app_tasks.h"

void App_Start(void)
{
    AppTasks_CreateAll();
}

int main(void)
{
    OS_ERR err;

    Board_EnableFpu();

    BSP_IntDisAll();

    CPU_Init();
    Mem_Init();
    Math_Init();

    OSInit(&err);
    if (err != OS_ERR_NONE) {
        while (DEF_TRUE) {
        }
    }

    if (!AppTasks_CreateStart()) {
        while (DEF_TRUE) {
        }
    }

    OSStart(&err);

    while (DEF_TRUE) {
    }
}

#endif /* APP_HOST_TEST */
