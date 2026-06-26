#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdbool.h>

#ifndef APP_HOST_TEST

bool AppTasks_CreateStart(void);
void AppTasks_CreateAll(void);

#endif /* APP_HOST_TEST */

#endif /* APP_TASKS_H */
