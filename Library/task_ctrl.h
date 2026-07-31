#ifndef __TASK_CTRL_H__
#define __TASK_CTRL_H__

#include "MSPM0.h"

typedef enum {
    TASK_INIT_PUSH = 0,
    TASK_WAIT_CMD,
    TASK_LOCK,
    TASK_SWING,
    TASK_HOLD,
    TASK_MENU
} TaskState;

void TaskCtrl_Init(void);
void TaskCtrl_Process(void);

#endif
