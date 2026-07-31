/**
 ******************************************************************************
 * @file    task_ctrl.c
 * @brief   MSPM0 任务状态机
 *
 *   S_INIT:     上电 PD 入死区 → 锁死
 *   S_LOCKED:   等 MSPM0. CMD2→S_HOLD_N50, CMD3/4→S_HOLD
 *   S_HOLD_N50: 稳定在 -50, 到位后稳定1s → 切 +50
 *   S_HOLD_P50: 稳定在 +50, 持续保持
 *   S_HOLD:     CMD3/4, 保持原点
 ******************************************************************************
 */

#include "task_ctrl.h"
#include "motor.h"
#include "maixpro.h"
#include <math.h>

typedef enum { S_INIT, S_LOCKED, S_HOLD_N50, S_HOLD_P50, S_HOLD } State;
static State    st = S_INIT;
static uint32_t stable_ms = 0;

void TaskCtrl_Init(void) { st = S_INIT; stable_ms = 0; }

void TaskCtrl_Process(void)
{
    Ball_Position pos;
    Mspm0_Cmd    cmd = MSPM0_CMD_NONE;
    uint32_t     now = HAL_GetTick();
    float e;

    if (Mspm0_DataReady()) { cmd = Mspm0_GetCmd(); Mspm0_ClearCmd(); }
    if (!MaixPro_GetPosition(&pos)) return;
    e = (pos.lr == 0) ? (float)pos.dist : -(float)pos.dist;

    switch (st) {

    case S_INIT:
        MaixPro_Process();
        if (pos.dist <= MAIXPRO_DEAD_ZONE) {
            if ((now - stable_ms) > 1000)
            { Motor_SetMode(MOTOR_MODE_BRAKE); st = S_LOCKED; }
        } else stable_ms = now;
        break;

    case S_LOCKED:
        if (cmd == MSPM0_CMD_2) {
            MaixPro_RequestOriginCapture();
            MaixPro_SetTarget(-50);
            stable_ms = 0;
            st = S_HOLD_N50;
        } else if (cmd == MSPM0_CMD_3 || cmd == MSPM0_CMD_4) {
            MaixPro_RequestOriginCapture();
            st = S_HOLD;
        }
        break;

    case S_HOLD_N50:
        MaixPro_SwingTo(-50.0f, e);   /* 快速推到 -50 */
        /* 球到 -40~-60 范围稳定 300ms 切 +50 */
        if (e <= -40.0f && e >= -60.0f) {
            if ((now - stable_ms) > 300) {
                MaixPro_SetTarget(50); stable_ms = 0; st = S_HOLD_P50;
            }
        } else stable_ms = now;
        if (cmd == MSPM0_CMD_0)
        { Motor_SetMode(MOTOR_MODE_BRAKE); st = S_LOCKED; }
        break;

    case S_HOLD_P50:
        MaixPro_SwingTo(50.0f, e);    /* 快速推到 +50 并保持 */
        if (cmd == MSPM0_CMD_0)
        { Motor_SetMode(MOTOR_MODE_BRAKE); st = S_LOCKED; }
        break;

    case S_HOLD:
        if (cmd == MSPM0_CMD_0)
        { Motor_SetMode(MOTOR_MODE_BRAKE); st = S_LOCKED; }
        else MaixPro_Process();
        break;
    }
}
