/**
 ******************************************************************************
 * @file    task_ctrl.c
 * @brief   业务逻辑状态机: MSPM0 命令 → 电机任务
 *
 *  上电 → 推球到死区并稳定 → 锁死 → 等 MSPM0 指令
 *  CMD 0: 回主菜单, 锁平衡角
 *  CMD 1: 电机锁死
 *  CMD 2: 设原点 → 经 -50px → 停 +50px
 *  CMD 3/4: 设原点 → 保持小球在原点
 ******************************************************************************
 */

#include "task_ctrl.h"
#include "motor.h"
#include "maixpro.h"

static TaskState  state = TASK_INIT_PUSH;
static uint32_t   state_tick = 0;

static Mspm0_Cmd  pending_cmd = MSPM0_CMD_NONE;

/* 前向声明 */
static void ResetAndLock(void);

void TaskCtrl_Init(void)
{
    state = TASK_INIT_PUSH;
    state_tick = HAL_GetTick();
    pending_cmd = MSPM0_CMD_NONE;
}

void TaskCtrl_Process(void)
{
    Ball_Position pos;
    Mspm0_Cmd cmd;
    uint32_t now = HAL_GetTick();

    /* ---- 读 MSPM0 命令(非阻塞) ---- */
    if (Mspm0_DataReady()) {
        pending_cmd = Mspm0_GetCmd();
        Mspm0_ClearCmd();
    }

    /* ---- 各状态 ---- */
    switch (state) {

    /* ================================================================
     * 上电推送: 用 MaixPro PD 把球推到死区, 稳定后锁死在平衡角
     * ================================================================ */
    case TASK_INIT_PUSH:
        MaixPro_Process();  /* 跑 PD  + 刹车 + 停球推 */
        /* 检测稳定: 球在死区内且速度 < 4px/s 持续 1s */
        if (MaixPro_GetPosition(&pos)) {
            if (pos.dist <= MAIXPRO_DEAD_ZONE) {
                if ((now - state_tick) > 1000) {
                    ResetAndLock();
                    state = TASK_WAIT_CMD;
                }
            } else {
                state_tick = now;
            }
        }
        break;

    /* ================================================================
     * 等指令
     * ================================================================ */
    case TASK_WAIT_CMD:
        if (pending_cmd == MSPM0_CMD_NONE) break;
        cmd = pending_cmd;
        pending_cmd = MSPM0_CMD_NONE;

        switch (cmd) {
        case MSPM0_CMD_0:
            state = TASK_MENU;
            ResetAndLock();
            break;
        case MSPM0_CMD_1:
            state = TASK_LOCK;
            Motor_SetMode(MOTOR_MODE_BRAKE);
            break;
        case MSPM0_CMD_2:
            MaixPro_RequestOriginCapture();
            state = TASK_SWING;
            state_tick = now;
            break;
        case MSPM0_CMD_3:
        case MSPM0_CMD_4:
            MaixPro_RequestOriginCapture();
            state = TASK_HOLD;
            break;
        default:
            break;
        }
        break;

    /* ================================================================
     * CMD 1: 锁死
     * ================================================================ */
    case TASK_LOCK:
        if (pending_cmd != MSPM0_CMD_NONE) {
            cmd = pending_cmd;
            pending_cmd = MSPM0_CMD_NONE;
            if (cmd == MSPM0_CMD_0) {
                state = TASK_MENU;
                ResetAndLock();
            } else if (cmd == MSPM0_CMD_2) {
                MaixPro_RequestOriginCapture();
                state = TASK_SWING;
                state_tick = now;
            } else if (cmd == MSPM0_CMD_3 || cmd == MSPM0_CMD_4) {
                MaixPro_RequestOriginCapture();
                state = TASK_HOLD;
            }
        }
        break;

    /* ================================================================
     * CMD 2: 设原点 → 经 -50px → 停 +50px
     * ================================================================ */
    case TASK_SWING:
        MaixPro_Process();  /* PD 正常控制 */
        /* 检测稳定: 球在 +50 附近且速度 < 4px/s */
        if (MaixPro_GetPosition(&pos)) {
            float e = (pos.lr == 0) ? (float)pos.dist : -(float)pos.dist;
            if (fabsf(e - 50.0f) <= MAIXPRO_DEAD_ZONE) {
                if ((now - state_tick) > 1000) {
                    ResetAndLock();
                    state = TASK_WAIT_CMD;
                }
            } else {
                state_tick = now;
            }
        }
        if (pending_cmd == MSPM0_CMD_0) {
            pending_cmd = MSPM0_CMD_NONE;
            state = TASK_MENU;
            ResetAndLock();
        }
        break;

    /* ================================================================
     * CMD 3/4: 设原点 → 保持原点
     * ================================================================ */
    case TASK_HOLD:
        MaixPro_Process();
        if (pending_cmd == MSPM0_CMD_0) {
            pending_cmd = MSPM0_CMD_NONE;
            state = TASK_MENU;
            ResetAndLock();
        }
        break;

    /* ================================================================
     * CMD 0: 回主菜单, 锁平衡角, 等新指令
     * ================================================================ */
    case TASK_MENU:
        if (pending_cmd != MSPM0_CMD_NONE) {
            cmd = pending_cmd;
            pending_cmd = MSPM0_CMD_NONE;
            if (cmd == MSPM0_CMD_1) {
                state = TASK_LOCK;
                Motor_SetMode(MOTOR_MODE_BRAKE);
            } else if (cmd == MSPM0_CMD_2) {
                MaixPro_RequestOriginCapture();
                state = TASK_SWING;
                state_tick = now;
            } else if (cmd == MSPM0_CMD_3 || cmd == MSPM0_CMD_4) {
                MaixPro_RequestOriginCapture();
                state = TASK_HOLD;
            }
        }
        break;
    }
}

/* 电机锁死在当前角度(刹车) */
static void ResetAndLock(void)
{
    Motor_SetMode(MOTOR_MODE_BRAKE);
}
