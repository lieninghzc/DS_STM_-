/**
 ******************************************************************************
 * @file    task_ctrl.c
 * @brief   MSPM0 任务状态机
 *
 *   CMD2: 开环摆动 -50→+50, SWING1/2/3 定时相位
 *   CMD3/4: 设原点后 PD 保持
 ******************************************************************************
 */

#include "task_ctrl.h"
#include "motor.h"
#include "maixpro.h"
#include "usart.h"
#include <math.h>

typedef enum
{
    S_INIT,
    S_LOCKED,
    S_SWING1,
    S_SWING2,
    S_SWING3,
    S_SWING4,
    S_HOLD
} State;

static State st = S_INIT;
static uint32_t phase_ms = 0;
static int32_t net_tilt = 0; /* 累计净倾角 (步) */

void TaskCtrl_Init (void)
{
    st = S_INIT;
    phase_ms = 0;
    net_tilt = 0;
}

void TaskCtrl_Process (void)
{
    Ball_Position pos;
    Mspm0_Cmd cmd = MSPM0_CMD_NONE;
    uint32_t now = HAL_GetTick();

    if (Mspm0_DataReady())
    {
        cmd = Mspm0_GetCmd();
        Mspm0_ClearCmd();
    }
    if (!MaixPro_GetPosition(&pos))
        return;

    switch (st)
    {
        case S_INIT:
            MaixPro_Process();
            if (pos.dist <= MAIXPRO_DEAD_ZONE)
            {
                if ((now - phase_ms) > 1000)
                {
                    Motor_SetMode(MOTOR_MODE_BRAKE);
                    st = S_LOCKED;
                }
            }
            else
                phase_ms = now;
            break;

        case S_LOCKED:
            if (cmd == MSPM0_CMD_2)
            {
                net_tilt = 0;
                Motor_SetSpeedHz(80);
                Motor_SetMode(MOTOR_MODE_FORWARD);
                phase_ms = now;
                st = S_SWING1;
            }
            else if (cmd == MSPM0_CMD_3 || cmd == MSPM0_CMD_4)
            {
                MaixPro_RequestOriginCapture();
                st = S_HOLD;
            }
            break;

        case S_SWING1:
            if ((now - phase_ms) >= 500)
            {
                net_tilt += 500 * 80 / 1000;
                Motor_SetSpeedHz(80);
                Motor_SetMode(MOTOR_MODE_REVERSE);
                phase_ms = now;
                st = S_SWING2;
            }
            break;

        case S_SWING2:
            if ((now - phase_ms) >= 1000)
            {
                net_tilt -= 1000 * 80 / 1000;
                Motor_SetSpeedHz(80);
                Motor_SetMode(MOTOR_MODE_FORWARD);
                phase_ms = now;
                st = S_SWING3;
            }
            break;

        case S_SWING3:
            if ((now - phase_ms) >= 450)
            {
                MaixPro_SetTarget(50);
                { uint8_t rsp = 0x32; HAL_UART_Transmit(&huart2, &rsp, 1, 10); }
                phase_ms = now; st = S_SWING4;
            }
            break;

        case S_SWING4:
            MaixPro_Process();
            if ((now - phase_ms) > 2000)
            { Motor_SetMode(MOTOR_MODE_BRAKE); st = S_LOCKED; }
            break;

        case S_HOLD:
            if (cmd == MSPM0_CMD_0)
            {
                Motor_SetMode(MOTOR_MODE_BRAKE);
                st = S_LOCKED;
            }
            else
                MaixPro_Process();
            break;
    }
}
