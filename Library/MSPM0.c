/**
 ******************************************************************************
 * @file    MSPM0.c
 * @brief   MSPM0 UART 协议 (USART2 中断接收单字节 0~4)
 ******************************************************************************
 */

#include "MSPM0.h"
#include "usart.h"
#include "maixpro.h"
#include "motor.h"

volatile uint8_t          mspm0_rx_byte;
static volatile Mspm0_Cmd rx_cmd   = MSPM0_CMD_NONE;
static volatile uint8_t   data_rdy = 0;
static Mspm0_Callback     callback = NULL;

static void Mspm0_StartRx(void)
{
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&mspm0_rx_byte, 1);
}

void Mspm0_Init(void)
{
    rx_cmd   = MSPM0_CMD_NONE;
    data_rdy = 0;
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    Mspm0_StartRx();
}

void Mspm0_RegisterCallback(Mspm0_Callback cb)  { callback = cb; }
uint8_t   Mspm0_DataReady(void)                 { return data_rdy; }
Mspm0_Cmd Mspm0_GetCmd(void)                    { return rx_cmd; }

void Mspm0_ClearCmd(void)
{
    data_rdy = 0;
    rx_cmd   = MSPM0_CMD_NONE;
}

/* 由 maixpro.c 的 HAL_UART_RxCpltCallback 统一调用 */
void Mspm0_RxByte(uint8_t byte)
{
    if (byte <= 4) {    /* MSPM0 发原始二进制 0x00~0x04 */
        rx_cmd   = (Mspm0_Cmd)byte;   /* 直接映射, 不要减 '0' */
        data_rdy = 1;
        if (callback) callback(rx_cmd);
        Mspm0_HandleCmd(rx_cmd);      /* 执行命令逻辑 */
    }
    Mspm0_StartRx();  /* 重新启动 USART2 中断接收 */
}

void Mspm0_RxError(void)
{
    Mspm0_StartRx();
}

/* ================================================================
 * MSPM0 命令执行逻辑
 *
 * 0: 待定 (暂回空挡)
 * 1: 电机锁死 (使能保持力矩)
 * 2: 待定
 * 3: 以接信号后相机第一个坐标为原点, 控制小球在其附近
 * 4: 同上
 * ================================================================ */
void Mspm0_HandleCmd(Mspm0_Cmd cmd)
{
    switch (cmd) {
    case MSPM0_CMD_0:
        /* TODO: 逻辑待定 — 暂回空挡 */
        Motor_SetMode(MOTOR_MODE_NEUTRAL);
        break;

    case MSPM0_CMD_1:
        /* 电机锁死: ENABLE=LOW + 无脉冲 = 保持力矩锁住电机 */
        Motor_SetMode(MOTOR_MODE_BRAKE);
        break;

    case MSPM0_CMD_2:
        /* TODO: 逻辑待定 */
        break;

    case MSPM0_CMD_3:
    case MSPM0_CMD_4:
        /* 重设原点: 相机下一个坐标记为原点, 开始原点保持 */
        MaixPro_RequestOriginCapture();
        break;

    default:
        break;
    }
}
