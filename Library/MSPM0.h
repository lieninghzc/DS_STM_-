/**
 ******************************************************************************
 * @file    MSPM0.h
 * @brief   MSPM0 UART 通信协议 (USART2, PA2/PA3)
 *          MSPM0 每次发送一个数字 0~4
 ******************************************************************************
 */

#ifndef __MSPM0_H__
#define __MSPM0_H__

#include "main.h"

/* 命令枚举 */
typedef enum {
    MSPM0_CMD_0    = 0,
    MSPM0_CMD_1    = 1,
    MSPM0_CMD_2    = 2,
    MSPM0_CMD_3    = 3,
    MSPM0_CMD_4    = 4,
    MSPM0_CMD_NONE = 0xFF
} Mspm0_Cmd;

typedef void (*Mspm0_Callback)(Mspm0_Cmd cmd);

extern volatile uint8_t mspm0_rx_byte;

void      Mspm0_Init(void);
void      Mspm0_RegisterCallback(Mspm0_Callback cb);
uint8_t   Mspm0_DataReady(void);
Mspm0_Cmd Mspm0_GetCmd(void);
void      Mspm0_ClearCmd(void);

/* 命令执行逻辑 */
void      Mspm0_HandleCmd(Mspm0_Cmd cmd);

/* 由 HAL 回调统一调用, 不直接使用 */
void      Mspm0_RxByte(uint8_t byte);
void      Mspm0_RxError(void);

#endif /* __MSPM0_H__ */
