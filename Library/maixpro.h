#ifndef __MAIXPRO_H__
#define __MAIXPRO_H__

#include "main.h"

typedef struct
{
        uint8_t lr;
        uint16_t dist;
} Ball_Position;

typedef void (*MaixPro_Callback)(const Ball_Position* pos);

#define MAIXPRO_UART huart1
#define MAIXPRO_LINE_BUF_SIZE 32
#define MAIXPRO_DEAD_ZONE 10
#define MAIXPRO_PX_PER_CM 10

/* 物理: 1步→42px/s², 刹车含电机响应时间补偿 */
#define MAIXPRO_CTRL_KP_BALL 0.03f
#define MAIXPRO_CTRL_KI_BALL 0.02f
#define MAIXPRO_CTRL_KD_BALL 0.05f
#define MAIXPRO_CTRL_INTEGRAL_LIM 30
#define MAIXPRO_CTRL_KP_ANG 44.0f
#define MAIXPRO_CTRL_THETA_MAX 267
#define MAIXPRO_CTRL_VEL_ALPHA 0.5f
#define MAIXPRO_CTRL_VEL_LIM 200.0f
#define MAIXPRO_CTRL_THETA_SLEW 267
#define MAIXPRO_CTRL_BRAKE_GAIN 60.0f /* 刹车增益, 越小越猛 */

#define MAIXPRO_SEEK_SPEED_HZ 44
#define MAIXPRO_SEEK_MOVE_THRESH 5
#define MAIXPRO_MAX_SPEED_HZ 120 /* 10°/s, 不抖 */

void MaixPro_Init (void);
void MaixPro_RegisterCallback (MaixPro_Callback callback);
void MaixPro_Process (void);
uint8_t MaixPro_GetPosition (Ball_Position* pos);
void MaixPro_RequestOriginCapture (void);

#endif
