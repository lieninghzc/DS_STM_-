#ifndef __MAIXPRO_H__
#define __MAIXPRO_H__

#include "main.h"

typedef struct { uint8_t lr; uint16_t dist; } Ball_Position;
typedef void (*MaixPro_Callback)(const Ball_Position* pos);

#define MAIXPRO_UART          huart1
#define MAIXPRO_LINE_BUF_SIZE 32
#define MAIXPRO_DEAD_ZONE     10
#define MAIXPRO_PX_PER_CM     10

/* 四变量全状态反推 + 积分 (放大增益, 快收敛 + 消假平衡) */
#define MAIXPRO_CTRL_K1           2.0f    /* 位置项: 30px→60Hz, 超死区 */
#define MAIXPRO_CTRL_K2           2.5f    /* 速度项: 阻尼 */
#define MAIXPRO_CTRL_K3          -6.0f    /* 加速度项 */
#define MAIXPRO_CTRL_KI           1.5f    /* 积分项: 30px→45Hz/s, 强力推 */
#define MAIXPRO_CTRL_KI_LIM       80.0f   /* 积分限幅 */
#define MAIXPRO_CTRL_K4          -12.0f   /* 角度项 */
#define MAIXPRO_ACCEL_PER_STEP    42.0f
#define MAIXPRO_CTRL_THETA_MAX    267
#define MAIXPRO_CTRL_VEL_ALPHA    0.5f
#define MAIXPRO_CTRL_VEL_LIM      200.0f

#define MAIXPRO_SEEK_SPEED_HZ     44
#define MAIXPRO_SEEK_MOVE_THRESH  5
#define MAIXPRO_MAX_SPEED_HZ      120

void MaixPro_Init(void);
void MaixPro_RegisterCallback(MaixPro_Callback callback);
void MaixPro_Process(void);
uint8_t MaixPro_GetPosition(Ball_Position* pos);
void MaixPro_RequestOriginCapture(void);

#endif
