#ifndef __MAIXPRO_H__
#define __MAIXPRO_H__

#include "main.h"

typedef struct { uint8_t lr; uint16_t dist; } Ball_Position;
typedef void (*MaixPro_Callback)(const Ball_Position* pos);

#define MAIXPRO_UART          huart1
#define MAIXPRO_LINE_BUF_SIZE 32
#define MAIXPRO_DEAD_ZONE     10
#define MAIXPRO_PX_PER_CM     10

/* 进死区那版: K1=2, K2=2.5, K3=-6, K4=-12, KI=1.5, 不加摩擦 */
#define MAIXPRO_CTRL_K1           2.0f
#define MAIXPRO_CTRL_K2           2.5f
#define MAIXPRO_CTRL_K3          -6.0f
#define MAIXPRO_CTRL_KI           1.5f
#define MAIXPRO_CTRL_KI_LIM       80.0f
#define MAIXPRO_CTRL_K4          -12.0f
#define MAIXPRO_CTRL_FRIC_COMP    0.0f
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
