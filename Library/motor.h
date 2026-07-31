/**
 ******************************************************************************
 * @file    motor.h
 * @brief   42步进电机控制模块 (A4988驱动) + PID 控制器
 * @note
 *          硬件接口:
 *          - PA8  (TIM1_CH1) -> A4988 STEP   (步进脉冲, PWM)
 *          - PB15             -> A4988 DIR    (方向: HIGH=正转, LOW=反转)
 *          - PB14             -> A4988 ENABLE (使能: LOW=使能, HIGH=禁止)
 *
 *          模式控制:
 *          | ENA(PB14) DIR(PB15) PWM  | 模式     | 说明                |
 *          | HIGH      x         OFF  | NEUTRAL  | 禁止驱动, 电机自由  |
 *          | LOW       HIGH      ON   | FORWARD  | 使能, DIR=HIGH, PWM |
 *          | LOW       LOW       ON   | REVERSE  | 使能, DIR=LOW, PWM  |
 *          | LOW       x         OFF  | BRAKE    | 使能, 无PWM, 力矩保 |
 *
 *          PID 控制模型:
 *          电机角度 θ → 平台倾角 → 钢球加速度 a = K * θ
 *          PID 输入: 钢球位置误差 (px)
 *          PID 输出: 电机步进频率 (Hz, 带符号, + = FORWARD)
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* Exported types ------------------------------------------------------------*/

/** @brief 电机运行模式 (逻辑模式, 非引脚编码) */
typedef enum {
    MOTOR_MODE_NEUTRAL = 0,  /* 空挡: ENABLE=HIGH, PWM=OFF     */
    MOTOR_MODE_FORWARD = 1,  /* 正转: ENABLE=LOW, DIR=HIGH, PWM */
    MOTOR_MODE_REVERSE = 2,  /* 反转: ENABLE=LOW, DIR=LOW, PWM  */
    MOTOR_MODE_BRAKE   = 3   /* 刹车: ENABLE=LOW, PWM=OFF       */
} Motor_Mode;

/** @brief PID 控制器 */
typedef struct {
    float   Kp;
    float   Ki;
    float   Kd;
    float   integral;
    float   prev_error;
    float   integral_limit;
    int16_t output_limit;
} Motor_PID;

/* Exported constants --------------------------------------------------------*/

/* 硬件引脚 */
#define MOTOR_PORT          GPIOB
#define MOTOR_PIN_EN        GPIO_PIN_14   /* PB14 -> A4988 ENABLE (LOW=使能) */
#define MOTOR_PIN_DIR       GPIO_PIN_15   /* PB15 -> A4988 DIR    (HIGH=正转) */

/* PWM 参数 */
#define MOTOR_TIM           htim1
#define MOTOR_TIM_CHANNEL   TIM_CHANNEL_1
#define MOTOR_TIM_CLK_HZ    1000000U
#define MOTOR_DEFAULT_ARR   999U

/* 速度限制 (1/16 微步: 3200 脉冲/转, 高灵敏度) */
#define MOTOR_MIN_FREQ_HZ   10U       /* 10Hz 起步                    */
#define MOTOR_MAX_FREQ_HZ   20000U    /* 20kHz = 6.25 rev/s    */

/* Exported functions prototypes ---------------------------------------------*/

/* --- 电机基础控制 --- */
void        Motor_Init(void);
void        Motor_SetMode(Motor_Mode mode);
Motor_Mode  Motor_GetMode(void);
void        Motor_SetSpeedHz(uint16_t freq_hz);
uint16_t    Motor_GetSpeedHz(void);
void        Motor_Start(void);
void        Motor_Stop(void);

/* --- PID 控制 --- */
void    Motor_PID_Init(Motor_PID *pid, float Kp, float Ki, float Kd,
                       float integral_limit, int16_t output_limit);
void    Motor_PID_Reset(Motor_PID *pid);
int16_t Motor_PID_Compute(Motor_PID *pid, float error, float dt);
void    Motor_PID_Apply(int16_t output);

#endif /* __MOTOR_H__ */
