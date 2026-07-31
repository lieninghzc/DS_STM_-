/**
 ******************************************************************************
 * @file    motor.c
 * @brief   42步进电机 + A4988 + PID 闭环控制
 *
 *          引脚功能 (硬件直连):
 *          PB14 → A4988 ENABLE (LOW=使能驱动, HIGH=禁止驱动)
 *          PB15 → A4988 DIR    (HIGH=正转/FORWARD, LOW=反转/REVERSE)
 *          PA8  → A4988 STEP   (PWM 脉冲)
 ******************************************************************************
 */

#include "motor.h"
#include "tim.h"

/* Private variables ---------------------------------------------------------*/

static Motor_Mode motor_mode    = MOTOR_MODE_NEUTRAL;
static uint16_t   motor_speed_hz = 1000;
static uint8_t    motor_running  = 0;

/* Private helpers -----------------------------------------------------------*/

static uint16_t Motor_CalcARR(uint16_t freq_hz)
{
    uint32_t arr;
    if (freq_hz == 0) return 0xFFFF;
    arr = MOTOR_TIM_CLK_HZ / freq_hz;
    if (arr < 2)  arr = 2;
    if (arr > 65535) arr = 65535;
    return (uint16_t)(arr - 1);
}

/* --- 电机基础控制 --------------------------------------------------------*/

void Motor_Init(void)
{
    /* 初始状态: NEUTRAL (ENABLE=HIGH=禁止, DIR=LOW) */
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_EN,  GPIO_PIN_SET);   /* HIGH = 禁止 */
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_DIR, GPIO_PIN_RESET);
    motor_mode    = MOTOR_MODE_NEUTRAL;
    motor_running = 0;

    /* 配置 TIM1 PWM */
    HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
    __HAL_TIM_SET_PRESCALER(&MOTOR_TIM, 71);
    __HAL_TIM_SET_AUTORELOAD(&MOTOR_TIM, MOTOR_DEFAULT_ARR);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_TIM_CHANNEL, MOTOR_DEFAULT_ARR / 2);
    MOTOR_TIM.Instance->EGR = TIM_EGR_UG;
    motor_speed_hz = MOTOR_TIM_CLK_HZ / (MOTOR_DEFAULT_ARR + 1);
}

void Motor_SetMode(Motor_Mode mode)
{
    /* 先停 PWM */
    HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
    motor_running = 0;

    switch (mode) {

    case MOTOR_MODE_NEUTRAL:
        /* ENABLE=HIGH → 禁止驱动, 电机自由旋转 */
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_EN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_DIR, GPIO_PIN_RESET);
        break;

    case MOTOR_MODE_FORWARD:
        /* ENABLE=LOW → 使能驱动, DIR=HIGH → 正转, PWM ON */
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_EN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_DIR, GPIO_PIN_SET);
        HAL_TIM_PWM_Start(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
        motor_running = 1;
        break;

    case MOTOR_MODE_REVERSE:
        /* ENABLE=LOW → 使能驱动, DIR=LOW → 反转, PWM ON */
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_EN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_DIR, GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
        motor_running = 1;
        break;

    case MOTOR_MODE_BRAKE:
        /* ENABLE=LOW → 使能驱动, PWM OFF → 线圈通电, 力矩保持 */
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN_EN, GPIO_PIN_RESET);
        /* DIR 保持不变, 不影响刹车 */
        break;
    }

    motor_mode = mode;
}

Motor_Mode Motor_GetMode(void)
{
    return motor_mode;
}

void Motor_SetSpeedHz(uint16_t freq_hz)
{
    uint16_t arr;

    if (freq_hz < MOTOR_MIN_FREQ_HZ) freq_hz = MOTOR_MIN_FREQ_HZ;
    if (freq_hz > MOTOR_MAX_FREQ_HZ) freq_hz = MOTOR_MAX_FREQ_HZ;

    motor_speed_hz = freq_hz;
    arr = Motor_CalcARR(freq_hz);

    __HAL_TIM_SET_AUTORELOAD(&MOTOR_TIM, arr);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_TIM_CHANNEL, arr / 2);
    MOTOR_TIM.Instance->EGR = TIM_EGR_UG;
}

uint16_t Motor_GetSpeedHz(void)
{
    return motor_speed_hz;
}

void Motor_Start(void)
{
    if ((motor_mode == MOTOR_MODE_FORWARD || motor_mode == MOTOR_MODE_REVERSE)
        && !motor_running) {
        HAL_TIM_PWM_Start(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
        motor_running = 1;
    }
}

void Motor_Stop(void)
{
    if (motor_running) {
        HAL_TIM_PWM_Stop(&MOTOR_TIM, MOTOR_TIM_CHANNEL);
        motor_running = 0;
    }
}

/* --- PID 控制器 -----------------------------------------------------------*/

void Motor_PID_Init(Motor_PID *pid, float Kp, float Ki, float Kd,
                    float integral_limit, int16_t output_limit)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
}

void Motor_PID_Reset(Motor_PID *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

int16_t Motor_PID_Compute(Motor_PID *pid, float error, float dt)
{
    float P_out, I_out, D_out, output;

    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.033f;
    }

    /* P */
    P_out = pid->Kp * error;

    /* I (带抗饱和) */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    I_out = pid->Ki * pid->integral;

    /* D (误差变化率 = -钢球速度) */
    D_out = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* 合成 + 限幅 */
    output = P_out + I_out + D_out;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return (int16_t)output;
}

void Motor_PID_Apply(int16_t output)
{
    if (output > (int16_t)MOTOR_MIN_FREQ_HZ) {
        /* 正转: 正值输出 */
        Motor_SetSpeedHz((uint16_t)output);
        Motor_SetMode(MOTOR_MODE_FORWARD);
    } else if (output < -(int16_t)MOTOR_MIN_FREQ_HZ) {
        /* 反转: 负值输出 */
        Motor_SetSpeedHz((uint16_t)(-output));
        Motor_SetMode(MOTOR_MODE_REVERSE);
    } else {
        /* 死区: 刹车保持 */
        Motor_SetMode(MOTOR_MODE_BRAKE);
    }
}
