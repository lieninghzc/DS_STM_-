/**
 ******************************************************************************
 * @file    maixpro.c
 * @brief   MaixCAM UART + 状态反推控制 + 寻平衡
 *
 *   ω = Kp·e + Kd·v − Ka·(θ̂−balance)
 *   位置+速度+加速度, 直出电机速度, 不经过角度环
 ******************************************************************************
 */

#include "maixpro.h"
#include "motor.h"
#include "usart.h"
#include "MSPM0.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static volatile uint8_t  rx_byte;
static char              line_buf[MAIXPRO_LINE_BUF_SIZE];
static volatile uint8_t  line_idx   = 0;
static volatile uint8_t  data_ready = 0;
static Ball_Position     current_pos;
static MaixPro_Callback  user_callback = NULL;

static uint8_t    need_balance   = 1;
static uint8_t    seeking        = 0;
static float      theta_est      = 0;
static int32_t    last_speed_hz  = 0;
static float      balance_theta  = 0;
static float      seek_hist[5];
static uint8_t    seek_hist_idx  = 0;
static uint16_t   prev_dist      = 0;
static float      vel_est        = 0;
static float      vel_prev       = 0;
static float      accel_est      = 0;
static float      err_integral   = 0;
static float      last_err       = 0;
static uint32_t   last_data_tick = 0;
static uint32_t   last_ctrl_tick = 0;
static uint32_t   last_frame_tick = 0;

static volatile uint8_t capture_origin_pending = 0;
static uint8_t          origin_hold_active  = 0;
static int16_t          origin_pos_signed   = 0;

static void   MaixPro_ParseLine(const char *line);
static void   MaixPro_StartRx(void);
static float  MaixPro_CalcError(const Ball_Position *pos);

void MaixPro_Init(void)
{
    memset(line_buf, 0, sizeof(line_buf));
    line_idx = 0; data_ready = 0;
    last_frame_tick = 0; last_ctrl_tick = 0; last_data_tick = 0;
    theta_est = 0; last_speed_hz = 0; balance_theta = 0;
    memset(seek_hist, 0, sizeof(seek_hist)); seek_hist_idx = 0; prev_dist = 0;
    vel_est = 0; vel_prev = 0; accel_est = 0; err_integral = 0; last_err = 0;
    need_balance = 1; seeking = 0;
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    MaixPro_StartRx();
}

void MaixPro_RegisterCallback(MaixPro_Callback cb) { user_callback = cb; }

void MaixPro_Process(void)
{
    Ball_Position pos;
    uint32_t now;
    float dt_ctrl, e, omega;

    now = HAL_GetTick();
    dt_ctrl = (float)(now - last_ctrl_tick) / 1000.0f;
    if (dt_ctrl <= 0.0f || dt_ctrl > 0.2f) dt_ctrl = 0.01f;
    last_ctrl_tick = now;

    theta_est += (float)last_speed_hz * dt_ctrl;
    if (theta_est >  MAIXPRO_CTRL_THETA_MAX) theta_est =  MAIXPRO_CTRL_THETA_MAX;
    if (theta_est < -MAIXPRO_CTRL_THETA_MAX) theta_est = -MAIXPRO_CTRL_THETA_MAX;

    if (!data_ready) {
        if (last_data_tick && (now - last_data_tick) > 1500) {
            Motor_SetMode(MOTOR_MODE_NEUTRAL);
            need_balance = 1; seeking = 0; last_data_tick = 0;
        }
        return;
    }

    pos = current_pos; data_ready = 0; last_data_tick = now;
    if (user_callback) user_callback(&pos);
    e = MaixPro_CalcError(&pos);

    /* ---- 寻平衡 ---- */
    if (need_balance) {
        if (pos.dist <= MAIXPRO_DEAD_ZONE) {
            balance_theta = theta_est; need_balance = 0; seeking = 0;
            err_integral = 0;
            Motor_SetMode(MOTOR_MODE_BRAKE); last_speed_hz = 0;
            last_err = e; last_frame_tick = now; return;
        }
        uint8_t dir = (pos.lr == 0) ? 1 : 0;
        if (!seeking) { seeking = 1; prev_dist = pos.dist;
            memset(seek_hist, 0, sizeof(seek_hist)); seek_hist_idx = 0; }
        seek_hist[seek_hist_idx % 5] = theta_est; seek_hist_idx++;
        int16_t dd = (int16_t)pos.dist - (int16_t)prev_dist;
        if (dd < 0) dd = -dd;
        if (dd >= MAIXPRO_SEEK_MOVE_THRESH) {
            balance_theta = seek_hist[(seek_hist_idx - 5) % 5];
            need_balance = 0; seeking = 0;
            theta_est = balance_theta; last_speed_hz = 0;
            last_err = e; last_frame_tick = now; goto normal_ctrl;
        }
        prev_dist = pos.dist;
        Motor_SetSpeedHz(MAIXPRO_SEEK_SPEED_HZ);
        Motor_SetMode(dir ? MOTOR_MODE_FORWARD : MOTOR_MODE_REVERSE);
        last_speed_hz = dir ? (int32_t)MAIXPRO_SEEK_SPEED_HZ : -(int32_t)MAIXPRO_SEEK_SPEED_HZ;
        return;
    }

    /* ---- 死区 ---- */
    if (pos.dist <= MAIXPRO_DEAD_ZONE && fabsf(vel_est) < 4.0f) {
        Motor_SetMode(MOTOR_MODE_BRAKE); last_speed_hz = 0;
        theta_est = balance_theta;
        err_integral = 0;
        last_frame_tick = 0; return;
    }

    /* ---- 正常控制 ---- */
normal_ctrl:
    if (last_frame_tick != 0) {
        float dt_f = (float)(now - last_frame_tick) / 1000.0f;
        if (dt_f > 0.001f) {
            float v_raw = (e - last_err) / dt_f;
            vel_est += MAIXPRO_CTRL_VEL_ALPHA * (v_raw - vel_est);
            accel_est += MAIXPRO_CTRL_VEL_ALPHA
                       * ((vel_est - vel_prev) / dt_f - accel_est);
        }
        vel_prev = vel_est;
    }
    last_err = e; last_frame_tick = now;

    err_integral += e * dt_ctrl;
    if (err_integral >  MAIXPRO_CTRL_KI_LIM) err_integral =  MAIXPRO_CTRL_KI_LIM;
    if (err_integral < -MAIXPRO_CTRL_KI_LIM) err_integral = -MAIXPRO_CTRL_KI_LIM;

    omega = MAIXPRO_CTRL_K1 * e
          + MAIXPRO_CTRL_K2 * vel_est
          + MAIXPRO_CTRL_K3 * (accel_est / MAIXPRO_ACCEL_PER_STEP)
          + MAIXPRO_CTRL_KI * err_integral
          + MAIXPRO_CTRL_K4 * (theta_est - balance_theta);

    /* 摩擦补偿: 近死区 (<20px) 加推力快速推入 */
    {
        float fric = MAIXPRO_CTRL_FRIC_COMP;
        if (fabsf(e) < 20.0f) fric *= 2.0f;  /* 近死区K1弱, 摩擦补加倍 */
        if (fabsf(e) > MAIXPRO_DEAD_ZONE)
            omega += (e > 0) ? fric : -fric;
    }

    if (omega >  MAIXPRO_MAX_SPEED_HZ) omega =  MAIXPRO_MAX_SPEED_HZ;
    if (omega < -MAIXPRO_MAX_SPEED_HZ) omega = -MAIXPRO_MAX_SPEED_HZ;

    if (omega > (int32_t)MOTOR_MIN_FREQ_HZ) {
        Motor_SetSpeedHz((uint16_t)omega); Motor_SetMode(MOTOR_MODE_FORWARD);
        last_speed_hz = (int32_t)omega;
    } else if (omega < -(int32_t)MOTOR_MIN_FREQ_HZ) {
        Motor_SetSpeedHz((uint16_t)(-omega)); Motor_SetMode(MOTOR_MODE_REVERSE);
        last_speed_hz = (int32_t)omega;
    } else {
        Motor_SetMode(MOTOR_MODE_BRAKE); last_speed_hz = 0;
    }
}

uint8_t MaixPro_GetPosition(Ball_Position *pos)
{ if (pos) { *pos = current_pos; return 1; } return 0; }

void MaixPro_RequestOriginCapture(void) { capture_origin_pending = 1; }

static float MaixPro_CalcError(const Ball_Position *pos)
{
    float p = (pos->lr == 0) ? (float)pos->dist : -(float)pos->dist;
    if (capture_origin_pending) {
        origin_pos_signed = (int16_t)p; capture_origin_pending = 0;
        origin_hold_active = 1; return 0.0f;
    }
    if (origin_hold_active) return p - (float)origin_pos_signed;
    return p;
}

static void MaixPro_ParseLine(const char *line)
{
    int lr_val, dist_val;
    if (sscanf(line, "%d,%d", &lr_val, &dist_val) == 2)
        if ((lr_val == 0 || lr_val == 1) && dist_val >= 0)
        { current_pos.lr = (uint8_t)lr_val; current_pos.dist = (uint16_t)dist_val; data_ready = 1; }
}

static void MaixPro_StartRx(void) { HAL_UART_Receive_IT(&MAIXPRO_UART, &rx_byte, 1); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) { Mspm0_RxByte(mspm0_rx_byte); return; }
    if (huart->Instance != USART1) return;
    if (rx_byte == '\n') {
        if (line_idx > 0 && line_idx < MAIXPRO_LINE_BUF_SIZE)
        { line_buf[line_idx] = '\0'; MaixPro_ParseLine(line_buf); }
        line_idx = 0;
    } else if (rx_byte != '\r') {
        if (line_idx < MAIXPRO_LINE_BUF_SIZE - 1) line_buf[line_idx++] = rx_byte;
        else line_idx = 0;
    }
    MaixPro_StartRx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) { Mspm0_RxError(); return; }
    if (huart->Instance != USART1) return;
    line_idx = 0; MaixPro_StartRx();
}
