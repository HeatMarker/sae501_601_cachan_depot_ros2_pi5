#ifndef INC_DRIVER_MOTOR_H_
#define INC_DRIVER_MOTOR_H_

#include "tim.h"
#include <stdbool.h>

typedef enum{
    MOTOR_STATE_NEUTRAL = 0,
    MOTOR_STATE_FORWARD_HOLD,
    MOTOR_STATE_FWD_BRAKE_TAP,
    MOTOR_STATE_FWD_NEUTRAL_GAP,
    MOTOR_STATE_REVERSE_HOLD,
    MOTOR_STATE_REV_BRAKE_TAP,
    MOTOR_STATE_REV_NEUTRAL_GAP,
    MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP,
    MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP
} MotorState_t;

typedef struct{
    int16_t  target_speed_mms;
    uint8_t  target_pwm;
    bool     target_forward;
    uint32_t deadline_ms;
} Motor_Context_t;

typedef struct{
    TIM_HandleTypeDef *htim;       /*!< Timer (ex: &htim2) */
    uint32_t channel;              /*!< Canal (ex: TIM_CHANNEL_1) */
    uint16_t min_pulse_ticks;      /*!< Ticks pour 0% (ex: 3200) */
    uint16_t max_pulse_ticks;      /*!< Ticks pour 100% (ex: 6400) */

    int16_t  max_speed_pos_mms;    /*!< Vitesse max avant (ex: 1000) */
    int16_t  max_speed_neg_mms;    /*!< Vitesse max arrière (ex: -500) */

    MotorState_t    state;
    bool            go_forward;
    Motor_Context_t ctx;
} Motor_Handle_t;

void motor_init(Motor_Handle_t *hmotor);
void motor_pwm_percent(Motor_Handle_t *hmotor, uint8_t percent);
void motor_set_speed_mms(Motor_Handle_t *hmotor, int16_t speed_mms);
void motor_process_1ms(Motor_Handle_t *hmotor, uint32_t now_ms);
void app_config(void);

#endif

