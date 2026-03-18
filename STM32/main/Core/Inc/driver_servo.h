#ifndef INC_DRIVER_SERVO_H_
#define INC_DRIVER_SERVO_H_

#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *htim;   // Pointeur vers le timer (ex: &htim1)
    uint32_t channel;          // Canal du timer (ex: TIM_CHANNEL_1)
    uint16_t min_pulse_ticks;  // Valeur registre pour min (ex: 3200)
    uint16_t max_pulse_ticks;  // Valeur registre pour max (ex: 6400)
} Servo_Handle_t;

void servo_initialisation(Servo_Handle_t *hservo);
void servo_pwm_percent(Servo_Handle_t *hservo, uint8_t percent);
void servo_pwm_angle_degree(Servo_Handle_t *hservo, int8_t angle);
void servo_pwm_angle_abs_value(Servo_Handle_t *hservo, uint16_t abs_value);
void app_test_config(void);
void app_test_loop(void);

#endif
