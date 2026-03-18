#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "app_main.h"
#include "driver_ins.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "driver_servo.h"
#include "driver_motor.h"
#include "dma.h"
#include "serial.h"
#include "serial_cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define GET_MICROS()        LL_TIM_GetCounter(TIM3)

#define TASK_MOTOR_US       1000   // 1ms
#define TASK_TELEMETRY_US   10000  // 10ms
#define FAILSAFE_TIMEOUT_MS 500    // 500ms

#define t_1_ms 3200
#define t_2_ms 6400

static Servo_Handle_t hServo1 = {
    .htim = &htim1,           		   // Instance du Timer
    .channel = TIM_CHANNEL_1, 	       // Canal PWM
    .min_pulse_ticks = t_1_ms,         // Ticks pour 0% (ex: 1ms)
    .max_pulse_ticks = t_2_ms          // Ticks pour 100% (ex: 2ms)
};

#define PWM_MIN_ESC 3200
#define PWM_MAX_ESC 6400

static Motor_Handle_t hMotor1 = {
    .htim = &htim2,					   // Instance du Timer
    .channel = TIM_CHANNEL_1,		   // Canal PWM
    .min_pulse_ticks = PWM_MIN_ESC,	   //
    .max_pulse_ticks = PWM_MAX_ESC,    //
    .max_speed_pos_mms = 1000,         // Limit max frwd
    .max_speed_neg_mms = -500		   // Limit max bkwd
};

static uint32_t last_cmd_time_ms = 0;  // Watchdog (ms)
static uint16_t last_motor_us = 0;     // Tâche moteur (µs)
static uint16_t last_telemetry_us = 0; // Tâche télémétrie (µs)

/**
 * @brief Traite les commandes reçues du PC
 * Utilise un SWITCH pour une extension facile future
 */
static void process_incoming_commands(void) {
    if (parser_state == PARSER_IDLE) {
        return;
    }

    last_cmd_time_ms = HAL_GetTick();

    switch (parser_state) {
        case PARSER_SERVO_CMD:
            servo_pwm_angle_degree(&hServo1, shadow_servo_cmd);
            break;

        case PARSER_MOTOR_CMD:
            motor_set_speed_mms(&hMotor1, shadow_motor_cmd);
            break;

        default:
            break;
    }

    parser_state = PARSER_IDLE;
}

/**
 * @brief Vérifie la sécurité (Dead Man's Switch)
 */
static void check_failsafe_security(void) {
    if ((HAL_GetTick() - last_cmd_time_ms) > FAILSAFE_TIMEOUT_MS) {
        motor_set_speed_mms(&hMotor1, 0);
    }
}

/**
 * @brief Gère la boucle d'asservissement/physique moteur (1kHz)
 */
static void task_motor_update(uint16_t now_us) {
    if ((uint16_t)(now_us - last_motor_us) >= TASK_MOTOR_US) {
        last_motor_us = now_us;
        motor_process_1ms(&hMotor1, HAL_GetTick());
    }
}

/**
 * @brief Gère l'envoi des données vers le PC (100Hz / 50Hz)
 */
static void task_telemetry_update(uint16_t now_us) {
    if ((uint16_t)(now_us - last_telemetry_us) >= TASK_TELEMETRY_US) {
        last_telemetry_us = now_us;
        serial_send_imu_frame();
    }
}

void app_config(void){
	LL_TIM_EnableCounter(TIM3);
	serial_init();
	BMI088_Init(&hspi1);

	servo_initialisation(&hServo1);
	motor_init(&hMotor1);
	motor_pwm_percent(&hMotor1, 50);

	last_cmd_time_ms = HAL_GetTick();
	last_motor_us = (uint16_t)GET_MICROS();
	last_telemetry_us = (uint16_t)GET_MICROS();
}

void app_loop(void){
	uint16_t now_us = (uint16_t)GET_MICROS();

	    serial_cmd_reader();

	    process_incoming_commands();
	    check_failsafe_security();

	    task_motor_update(now_us);
	    task_telemetry_update(now_us);
}

