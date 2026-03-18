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
#include "driver_speedometer.h"
#include "driver_pid_motor.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static uint32_t last_print_us = 0;

#define TASK_MOTOR_US       1000   // 1ms
#define TASK_TELEMETRY_US   10000  // 10ms
#define TASK_SPEED_US		20000  // 20ms
#define FAILSAFE_TIMEOUT_MS 500    // 500ms
#define TASK_PID_US			20000 // 2Oms
#define TASK_PRINT_US       50000

#define t_1_ms 3200
#define t_2_ms 6400

#define PWM_MIN_ESC 3200
#define PWM_MAX_ESC 6400

#define SPEED_FILTER_ALPHA 0.50f

static Servo_Handle_t hServo1 = {
    .htim = &htim1,           		   // Instance du Timer
    .channel = TIM_CHANNEL_1, 	       // Canal PWM
    .min_pulse_ticks = t_1_ms,         // Ticks pour 0% (ex: 1ms)
    .max_pulse_ticks = t_2_ms          // Ticks pour 100% (ex: 2ms)
};

static Motor_Handle_t hMotor1 = {
    .htim = &htim2,					   // Instance du Timer
    .channel = TIM_CHANNEL_1,		   // Canal PWM
    .min_pulse_ticks = PWM_MIN_ESC,	   //
    .max_pulse_ticks = PWM_MAX_ESC,    //
    .max_speed_pos_mms = 1000,         // Limit max frwd
    .max_speed_neg_mms = -500		   // Limit max bkwd
};

Speedometer_Handle_t hSpeedo;
PID_Handle_t hpid;

static uint32_t last_cmd_time_ms = 0;  // Watchdog (ms)
static uint32_t last_motor_us = 0;     // Tâche moteur (µs)
static uint32_t last_telemetry_us = 0; // Tâche télémétrie (µs)
static uint32_t last_speed_us = 0;
static uint32_t last_pid_us = 0;

float speed_speedo_data = 0.0f;

volatile uint32_t tim3_overflow_cnt = 0;

static uint32_t GetMicrosTotal(void);
static void process_incoming_commands(void);
static void check_failsafe_security(void);
static void task_motor_update(uint32_t now_us);
static void task_telemetry_update(uint32_t now_us);
static void task_get_speed(uint32_t now_us);
static void task_pid_update(uint32_t now_us);
static void task_debug_terminal(uint32_t now_us);

static uint32_t GetMicrosTotal(void){
    uint32_t m_overflow;
    uint16_t m_counter;

    do{
        m_overflow = tim3_overflow_cnt;
        m_counter = LL_TIM_GetCounter(TIM3);
    }
    while(m_overflow != tim3_overflow_cnt);

    return (m_overflow << 16) + m_counter;
}

/**
 * @brief Traite les commandes reçues du PC
 * Utilise un SWITCH pour une extension facile future
 */
static void process_incoming_commands(void){
    if(parser_state == PARSER_IDLE){
        return;
    }

    last_cmd_time_ms = HAL_GetTick();

    switch(parser_state){
        case PARSER_SERVO_CMD:
            servo_pwm_angle_degree(&hServo1, shadow_servo_cmd);
            break;

        case PARSER_MOTOR_CMD:

            float target_m_s = (float)shadow_motor_cmd / 1000.0f;

            //motor_set_power(&hMotor1, shadow_motor_cmd);
            pid_set_target_speed(&hpid, target_m_s);

            break;

        default:
            break;
    }

    parser_state = PARSER_IDLE;
}

/**
 * @brief Vérifie la sécurité (Dead Man's Switch)
 */
static void check_failsafe_security(void){
    if((HAL_GetTick() - last_cmd_time_ms) > FAILSAFE_TIMEOUT_MS){
    	motor_set_power(&hMotor1, 0);
    }
}

/**
 * @brief Gère la boucle d'asservissement/physique moteur (1kHz)
 */
static void task_motor_update(uint32_t now_us){
    if((uint32_t)(now_us - last_motor_us) >= TASK_MOTOR_US){
        last_motor_us = now_us;
        motor_process_1ms(&hMotor1, HAL_GetTick());
    }
}

/**
 * @brief Gère l'envoi des données vers le PC (100Hz / 50Hz)
 */
static void task_telemetry_update(uint32_t now_us){
    if((uint32_t)(now_us - last_telemetry_us) >= TASK_TELEMETRY_US){
        last_telemetry_us = now_us;
        serial_send_data_frame();
    }
}

static void task_get_speed(uint32_t now_us){
    if((uint32_t)(now_us - last_speed_us) >= TASK_SPEED_US){
        last_speed_us = now_us;

        float raw_speed = speedometer_solve_speed(&hSpeedo);

        if (hpid.last_output_power < -0.1f) {
            raw_speed = -raw_speed;
        }

        speed_speedo_data = (SPEED_FILTER_ALPHA * raw_speed) +
                            ((1.0f - SPEED_FILTER_ALPHA) * speed_speedo_data);

        hSpeedo.current_speed_ms = speed_speedo_data;
    }
}

static void task_pid_update(uint32_t now_us){
    if((uint32_t)(now_us - last_pid_us) >= TASK_PID_US){
        last_pid_us = now_us;
        pid_process(&hpid);
    }
}

static void task_debug_terminal(uint32_t now_us){
    if((uint32_t)(now_us - last_print_us) >= TASK_PRINT_US){
        last_print_us = now_us;

        uint32_t raw_cnt = __HAL_TIM_GET_COUNTER(&htim4);

        float speed = speed_speedo_data;

        printf("Raw: %lu | Vitesse: %.2f m/s\r\n", raw_cnt, speed);
    }
}

void app_config(void){

	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	HAL_TIM_Base_Start(&htim4);

	serial_init();
	servo_initialisation(&hServo1);
	motor_init(&hMotor1);
	speedometer_init(&hSpeedo, &htim4);
	pid_init(&hpid, &hMotor1, &hSpeedo);

	last_cmd_time_ms  = HAL_GetTick();
	last_motor_us     = GetMicrosTotal();
	last_telemetry_us = GetMicrosTotal();
	last_speed_us     = GetMicrosTotal();
	last_pid_us       = GetMicrosTotal();

	servo_pwm_angle_degree(&hServo1, 0);
	motor_set_power(&hMotor1, 0);
	pid_set_target_speed(&hpid, 0.0f);
}

void app_loop(void){
	uint32_t now_us = GetMicrosTotal(); //70 minutes max

	serial_cmd_reader();//
    process_incoming_commands();
    //check_failsafe_security();
    task_motor_update(now_us);
    task_get_speed(now_us);
    task_telemetry_update(now_us);//
    task_pid_update(now_us);//
    //task_debug_terminal(now_us);
}
