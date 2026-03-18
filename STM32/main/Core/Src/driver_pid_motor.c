/**
 * @file driver_pid_motor.c
 * @brief Implémentation du régulateur PID avec logique de séparation Avant/Arrière (Split)
 */

#include "driver_pid_motor.h"
#include "tim.h"

/**
 * @def MAX_PWM_OUTPUT
 * @brief Valeur maximale autorisée pour la commande de sortie.
 */
#define MAX_PWM_OUTPUT 1000.0f

/**
 * @brief Initialise le régulateur PID avec les paramètres par défaut et lie les périphériques.
 * @param hpid Pointeur vers la structure PID.
 * @param hmotor Pointeur vers le handler du moteur.
 * @param hspeedo Pointeur vers le handler du tachymètre.
 */
void pid_init(PID_Handle_t *hpid, Motor_Handle_t *hmotor, Speedometer_Handle_t *hspeedo){
    if(!hpid || !hmotor || !hspeedo) return;

    hpid->motor = hmotor;
    hpid->speedo = hspeedo;

    hpid->Kp = 10.0f;
    hpid->Ki = 25.0f;
    hpid->Kd = 0.0f;
    hpid->max_integral = PID_MAX_INTEGRAL;

    hpid->target_speed_ms = 0.0f;
    hpid->integral_sum = 0.0f;
    hpid->prev_error = 0.0f;
    hpid->last_time_ms = HAL_GetTick();
}

/**
 * @brief Définit la vitesse cible et réinitialise les accumulateurs si la consigne est nulle.
 * @param hpid Pointeur vers la structure PID.
 * @param speed_ms Vitesse cible en m/s (bornée entre -15 et 15).
 */
void pid_set_target_speed(PID_Handle_t *hpid, float speed_ms){
    if(!hpid) return;

    if (speed_ms > 15.0f) speed_ms = 15.0f;
    if (speed_ms < -15.0f) speed_ms = -15.0f;

    hpid->target_speed_ms = speed_ms;

    if (speed_ms == 0.0f) {
        hpid->integral_sum = 0.0f;
        hpid->prev_error = 0.0f;
        motor_set_power(hpid->motor, 0);
    }
}

/**
 * @brief Calcule et applique l'asservissement PID.
 * * Cette version implémente une logique "Split" qui empêche le régulateur d'inverser
 * la puissance moteur par rapport au signe de la consigne, évitant ainsi les
 * oscillations brutales au passage par zéro.
 * * @param hpid Pointeur vers la structure PID.
 */
void pid_process(PID_Handle_t *hpid){
    if(!hpid || !hpid->motor || !hpid->speedo) return;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - hpid->last_time_ms) / 1000.0f;
    if (dt <= 0.0001f) return;
    hpid->last_time_ms = now;

    float actual_speed_ms = hpid->speedo->current_speed_ms;
    float error = hpid->target_speed_ms - actual_speed_ms;

    float p_out = hpid->Kp * error;

    if (hpid->target_speed_ms != 0.0f) {
        hpid->integral_sum += (error * dt);
        if (hpid->integral_sum > hpid->max_integral) hpid->integral_sum = hpid->max_integral;
        else if (hpid->integral_sum < -hpid->max_integral) hpid->integral_sum = -hpid->max_integral;
    } else {
        hpid->integral_sum = 0.0f;
    }
    float i_out = hpid->Ki * hpid->integral_sum;

    float d_out = hpid->Kd * ((error - hpid->prev_error) / dt);

    float output_power = p_out + i_out + d_out;

    if (output_power > 0.01f) output_power += MOTOR_DEADBAND;
    else if (output_power < -0.01f) output_power -= MOTOR_DEADBAND;

    if (hpid->target_speed_ms > 0.0f) {
        if (output_power < 0.0f) output_power = 0.0f;
    }
    else if (hpid->target_speed_ms < 0.0f) {
        if (output_power > 0.0f) output_power = 0.0f;
    }
    else {
        output_power = 0.0f;
    }

    if (output_power > MAX_PWM_OUTPUT) output_power = MAX_PWM_OUTPUT;
    else if (output_power < -MAX_PWM_OUTPUT) output_power = -MAX_PWM_OUTPUT;

    hpid->prev_error = error;
    hpid->last_output_power = output_power;

    motor_set_power(hpid->motor, (int16_t)output_power);
}
