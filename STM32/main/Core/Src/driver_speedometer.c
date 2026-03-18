/**
 * @file driver_speedometer.c
 * @brief Implémentation des fonctions de mesure de vitesse
 */

#include "driver_speedometer.h"

/**
 * @brief Initialise le tachymètre et lance le comptage matériel.
 * @param hSpeedo Pointeur vers la structure de contrôle.
 * @param htim Pointeur vers le handle du timer configuré en mode horloge externe.
 */
void speedometer_init(Speedometer_Handle_t *hSpeedo, TIM_HandleTypeDef *htim){
    hSpeedo->htim = htim;
    hSpeedo->current_speed_ms = 0.0f;
    hSpeedo->last_process_time = HAL_GetTick();

    HAL_TIM_Base_Start(hSpeedo->htim);

    hSpeedo->last_counter_val = __HAL_TIM_GET_COUNTER(hSpeedo->htim);
}

/**
 * @brief Calcule la vitesse réelle en m/s.
 * * Cette fonction utilise la différence de ticks du timer (gestion de l'overflow incluse)
 * et le temps écoulé (dt) pour déterminer la vitesse instantanée.
 * * @param hSpeedo Pointeur vers la structure de contrôle.
 * @return float La vitesse calculée en mètres par seconde.
 */
float speedometer_solve_speed(Speedometer_Handle_t *hSpeedo){
    uint32_t now = HAL_GetTick();
    uint32_t time_diff_ms = now - hSpeedo->last_process_time;

    if(time_diff_ms == 0){
        return hSpeedo->current_speed_ms;
    }

    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(hSpeedo->htim);

    uint16_t diff_ticks = (uint16_t)(current_cnt - hSpeedo->last_counter_val);

    float distance_m = (float)diff_ticks * METERS_PER_TICK;

    float speed = (distance_m * 1000.0f) / (float)time_diff_ms;

    hSpeedo->last_counter_val = current_cnt;
    hSpeedo->last_process_time = now;
    hSpeedo->current_speed_ms = speed;

    return speed;
}
