/**
 * @file driver_speedometer.h
 * @brief Driver pour la mesure de vitesse via capteur à effet Hall (A3141)
 */

#ifndef DRIVER_SPEEDOMETER_H
#define DRIVER_SPEEDOMETER_H

#include "main.h"
#include <math.h>

/**
 * @def METERS_PER_TICK
 * @brief Facteur de conversion distance/impulsion.
 * @note Valeur calibrée : 3 mètres correspondent à 146 impulsions.
 */
#define METERS_PER_TICK     (3.0f / 146.0f)

/**
 * @struct Speedometer_Handle_t
 * @brief Structure de contrôle du tachymètre.
 */
typedef struct {
    /** @brief Pointeur vers le timer matériel utilisé pour le comptage. */
    TIM_HandleTypeDef *htim;

    /** @brief Dernière valeur lue du registre CNT du timer. */
    uint32_t last_counter_val;

    /** @brief Timestamp du dernier calcul en millisecondes. */
    uint32_t last_process_time;

    /** @brief Vitesse actuelle calculée en mètres par seconde (m/s). */
    float current_speed_ms;

} Speedometer_Handle_t;

/**
 * @brief Initialise le tachymètre et démarre le timer associé.
 * @param hSpeedo Pointeur vers la structure de contrôle.
 * @param htim Pointeur vers le handle du timer STM32.
 */
void speedometer_init(Speedometer_Handle_t *hSpeedo, TIM_HandleTypeDef *htim);

/**
 * @brief Calcule la vitesse instantanée en fonction de la différence de ticks et du temps écoulé.
 * @param hSpeedo Pointeur vers la structure de contrôle.
 * @return La vitesse calculée en m/s.
 */
float speedometer_solve_speed(Speedometer_Handle_t *hSpeedo);

#endif
