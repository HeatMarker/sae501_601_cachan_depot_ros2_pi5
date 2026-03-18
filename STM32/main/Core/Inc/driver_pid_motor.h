/**
 * @file driver_pid_motor.h
 * @brief Header du contrôleur PID pour l'asservissement du moteur
 */

#ifndef INC_DRIVER_PID_MOTOR_H_
#define INC_DRIVER_PID_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver_motor.h"
#include "driver_speedometer.h"

/** @defgroup PID_Config Configuration par défaut du régulateur */
/** @{ */
#define PID_DEFAULT_KP   20.0f
#define PID_DEFAULT_KI   25.0f
#define PID_DEFAULT_KD   0.0f
#define PID_MAX_INTEGRAL 100.0f
/** @} */

/**
 * @def MOTOR_DEADBAND
 * @brief Valeur de compensation de la zone morte moteur (Feedforward).
 */
#define MOTOR_DEADBAND 0.0f

/**
 * @struct PID_Handle_t
 * @brief Structure de contrôle regroupant les paramètres et l'état du régulateur PID.
 */
typedef struct {
    float Kp;               /**< Gain proportionnel */
    float Ki;               /**< Gain intégral */
    float Kd;               /**< Gain dérivé */
    float max_integral;     /**< Borne anti-windup pour la somme intégrale */

    float target_speed_ms;  /**< Consigne de vitesse cible en m/s */
    float integral_sum;     /**< Accumulateur de l'erreur pour le terme intégral */
    float prev_error;       /**< Erreur de l'itération précédente pour le terme dérivé */
    uint32_t last_time_ms;  /**< Timestamp de la dernière exécution */

    float last_output_power; /**< Dernière commande de puissance calculée (-100 à 100) */

    Motor_Handle_t *motor;        /**< Pointeur vers le handler du moteur */
    Speedometer_Handle_t *speedo; /**< Pointeur vers le handler du tachymètre */

} PID_Handle_t;

/**
 * @brief Initialise le contrôleur PID et lie les drivers moteur/vitesse.
 * @param hpid Pointeur vers la structure PID.
 * @param hmotor Pointeur vers le handler moteur.
 * @param hspeedo Pointeur vers le handler speedometer.
 */
void pid_init(PID_Handle_t *hpid, Motor_Handle_t *hmotor, Speedometer_Handle_t *hspeedo);

/**
 * @brief Définit la consigne de vitesse cible avec bridage automatique.
 * @param hpid Pointeur vers la structure PID.
 * @param speed_ms Vitesse en m/s (bornée par les constantes de sécurité).
 */
void pid_set_target_speed(PID_Handle_t *hpid, float speed_ms);

/**
 * @brief Exécute la boucle d'asservissement (calcul PID et commande moteur).
 * @note Cette fonction doit être appelée de manière périodique dans la boucle principale ou un timer.
 * @param hpid Pointeur vers la structure PID.
 */
void pid_process(PID_Handle_t *hpid);

#endif
