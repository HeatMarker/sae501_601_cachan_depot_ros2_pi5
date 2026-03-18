/**
 * @file driver_motor.c
 * @brief Driver Moteur Non-Bloquant (State Machine) - Version Objet
 */

#include "driver_motor.h"
#include "tim.h"

// 64Mhz - PSC=19 - ARR=63999 soit PWM{50Hz, duty=50%}

/** @brief Durée de freinage en ms lors d'un changement de sens. */
#define T_BRAKE_MS          120
/** @brief Temps de pause au neutre en ms après le freinage. */
#define T_NEUTRAL_GAP_MS    120
/** @brief Valeur de pourcentage PWM pour le neutre (arrêt). */
#define PWM_NEUTRAL         50u
/** @brief Valeur de pourcentage PWM pour le freinage en marche arrière. */
#define PWM_BRAKE_REV       40u
/** @brief Valeur de pourcentage PWM pour le freinage en marche avant. */
#define PWM_BRAKE_FWD       60u

static uint8_t motor_speed_mms_to_pwm_percent(Motor_Handle_t *hmotor, int16_t value);

/**
 * @brief  Applique la valeur brute au registre de comparaison du timer.
 * @note   Fonction interne statique.
 * @param  hmotor Pointeur vers la structure de l'objet moteur.
 * @param  value  Valeur en ticks à écrire dans le registre CCR.
 */
static inline void pwm_pulse(Motor_Handle_t *hmotor, uint16_t value){
    if (hmotor && hmotor->htim) {
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, value);
    }
}

/**
 * @brief  Convertit un pourcentage (0-100%) en ticks timer.
 * @note   Utilise les bornes min/max définies dans l'objet moteur.
 * @param  hmotor  Pointeur vers l'objet moteur.
 * @param  percent Pourcentage cible.
 * @retval Valeur en ticks calculée.
 */
static inline uint16_t motor_map_percent(Motor_Handle_t *hmotor, int16_t percent){
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    const uint16_t min = hmotor->min_pulse_ticks;
    const uint16_t max = hmotor->max_pulse_ticks;

    return (uint16_t)(min + ((uint32_t)(max - min) * (uint32_t)percent) / 100u);
}

/**
 * @brief  Convertit une vitesse en mm/s vers un pourcentage PWM.
 * @note   Gère l'asymétrie des vitesses max avant/arrière définies dans l'objet.
 * @param  hmotor Pointeur vers l'objet moteur.
 * @param  value  Vitesse en mm/s.
 * @retval Pourcentage PWM (0-100) correspondant.
 */
static uint8_t motor_speed_mms_to_pwm_percent(Motor_Handle_t *hmotor, int16_t value){
    if (value >= hmotor->max_speed_pos_mms) return 100;
    if (value <= hmotor->max_speed_neg_mms) return 0;

    if (value >= 0)
        return (uint8_t)(50 + ((int32_t)value * 50) / hmotor->max_speed_pos_mms);
    else
        return (uint8_t)(50 + ((int32_t)value * 50) / (-hmotor->max_speed_neg_mms));
}

/**
 * @brief  Vérifie si une échéance temporelle est atteinte.
 * @note   Gère le débordement (overflow) du compteur uint32_t.
 * @param  now      Temps actuel en ms.
 * @param  deadline Temps cible en ms.
 * @retval true si le temps est atteint ou dépassé, false sinon.
 */
static inline bool time_reached(uint32_t now, uint32_t deadline){
    return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief  Initialise le driver moteur.
 * @note   Met le moteur au neutre, initialise la machine à états et démarre le PWM.
 * @param  hmotor Pointeur vers la structure de configuration du moteur.
 */
void motor_init(Motor_Handle_t *hmotor){
    if(hmotor && hmotor->htim){
        hmotor->state = MOTOR_STATE_NEUTRAL;
        hmotor->go_forward = true;
        hmotor->ctx.target_speed_mms = 0;
        hmotor->ctx.target_pwm = PWM_NEUTRAL;
        hmotor->ctx.target_forward = true;
        hmotor->ctx.deadline_ms = 0;

        HAL_TIM_PWM_Start(hmotor->htim, hmotor->channel);
    }
}

/**
 * @brief  Commande directe du moteur en pourcentage PWM.
 * @note   Contourne la logique de vitesse mm/s (utilisé pour forcer le neutre ou le frein).
 * @param  hmotor  Pointeur vers l'objet moteur.
 * @param  percent Pourcentage cible (0-100).
 */
void motor_pwm_percent(Motor_Handle_t *hmotor, uint8_t percent){
    uint16_t value = motor_map_percent(hmotor, (int16_t)percent);
    pwm_pulse(hmotor, value);
}

/**
 * @brief  Définit la consigne de vitesse en mm/s.
 * @note   Cette fonction est non-bloquante. Elle met à jour le contexte cible.
 * La machine à états appliquera les transitions nécessaires (freinage, etc.).
 * @param  hmotor    Pointeur vers l'objet moteur.
 * @param  speed_mms Vitesse cible en mm/s (positif = avant, négatif = arrière).
 */
void motor_set_speed_mms(Motor_Handle_t *hmotor, int16_t speed_mms){
    hmotor->ctx.target_speed_mms = speed_mms;

    if(speed_mms == 0){
        hmotor->ctx.target_pwm = PWM_NEUTRAL;
    }
    else{
        hmotor->ctx.target_pwm = motor_speed_mms_to_pwm_percent(hmotor, speed_mms);
        hmotor->ctx.target_forward = (speed_mms > 0);
    }
}

/**
 * @brief  Machine à états principale du moteur.
 * @note   DOIT être appelée cycliquement (ex: toutes les 1ms) pour gérer les temporisations
 * de sécurité (freinage, pause neutre) lors des changements de sens.
 * @param  hmotor Pointeur vers l'objet moteur.
 * @param  now_ms Temps système actuel en millisecondes (ex: HAL_GetTick()).
 */
void motor_process_1ms(Motor_Handle_t *hmotor, uint32_t now_ms){
    if (!hmotor) return;

    const bool want_neutral = (hmotor->ctx.target_speed_mms == 0);

    switch (hmotor->state){
        case MOTOR_STATE_NEUTRAL:
            motor_pwm_percent(hmotor, PWM_NEUTRAL);

            if(!want_neutral){
                if(hmotor->ctx.target_forward){
                    motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                    hmotor->go_forward = true;
                    hmotor->state = MOTOR_STATE_FORWARD_HOLD;
                }
                else{
                    motor_pwm_percent(hmotor, PWM_BRAKE_REV);
                    hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                    hmotor->state = MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP;
                }
            }
            break;

        case MOTOR_STATE_NEUTRAL_TO_REVERSE_TAP:
            if (!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP;
            break;

        case MOTOR_STATE_NEUTRAL_TO_REVERSE_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if (hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            break;

        case MOTOR_STATE_FORWARD_HOLD:
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
                break;
            }
            if(hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
            }
            else{
                motor_pwm_percent(hmotor, PWM_BRAKE_REV);
                hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                hmotor->state = MOTOR_STATE_FWD_BRAKE_TAP;
            }
            break;

        case MOTOR_STATE_FWD_BRAKE_TAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_FWD_NEUTRAL_GAP;
            break;

        case MOTOR_STATE_FWD_NEUTRAL_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if(hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            break;

        case MOTOR_STATE_REVERSE_HOLD:
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
                break;
            }
            if(!hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
            }
            else{
                motor_pwm_percent(hmotor, PWM_BRAKE_FWD);
                hmotor->ctx.deadline_ms = now_ms + T_BRAKE_MS;
                hmotor->state = MOTOR_STATE_REV_BRAKE_TAP;
            }
            break;

        case MOTOR_STATE_REV_BRAKE_TAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            motor_pwm_percent(hmotor, PWM_NEUTRAL);
            hmotor->ctx.deadline_ms = now_ms + T_NEUTRAL_GAP_MS;
            hmotor->state = MOTOR_STATE_REV_NEUTRAL_GAP;
            break;

        case MOTOR_STATE_REV_NEUTRAL_GAP:
            if(!time_reached(now_ms, hmotor->ctx.deadline_ms)) break;
            if(want_neutral){
                motor_pwm_percent(hmotor, PWM_NEUTRAL);
                hmotor->state = MOTOR_STATE_NEUTRAL;
            }
            else if(!hmotor->ctx.target_forward){
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = false;
                hmotor->state = MOTOR_STATE_REVERSE_HOLD;
            }
            else{
                motor_pwm_percent(hmotor, hmotor->ctx.target_pwm);
                hmotor->go_forward = true;
                hmotor->state = MOTOR_STATE_FORWARD_HOLD;
            }
            break;
    }
}

/**
 * @brief  Commande l'effort du moteur en pourcentage (-100 à +100).
 * @note   Remplace motor_set_speed_mms. Respecte la machine à états.
 * @param  hmotor Pointeur moteur
 * @param  power  Puissance : -100 (Arrière Max) à +100 (Avant Max).
 */
void motor_set_power(Motor_Handle_t *hmotor, int16_t power) {
    if (!hmotor) return;

    // 1. Saturation de sécurité (-100 à +100)
    if (power > 100) power = 100;
    if (power < -100) power = -100;

    // 2. Gestion du Neutre (Arrêt)
    if (power == 0) {
        // La machine à états surveille "target_speed_mms == 0" pour savoir si on veut s'arrêter
        hmotor->ctx.target_speed_mms = 0;
        hmotor->ctx.target_pwm = 50;      // 50% = PWM Neutre standard
    }
    // 3. Gestion du Mouvement
    else {
        // On trompe la machine à états : on lui dit "on roule" (vitesse != 0)
        // pour qu'elle accepte de quitter l'état NEUTRAL.
        hmotor->ctx.target_speed_mms = (power > 0) ? 1 : -1;

        // Calcul du PWM (Mapping : -100..100 -> 0..100 sur une base de 50)
        // 0   -> 50
        // 100 -> 100
        // -100 -> 0
        hmotor->ctx.target_pwm = (uint8_t)(50 + (power / 2));

        // Indication du sens pour que la machine à états gère le freinage si on inverse
        hmotor->ctx.target_forward = (power > 0);
    }
}
