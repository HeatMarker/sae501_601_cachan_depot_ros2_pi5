/**
 * @file driver_servo.c
 * @brief Driver PWM pour servo (fonctions internes + publiques)
 */

#include "tim.h"
#include "driver_servo.h"

#define SERVO_OFFSET_PERCENT  5
#define SERVO_CLAMP_MIN       -20
#define SERVO_CLAMP_MAX       20

static inline void pwm_pulse(Servo_Handle_t *hservo, uint16_t value);

/**
 * @brief  Modifie le rapport cyclique (Duty Cycle) du signal PWM.
 * @note   Utilise la macro bas niveau __HAL_TIM_SET_COMPARE.
 * @param  hservo Pointeur vers la structure de l'objet servo.
 * @param  value  Valeur de comparaison (nombre de ticks).
 */
static inline void pwm_pulse(Servo_Handle_t *hservo, uint16_t value){
    if (hservo && hservo->htim) {
        __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, value);
    }
}

/**
 * @brief  Fonction utilitaire de mappage (produit en croix) en entiers.
 * @note   Transforme x de la plage [in_min, in_max] vers [out_min, out_max].
 * Utilise int32_t pour éviter l'overflow pendant la multiplication interm.
 */
static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Convertit un pourcentage en valeur de registre (ticks) pour le timer.
 * @note   Applique un offset (SERVO_OFFSET_PERCENT) et borne le résultat entre 0 et 100.
 * Utilise les bornes min/max définies dans la structure hservo.
 * @param  hservo  Pointeur vers la structure de l'objet servo.
 * @param  percent Pourcentage d'entrée (signé pour gérer l'offset).
 * @retval Valeur calculée pour le registre CCR du timer.
 */
static inline uint16_t servo_map_percent(Servo_Handle_t *hservo, int16_t percent){
    int16_t corrected = percent + SERVO_OFFSET_PERCENT;

    if (corrected < 0)   corrected = 0;
    if (corrected > 100) corrected = 100;

    uint16_t min = hservo->min_pulse_ticks;
    uint16_t max = hservo->max_pulse_ticks;
    return min + ((max - min) * corrected) / 100;
}

/**
 * @brief  Commande le servo moteur en utilisant un pourcentage.
 * @param  hservo  Pointeur vers la structure de l'objet servo.
 * @param  percent Position cible en pourcentage (0 à 100).
 */
void servo_pwm_percent(Servo_Handle_t *hservo, uint8_t percent){
    uint16_t value = servo_map_percent(hservo, percent);
    pwm_pulse(hservo, value);
}

/**
 * @brief  Commande le servo moteur en utilisant un angle en degrés.
 * @note   L'angle est borné (clamped) par les constantes SERVO_CLAMP_MIN et SERVO_CLAMP_MAX.
 * Effectue une conversion de l'angle (-20°..20°) vers un pourcentage.
 * @param  hservo Pointeur vers la structure de l'objet servo.
 * @param  angle  Angle cible en degrés (signé sur 8 bits).
 */
void servo_pwm_angle_degree(Servo_Handle_t *hservo, int8_t angle){
    if (angle < SERVO_CLAMP_MIN) angle = SERVO_CLAMP_MIN;
    if (angle > SERVO_CLAMP_MAX) angle = SERVO_CLAMP_MAX;

    int16_t percent = ((angle + 35) * 100) / 70;
    uint16_t value = servo_map_percent(hservo, percent);

    pwm_pulse(hservo, value);
}

/**
 * @brief  Commande absolue haute précision sans float.
 * @note   Intègre maintenant l'offset défini par SERVO_OFFSET_PERCENT pour être
 * compatible avec le comportement des autres fonctions.
 * @param  hservo    Pointeur vers la structure de l'objet servo.
 * @param  abs_value Valeur 0 à 65535.
 */
void servo_pwm_angle_abs_value(Servo_Handle_t *hservo, uint16_t abs_value){
    int32_t angle_centi = map(abs_value, 0, 65535, -4500, 4500);

    if (angle_centi < -2000) angle_centi = -2000;
    if (angle_centi >  2000) angle_centi =  2000;

    int32_t pwm_ticks = map(angle_centi, -3500, 3500, hservo->min_pulse_ticks, hservo->max_pulse_ticks);

    int32_t range = hservo->max_pulse_ticks - hservo->min_pulse_ticks;
    int32_t offset_ticks = (range * SERVO_OFFSET_PERCENT) / 100;

    pwm_ticks += offset_ticks;

    // Sécurité bornes hardware
    if (pwm_ticks > hservo->max_pulse_ticks) pwm_ticks = hservo->max_pulse_ticks;
    if (pwm_ticks < hservo->min_pulse_ticks) pwm_ticks = hservo->min_pulse_ticks;

    pwm_pulse(hservo, (uint16_t)pwm_ticks);
}

/**
 * @brief  Initialise le driver du servo.
 * @note   Positionne le servo à l'angle 0° (neutre) et démarre le timer PWM.
 */
void servo_initialisation(Servo_Handle_t *hservo){
    if(hservo && hservo->htim){
        servo_pwm_angle_degree(hservo, 0);
        HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
    }
}


