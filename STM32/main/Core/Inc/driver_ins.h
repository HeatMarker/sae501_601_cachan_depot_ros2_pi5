#ifndef BMI088_DRIVER_H
#define BMI088_DRIVER_H

#include "stm32g0xx_hal.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"

/* ========== CONFIGURATION ========== */

// Pins des Chip Select pour le BMI088
//	ACCEL --> PA4
//  GYRO  --> PA1
#define BMI088_CS_ACC_GPIO_Port     GPIOB
#define BMI088_CS_ACC_Pin           GPIO_PIN_13
#define BMI088_CS_GYRO_GPIO_Port    GPIOB
#define BMI088_CS_GYRO_Pin          GPIO_PIN_14

// Constantes de conversion
#define ACCEL_RANGE_3G_LSB 			10922.67f
#define ACCEL_RANGE_6G_LSB          5461.33f    // LSB par g pour ±6g
#define ACCEL_RANGE_12G_LSB 		2730.67f
#define ACCEL_RANGE_24G_LSB 		1365.33f
#define G_TO_MM_S2                  9806.65f    // 1g = 9806.65 mm/s²

#define GYRO_RANGE_125DPS_LSB 		262.4f
#define GYRO_RANGE_250DPS_LSB 		131.2f
#define GYRO_RANGE_500DPS_LSB 		65.6f
#define GYRO_RANGE_1000DPS_LSB      32.768f     // LSB par °/s pour ±1000°/s
#define GYRO_RANGE_2000DPS_LSB 		16.4f
#define DEG_TO_RAD                  0.017453292519943295f  // π/180

/* ========== STRUCTURES ========== */

/**
 * @brief Structure pour gérer les pins CS (Chip Select)
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} bmi088_cs_t;

/**
 * @brief Structure contenant les données converties du BMI088
 */
typedef struct {
    float accel_x_mms2;     // Accélération X en mm/s²
    float accel_y_mms2;     // Accélération Y en mm/s²
    float accel_z_mms2;     // Accélération Z en mm/s²

    float gyro_x_rads;      // Vitesse angulaire X en rad/s
    float gyro_y_rads;      // Vitesse angulaire Y en rad/s
    float gyro_z_rads;      // Vitesse angulaire Z en rad/s

    uint32_t timestamp_ms;  // Timestamp de la mesure
} bmi088_data_t;

/* ========== FONCTIONS PUBLIQUES ========== */

/**
 * @brief Initialise le BMI088 (accéléromètre et gyroscope)
 * @param hspi Pointeur vers le handle SPI à utiliser
 * @return BMI08_OK si succès, code d'erreur sinon
 */
int8_t BMI088_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief Lit les données brutes de l'accéléromètre
 * @param accel_data Structure pour stocker les données brutes
 * @return BMI08_OK si succès, code d'erreur sinon
 */
int8_t BMI088_Read_Accel_Raw(struct bmi08_sensor_data *accel_data);

/**
 * @brief Lit les données brutes du gyroscope
 * @param gyro_data Structure pour stocker les données brutes
 * @return BMI08_OK si succès, code d'erreur sinon
 */
int8_t BMI088_Read_Gyro_Raw(struct bmi08_sensor_data *gyro_data);

/**
 * @brief Lit et convertit toutes les données du BMI088
 * @param data Structure pour stocker les données converties
 * @return BMI08_OK si succès, code d'erreur sinon
 */
int8_t BMI088_Read_All(bmi088_data_t *data);

/**
 * @brief Convertit les données brutes de l'accéléromètre en mm/s²
 * @param accel_raw Données brutes du capteur
 * @param accel_mms2 Tableau de sortie [x, y, z] en mm/s²
 */
void BMI088_Convert_Accel(struct bmi08_sensor_data *accel_raw, float *accel_mms2);

/**
 * @brief Convertit les données brutes du gyroscope en rad/s
 * @param gyro_raw Données brutes du capteur
 * @param gyro_rads Tableau de sortie [x, y, z] en rad/s
 */
void BMI088_Convert_Gyro(struct bmi08_sensor_data *gyro_raw, float *gyro_rads);

/**
 * @brief Vérifie la communication avec le BMI088
 * @param print_result Si 1, affiche les résultats via printf
 * @return 1 si les deux capteurs répondent correctement, 0 sinon
 */
uint8_t BMI088_Test_Communication(uint8_t print_result);

/**
 * @brief Effectue un soft reset du BMI088
 * @return BMI08_OK si succès, code d'erreur sinon
 */
int8_t BMI088_Soft_Reset(void);

#endif /* BMI088_DRIVER_H */
