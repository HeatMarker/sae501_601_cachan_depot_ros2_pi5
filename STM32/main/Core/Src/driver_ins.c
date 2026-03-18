#include "stm32g0xx_hal.h"
#include "driver_ins.h"
#include <stdio.h>
#include <string.h>

/* ========== VARIABLES PRIVÉES ========== */

static struct bmi08_dev bmi088_dev;
static SPI_HandleTypeDef *bmi088_hspi;

static bmi088_cs_t cs_accel = {
    .port = BMI088_CS_ACC_GPIO_Port,
    .pin  = BMI088_CS_ACC_Pin
};

static bmi088_cs_t cs_gyro = {
    .port = BMI088_CS_GYRO_GPIO_Port,
    .pin  = BMI088_CS_GYRO_Pin
};

/* ========== FONCTIONS PRIVÉES (CALLBACKS POUR L'API BOSCH) ========== */

/**
 * @brief Fonction de lecture SPI pour l'API Bosch
 */
static int8_t bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
    if (reg_data == NULL || intf_ptr == NULL) return BMI08_E_NULL_PTR;
    if (len == 0 || len > BMI08_MAX_LEN) return BMI08_E_RD_WR_LENGTH_INVALID;

    bmi088_cs_t *cs = (bmi088_cs_t*)intf_ptr;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);

    if(cs->pin == BMI088_CS_ACC_Pin){
        // --- Accéléromètre : adresse + dummy + données ---
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};

        tx_buf[0] = reg_addr | 0x80; // adresse avec bit lecture
        tx_buf[1] = 0x00;            // dummy byte

        // Envoyer adresse + dummy + lire len octets
        status = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, len + 2, HAL_MAX_DELAY);

        /*
        for(uint32_t i = 0; i < len + 2; i++){
            printf("REG RX i : %d : %02X\r\n", i, rx_buf[i]);
        }
        */

        // CS haut
        HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

        if(status != HAL_OK) return BMI08_E_COM_FAIL;

        // Copier uniquement les données utiles (après dummy)
        for(uint32_t i = 0; i < len; i++){
            reg_data[i] = rx_buf[i + 1];
        }
    }
    else{
        // --- Gyroscope : adresse + données (pas de dummy) ---
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};

        tx_buf[0] = reg_addr | 0x80; // adresse avec bit lecture

        status = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);

        // CS haut
        HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

        if(status != HAL_OK) return BMI08_E_COM_FAIL;

        // Copier les données utiles
        for(uint32_t i = 0; i < len; i++){
            reg_data[i] = rx_buf[i + 1];
        }
    }

    return BMI08_OK;
}



/**
 * @brief Fonction d'écriture SPI pour l'API Bosch
 */
static int8_t bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    bmi088_cs_t *cs = (bmi088_cs_t*)intf_ptr;

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);

    // Envoyer l'adresse (bit 7 = 0 pour écriture)
    uint8_t addr = reg_addr & 0x7F;
    HAL_SPI_Transmit(bmi088_hspi, &addr, 1, HAL_MAX_DELAY);

    // Écrire les données
    HAL_SPI_Transmit(bmi088_hspi, (uint8_t*)reg_data, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);

    return BMI08_OK;
}

/**
 * @brief Fonction de délai pour l'API Bosch
 */
static void bmi088_delay_us(uint32_t period, void *intf_ptr){
    // Convertir µs en ms (approximation)
    uint32_t delay_ms = period / 1000;
    if(delay_ms == 0) delay_ms = 1;
    HAL_Delay(delay_ms);
}

/* ========== FONCTIONS PUBLIQUES ========== */

int8_t BMI088_Init(SPI_HandleTypeDef *hspi){
    if(hspi == NULL){
        return BMI08_E_NULL_PTR;
    }

    bmi088_hspi = hspi;

    // Initialiser les CS à l'état haut
    HAL_GPIO_WritePin(BMI088_CS_ACC_GPIO_Port, BMI088_CS_ACC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_Port, BMI088_CS_GYRO_Pin, GPIO_PIN_SET);

    // Configuration de la structure device
    bmi088_dev.intf = BMI08_SPI_INTF;
    bmi088_dev.read = bmi088_spi_read;
    bmi088_dev.write = bmi088_spi_write;
    bmi088_dev.delay_us = bmi088_delay_us;
    bmi088_dev.intf_ptr_accel = &cs_accel;
    bmi088_dev.intf_ptr_gyro = &cs_gyro;

    // Initialisation de l'accéléromètre
    int8_t rslt_accel = bmi08a_init(&bmi088_dev);

#ifdef DEBUG
    //printf("BMI088 Init Accel: %d (0=OK)\r\n", rslt_accel);
    //printf("Accel Chip ID: 0x%02X (attendu: 0x1E)\r\n", bmi088_dev.accel_chip_id);
#endif

    // Initialisation du gyroscope
    int8_t rslt_gyro = bmi08g_init(&bmi088_dev);

#ifdef DEBUG
    //printf("BMI088 Init Gyro: %d (0=OK)\r\n", rslt_gyro);
    //printf("Gyro Chip ID: 0x%02X (attendu: 0x0F)\r\n", bmi088_dev.gyro_chip_id);
#endif

    // Vérifier que les deux sont OK
    if(rslt_accel != BMI08_OK || rslt_gyro != BMI08_OK){
        return BMI08_E_DEV_NOT_FOUND;
    }

    // Configuration de l'accéléromètre
    bmi088_dev.accel_cfg.odr   = BMI08_ACCEL_ODR_100_HZ;
    bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    bmi088_dev.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
    bmi088_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    rslt_accel  = bmi08a_set_power_mode(&bmi088_dev);
    rslt_accel |= bmi08a_set_meas_conf(&bmi088_dev);

    // Configuration du gyroscope
    bmi088_dev.gyro_cfg.odr   = BMI08_GYRO_BW_23_ODR_200_HZ;
    bmi088_dev.gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
    bmi088_dev.gyro_cfg.bw    = BMI08_GYRO_BW_23_ODR_200_HZ;
    bmi088_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    rslt_gyro  = bmi08g_set_power_mode(&bmi088_dev);
    rslt_gyro |= bmi08g_set_meas_conf(&bmi088_dev);

    if(rslt_accel != BMI08_OK || rslt_gyro != BMI08_OK){
        return BMI08_E_COM_FAIL;
    }

#ifdef DEBUG
    //printf("BMI088 initialisé avec succès!\r\n");
#endif

    return BMI08_OK;
}

int8_t BMI088_Read_Accel_Raw(struct bmi08_sensor_data *accel_data){
    if(accel_data == NULL){
        return BMI08_E_NULL_PTR;
    }

    return bmi08a_get_data(accel_data, &bmi088_dev);
}

int8_t BMI088_Read_Gyro_Raw(struct bmi08_sensor_data *gyro_data){
    if(gyro_data == NULL){
        return BMI08_E_NULL_PTR;
    }

    return bmi08g_get_data(gyro_data, &bmi088_dev);
}

int8_t BMI088_Read_All(bmi088_data_t *data){
    if(data == NULL){
        return BMI08_E_NULL_PTR;
    }

    struct bmi08_sensor_data accel_raw, gyro_raw;

    // Lecture des données brutes
    int8_t rslt = BMI088_Read_Accel_Raw(&accel_raw);
    if(rslt != BMI08_OK){
        return rslt;
    }

    rslt = BMI088_Read_Gyro_Raw(&gyro_raw);
    if(rslt != BMI08_OK){
        return rslt;
    }

    // Conversion en unités physiques
    data->accel_x_mms2 = ((float)accel_raw.x / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    data->accel_y_mms2 = ((float)accel_raw.y / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    data->accel_z_mms2 = ((float)accel_raw.z / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;

    data->gyro_x_rads = ((float)gyro_raw.x / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    data->gyro_y_rads = ((float)gyro_raw.y / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    data->gyro_z_rads = ((float)gyro_raw.z / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;

    data->timestamp_ms = HAL_GetTick();

    return BMI08_OK;
}

void BMI088_Convert_Accel(struct bmi08_sensor_data *accel_raw, float *accel_mms2){
    if(accel_raw == NULL || accel_mms2 == NULL){
        return;
    }

    accel_mms2[0] = ((float)accel_raw->x / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    accel_mms2[1] = ((float)accel_raw->y / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
    accel_mms2[2] = ((float)accel_raw->z / ACCEL_RANGE_6G_LSB) * G_TO_MM_S2;
}

void BMI088_Convert_Gyro(struct bmi08_sensor_data *gyro_raw, float *gyro_rads){
    if(gyro_raw == NULL || gyro_rads == NULL){
        return;
    }

    gyro_rads[0] = ((float)gyro_raw->x / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    gyro_rads[1] = ((float)gyro_raw->y / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
    gyro_rads[2] = ((float)gyro_raw->z / GYRO_RANGE_1000DPS_LSB) * DEG_TO_RAD;
}

uint8_t BMI088_Test_Communication(uint8_t print_result){
    uint8_t accel_ok = (bmi088_dev.accel_chip_id == BMI088_ACCEL_CHIP_ID);
    uint8_t gyro_ok = (bmi088_dev.gyro_chip_id == BMI08_GYRO_CHIP_ID);

    if(print_result){

    	/*
    	printf("=== BMI088 Communication Test ===\r\n");
        printf("Accelerometer: %s (ID: 0x%02X)\r\n",
               accel_ok ? "OK" : "FAIL", bmi088_dev.accel_chip_id);
        printf("Gyroscope: %s (ID: 0x%02X)\r\n",
               gyro_ok ? "OK" : "FAIL", bmi088_dev.gyro_chip_id);
        printf("================================\r\n");
    	*/
    }

    return (accel_ok && gyro_ok);
}

int8_t BMI088_Soft_Reset(void){
    uint8_t soft_reset_cmd = BMI08_SOFT_RESET_CMD;
    int8_t rslt = bmi088_spi_write(BMI08_REG_ACCEL_SOFTRESET, &soft_reset_cmd, 1, &cs_accel);

    HAL_Delay(50);

    rslt |= bmi088_spi_write(BMI08_REG_GYRO_SOFTRESET, &soft_reset_cmd, 1, &cs_gyro);

    HAL_Delay(50);

    return rslt;
}
