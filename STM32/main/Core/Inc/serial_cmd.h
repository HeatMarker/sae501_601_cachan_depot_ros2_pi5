#ifndef INC_SERIAL_CMD_H_
#define INC_SERIAL_CMD_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define REG_SERVO_CMD 0x00
#define REG_MOTOR_CMD 0x01
#define REG_BMI       0x02

typedef struct __attribute__((packed)) {
    uint8_t head1;      // 0xAA
    uint8_t head2;      // 0x55
    uint8_t type;       // 0x01
    uint8_t len;        // 28 (Taille payload)
    uint32_t timestamp; // HAL_GetTick()
    float accel[3];     // 3 floats (x, y, z)
    float gyro[3];      // 3 floats (x, y, z)
    uint8_t crc;        // Checksum
} SerialImuFrame_t;

typedef enum{
    PARSER_IDLE,
    PARSER_SERVO_CMD,
    PARSER_MOTOR_CMD,
    PARSER_BMI_CMD,
    PARSER_OTHERS
} ParserSwitch;

extern ParserSwitch parser_state;
extern int8_t shadow_servo_cmd;
extern int16_t shadow_motor_cmd;

void serial_cmd_reader(void);
void serial_send_imu_frame(void);

#endif
