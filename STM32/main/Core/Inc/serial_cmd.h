#ifndef INC_SERIAL_CMD_H_
#define INC_SERIAL_CMD_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define REG_SERVO_CMD 0x00
#define REG_MOTOR_CMD 0x01
#define REG_BMI       0x02
#define REG_PID_KP    0x03
#define REG_PID_KI    0x04
#define REG_PID_KD    0x05
#define PID_SCALER    100.0f
/*
typedef struct __attribute__((packed)) {
    uint8_t head1;      // 0xAA
    uint8_t head2;      // 0x55
    uint8_t type;       // 0x02
    uint8_t len;        // 12
    uint32_t timestamp; // 4 octets
    float target;       // 4 octets
    float actual;       // 4 octets
    float error;        // 4 octets
    uint8_t crc;        // 1 octet
} SerialPIDFrame_t;
*/
///*
typedef struct __attribute__((packed)) {
    uint8_t head1;      // 0xAA
    uint8_t head2;      // 0x55
    uint8_t type;       // 0x02
    uint8_t len;        // 4 (C'était 12 avant : 1 seul float = 4 octets)
    uint32_t timestamp; // 4 octets
    float speed;        // 4 octets (La vitesse réelle uniquement)
    uint8_t crc;        // 1 octet
} SerialSpeedFrame_t;
//*/
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
void serial_send_data_frame(void);

#endif
