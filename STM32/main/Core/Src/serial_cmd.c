#include "serial_cmd.h"
#include "serial.h"
#include "driver_motor.h"
#include "driver_servo.h"
#include "driver_ins.h"
#include "app_main.h"
#include <string.h>

ParserSwitch parser_state = PARSER_IDLE;

typedef enum{
    S_HDR=0,
    S_D0,
    S_D1,
    S_CRC
} ParseState;

static ParseState st=S_HDR;
static uint8_t hdr = 0;
static uint8_t d0  = 0;
static uint8_t d1  = 0;
int8_t  shadow_servo_cmd = 0;
int16_t shadow_motor_cmd = 0;

static inline uint16_t to_u16(uint8_t lo,uint8_t hi){return(uint16_t)lo|((uint16_t)hi<<8);}
static inline int16_t to_i16(uint8_t lo,uint8_t hi){return(int16_t)to_u16(lo,hi);}

static int16_t read_reg16(uint8_t addr){
    switch(addr){
        case REG_SERVO_CMD:return shadow_servo_cmd;
        case REG_MOTOR_CMD:return shadow_motor_cmd;
        case REG_BMI:return 0;
        default:return 0;
    }
}

static void handle_frame(uint8_t hdr_b,uint8_t d0_b,uint8_t d1_b){
    const uint8_t  is_read = PROTO_IS_READ(hdr_b)?1u:0u;
    const uint8_t  addr    = PROTO_ADDR(hdr_b);
    const uint16_t udata16 = to_u16(d0_b,d1_b);
    const int16_t  data16  = to_i16(d0_b,d1_b);

    (void)udata16;

    if(is_read){
        uint8_t count=d0_b;
        for(uint8_t i=0; i<count; i++){
            uint8_t a = (uint8_t)((addr+i)&PROTO_HDR_ADDR_MASK);
            int16_t v = read_reg16(a);
            (void)proto_send_data16(a,v);
        }
        return;
    }

    switch(addr){
        case REG_SERVO_CMD:
            parser_state = PARSER_SERVO_CMD;
            shadow_servo_cmd = data16;
        break;

        case REG_MOTOR_CMD:
            parser_state = PARSER_MOTOR_CMD;
            shadow_motor_cmd = data16;
        break;

        case REG_BMI:
            parser_state = PARSER_BMI_CMD;
        break;

        default:
            parser_state = PARSER_OTHERS;
        break;
    }
}

static void parse_byte(uint8_t b){
    switch(st){
        case S_HDR:hdr=b;st=S_D0;break;
        case S_D0:d0=b;st=S_D1;break;
        case S_D1:d1=b;st=S_CRC;break;
        case S_CRC:{
            uint8_t buf[3]={hdr,d0,d1};
            uint8_t crc=serial_crc8_atm(buf,3);
            if(crc==b)handle_frame(hdr,d0,d1);
            st=S_HDR;
        }
        break;
        default:st=S_HDR;break;
    }
}

void serial_cmd_reader(void){
    uint8_t tmp[64];
    size_t n = serial_read(tmp,sizeof(tmp));
    if(!n)return;
    for(size_t i=0;i<n;i++){
        parse_byte(tmp[i]);
    }
}

void serial_send_imu_frame(void) {
    bmi088_data_t imu_data;

    if (BMI088_Read_All(&imu_data) != BMI08_OK) {
        return;
    }

    static SerialImuFrame_t frame;

    frame.head1 = 0xAA;
    frame.head2 = 0x55;
    frame.type  = 0x01;
    frame.len   = 28;
    frame.timestamp = HAL_GetTick();

    frame.accel[0] = imu_data.accel_x_mms2;
    frame.accel[1] = imu_data.accel_y_mms2;
    frame.accel[2] = imu_data.accel_z_mms2;

    frame.gyro[0]  = imu_data.gyro_x_rads;
    frame.gyro[1]  = imu_data.gyro_y_rads;
    frame.gyro[2]  = imu_data.gyro_z_rads;

    uint8_t *raw_bytes = (uint8_t*)&frame;

    frame.crc = serial_crc8_atm(raw_bytes, sizeof(SerialImuFrame_t) - 1);

    serial_write_all_nb(raw_bytes, sizeof(SerialImuFrame_t));
}
