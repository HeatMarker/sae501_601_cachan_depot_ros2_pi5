#include "serial_cmd.h"
#include "serial.h"
#include "driver_motor.h"
#include "driver_servo.h"
#include "driver_ins.h"
#include "app_main.h"
#include "driver_pid_motor.h"
#include <string.h>

ParserSwitch parser_state = PARSER_IDLE;


extern PID_Handle_t hpid;
extern float speed_speedo_data;

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
        case REG_SERVO_CMD: return shadow_servo_cmd;
        case REG_MOTOR_CMD: return shadow_motor_cmd;
        case REG_BMI:       return 0;

        // --- NOUVEAU : Lecture des gains PID ---
        // On renvoie l'entier scalé (ex: 1.5 -> 150)
        case REG_PID_KP:    return (int16_t)(hpid.Kp * PID_SCALER);
        case REG_PID_KI:    return (int16_t)(hpid.Ki * PID_SCALER);
        case REG_PID_KD:    return (int16_t)(hpid.Kd * PID_SCALER);

        default: return 0;
    }
}

static void handle_frame(uint8_t hdr_b, uint8_t d0_b, uint8_t d1_b){
    const uint8_t  is_read = PROTO_IS_READ(hdr_b) ? 1u : 0u;
    const uint8_t  addr    = PROTO_ADDR(hdr_b);
    const uint16_t udata16 = to_u16(d0_b, d1_b);
    const int16_t  data16  = to_i16(d0_b, d1_b);

    (void)udata16;

    if(is_read){
        uint8_t count = d0_b; // Nombre de registres à lire
        for(uint8_t i=0; i<count; i++){
            uint8_t a = (uint8_t)((addr+i) & PROTO_HDR_ADDR_MASK);
            int16_t v = read_reg16(a);
            (void)proto_send_data16(a, v);
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

        // --- NOUVEAU : Modification des gains PID ---
        case REG_PID_KP:
            // On divise par le SCALER pour retrouver le float (150 / 100.0 = 1.5)
            hpid.Kp = (float)data16 / PID_SCALER;
            // Optionnel : Reset de l'intégrale quand on change les gains pour éviter les à-coups
            hpid.integral_sum = 0.0f;
            break;

        case REG_PID_KI:
            hpid.Ki = (float)data16 / PID_SCALER;
            hpid.integral_sum = 0.0f;
            break;

        case REG_PID_KD:
            hpid.Kd = (float)data16 / PID_SCALER;
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

///*
void serial_send_data_frame(void) {
    // On utilise la nouvelle structure allégée
    static SerialSpeedFrame_t frame;

    frame.head1 = 0xAA;
    frame.head2 = 0x55;
    frame.type  = 0x02;

    // IMPORTANT : La longueur des données change !
    // On envoie 1 seul float (vitesse), donc 4 octets.
    frame.len   = 4;

    frame.timestamp = HAL_GetTick();

    // On affecte uniquement la vitesse réelle filtrée
    frame.speed = speed_speedo_data;

    // Calcul du CRC
    // La taille a changé, mais sizeof() s'adapte tout seul
    uint8_t *raw_bytes = (uint8_t*)&frame;
    frame.crc = serial_crc8_atm(raw_bytes, sizeof(SerialSpeedFrame_t) - 1);

    // Envoi
    serial_write_all_nb(raw_bytes, sizeof(SerialSpeedFrame_t));
}
//*/
/*
void serial_send_data_frame(void) {
    // On utilise la structure SerialPIDFrame_t définie dans serial_cmd.h
    static SerialPIDFrame_t frame;

    frame.head1 = 0xAA;
    frame.head2 = 0x55;
    frame.type  = 0x02; // Type 0x02 pour la trame PID
    frame.len   = 12;   // 3 floats * 4 octets = 12 octets de données
    frame.timestamp = HAL_GetTick();

    // 1. Récupération de la CONSIGNE
    frame.target = hpid.target_speed_ms;

    // 2. Récupération du RÉEL
    // CORRECTION : On affecte directement la valeur.
    // Le driver speedometer renvoie déjà du négatif (-X m/s) quand on recule.
    // L'ancienne inversion manuelle ici créait un double négatif (donc du positif).
    frame.actual = speed_speedo_data;

    // 3. Calcul de l'ERREUR
    frame.error = frame.target - frame.actual;

    // 4. Calcul du CRC
    uint8_t *raw_bytes = (uint8_t*)&frame;
    // On calcule le CRC sur toute la trame SAUF le dernier octet
    frame.crc = serial_crc8_atm(raw_bytes, sizeof(SerialPIDFrame_t) - 1);

    // 5. Envoi
    serial_write_all_nb(raw_bytes, sizeof(SerialPIDFrame_t));
}
*/
