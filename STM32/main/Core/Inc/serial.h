#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---------- DMA / UART binding ---------- */
#define SERIAL_RX_RING_SIZE   1024u
#define SERIAL_RX_CHUNK_SIZE  256u
#define SERIAL_TX_CHUNK_MAX   255u
#define SERIAL_UART           huart2

/* ---------- Protocole 4B : [HDR | D0 | D1 | CRC8] ---------- */
/* HDR: bit7 = R(1)/W(0), bits6..0 = adresse registre (0..127) */
#define PROTO_HDR_RW_MASK     0x80u
#define PROTO_HDR_ADDR_MASK   0x7Fu
#define PROTO_MAKE_HDR(rw, addr)   ( (uint8_t)(((rw) ? 0x80u : 0x00u) | ((addr) & 0x7Fu)) )
#define PROTO_IS_READ(hdr)         ( ((hdr) & 0x80u) != 0 )
#define PROTO_ADDR(hdr)            ( (hdr) & 0x7Fu )

/* ---------- Link layer ---------- */
void     serial_init(void);
int      serial_write_nb(const uint8_t *data, uint16_t len);
int      serial_write_all_nb(const uint8_t *data, uint16_t len);
int      serial_write(const uint8_t *data, uint16_t len);
size_t   serial_available(void);
size_t   serial_read(uint8_t *dst, size_t max_len);
size_t   serial_read_until(uint8_t *dst, size_t max_len, uint8_t delim);

/* ---------- CRC8/ATM (poly 0x07, init 0x00) ---------- */
uint8_t  serial_crc8_atm(const uint8_t *data, uint16_t len);

/* ---------- Helpers protocole ---------- */
/* WRITE/DATA: envoie une trame [HDR(b7=0,addr) | value(16) | CRC8] */
int      proto_send_write16(uint8_t addr, int16_t value);
/* READ request: [HDR(b7=1,addr) | count | flags | CRC8] */
int      proto_send_read_burst(uint8_t addr, uint8_t count, uint8_t flags);
/* Alias DATA (télémétrie) = WRITE côté STM32 */
static inline int proto_send_data16(uint8_t addr, int16_t value){
    return proto_send_write16(addr, value);
}

#endif
