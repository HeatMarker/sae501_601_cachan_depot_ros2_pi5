#!/usr/bin/env python3
"""
Moniteur de vitesse STM32 - Affiche la vitesse encodeur en temps réel.
Usage : python3 stm32_speed_monitor.py
        Arrête le stm32_bridge avant de lancer (port exclusif).
"""
import serial
import struct
import sys

PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF505075894967183959-if02'
BAUD = 115200
FRAME_SIZE = 13  # [AA][55][type][len][ts:4][speed:4][crc]

def crc8_atm(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def main():
    print(f"Connexion sur {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Erreur : {e}")
        sys.exit(1)

    print("Lecture en cours... (Ctrl+C pour arrêter)\n")
    buf = bytearray()
    count = 0

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            buf.extend(data)

            while len(buf) >= FRAME_SIZE:
                # Synchronisation sur l'en-tête 0xAA 0x55
                if buf[0] != 0xAA or buf[1] != 0x55:
                    del buf[0]
                    continue

                frame = bytes(buf[:FRAME_SIZE])

                if crc8_atm(frame[:-1]) != frame[-1]:
                    del buf[0]
                    continue

                # Trame valide
                buf = buf[FRAME_SIZE:]
                _, _, ftype, _, ts, speed = struct.unpack('<BBBBIf', frame[:-1])

                if ftype == 0x02:
                    count += 1
                    bar_len = int(abs(speed) * 20)
                    direction = ">>>" if speed >= 0 else "<<<"
                    bar = direction * min(bar_len, 20)
                    print(f"\r[{count:5d}] {speed:+6.3f} m/s  {speed*1000:+7.1f} mm/s  {bar:<60}", end='', flush=True)

    except KeyboardInterrupt:
        print(f"\n\nArrêt. {count} trames reçues.")
        ser.close()

if __name__ == '__main__':
    main()
