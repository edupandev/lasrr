#!/usr/bin/env python3
"""
Leitor, Barreira Laser e Detector de Objetos para Meskernel PTFH
Requisitos: pip install pyserial
Uso: python visualizer.py [COM_PORT] [BAUD_RATE]
"""

import sys
import time
import serial
import serial.tools.list_ports

# Comandos Oficiais Meskernel PTFH
CMD_SINGLE_MEASURE = bytes([0xFA, 0x01, 0xFF, 0x04, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00])
CMD_CONTINUOUS     = bytes([0xFA, 0x01, 0xFF, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF])
CMD_STOP           = bytes([0xFA, 0x01, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFE])

def calculate_checksum(data_bytes: bytes) -> int:
    return sum(data_bytes) & 0xFF

def list_available_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[!] Nenhuma porta COM encontrada.")
        return []
    print("\n--- Portas Seriais Disponíveis ---")
    for i, p in enumerate(ports):
        print(f" [{i+1}] {p.device} - {p.description}")
    return ports

def parse_ptfh_packet(raw_frame: bytes):
    if len(raw_frame) < 9:
        return None

    calc_crc = calculate_checksum(raw_frame[:-1])
    recv_crc = raw_frame[-1]

    if calc_crc != recv_crc:
        return None

    valid_indicator = raw_frame[4] | (raw_frame[5] << 8)
    is_valid = (valid_indicator == 1)

    raw_distance = raw_frame[6] | (raw_frame[7] << 8)
    distance_meters = raw_distance / 10.0  # decímetros para metros

    return {
        "valid": is_valid,
        "distance_m": distance_meters,
        "raw_dm": raw_distance
    }

def main():
    print("=" * 65)
    print("  BARREIRA LASER & DETECTOR DE OBJETOS - MESKERNEL PTFH")
    print("=" * 65)

    port_name = None
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        ports = list_available_ports()
        if not ports:
            sys.exit(1)
        choice = input("\nEscolha a porta serial [ou digite ex: COM3]: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            port_name = ports[int(choice) - 1].device
        else:
            port_name = choice

    baud_rate = 115200

    # Configuração de Limiar
    print("\n--- Configuração da Barreira Óptica ---")
    thresh_input = input("Digite a distância de corte em metros [Padrão: 1.00m]: ").strip()
    try:
        alarm_threshold = float(thresh_input) if thresh_input else 1.00
    except ValueError:
        alarm_threshold = 1.00

    print(f"[*] Limiar definido: < {alarm_threshold:.2f} m para disparar alarme.")
    print(f"[*] Conectando a {port_name}...")

    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except Exception as e:
        print(f"[!] Erro ao abrir a porta: {e}")
        sys.exit(1)

    # Inicia medição contínua
    ser.write(CMD_CONTINUOUS)
    print("[+] Conectado! Sensor em medição contínua.")
    print("[*] Pressione Ctrl+C para encerrar.\n")

    rx_buffer = bytearray()
    object_count = 0
    is_object_present = False
    start_time = 0
    min_dist_in_passage = 999.0

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                rx_buffer.extend(chunk)

                while len(rx_buffer) >= 2:
                    if rx_buffer[0] == 0xFB and rx_buffer[1] == 0x03:
                        if len(rx_buffer) < 4:
                            break
                        payload_len = rx_buffer[3]
                        total_expected_len = 4 + payload_len + 1

                        if len(rx_buffer) < total_expected_len:
                            break

                        packet = bytes(rx_buffer[:total_expected_len])
                        rx_buffer = rx_buffer[total_expected_len:]

                        parsed = parse_ptfh_packet(packet)
                        if parsed and parsed["valid"]:
                            dist = parsed["distance_m"]

                            # Lógica da Barreira / Alarme
                            breached = (dist < alarm_threshold)

                            if breached:
                                if not is_object_present:
                                    is_object_present = True
                                    start_time = time.time()
                                    min_dist_in_passage = dist
                                    object_count += 1
                                    # Emite sinal sonoro de terminal (Bell)
                                    print(f"\a\n🚨 [ALARME] OBJETO #{object_count} PASSANDO! Dist: {dist:.2f}m")
                                else:
                                    if dist < min_dist_in_passage:
                                        min_dist_in_passage = dist
                            else:
                                if is_object_present:
                                    duration_ms = int((time.time() - start_time) * 1000)
                                    is_object_present = False
                                    print(f"✓ [LIVRE] Objeto #{object_count} saiu. Duração: {duration_ms}ms | Dist Mín: {min_dist_in_passage:.2f}m\n")

                            if not is_object_present:
                                sys.stdout.write(f"\rMonitor: Distância: {dist:5.2f} m | Limiar: < {alarm_threshold:.2f} m | Objetos: {object_count}")
                                sys.stdout.flush()
                    else:
                        rx_buffer.pop(0)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[*] Finalizando e parando sensor...")
        ser.write(CMD_STOP)
        ser.close()
        print(f"[*] Total de objetos detectados: {object_count}")

if __name__ == "__main__":
    main()
