/*
 * Velocímetro Laser de Barreira Dupla com 2 Sensores Meskernel PTFH
 * Plataforma recomendada: ESP32 (utiliza 2 UARTs por Hardware + Timer em microssegundos)
 * 
 * Ligações Elétricas (3.3V TTL):
 *   SENSOR A (Feixe 1 - Entrada):
 *     - VCC  --> 3.3V
 *     - GND  --> GND
 *     - TX   --> ESP32 GPIO 18 (RX1)
 *     - RX   --> ESP32 GPIO 19 (TX1)
 * 
 *   SENSOR B (Feixe 2 - Saída):
 *     - VCC  --> 3.3V
 *     - GND  --> GND
 *     - TX   --> ESP32 GPIO 16 (RX2)
 *     - RX   --> ESP32 GPIO 17 (TX2)
 * 
 *   SAÍDA DE ALARME / RELÉ (Opcional):
 *     - ESP32 GPIO 2 (LED interno ou Módulo Relé/Buzzer)
 * 
 * Fórmula: v = Δs / Δt (SEM distância mínima entre os sensores)
 */

#include <Arduino.h>

#define SENSOR_A_RX 18
#define SENSOR_A_TX 19
#define SENSOR_B_RX 16
#define SENSOR_B_TX 17
#define ALARM_PIN 2

HardwareSerial SerialA(1);
HardwareSerial SerialB(2);

// --- PARÂMETROS CONFIGURÁVEIS DA ARMADILHA ÓPTICA ---
// Distância física entre o Sensor A e o Sensor B (SEM limite mínimo restritivo):
float trapDistanceMeters = 0.50;     // Ex: 0.10m, 0.20m, 0.50m, 1.00m, etc.

// Calibração de Fundo para cada sensor (Tolerância padrão 0.50m)
float bgDistA = 4.00, tolA = 0.50;
float bgDistB = 4.00, tolB = 0.50;
float lastDistA = 0.0, lastDistB = 0.0;

float maxSpeedLimitKmH = 60.0;       // Limite para acionar o alarme de excesso (km/h)
const unsigned long timeoutMicros = 5000000; // 5 segundos de timeout

const uint8_t CMD_CONTINUOUS[] = { 0xFA, 0x01, 0xFF, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF };

// Máquina de estados do velocímetro
enum TrapState { IDLE, WAITING_B, WAITING_A };
TrapState trapState = IDLE;

unsigned long triggerMicrosA = 0;
unsigned long triggerMicrosB = 0;
bool isBlockedA = false;
bool isBlockedB = false;
uint8_t countBlockedA = 0;
uint8_t countBlockedB = 0;
unsigned long pendingMicrosA = 0;
unsigned long pendingMicrosB = 0;
unsigned long passCounter = 0;

// Buffers de recepção serial
#define BUF_SIZE 64
uint8_t bufA[BUF_SIZE], bufB[BUF_SIZE];
uint8_t idxA = 0, idxB = 0;

uint8_t calcCRC(const uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

void finalizeSpeed(unsigned long deltaMicros, const char *direction) {
  float deltaSeconds = (float)deltaMicros / 1000000.0f;
  if (deltaSeconds <= 0.000001f) return;

  float speed_m_s = trapDistanceMeters / deltaSeconds;
  float speed_km_h = speed_m_s * 3.6f;
  float deltaMs = (float)deltaMicros / 1000.0f;

  passCounter++;
  bool overspeed = (speed_km_h > maxSpeedLimitKmH);

  if (overspeed) {
    digitalWrite(ALARM_PIN, HIGH);
    delay(100);
    digitalWrite(ALARM_PIN, LOW);
  }

  Serial.println();
  Serial.println("==================================================");
  Serial.printf("⚡ PASSAGEM #%lu DETECTADA!\n", passCounter);
  Serial.printf("   Sentido: %s\n", direction);
  Serial.printf("   Distância entre sensores (Δs): %.3f m\n", trapDistanceMeters);
  Serial.printf("   Tempo (Δt): %.2f ms (%.6f s)\n", deltaMs, deltaSeconds);
  Serial.printf("   VELOCIDADE: %.1f km/h  (%.2f m/s)\n", speed_km_h, speed_m_s);
  if (overspeed) {
    Serial.println("   ⚠ ALERTA: EXCESSO DE VELOCIDADE!");
  } else {
    Serial.println("   ✓ Velocidade dentro do limite.");
  }
  Serial.println("==================================================");
}

unsigned long lastTriggerMicros = 0;

void onSensorTriggered(char sensor, unsigned long nowMicros) {
  // Cooldown de 350ms para evitar falsos rebotes da traseira do objeto
  if (nowMicros - lastTriggerMicros < 350000UL) return;

  if (trapState == IDLE) {
    if (sensor == 'A') {
      trapState = WAITING_B;
      triggerMicrosA = nowMicros;
      Serial.println("[Feixe A] 🚨 Objeto cortou Feixe A INSTANTANEAMENTE. Cronometrando...");
    } else {
      trapState = WAITING_A;
      triggerMicrosB = nowMicros;
      Serial.println("[Feixe B] 🚨 Objeto cortou Feixe B INSTANTANEAMENTE. Cronometrando...");
    }
  } else if (trapState == WAITING_B && sensor == 'B') {
    unsigned long deltaMicros = nowMicros - triggerMicrosA;
    if (deltaMicros > 500 && deltaMicros <= timeoutMicros) {
      lastTriggerMicros = nowMicros;
      finalizeSpeed(deltaMicros, "Sensor A ➔ Sensor B (Ida)");
    }
    trapState = IDLE;
    isBlockedA = false;
    isBlockedB = false;
  } else if (trapState == WAITING_A && sensor == 'A') {
    unsigned long deltaMicros = nowMicros - triggerMicrosB;
    if (deltaMicros > 500 && deltaMicros <= timeoutMicros) {
      lastTriggerMicros = nowMicros;
      finalizeSpeed(deltaMicros, "Sensor B ➔ Sensor A (Volta)");
    }
    trapState = IDLE;
    isBlockedA = false;
    isBlockedB = false;
  }
}

void parseSensorStream(HardwareSerial &port, uint8_t *buf, uint8_t &idx, char sensorName) {
  while (port.available()) {
    uint8_t b = port.read();
    if (idx < BUF_SIZE) buf[idx++] = b;
    else {
      memmove(buf, buf + 1, BUF_SIZE - 1);
      buf[BUF_SIZE - 1] = b;
    }

    while (idx >= 2) {
      if (buf[0] == 0xFB && buf[1] == 0x03) {
        if (idx < 4) break;
        uint8_t payloadLen = buf[3];
        uint8_t totalLen = 4 + payloadLen + 1;
        if (idx < totalLen) break;

        if (calcCRC(buf, totalLen - 1) == buf[totalLen - 1]) {
          bool isValid = ((buf[4] | (buf[5] << 8)) == 1);
          uint16_t rawDist = buf[6] | (buf[7] << 8);
          float distM = (float)rawDist / 10.0f; // decímetros para metros

          float bg = (sensorName == 'A') ? bgDistA : bgDistB;
          float tol = (sensorName == 'A') ? tolA : tolB;
          float cutoff = bg - tol;

          if (sensorName == 'A') lastDistA = distM;
          else lastDistB = distM;

          // DISPARO INSTANTÂNEO (SEM EXIGIR PERMANÊNCIA):
          // Dispara imediatamente no 1º milissegundo se:
          // 1. Objeto detectado na frente do fundo (distM < cutoff)
          // 2. OU feixe for cortado e perder o eco da parede de fundo (!isValid ou distM <= 0.04m)
          bool hasBgCalibrated = (bg > 0.30f);
          bool isDistanceTrigger = (isValid && distM < cutoff && distM > 0.04f);
          bool isBeamInterrupted = (hasBgCalibrated && (!isValid || distM <= 0.04f));

          bool blocked = isDistanceTrigger || isBeamInterrupted;
          bool &isBlocked = (sensorName == 'A') ? isBlockedA : isBlockedB;
          uint8_t &countBlocked = (sensorName == 'A') ? countBlockedA : countBlockedB;
          unsigned long &pendingMicros = (sensorName == 'A') ? pendingMicrosA : pendingMicrosB;

          if (blocked) {
            if (countBlocked == 0) pendingMicros = micros();
            countBlocked++;
            // Confirmação de 3 amostras (~6ms) para eliminar disparos falsos de ruído
            if (countBlocked >= 3 && !isBlocked) {
              isBlocked = true;
              onSensorTriggered(sensorName, pendingMicros);
            }
          } else {
            countBlocked = 0;
            if (isBlocked && isValid && distM >= cutoff) {
              isBlocked = false;
            }
          }
        }
        uint8_t rem = idx - totalLen;
        memmove(buf, buf + totalLen, rem);
        idx = rem;
      } else {
        memmove(buf, buf + 1, idx - 1);
        idx--;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  delay(1000);
  Serial.println("\n=== Velocímetro Laser Duplo Meskernel PTFH ===");
  Serial.printf("Distância entre sensores (Δs): %.3f metros (Sem limite mínimo)\n", trapDistanceMeters);
  Serial.printf("Fundo A: %.2f m (Corte: < %.2f m) | Fundo B: %.2f m (Corte: < %.2f m)\n", bgDistA, bgDistA - tolA, bgDistB, bgDistB - tolB);
  Serial.println("Comandos no Monitor Serial:");
  Serial.println("  'b' -> Capturar distâncias atuais como Fundos de A e B");
  Serial.println("  'd' -> Ajustar distância entre sensores (ex: d0.25 para 25cm)");
  Serial.println("  'w' -> Reenviar comandos de inicialização para os sensores");
  Serial.println("  'z' -> Zerar contador de passagens");

  SerialA.begin(115200, SERIAL_8N1, SENSOR_A_RX, SENSOR_A_TX);
  SerialB.begin(115200, SERIAL_8N1, SENSOR_B_RX, SENSOR_B_TX);

  delay(500);
  // Inicialização + Modo Ultra Rápido (100Hz/200Hz)
  SerialA.print("O\r\n"); SerialB.print("O\r\n");
  delay(80);
  const uint8_t CMD_FREQ_100HZ[] = {0xFA, 0x04, 0x0A, 0x64, 0x94};
  SerialA.write(CMD_FREQ_100HZ, sizeof(CMD_FREQ_100HZ));
  SerialB.write(CMD_FREQ_100HZ, sizeof(CMD_FREQ_100HZ));
  delay(80);
  SerialA.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  SerialB.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  delay(50);
  SerialA.print("C\r\n"); SerialB.print("C\r\n");

  Serial.println("Sensores inicializados e prontos para medir em ALTA VELOCIDADE!");
}

void loop() {
  parseSensorStream(SerialA, bufA, idxA, 'A');
  parseSensorStream(SerialB, bufB, idxB, 'B');

  // Timeout Watchdog
  if (trapState == WAITING_B && (micros() - triggerMicrosA > timeoutMicros)) {
    Serial.println("[Timeout] Objeto não alcançou o Sensor B. Medição cancelada.");
    trapState = IDLE;
  } else if (trapState == WAITING_A && (micros() - triggerMicrosB > timeoutMicros)) {
    Serial.println("[Timeout] Objeto não alcançou o Sensor A. Medição cancelada.");
    trapState = IDLE;
  }

  // Comandos pelo Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("d") || cmd.startsWith("D")) {
      float newD = cmd.substring(1).toFloat();
      if (newD > 0.001f) {
        trapDistanceMeters = newD;
        Serial.printf("Distância entre sensores ajustada para: %.3f m (%.1f cm)\n", trapDistanceMeters, trapDistanceMeters * 100.0f);
      }
    } else if (cmd.equalsIgnoreCase("b")) {
      if (lastDistA > 0.1f) bgDistA = lastDistA;
      if (lastDistB > 0.1f) bgDistB = lastDistB;
      Serial.printf("🎯 Novos Fundos: A = %.2f m (Corte: < %.2f m) | B = %.2f m (Corte: < %.2f m)\n", bgDistA, bgDistA - tolA, bgDistB, bgDistB - tolB);
    } else if (cmd.equalsIgnoreCase("w")) {
      Serial.println("[Acordando Sensores] Enviando comandos...");
      SerialA.print("O\r\n"); SerialB.print("O\r\n");
      delay(50);
      SerialA.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
      SerialB.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
      delay(50);
      SerialA.print("C\r\n"); SerialB.print("C\r\n");
    } else if (cmd.equalsIgnoreCase("z")) {
      passCounter = 0;
      Serial.println("Contador de passagens zerado.");
    }
  }
}
