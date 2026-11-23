/*
 * Barreira Laser Óptica e Detector de Objetos - Meskernel PTFH
 * Compatível com ESP32, Arduino Mega, RP2040 (Raspberry Pi Pico) ou STM32
 * 
 * Ligações Elétricas (Tensão de Nível Lógico: 3.3V TTL):
 *   Sensor VCC  --> 3.3V (ou 5V se a sua variante tiver regulador integrado)
 *   Sensor GND  --> GND
 *   Sensor TX   --> MCU RX (Ex: ESP32 GPIO 16)
 *   Sensor RX   --> MCU TX (Ex: ESP32 GPIO 17)
 * 
 * Saída de Alarme / Relé / Buzzer Físico:
 *   ESP32 GPIO 2 (LED embutido) ou GPIO 4 (Buzzer / Módulo Relé)
 */

#include <Arduino.h>

#if defined(ESP32)
  #define SENSOR_SERIAL Serial2
  #define RX_PIN 16
  #define TX_PIN 17
  #define ALARM_OUTPUT_PIN 2  // LED embutido ou pino de relé/buzzer
#elif defined(ARDUINO_AVR_MEGA2560)
  #define SENSOR_SERIAL Serial1
  #define ALARM_OUTPUT_PIN 13
#else
  #define SENSOR_SERIAL Serial1
  #define ALARM_OUTPUT_PIN 13
#endif

// Comandos Hexadecimais Meskernel PTFH
const uint8_t CMD_SINGLE[]     = { 0xFA, 0x01, 0xFF, 0x04, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00 };
const uint8_t CMD_CONTINUOUS[] = { 0xFA, 0x01, 0xFF, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF };
const uint8_t CMD_STOP[]       = { 0xFA, 0x01, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE };

// --- CONFIGURAÇÕES DA BARREIRA / DETECÇÃO DE FUNDO E VELOCIDADE ---
// Distância da parede/fundo de referência (em metros):
float backgroundDistanceMeters = 4.00;
float toleranceMeters = 0.20; // Margem de tolerância (ex: 20 cm)

// Comprimento do objeto no sentido do movimento (em metros):
float objectLengthMeters = 0.30; // Padrão: 30 cm (ajuste conforme seu objeto)

// Variáveis de controle de passagem
bool isObjectPresent = false;
unsigned long objectStartTimeMicros = 0;
float objectMinDistance = 999.0;
unsigned int objectCount = 0;
float lastValidDistance = 0.0;

// Buffer de recepção serial
#define RX_BUF_SIZE 64
uint8_t rxBuffer[RX_BUF_SIZE];
uint8_t rxIndex = 0;

uint8_t calculateCRC(const uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return (uint8_t)(sum & 0xFF);
}

void sendCommand(const uint8_t *cmd, size_t len) {
  SENSOR_SERIAL.write(cmd, len);
}

void handleObjectDetection(float distanceMeters, bool isValid) {
  if (!isValid) return;

  lastValidDistance = distanceMeters;

  // REGRA: MARCAR QUALQUER OBJETO NA FRENTE DO FUNDO
  float cutoff = backgroundDistanceMeters - toleranceMeters;
  bool breached = (distanceMeters < cutoff && distanceMeters > 0.05);

  if (breached) {
    if (!isObjectPresent) {
      // Início do corte do feixe pelo objeto
      isObjectPresent = true;
      objectStartTimeMicros = micros();
      objectMinDistance = distanceMeters;
      objectCount++;

      // Aciona relé / buzzer físico
      digitalWrite(ALARM_OUTPUT_PIN, HIGH);

      Serial.println();
      Serial.println("==================================================");
      Serial.printf("🚨 [FEIXE CORTADO] Objeto #%u detectado a %.2f m!\n", objectCount, distanceMeters);
      Serial.println("   Cronometrando tempo de passagem do objeto...");
      Serial.println("==================================================");
    } else {
      // Objeto continua passando pelo feixe
      if (distanceMeters < objectMinDistance) {
        objectMinDistance = distanceMeters;
      }
    }
  } else {
    // Objeto saiu do feixe laser -> CALCULA VELOCIDADE (v = L / Δt)
    if (isObjectPresent) {
      unsigned long durationMicros = micros() - objectStartTimeMicros;
      isObjectPresent = false;

      // Desliga relé / buzzer físico
      digitalWrite(ALARM_OUTPUT_PIN, LOW);

      float durationSec = (float)durationMicros / 1000000.0f;
      float durationMs = (float)durationMicros / 1000.0f;

      if (durationSec > 0.005f) {
        float speed_m_s = objectLengthMeters / durationSec;
        float speed_km_h = speed_m_s * 3.6f;

        Serial.println("--------------------------------------------------");
        Serial.printf("⚡ [VELOCIDADE CALCULADA] Objeto #%u:\n", objectCount);
        Serial.printf("   VELOCIDADE: %.1f km/h  (%.2f m/s)\n", speed_km_h, speed_m_s);
        Serial.printf("   Tempo de Corte (Δt): %.1f ms\n", durationMs);
        Serial.printf("   Comprimento do Objeto (L): %.2f m (%.0f cm)\n", objectLengthMeters, objectLengthMeters * 100.0f);
        Serial.printf("   Distância de Passagem: %.2f m\n", objectMinDistance);
        Serial.println("--------------------------------------------------");
      }
    }
  }
}

void parseFrame(const uint8_t *frame, uint8_t len) {
  uint8_t expectedCrc = calculateCRC(frame, len - 1);
  uint8_t receivedCrc = frame[len - 1];

  if (expectedCrc != receivedCrc) {
    return; // CRC inválido
  }

  uint16_t validInd = frame[4] | (frame[5] << 8);
  bool isValid = (validInd == 1);

  uint16_t rawDist = frame[6] | (frame[7] << 8);
  // Unidade decímetros (dm): 1 dm = 0.1 m
  float distanceMeters = (float)rawDist / 10.0;

  if (isValid) {
    // Avalia detecção de objeto
    handleObjectDetection(distanceMeters, true);

    // Se não estiver em alarme no momento, exibe leitura normal de monitoramento
    if (!isObjectPresent) {
      Serial.print("Distância: ");
      Serial.print(distanceMeters, 2);
      Serial.print(" m | Limiar de Corte: < ");
      Serial.print(alarmThresholdMeters, 2);
      Serial.print(" m | Objetos: ");
      Serial.println(objectCount);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ALARM_OUTPUT_PIN, OUTPUT);
  digitalWrite(ALARM_OUTPUT_PIN, LOW);

  delay(1000);
  Serial.println("\n=== Barreira Laser Meskernel PTFH - Detector de Fundo ===");
  Serial.print("Fundo Inicial: ");
  Serial.print(backgroundDistanceMeters);
  Serial.print(" m | Corte: < ");
  Serial.print(backgroundDistanceMeters - toleranceMeters);
  Serial.println(" m");
  Serial.println("Comandos no Monitor Serial:");
  Serial.println("  'b' -> Capturar distância atual como Fundo");
  Serial.println("  'w' -> Reenviar comandos de inicialização para acordar o sensor");
  Serial.println("  'z' -> Zerar contador de objetos");

#if defined(ESP32)
  SENSOR_SERIAL.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
#else
  SENSOR_SERIAL.begin(115200);
#endif

  delay(500);
  // Envia rajada de inicialização para garantir que o sensor responda
  SENSOR_SERIAL.print("O\r\n"); // Laser ON
  delay(100);
  sendCommand(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  delay(50);
  SENSOR_SERIAL.print("C\r\n"); // ASCII continuous
}

void loop() {
  // Leitura do fluxo serial do sensor
  while (SENSOR_SERIAL.available()) {
    uint8_t b = SENSOR_SERIAL.read();

    if (rxIndex < RX_BUF_SIZE) {
      rxBuffer[rxIndex++] = b;
    } else {
      memmove(rxBuffer, rxBuffer + 1, RX_BUF_SIZE - 1);
      rxBuffer[RX_BUF_SIZE - 1] = b;
    }

    while (rxIndex >= 2) {
      if (rxBuffer[0] == 0xFB && rxBuffer[1] == 0x03) {
        if (rxIndex < 4) break;
        uint8_t payloadLen = rxBuffer[3];
        uint8_t totalLen = 4 + payloadLen + 1;

        if (rxIndex < totalLen) break;

        parseFrame(rxBuffer, totalLen);

        uint8_t remaining = rxIndex - totalLen;
        memmove(rxBuffer, rxBuffer + totalLen, remaining);
        rxIndex = remaining;
      } else {
        memmove(rxBuffer, rxBuffer + 1, rxIndex - 1);
        rxIndex--;
      }
    }
  }

  // Interatividade via teclado no Monitor Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      if (lastValidDistance > 0.1) {
        backgroundDistanceMeters = lastValidDistance;
        Serial.println();
        Serial.print("🎯 Fundo Capturado: ");
        Serial.print(backgroundDistanceMeters, 2);
        Serial.print(" m -> Qualquer objeto na frente de ");
        Serial.print(backgroundDistanceMeters - toleranceMeters, 2);
        Serial.println(" m será marcado!");
      }
    } else if (c == 'w' || c == 'W') {
      Serial.println("[Acordando sensor] Enviando comandos...");
      SENSOR_SERIAL.print("O\r\n");
      delay(50);
      sendCommand(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
      delay(50);
      SENSOR_SERIAL.print("C\r\n");
    } else if (c == 'l' || c == 'L') {
      Serial.println("\nDigite o novo comprimento do objeto em metros (ex: 0.35 para 35cm):");
      while (!Serial.available()) delay(10);
      float newL = Serial.parseFloat();
      if (newL > 0.005) {
        objectLengthMeters = newL;
        Serial.printf("Novo comprimento configurado: %.2f m (%.0f cm)\n", objectLengthMeters, objectLengthMeters * 100.0f);
      }
    } else if (c == 'z' || c == 'Z') {
      objectCount = 0;
      Serial.println("Contador de objetos zerado.");
    }
  }
}
