/*
 * ==============================================================================
 * VELOCÍMETRO LASER DE BARREIRA DUPLA DE ALTA PRECISÃO (HARDWARE TIMING)
 * Plataforma: ARDUINO DUE (ARM Cortex-M3 @ 84 MHz - Atmel SAM3X8E)
 * Sensores: 2x Meskernel PTFH LiDAR (UART 115200 bps, ~490 Hz streaming)
 * Saída de Velocidade / Telemetria: Porta de Programação USB (Serial a 115200 bps)
 * ==============================================================================
 * 
 * MAPA DE PINOS EXATO (CONFORME SEU CIRCUITO / ESQUEMA):
 * ------------------------------------------------------------------------------
 * SENSOR 1 (Feixe 1 - Entrada / Ida):
 *   - VCC    --> 3.3V
 *   - GND    --> GND
 *   - TX     --> Arduino Due Pino 17 (RX2)
 *   - RX     --> Arduino Due Pino 16 (TX2)
 *   - PWR_EN --> Arduino Due Pino 2   (D2 - Controle de Alimentação)
 *   - Porta Serial no Due: Serial2
 * 
 * SENSOR 2 (Feixe 2 - Saída / Volta):
 *   - VCC    --> 3.3V
 *   - GND    --> GND
 *   - TX     --> Arduino Due Pino 15 (RX3)
 *   - RX     --> Arduino Due Pino 14 (TX3)
 *   - PWR_EN --> Arduino Due Pino 3   (D3 - Controle de Alimentação)
 *   - Porta Serial no Due: Serial3
 * 
 * SAÍDA DE ALARME / INDICADORES:
 *   - Pino 13 --> LED Onboard do Arduino Due (acende durante detecção de passagem)
 *   - Pino 4  --> Saída Digital de Alarme / Relé / Buzzer de excesso (3.3V)
 * 
 * COMUNICAÇÃO COM O COMPUTADOR / WEB APP:
 *   - Cabo micro-USB conectado diretamente na porta "PROGRAMMING" do Arduino Due
 *   - Taxa de comunicação: 115200 bps
 * ==============================================================================
 */

#include <Arduino.h>

// Definição de Pinos conforme seu circuito
const int PIN_PWR_EN_1   = 2;  // Pino D2: Habilita alimentação do Sensor 1
const int PIN_PWR_EN_2   = 3;  // Pino D3: Habilita alimentação do Sensor 2
const int PIN_ALARM      = 4;  // Pino D4: Saída opcional de alarme de excesso
const int PIN_LED_ONBOARD = 13; // LED nativo da placa Due

// --- PARÂMETROS CONFIGURÁVEIS DA ARMADILHA ÓPTICA ---
float trapDistanceMeters = 0.50f;     // Distância física entre Feixe A e Feixe B (sem limite mínimo)
float maxSpeedLimitKmH = 60.0f;       // Limite para acionar o alarme de excesso (km/h)
float bgDistA = 4.00f;                // Distância da parede de fundo do Sensor 1 (metros)
float tolA = 0.50f;                   // Tolerância de corte do Sensor 1
float bgDistB = 4.00f;                // Distância da parede de fundo do Sensor 2 (metros)
float tolB = 0.50f;                   // Tolerância de corte do Sensor 2
uint8_t filterSamplesRequired = 3;    // Amostras consecutivas para confirmação anti-ruído (~6ms)
const unsigned long timeoutMicros = 5000000UL; // 5 segundos de timeout para passagem incompleta

// Buffers de Recepção dos Sensores
#define BUF_SIZE 64
uint8_t bufA[BUF_SIZE], bufB[BUF_SIZE];
uint8_t idxA = 0, idxB = 0;

// Estado Atual dos Sensores
float currentDistA = 0.0f;
float currentDistB = 0.0f;
bool currentValidA = false;
bool currentValidB = false;
unsigned long packetCountA = 0;
unsigned long packetCountB = 0;
float rateA_Hz = 0.0f;
float rateB_Hz = 0.0f;

// Máquina de Estados do Velocímetro de Barreira Dupla
enum TrapState { IDLE, WAITING_B, WAITING_A };
TrapState trapState = IDLE;

unsigned long triggerMicrosA = 0;
unsigned long triggerMicrosB = 0;
unsigned long clearMicrosA = 0;
unsigned long clearMicrosB = 0;
unsigned long pendingMicrosA = 0;
unsigned long pendingMicrosB = 0;
uint8_t countBlockedA = 0;
uint8_t countBlockedB = 0;
bool isBlockedA = false;
bool isBlockedB = false;

unsigned long lastPassMicros = 0;
unsigned long passCounter = 0;

// Timers de Telemetria Periódica para o Web App
unsigned long lastTelemetryMillis = 0;
unsigned long lastRateCalcMillis = 0;

// Comandos do Protocolo Meskernel PTFH
const uint8_t CMD_CONTINUOUS[] = { 0xFA, 0x01, 0xFF, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF };
const uint8_t CMD_FREQ_100HZ[] = { 0xFA, 0x04, 0x0A, 0x64, 0x94 };

// CRC padrão Meskernel (Soma de bytes módulo 256)
uint8_t calcCRC(const uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

// Envia comandos de inicialização para os sensores
void wakeSensors() {
  Serial.println(F("[DUE] Habilitando alimentação dos Sensores 1 e 2 (D2/D3 = HIGH)..."));
  digitalWrite(PIN_PWR_EN_1, HIGH);
  digitalWrite(PIN_PWR_EN_2, HIGH);
  delay(120); // Aguarda inicialização do circuito interno do sensor

  Serial.println(F("[DUE] Enviando comandos de ativação (Sensor 1 em Serial2 / Sensor 2 em Serial3)..."));
  
  // Laser ON (ASCII 'O')
  Serial2.print("O\r\n");
  Serial3.print("O\r\n");
  delay(60);

  // Frequência rápida (100 Hz / ~490 Hz streaming)
  Serial2.write(CMD_FREQ_100HZ, sizeof(CMD_FREQ_100HZ));
  Serial3.write(CMD_FREQ_100HZ, sizeof(CMD_FREQ_100HZ));
  delay(60);

  // Modo contínuo binário
  Serial2.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  Serial3.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
  delay(60);

  // Comando C (Contínuo)
  Serial2.print("C\r\n");
  Serial3.print("C\r\n");
  
  Serial.println(F("[DUE] Sensores ativados no modo contínuo de alta taxa!"));
  Serial.println(F("{\"type\":\"ACK\",\"msg\":\"Sensores ativados com sucesso!\"}"));
}

// Power Cycle (desliga e liga os pinos PWR_EN para reset físico dos sensores)
void powerCycleSensors() {
  Serial.println(F("[DUE] Executando Power Cycle nos sensores (D2/D3 -> LOW -> HIGH)..."));
  digitalWrite(PIN_PWR_EN_1, LOW);
  digitalWrite(PIN_PWR_EN_2, LOW);
  delay(350);
  digitalWrite(PIN_PWR_EN_1, HIGH);
  digitalWrite(PIN_PWR_EN_2, HIGH);
  delay(200);
  wakeSensors();
}

// Finaliza o cálculo de velocidade quando o segundo feixe é acionado
void finalizeSpeed(unsigned long deltaMicros, const char *dirCode, const char *dirDescription) {
  float deltaSec = (float)deltaMicros / 1000000.0f;
  if (deltaSec <= 0.000001f) return;

  float speed_m_s = trapDistanceMeters / deltaSec;
  float speed_km_h = speed_m_s * 3.6f;
  float deltaMs = (float)deltaMicros / 1000.0f;

  passCounter++;
  lastPassMicros = micros();
  bool overspeed = (speed_km_h > maxSpeedLimitKmH);

  // Duração de oclusão de feixe (estimativa de comprimento do veículo)
  unsigned long cutMicrosA = (clearMicrosA > triggerMicrosA) ? (clearMicrosA - triggerMicrosA) : 0;
  unsigned long cutMicrosB = (clearMicrosB > triggerMicrosB) ? (clearMicrosB - triggerMicrosB) : 0;
  float cutDurationSec = 0.0f;
  if (cutMicrosA > 0 && cutMicrosB > 0) cutDurationSec = ((float)cutMicrosA + (float)cutMicrosB) / 2000000.0f;
  else if (cutMicrosA > 0) cutDurationSec = (float)cutMicrosA / 1000000.0f;
  else if (cutMicrosB > 0) cutDurationSec = (float)cutMicrosB / 1000000.0f;
  float estLengthM = speed_m_s * cutDurationSec;

  // Acionamento de alarme físico
  if (overspeed) {
    digitalWrite(PIN_ALARM, HIGH);
    digitalWrite(PIN_LED_ONBOARD, HIGH);
  }

  // 1. Saída Formatada Amigável (Visualização em qualquer Monitor Serial)
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print(F("⚡ PASSAGEM #")); Serial.print(passCounter);
  Serial.println(F(" DETECTADA (ARDUINO DUE - HARDWARE TIMING)!"));
  Serial.print(F("   Sentido: ")); Serial.println(dirDescription);
  Serial.print(F("   Distância entre sensores (Δs): ")); Serial.print(trapDistanceMeters, 3); Serial.println(F(" m"));
  Serial.print(F("   Tempo de Trânsito (Δt): ")); Serial.print(deltaMs, 3);
  Serial.print(F(" ms (")); Serial.print(deltaMicros); Serial.println(F(" µs)"));
  Serial.print(F("   VELOCIDADE: ")); Serial.print(speed_km_h, 2); Serial.print(F(" km/h  ("));
  Serial.print(speed_m_s, 2); Serial.println(F(" m/s)"));
  if (estLengthM > 0.05f) {
    Serial.print(F("   Comprimento Estimado: ")); Serial.print(estLengthM, 2); Serial.println(F(" m"));
  }
  if (overspeed) {
    Serial.println(F("   ⚠ ALERTA: VELOCIDADE ACIMA DO LIMITE CONFIGURADO!"));
  } else {
    Serial.println(F("   ✓ Velocidade regular."));
  }
  Serial.println(F("=================================================="));

  // 2. Saída em JSON para o Web App (lido diretamente pela interface)
  Serial.print(F("{\"type\":\"PASS\",\"id\":"));
  Serial.print(passCounter);
  Serial.print(F(",\"kmh\":"));
  Serial.print(speed_km_h, 2);
  Serial.print(F(",\"ms\":"));
  Serial.print(speed_m_s, 2);
  Serial.print(F(",\"deltaMs\":"));
  Serial.print(deltaMs, 3);
  Serial.print(F(",\"deltaUs\":"));
  Serial.print(deltaMicros);
  Serial.print(F(",\"trapDist\":"));
  Serial.print(trapDistanceMeters, 3);
  Serial.print(F(",\"dir\":\""));
  Serial.print(dirCode);
  Serial.print(F("\",\"overspeed\":"));
  Serial.print(overspeed ? F("true") : F("false"));
  Serial.print(F(",\"cutMsA\":"));
  Serial.print((float)cutMicrosA / 1000.0f, 1);
  Serial.print(F(",\"cutMsB\":"));
  Serial.print((float)cutMicrosB / 1000.0f, 1);
  Serial.print(F(",\"estLengthM\":"));
  Serial.print(estLengthM, 2);
  Serial.println(F("}"));

  if (overspeed) {
    delay(120);
    digitalWrite(PIN_ALARM, LOW);
    digitalWrite(PIN_LED_ONBOARD, LOW);
  }
}

// Disparo instantâneo do feixe
void onSensorTriggered(char sensor, unsigned long instantMicros) {
  // Cooldown de 350ms para evitar falsos múltiplos disparos pela lataria/reboque
  if (micros() - lastPassMicros < 350000UL) return;

  digitalWrite(PIN_LED_ONBOARD, HIGH);

  if (trapState == IDLE) {
    if (sensor == 'A') {
      trapState = WAITING_B;
      triggerMicrosA = instantMicros;
      Serial.print(F("{\"type\":\"TRIGGER\",\"sensor\":\"A\",\"micros\":"));
      Serial.print(instantMicros);
      Serial.println(F("}"));
      Serial.println(F("[Feixe 1 - Sensor 1] 🚨 Cortado! Cronômetro em hardware iniciado..."));
    } else {
      trapState = WAITING_A;
      triggerMicrosB = instantMicros;
      Serial.print(F("{\"type\":\"TRIGGER\",\"sensor\":\"B\",\"micros\":"));
      Serial.print(instantMicros);
      Serial.println(F("}"));
      Serial.println(F("[Feixe 2 - Sensor 2] 🚨 Cortado! Cronômetro em hardware iniciado..."));
    }
  } else if (trapState == WAITING_B && sensor == 'B') {
    unsigned long deltaMicros = instantMicros - triggerMicrosA;
    if (deltaMicros > 400 && deltaMicros <= timeoutMicros) {
      finalizeSpeed(deltaMicros, "A->B", "Sensor 1 ➔ Sensor 2 (Ida)");
    }
    trapState = IDLE;
    isBlockedA = false;
    isBlockedB = false;
    digitalWrite(PIN_LED_ONBOARD, LOW);
  } else if (trapState == WAITING_A && sensor == 'A') {
    unsigned long deltaMicros = instantMicros - triggerMicrosB;
    if (deltaMicros > 400 && deltaMicros <= timeoutMicros) {
      finalizeSpeed(deltaMicros, "B->A", "Sensor 2 ➔ Sensor 1 (Volta)");
    }
    trapState = IDLE;
    isBlockedA = false;
    isBlockedB = false;
    digitalWrite(PIN_LED_ONBOARD, LOW);
  }
}

void onSensorCleared(char sensor, unsigned long instantMicros) {
  if (sensor == 'A') clearMicrosA = instantMicros;
  else clearMicrosB = instantMicros;

  if (!isBlockedA && !isBlockedB) {
    digitalWrite(PIN_LED_ONBOARD, LOW);
  }
}

// Parser de stream binário dos sensores LiDAR
void parseSensorStream(Stream &port, uint8_t *buf, uint8_t &idx, char sensorName) {
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

          if (sensorName == 'A') {
            currentDistA = distM;
            currentValidA = isValid;
            packetCountA++;
          } else {
            currentDistB = distM;
            currentValidB = isValid;
            packetCountB++;
          }

          float bg = (sensorName == 'A') ? bgDistA : bgDistB;
          float tol = (sensorName == 'A') ? tolA : tolB;
          float cutoff = bg - tol;

          // DISPARO INSTANTÂNEO COM FILTRO ANTI-RUÍDO:
          // 1. Objeto passou em frente da parede de fundo (distM < cutoff)
          // 2. OU feixe interrompido / perdeu eco da parede (!isValid ou <= 0.04m)
          bool hasBgCalibrated = (bg > 0.30f);
          bool isDistanceTrigger = (isValid && distM < cutoff && distM > 0.04f);
          bool isBeamInterrupted = (hasBgCalibrated && (!isValid || distM <= 0.04f));
          bool blocked = isDistanceTrigger || isBeamInterrupted;

          bool &isBlocked = (sensorName == 'A') ? isBlockedA : isBlockedB;
          uint8_t &countBlocked = (sensorName == 'A') ? countBlockedA : countBlockedB;
          unsigned long &pendingMicros = (sensorName == 'A') ? pendingMicrosA : pendingMicrosB;

          if (blocked) {
            // Guarda o timestamp exato do 1º pacote do corte
            if (countBlocked == 0) pendingMicros = micros();
            countBlocked++;

            if (countBlocked >= filterSamplesRequired && !isBlocked) {
              isBlocked = true;
              onSensorTriggered(sensorName, pendingMicros);
            }
          } else {
            countBlocked = 0;
            if (isBlocked && isValid && distM >= cutoff) {
              isBlocked = false;
              onSensorCleared(sensorName, micros());
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

// Processamento de comandos vindos do Computador / Web App pela Porta de Programação
void processSerialCommands() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) continue;

    // Ajuste de Distância entre Sensores (ex: d0.50 ou d1.25)
    if (cmd.startsWith("d") || cmd.startsWith("D")) {
      float newD = cmd.substring(1).toFloat();
      if (newD > 0.0005f) {
        trapDistanceMeters = newD;
        Serial.print(F("[DUE] Distância entre sensores (Δs) ajustada para: "));
        Serial.print(trapDistanceMeters, 3);
        Serial.println(F(" m"));
        Serial.print(F("{\"type\":\"CONFIG\",\"trapDist\":"));
        Serial.print(trapDistanceMeters, 3);
        Serial.println(F("}"));
      }
    }
    // Captura dos Fundos Atuais (Comando 'b')
    else if (cmd.equalsIgnoreCase("b")) {
      if (currentDistA > 0.1f) bgDistA = currentDistA;
      if (currentDistB > 0.1f) bgDistB = currentDistB;
      Serial.print(F("[DUE] 🎯 Novos Fundos Capturados: Sensor 1 = "));
      Serial.print(bgDistA, 2);
      Serial.print(F("m (Corte: < "));
      Serial.print(bgDistA - tolA, 2);
      Serial.print(F("m) | Sensor 2 = "));
      Serial.print(bgDistB, 2);
      Serial.print(F("m (Corte: < "));
      Serial.print(bgDistB - tolB, 2);
      Serial.println(F("m)"));
      Serial.print(F("{\"type\":\"BG_UPDATE\",\"bgA\":"));
      Serial.print(bgDistA, 2);
      Serial.print(F(",\"tolA\":"));
      Serial.print(tolA, 2);
      Serial.print(F(",\"bgB\":"));
      Serial.print(bgDistB, 2);
      Serial.print(F(",\"tolB\":"));
      Serial.print(tolB, 2);
      Serial.println(F("}"));
    }
    // Ajuste de Tolerância (ex: t0.50 ou t0.30)
    else if (cmd.startsWith("t") || cmd.startsWith("T")) {
      float newTol = cmd.substring(1).toFloat();
      if (newTol >= 0.05f && newTol <= 5.0f) {
        tolA = newTol;
        tolB = newTol;
        Serial.print(F("[DUE] Tolerância ajustada para: "));
        Serial.print(newTol, 2);
        Serial.println(F(" m"));
        Serial.print(F("{\"type\":\"CONFIG\",\"tol\":"));
        Serial.print(newTol, 2);
        Serial.println(F("}"));
      }
    }
    // Ajuste do Filtro de Amostras (ex: f2 ou f3)
    else if (cmd.startsWith("f") || cmd.startsWith("F")) {
      int newF = cmd.substring(1).toInt();
      if (newF >= 1 && newF <= 10) {
        filterSamplesRequired = (uint8_t)newF;
        Serial.print(F("[DUE] Filtro anti-ruído ajustado para: "));
        Serial.print(filterSamplesRequired);
        Serial.println(F(" amostras consecutivas."));
        Serial.print(F("{\"type\":\"CONFIG\",\"filterSamples\":"));
        Serial.print(filterSamplesRequired);
        Serial.println(F("}"));
      }
    }
    // Acordar Sensores (Comando 'w')
    else if (cmd.equalsIgnoreCase("w")) {
      wakeSensors();
    }
    // Power Cycle / Reiniciar alimentação (Comando 'p')
    else if (cmd.equalsIgnoreCase("p")) {
      powerCycleSensors();
    }
    // Zerar Contador de Passagens (Comando 'z')
    else if (cmd.equalsIgnoreCase("z")) {
      passCounter = 0;
      Serial.println(F("[DUE] Contador de passagens zerado."));
      Serial.println(F("{\"type\":\"CONFIG\",\"passCounter\":0}"));
    }
    // Ajuda (Comando '?')
    else if (cmd.equals("?") || cmd.equalsIgnoreCase("help")) {
      Serial.println(F("\n--- COMANDOS DISPONÍVEIS NO ARDUINO DUE ---"));
      Serial.println(F("  b        -> Captura distâncias atuais como parede de fundo Sensor 1 e 2"));
      Serial.println(F("  d<valor> -> Ajusta distância física entre sensores (ex: d0.50)"));
      Serial.println(F("  t<valor> -> Ajusta tolerância do feixe em metros (ex: t0.50)"));
      Serial.println(F("  f<n>     -> Ajusta filtro anti-ruído em amostras (ex: f3)"));
      Serial.println(F("  w        -> Acorda e reinicializa os sensores LiDAR"));
      Serial.println(F("  p        -> Reinicia alimentação dos sensores via D2/D3 (Power Cycle)"));
      Serial.println(F("  z        -> Zera o contador de passagens"));
      Serial.println(F("-------------------------------------------\n"));
    }
  }
}

void setup() {
  // Inicializa a Porta de Programação do Arduino Due (Pinos 0 e 1 via ATmega16U2)
  Serial.begin(115200);

  // 1. Configura os pinos de alimentação D2 (Sensor 1) e D3 (Sensor 2)
  pinMode(PIN_PWR_EN_1, OUTPUT);
  pinMode(PIN_PWR_EN_2, OUTPUT);
  
  // Habilita a alimentação (PWR_EN = HIGH)
  digitalWrite(PIN_PWR_EN_1, HIGH);
  digitalWrite(PIN_PWR_EN_2, HIGH);

  // Pinos de Alarme e LED Onboard
  pinMode(PIN_ALARM, OUTPUT);
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  digitalWrite(PIN_ALARM, LOW);
  digitalWrite(PIN_LED_ONBOARD, LOW);

  // Aguarda estabilização da conexão serial e inicialização do circuito
  delay(1200);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   VELOCÍMETRO LASER DUPLO MESKERNEL PTFH - ARDUINO DUE  "));
  Serial.println(F("          Hardware Timing @ 84 MHz (Microsegundos)       "));
  Serial.println(F("========================================================"));
  Serial.println(F("Pinagem Ativa:"));
  Serial.println(F("  Sensor 1 -> Serial2 (RX2=17, TX2=16) | PWR_EN: Pino D2"));
  Serial.println(F("  Sensor 2 -> Serial3 (RX3=15, TX3=14) | PWR_EN: Pino D3"));
  Serial.println(F("  Saída Velocidade: Porta de Programação USB (115200 bps)"));
  Serial.print(F("Distância Inicial entre Sensores (Δs): "));
  Serial.print(trapDistanceMeters, 3);
  Serial.println(F(" m"));
  Serial.print(F("Filtro Anti-Ruído: "));
  Serial.print(filterSamplesRequired);
  Serial.println(F(" amostras consecutivas"));

  // Inicializa as UARTs de Hardware conforme seu circuito
  // Sensor 1: Serial2 (Pino 17 RX2, Pino 16 TX2)
  // Sensor 2: Serial3 (Pino 15 RX3, Pino 14 TX3)
  Serial2.begin(115200);
  Serial3.begin(115200);

  delay(200);
  wakeSensors();
}

void loop() {
  // 1. Processa stream de dados do Sensor 1 (Serial2 - RX2=Pino 17 / TX2=Pino 16)
  parseSensorStream(Serial2, bufA, idxA, 'A');

  // 2. Processa stream de dados do Sensor 2 (Serial3 - RX3=Pino 15 / TX3=Pino 14)
  parseSensorStream(Serial3, bufB, idxB, 'B');

  // 3. Processa comandos recebidos do computador pela Porta de Programação
  processSerialCommands();

  // 4. Watchdog de Timeout de Passagem Incompleta (5 segundos)
  if (trapState == WAITING_B && (micros() - triggerMicrosA > timeoutMicros)) {
    Serial.println(F("[Timeout] Objeto cortou Feixe 1 mas não atingiu Feixe 2. Passagem cancelada."));
    Serial.println(F("{\"type\":\"TIMEOUT\",\"from\":\"A\"}"));
    trapState = IDLE;
    digitalWrite(PIN_LED_ONBOARD, LOW);
  } else if (trapState == WAITING_A && (micros() - triggerMicrosB > timeoutMicros)) {
    Serial.println(F("[Timeout] Objeto cortou Feixe 2 mas não atingiu Feixe 1. Passagem cancelada."));
    Serial.println(F("{\"type\":\"TIMEOUT\",\"from\":\"B\"}"));
    trapState = IDLE;
    digitalWrite(PIN_LED_ONBOARD, LOW);
  }

  // 5. Cálculo de Taxa de Amostragem (Hz) a cada 1 segundo
  unsigned long nowMs = millis();
  if (nowMs - lastRateCalcMillis >= 1000UL) {
    float elapsedSec = (float)(nowMs - lastRateCalcMillis) / 1000.0f;
    rateA_Hz = (float)packetCountA / elapsedSec;
    rateB_Hz = (float)packetCountB / elapsedSec;
    packetCountA = 0;
    packetCountB = 0;
    lastRateCalcMillis = nowMs;
  }

  // 6. Telemetria Contínua para a Interface Web (10 vezes por segundo = a cada 100ms)
  if (nowMs - lastTelemetryMillis >= 100UL) {
    lastTelemetryMillis = nowMs;
    Serial.print(F("{\"type\":\"TELEMETRY\",\"distA\":"));
    Serial.print(currentDistA, 2);
    Serial.print(F(",\"distB\":"));
    Serial.print(currentDistB, 2);
    Serial.print(F(",\"validA\":"));
    Serial.print(currentValidA ? F("true") : F("false"));
    Serial.print(F(",\"validB\":"));
    Serial.print(currentValidB ? F("true") : F("false"));
    Serial.print(F(",\"blockedA\":"));
    Serial.print(isBlockedA ? F("true") : F("false"));
    Serial.print(F(",\"blockedB\":"));
    Serial.print(isBlockedB ? F("true") : F("false"));
    Serial.print(F(",\"rateA\":"));
    Serial.print((int)rateA_Hz);
    Serial.print(F(",\"rateB\":"));
    Serial.print((int)rateB_Hz);
    Serial.print(F(",\"trapState\":\""));
    Serial.print((trapState == IDLE) ? F("IDLE") : ((trapState == WAITING_B) ? F("WAIT_B") : F("WAIT_A")));
    Serial.println(F("\"}"));
  }
}
