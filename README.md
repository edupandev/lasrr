# Velocímetro Laser de Barreira Dupla - Meskernel PTFH

Sistema de medição de velocidade de alta precisão utilizando **dois sensores LiDAR laser Meskernel PTFH** com cálculo de tempo de trânsito em hardware ($\Delta t$ em microssegundos) e velocidade ($v = \Delta s / \Delta t$), calibração de fundo (tripwire), filtro inteligente anti-ruído e tabela comparativa com a velocidade real do veículo para ajuste fino.

Suporta **Arduino Due** (Hardware Timing nativo a 84 MHz com saída pela Porta de Programação) e modo legado direto no PC (Web Serial com 2 conversores USB).

---

## 1. Vantagens do Arduino Due (Alta Precisão)

| Característica | Modo PC Direto (2 Dongles USB) | Modo Arduino Due (Hardware Timing) |
|:---|:---|:---|
| **Resolução Temporal** | ~1 a 4 ms (Jitter de polling USB do PC) | **1 microssegundo (1 µs)** no clock de 84 MHz |
| **Simetria $A \to B$ vs $B \to A$** | Pode variar devido a latências separadas do SO | **Matematicamente idêntico** nos dois sentidos |
| **Processamento** | Duas portas USB assíncronas no PC | **Mesmo microcontrolador SAM3X8E** via UARTs de hardware (Serial1 e Serial2) |
| **Nível Lógico** | Requer adaptadores USB-TTL | **3.3V nativo** compatível direto com o PTFH |
| **Conexão com o PC** | 2 cabos USB | **1 único cabo USB** na Porta de Programação |

---

## 2. Esquema de Ligação Elétrica no Arduino Due (Conforme seu Circuito)

> [!IMPORTANT]
> O Arduino Due opera exclusivamente em **3.3V**. Os sensores Meskernel PTFH possuem lógica de 3.3V TTL, sendo 100% compatíveis diretamente com o Due.
> Os pinos **D2** e **D3** são configurados como saídas digitais em nível ALTO (`HIGH`) no programa para acionar o sinal `PWR_EN` e ligar os sensores.

```
       [SENSOR 1 - Feixe 1]                   [ARDUINO DUE (SAM3X8E)]
       - VCC    ───────────────> 3.3V
       - GND    ───────────────> GND
       - TX     ───────────────> Pino 17 (RX2)
       - RX     ───────────────> Pino 16 (TX2)
       - PWR_EN ───────────────> Pino 2  (D2 - Habilita Alimentação)
       Porta Serial: Serial2

       [SENSOR 2 - Feixe 2]                   [ARDUINO DUE (SAM3X8E)]
       - VCC    ───────────────> 3.3V
       - GND    ───────────────> GND
       - TX     ───────────────> Pino 15 (RX3)
       - RX     ───────────────> Pino 14 (TX3)
       - PWR_EN ───────────────> Pino 3  (D3 - Habilita Alimentação)
       Porta Serial: Serial3

       [CONEXÃO COM O PC]
       - Cabo micro-USB conectado diretamente na porta "PROGRAMMING" do Arduino Due
       - Saída Serial: Serial a 115200 bps
```

---

## 3. Como Gravar o Programa no Arduino Due

1. Abra a **Arduino IDE**.
2. Abra o arquivo [`arduino_due_dual_ptfh_speed.ino`](file:///C:/Users/fidal/.gemini/antigravity/scratch/meskernel-ptfh/arduino_due_dual_ptfh_speed.ino).
3. No menu **Ferramentas (Tools)**:
   - **Placa (Board):** Selecione `Arduino Due (Programming Port)` *(se não estiver instalado, vá em Gerenciador de Placas e instale o pacote "Arduino SAM Boards (32-bits ARM Cortex-M3)")*.
   - **Porta (Port):** Selecione a porta COM correspondente ao `Arduino Due (Programming Port)`.
4. Clique em **Carregar (Upload)**.
5. Quando o carregamento terminar, o programa iniciará automaticamente e a velocidade será transmitida pela porta de programação a **115200 bps**.

---

## 4. Como Usar o Visualizador Web (`http://localhost:3333`)

O servidor do projeto roda em background na porta **3333**:
```bash
node server.js
```
Abra o navegador Google Chrome ou Microsoft Edge em: **`http://localhost:3333`**

### Operando com o Arduino Due:
1. No topo da página, no painel azul **Arduino Due &bull; Alta Precisão Hardware**, clique em **`🔌 Conectar Arduino Due`**.
2. Selecione a porta do Arduino Due e clique em **Conectar**.
3. **Calibrar Paredes de Fundo:**
   - Com a pista vazia, aponte os feixes para o anteparo/parede de fundo (ex: 13 metros).
   - Clique em **`🎯 Capturar Fundos no Due ('b')`** ou nos botões de captura individuais.
4. **Passagem do Carro:**
   - Ao passar pelo feixe, a velocidade é calculada em hardware com precisão de microssegundos.
   - A interface mostra o velocímetro digital, o gráfico de distância dos dois feixes e a linha na tabela de histórico.
5. **Calibração com a Velocidade Real do Carro:**
   - Na tabela de histórico, digite a velocidade real do carro no campo azul **`🚗 Vel. Real (Carro)`** (ex: `30.0` km/h).
   - O sistema calculará na hora a diferença, o erro percentual e a distância ideal entre sensores ($\Delta s$).
   - Clique no botão do assistente **`🎯 Aplicar Ajuste Calibrado em Δs`** para sincronizar o novo espaçamento calibrado diretamente para o Arduino Due!
6. **Exportação CSV:**
   - Clique em **Exportar CSV** para salvar o relatório completo com tempo em $\mu\text{s}$, velocidade medida, velocidade real e fatores de correção.

---

## 5. Comandos Seriais Disponíveis (Porta de Programação)

Você pode enviar comandos pela interface Web ou pelo Monitor Serial da Arduino IDE (115200 bps):

| Comando | Descrição | Exemplo |
|:---|:---|:---|
| `b` | Captura as distâncias atuais como parede de fundo para os sensores A e B | `b` |
| `d<valor>` | Ajusta a distância física entre os sensores em metros ($\Delta s$) | `d0.50` ou `d1.25` |
| `t<valor>` | Ajusta a tolerância de detecção da parede de fundo em metros | `t0.50` |
| `f<n>` | Ajusta o filtro anti-ruído em número de amostras consecutivas | `f3` |
| `w` | Envia rajada para acordar e colocar os sensores no modo contínuo de alta taxa | `w` |
| `z` | Zera o contador de passagens | `z` |
| `?` | Exibe o menu de ajuda com todos os comandos | `?` |
