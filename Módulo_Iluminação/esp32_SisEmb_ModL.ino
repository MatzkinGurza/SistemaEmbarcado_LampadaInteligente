#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// =================================================================
// 1. DEFINIÇÕES E CONSTANTES DE HARDWARE
// =================================================================
#define PIN_DIMMER_ZERO_CROSS 2
#define PIN_DIMMER_TRIAC_GATE 4
#define PIN_RSWITCH 34
#define PIN_BUTTON 23
#define PIN_LED_VERMELHO 27
#define PIN_LED_AZUL 26

#define DELAY_MIN_US 800
#define DELAY_MAX_US 7500
#define TRIGGER_TRIAC_INTERVAL 20
#define IDLE_STATE 8333

// Tempo de um semiciclo em microssegundos: 8333 µs para 60Hz
const int TEMPO_SEMICICLO = 1000000 / (2 * 60); // Assumindo 60Hz

// =================================================================
// 2. DEFINIÇÕES DA MÁQUINA DE ESTADOS
// =================================================================
enum EstadosModL { LAMP_IND, PROC_WIFI, RECEBENDO, NUM_ESTADOS_MOD_L };
enum EventosModL { AJUST_SWITCH, BOT_SETP, DADOS_RECEBIDOS, TIMEOUT_WIFI, LUM_ACIMA, LUM_ABAIXO, SEM_EVENTO, NUM_EVENTOS_MOD_L };
enum AcoesModL { NA, A01_AJUSTAR_BRILHO, A02_INICIAR_SETP, A03_WIFI_CONECTADO, A04_DIMINUIR_BRILHO, A05_AUMENTAR_BRILHO };

struct TransicaoModL {
  EstadosModL proximoEstado;
  AcoesModL acao;
  TransicaoModL(EstadosModL p = LAMP_IND, AcoesModL a = NA) : proximoEstado(p), acao(a) {}
};

// =================================================================
// 3. VARIÁVEIS GLOBAIS E ESTRUTURAS DE DADOS
// =================================================================
typedef struct struct_message {
  int luminosityPercent;
} struct_message;

struct_message receivedData;

volatile long currentBrightnessDelay = IDLE_STATE;
volatile bool isTriacPulseScheduled = false;
volatile int setpointLuminosidade = 50;
volatile bool novosDadosRecebidos = false;
volatile long ultimoDadoRecebido = 0;
const long WIFI_TIMEOUT_MS = 10000;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

EstadosModL estadoAtualModL = LAMP_IND;

// =================================================================
// 4. CALLBACK ESP-NOW E FUNÇÕES DE DEBUG
// =================================================================
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  novosDadosRecebidos = true;
  ultimoDadoRecebido = millis();
}

const char* estadoParaString(EstadosModL estado) {
  switch (estado) {
    case LAMP_IND:   return "LAMP_IND (Controle Manual)";
    case PROC_WIFI:  return "PROC_WIFI (Aguardando Dados)";
    case RECEBENDO:  return "RECEBENDO (Controle Automatico)";
    default:         return "ESTADO DESCONHECIDO";
  }
}

const char* eventoParaString(EventosModL evento) {
  switch (evento) {
    case AJUST_SWITCH:     return "AJUST_SWITCH (Potenciometro)";
    case BOT_SETP:         return "BOT_SETP (Botao Pressionado)";
    case DADOS_RECEBIDOS:  return "DADOS_RECEBIDOS (ESP-NOW)";
    case TIMEOUT_WIFI:     return "TIMEOUT_WIFI (Comunicacao perdida)";
    case LUM_ACIMA:        return "LUM_ACIMA (Luz Ambiente Alta)";
    case LUM_ABAIXO:       return "LUM_ABAIXO (Luz Ambiente Baixa)";
    case SEM_EVENTO:       return "SEM_EVENTO";
    default:               return "EVENTO DESCONHECIDO";
  }
}

const char* acaoParaString(AcoesModL acao) {
  switch (acao) {
    case NA:                  return "Nenhuma";
    case A01_AJUSTAR_BRILHO:  return "A01: Ajustar brilho via potenciometro";
    case A02_INICIAR_SETP:    return "A02: Iniciar modo automatico (ESP-NOW)";
    case A03_WIFI_CONECTADO:  return "A03: Primeiro dado recebido";
    case A04_DIMINUIR_BRILHO: return "A04: Diminuir brilho da lampada";
    case A05_AUMENTAR_BRILHO: return "A05: Aumentar brilho da lampada";
    default:                  return "ACAO DESCONHECIDA";
  }
}

// =================================================================
// 5. MOTOR DO DIMMER (INTERRUPÇÕES E TIMERS) 
// =================================================================
volatile bool flagZeroCross = false;
volatile unsigned long ultimaInterrupcao = 0;
const long tempoDebounce = 7; // Debounce em milissegundos

void IRAM_ATTR detectaZeroCross() {
  // Lógica de Debounce
  if (millis() - ultimaInterrupcao > tempoDebounce) {
    flagZeroCross = true;
    ultimaInterrupcao = millis();
  }
}

// =================================================================
// 6. CLASSES DE COMPONENTES DE HARDWARE
// =================================================================
class RotarySwitch {
public:
  RotarySwitch(int pin, int tolerance) : _pin(pin), _tolerance(tolerance) {
    _last_value = analogRead(_pin);
  }
  bool hasChanged() {
    int current_value = analogRead(_pin);
    if (abs(_last_value - current_value) > _tolerance) {
      _last_value = current_value;
      return true;
    }
    return false;
  }
  int getValue() { return _last_value; }
private:
  int _pin, _last_value, _tolerance;
};

class Button {
public:
  Button(int pin, long debounce_ms) : _pin(pin), _debounce_ms(debounce_ms), _last_press_time(0) {
    pinMode(_pin, INPUT_PULLUP);
  }
  bool wasClicked() {
    if (digitalRead(_pin) == LOW && (millis() - _last_press_time) > _debounce_ms) {
      _last_press_time = millis();
      return true;
    }
    return false;
  }
private:
  int _pin;
  long _debounce_ms, _last_press_time;
};

// =================================================================
// 7. OBJETOS GLOBAIS E MATRIZ DE ESTADOS
// =================================================================
RotarySwitch rSwitch(PIN_RSWITCH, 500);
Button button(PIN_BUTTON, 200);
TransicaoModL MatrizModL[NUM_ESTADOS_MOD_L][NUM_EVENTOS_MOD_L];

// =================================================================
// 8. FUNÇÕES DA MÁQUINA DE ESTADOS
// =================================================================
void setLampBrightness(int potValue) {
  long newDelay = map(potValue, 0, 4095, DELAY_MAX_US, DELAY_MIN_US);
  portENTER_CRITICAL(&mux);
  currentBrightnessDelay = (potValue < 100) ? IDLE_STATE : newDelay;
  portEXIT_CRITICAL(&mux);
}

void inicializarMatrizModL() {
  for (int i = 0; i < NUM_ESTADOS_MOD_L; i++) {
    for (int j = 0; j < NUM_EVENTOS_MOD_L; j++) {
      MatrizModL[i][j] = TransicaoModL((EstadosModL)i, NA);
    }
  }

  MatrizModL[LAMP_IND][AJUST_SWITCH] = { LAMP_IND, A01_AJUSTAR_BRILHO };
  MatrizModL[LAMP_IND][BOT_SETP] = { PROC_WIFI, A02_INICIAR_SETP };

  MatrizModL[PROC_WIFI][AJUST_SWITCH] = { LAMP_IND, A01_AJUSTAR_BRILHO };
  MatrizModL[PROC_WIFI][DADOS_RECEBIDOS] = { RECEBENDO, A03_WIFI_CONECTADO };

  MatrizModL[RECEBENDO][AJUST_SWITCH] = { LAMP_IND, A01_AJUSTAR_BRILHO };
  MatrizModL[RECEBENDO][TIMEOUT_WIFI] = { PROC_WIFI, A02_INICIAR_SETP };
  MatrizModL[RECEBENDO][LUM_ACIMA] = { RECEBENDO, A04_DIMINUIR_BRILHO };
  MatrizModL[RECEBENDO][LUM_ABAIXO] = { RECEBENDO, A05_AUMENTAR_BRILHO };
}

EventosModL gerarEvento() {
  if (rSwitch.hasChanged()) return AJUST_SWITCH;
  if (button.wasClicked()) return BOT_SETP;

  if (estadoAtualModL == RECEBENDO && ((millis() - ultimoDadoRecebido) > WIFI_TIMEOUT_MS)) {
    return TIMEOUT_WIFI;
  }

  if ((estadoAtualModL == PROC_WIFI) && (WiFi.status() != WL_CONNECTED)) {
    Serial.println("Conexao Wi-Fi: Tentando Conectar...");
    WiFi.reconnect(); // Tenta reconectar usando as últimas credenciais
    }

  if (novosDadosRecebidos) {
    novosDadosRecebidos = false;

    if (estadoAtualModL == PROC_WIFI) {
      return DADOS_RECEBIDOS;
    }

    if (estadoAtualModL == RECEBENDO) {
      if (receivedData.luminosityPercent > setpointLuminosidade) {
        Serial.printf("[ESP-NOW] Dado recebido: %d%%\n", receivedData.luminosityPercent);
        return LUM_ACIMA;
      } else if (receivedData.luminosityPercent < setpointLuminosidade) {
        Serial.printf("[ESP-NOW] Dado recebido: %d%%\n", receivedData.luminosityPercent);
        return LUM_ABAIXO;
      }
    }
  }
  return SEM_EVENTO;
}

void executarAcao(AcoesModL acao) {
  digitalWrite(PIN_LED_VERMELHO, LOW);

  switch (acao) {
    case A01_AJUSTAR_BRILHO:
      setLampBrightness(rSwitch.getValue());
      digitalWrite(PIN_LED_AZUL, LOW);
      break;

    case A02_INICIAR_SETP:
      Serial.println("[Acao] Modo automatico. Aguardando dados via ESP-NOW...");
      break;

    case A03_WIFI_CONECTADO:
      Serial.println("[Acao] Primeiro dado recebido! Controle automatico ativo.");
      digitalWrite(PIN_LED_AZUL, HIGH);
      setpointLuminosidade = receivedData.luminosityPercent;
      Serial.printf("SetPoint recebido: %d%%\n", setpointLuminosidade);
      digitalWrite(PIN_LED_AZUL, HIGH);
      break;

    case A04_DIMINUIR_BRILHO: {
      Serial.println("[Acao] Ambiente claro, diminuindo brilho...");
      int erro_dim = receivedData.luminosityPercent-setpointLuminosidade;
      int KP_dim = 5;
      portENTER_CRITICAL(&mux);
      currentBrightnessDelay = currentBrightnessDelay + KP_dim*erro_dim;
      if (currentBrightnessDelay < DELAY_MIN_US) currentBrightnessDelay = DELAY_MIN_US;
      portEXIT_CRITICAL(&mux);
      break;
    }
    case A05_AUMENTAR_BRILHO: {
      Serial.println("[Acao] Ambiente escuro, aumentando brilho...");
      int erro_aum = receivedData.luminosityPercent-setpointLuminosidade;
      int KP_aum = 5;
      portENTER_CRITICAL(&mux);
      currentBrightnessDelay = currentBrightnessDelay + KP_aum*erro_aum;
      if (currentBrightnessDelay > DELAY_MAX_US) currentBrightnessDelay = DELAY_MAX_US;
      portEXIT_CRITICAL(&mux);
      break;
    }
    case NA:
    default:
      break;
  }
}

// =================================================================
// 9. SETUP E LOOP PRINCIPAL
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=============================================");
  Serial.println("Modulo da Lampada (Receptor ESP-NOW) - Inicializando...");
  Serial.println("=============================================");

  pinMode(PIN_DIMMER_TRIAC_GATE, OUTPUT);
  pinMode(PIN_LED_VERMELHO, OUTPUT);
  pinMode(PIN_LED_AZUL, OUTPUT);

  WiFi.mode(WIFI_STA);
  Serial.print("[WIFI] Endereço MAC deste dispositivo: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Erro ao inicializar");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  inicializarMatrizModL();

  // Configura os pinos do dimmer
  pinMode(PIN_DIMMER_TRIAC_GATE, OUTPUT);
  digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
  pinMode(PIN_DIMMER_ZERO_CROSS, INPUT);

  // Anexa a interrupção para o Zero Cross
  attachInterrupt(digitalPinToInterrupt(PIN_DIMMER_ZERO_CROSS), detectaZeroCross, RISING);

  setLampBrightness(rSwitch.getValue());
  Serial.print("Estado Inicial: ");
  Serial.println(estadoParaString(estadoAtualModL));
}

void loop() {
  EventosModL evento = gerarEvento();

  // --- LÓGICA DE DISPARO DO DIMMER (Adaptada) ---
  if (flagZeroCross) {
    flagZeroCross = false; // "Consome" a flag

    // Calcula o atraso para o disparo do TRIAC
    int atraso_us = currentBrightnessDelay;

    // Aguarda o tempo de atraso
    if (atraso_us != IDLE_STATE) { // Impede o disparo se o brilho estiver desligado
      delayMicroseconds(atraso_us);

      // Dispara o TRIAC com um pulso curto
      digitalWrite(PIN_DIMMER_TRIAC_GATE, HIGH);
      delayMicroseconds(15);
      digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
    }
  }

  if (evento != SEM_EVENTO) {
    EstadosModL estadoAnterior = estadoAtualModL;
    AcoesModL acao = MatrizModL[estadoAnterior][evento].acao;
    estadoAtualModL = MatrizModL[estadoAnterior][evento].proximoEstado;

    Serial.println("---------------------------------------------");
    Serial.print("TRANSICAO: [");
    Serial.print(estadoParaString(estadoAnterior));
    Serial.print("] + [");
    Serial.print(eventoParaString(evento));
    Serial.print("] -> [");
    Serial.print(estadoParaString(estadoAtualModL));
    Serial.println("]");
    Serial.print("  -> ACAO: ");
    Serial.println(acaoParaString(acao));
    //Serial.println(currentBrightnessDelay);
    executarAcao(acao);
  }
}