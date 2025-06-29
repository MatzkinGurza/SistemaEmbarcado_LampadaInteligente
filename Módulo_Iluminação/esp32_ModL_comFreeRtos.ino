// =================================================================
// Módulo de Iluminação 
// =================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// =================================================================
// 1. DEFINIÇÕES E CONSTANTES DE HARDWARE 
// =================================================================
#define PIN_DIMMER_ZERO_CROSS 4  
#define PIN_DIMMER_TRIAC_GATE 2  
#define PIN_RSWITCH 34            // Pino ADC1, seguro para analogRead
#define PIN_BUTTON 23             // Pino seguro de uso geral

#define DELAY_MIN_US 800
#define DELAY_MAX_US 7500
#define IDLE_STATE 8333

// =================================================================
// 2. DEFINIÇÕES DA MÁQUINA DE ESTADOS E DADOS
// =================================================================
enum EstadosModL { LAMP_IND, PROC_WIFI, RECEBENDO, NUM_ESTADOS_MOD_L };
enum EventosModL { AJUST_SWITCH, BOT_SETP, DADOS_RECEBIDOS, TIMEOUT_WIFI, AJUSTE_LUMINOSIDADE_AUTO, SEM_EVENTO, NUM_EVENTOS_MOD_L };
enum AcoesModL { NA, A01_AJUSTAR_BRILHO_MANUAL, A02_INICIAR_MODO_AUTO, A03_DADO_INICIAL_RECEBIDO, A04_AJUSTAR_BRILHO_AUTO };
struct TransicaoModL { EstadosModL proximoEstado; AcoesModL acao; TransicaoModL(EstadosModL p = LAMP_IND, AcoesModL a = NA) : proximoEstado(p), acao(a) {} };
typedef struct struct_message { int luminosityValue; } struct_message;

// =================================================================
// 3. VARIÁVEIS GLOBAIS E DE CONTROLE
// =================================================================
volatile long currentBrightnessDelay = IDLE_STATE;
int lampBrightnessPercent = 0;
const float Kp = 0.4;
const int LARGURA_ZONA_MORTA = 20;
const int MUDANCA_MAX_BRILHO_POR_CICLO = 5;

struct_message receivedData;
int setpointLuminosidade = 300;
volatile long ultimoDadoRecebido = 0;
const long WIFI_TIMEOUT_MS = 10000;
EstadosModL estadoAtualModL = LAMP_IND;

// --- Handles e Mutex ---
QueueHandle_t espNowQueue = NULL;
hw_timer_t *dimmerTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// =================================================================
// 4. ISRs E CALLBACKS
// =================================================================
void IRAM_ATTR onTimer() {
  timerAlarmDisable(dimmerTimer);
  digitalWrite(PIN_DIMMER_TRIAC_GATE, HIGH);
  delayMicroseconds(20);
  digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
}

void IRAM_ATTR detectaZeroCross() {
  portENTER_CRITICAL_ISR(&timerMux);
  long atraso_us = currentBrightnessDelay;
  portEXIT_CRITICAL_ISR(&timerMux);
  if (atraso_us != IDLE_STATE) {
    timerAlarmWrite(dimmerTimer, atraso_us, false);
    timerAlarmEnable(dimmerTimer);
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  struct_message data;
  if (len == sizeof(data)) {
    memcpy(&data, incomingData, sizeof(data));
    xQueueSend(espNowQueue, &data, 0);
    ultimoDadoRecebido = millis();
  }
}

// =================================================================
// 5. CLASSES DE COMPONENTES 
// =================================================================
class RotarySwitch { public: RotarySwitch(int pin, int tolerance) : _pin(pin), _tolerance(tolerance) { _last_value = analogRead(_pin); } bool hasChanged() { int current_value = analogRead(_pin); if (abs(_last_value - current_value) > _tolerance) { _last_value = current_value; return true; } return false; } int getValue() { return _last_value; } private: int _pin, _last_value, _tolerance; };
class Button { public: Button(int pin, long debounce_ms) : _pin(pin), _debounce_ms(debounce_ms), _last_press_time(0) { pinMode(_pin, INPUT_PULLUP); } bool wasClicked() { if (digitalRead(_pin) == LOW && (millis() - _last_press_time) > _debounce_ms) { _last_press_time = millis(); return true; } return false; } private: int _pin; long _debounce_ms, _last_press_time; };

// =================================================================
// 6. OBJETOS GLOBAIS E FUNÇÕES DA MÁQUINA DE ESTADOS
// =================================================================
RotarySwitch rSwitch(PIN_RSWITCH, 200);
Button button(PIN_BUTTON, 200);
TransicaoModL MatrizModL[NUM_ESTADOS_MOD_L][NUM_EVENTOS_MOD_L];

void setLampBrightnessFromPercentage(int percentage) {
  lampBrightnessPercent = constrain(percentage, 0, 100);
  long newDelay = (lampBrightnessPercent < 5) ? IDLE_STATE : map(lampBrightnessPercent, 0, 100, DELAY_MAX_US, DELAY_MIN_US);
  portENTER_CRITICAL(&timerMux);
  currentBrightnessDelay = newDelay;
  portEXIT_CRITICAL(&timerMux);
}

void inicializarMatrizModL() {
  for (int i = 0; i < NUM_ESTADOS_MOD_L; i++) {
    for (int j = 0; j < NUM_EVENTOS_MOD_L; j++) {
      MatrizModL[i][j] = TransicaoModL((EstadosModL)i, NA);
    }
  }
  MatrizModL[LAMP_IND][AJUST_SWITCH]               = TransicaoModL(LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL);
  MatrizModL[LAMP_IND][BOT_SETP]                  = TransicaoModL(PROC_WIFI, A02_INICIAR_MODO_AUTO);
  MatrizModL[PROC_WIFI][AJUST_SWITCH]             = TransicaoModL(LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL);
  MatrizModL[PROC_WIFI][DADOS_RECEBIDOS]          = TransicaoModL(RECEBENDO, A03_DADO_INICIAL_RECEBIDO);
  MatrizModL[RECEBENDO][AJUST_SWITCH]             = TransicaoModL(LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL);
  MatrizModL[RECEBENDO][TIMEOUT_WIFI]             = TransicaoModL(PROC_WIFI, A02_INICIAR_MODO_AUTO);
  MatrizModL[RECEBENDO][AJUSTE_LUMINOSIDADE_AUTO] = TransicaoModL(RECEBENDO, A04_AJUSTAR_BRILHO_AUTO);
}

// =================================================================
// 7. TAREFA FreeRTOS (Máquina de Estados)
// =================================================================
void taskStateMachine(void *pvParameters) {
  Serial.println("[FreeRTOS] Tarefa da Máquina de Estados iniciada.");
  inicializarMatrizModL();
  setLampBrightnessFromPercentage(map(rSwitch.getValue(), 0, 4095, 0, 100));

  for (;;) {
    EventosModL evento = SEM_EVENTO;
    struct_message dataFromQueue;

    if (rSwitch.hasChanged()) { evento = AJUST_SWITCH; }
    else if (button.wasClicked()) { evento = BOT_SETP; }
    else if (estadoAtualModL == RECEBENDO && (millis() - ultimoDadoRecebido > WIFI_TIMEOUT_MS)) { evento = TIMEOUT_WIFI; }
    else if (xQueueReceive(espNowQueue, &dataFromQueue, 0) == pdTRUE) {
      receivedData = dataFromQueue;
      if (estadoAtualModL == PROC_WIFI) evento = DADOS_RECEBIDOS;
      else if (estadoAtualModL == RECEBENDO) evento = AJUSTE_LUMINOSIDADE_AUTO;
    }

    if (evento != SEM_EVENTO) {
      AcoesModL acao = MatrizModL[estadoAtualModL][evento].acao;
      switch (acao) {
        case A01_AJUSTAR_BRILHO_MANUAL:
          setLampBrightnessFromPercentage(map(rSwitch.getValue(), 0, 4095, 0, 100));
          break;
        case A02_INICIAR_MODO_AUTO:
          vTaskDelay(pdMS_TO_TICKS(100));
          break;
        case A03_DADO_INICIAL_RECEBIDO:
          setLampBrightnessFromPercentage(50);
          break;
        case A04_AJUSTAR_BRILHO_AUTO: {
          int erro = setpointLuminosidade - receivedData.luminosityValue;
          if (abs(erro) > LARGURA_ZONA_MORTA) {
            int ajuste = (int)(erro * Kp);
            ajuste = constrain(ajuste, -MUDANCA_MAX_BRILHO_POR_CICLO, MUDANCA_MAX_BRILHO_POR_CICLO);
            int novoBrilho = lampBrightnessPercent + ajuste;
            setLampBrightnessFromPercentage(novoBrilho);
          }
          break;
        }
      }
      estadoAtualModL = MatrizModL[estadoAtualModL][evento].proximoEstado;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =================================================================
// 8. SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Módulo Lâmpada  ---");

  espNowQueue = xQueueCreate(5, sizeof(struct_message));
  if (espNowQueue == NULL) { Serial.println("ERRO: Falha ao criar Fila."); while(1); }

  pinMode(PIN_DIMMER_TRIAC_GATE, OUTPUT);
  digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
  pinMode(PIN_DIMMER_ZERO_CROSS, INPUT_PULLUP);

  dimmerTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(dimmerTimer, &onTimer, true);
  attachInterrupt(digitalPinToInterrupt(PIN_DIMMER_ZERO_CROSS), detectaZeroCross, RISING);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Erro ESP-NOW"); return; }
  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreatePinnedToCore(taskStateMachine, "StateMachineTask", 10240, NULL, 1, NULL, 0);

  Serial.println("Setup completo. Sistema iniciado.");
}

void loop() {
  vTaskDelete(NULL);
}
