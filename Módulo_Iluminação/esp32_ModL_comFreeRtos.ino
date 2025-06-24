// =================================================================
// Módulo de Iluminação (Receptor ESP-NOW) com FreeRTOS
// =================================================================

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

#define DELAY_MIN_US 800
#define DELAY_MAX_US 7500
#define IDLE_STATE 8333

const int TEMPO_SEMICICLO = 1000000 / (2 * 60); // Assumindo 60Hz

// =================================================================
// 2. DEFINIÇÕES DA MÁQUINA DE ESTADOS E DADOS
// =================================================================
enum EstadosModL { LAMP_IND, PROC_WIFI, RECEBENDO, NUM_ESTADOS_MOD_L };
enum EventosModL { AJUST_SWITCH, BOT_SETP, DADOS_RECEBIDOS, TIMEOUT_WIFI, AJUSTE_LUMINOSIDADE_AUTO, SEM_EVENTO, NUM_EVENTOS_MOD_L };
enum AcoesModL { NA, A01_AJUSTAR_BRILHO_MANUAL, A02_INICIAR_MODO_AUTO, A03_DADO_INICIAL_RECEBIDO, A04_AJUSTAR_BRILHO_AUTO };
struct TransicaoModL { EstadosModL proximoEstado; AcoesModL acao; TransicaoModL(EstadosModL p = LAMP_IND, AcoesModL a = NA) : proximoEstado(p), acao(a) {} };
typedef struct struct_message { int luminosityValue; } struct_message;

// =================================================================
// 3. VARIÁVEIS GLOBAIS E HANDLES FreeRTOS
// =================================================================
// --- Variáveis do Dimmer e Controle
volatile long currentBrightnessDelay = IDLE_STATE;
int lampBrightnessPercent = 0;
const float Kp = 5; // Ganho Proporcional 

// --- Variáveis da Máquina de Estados e ESP-NOW
struct_message receivedData;
int setpointLuminosidade = 300; // Setpoint em Lux
volatile long ultimoDadoRecebido = 0;
const long WIFI_TIMEOUT_MS = 10000;
EstadosModL estadoAtualModL = LAMP_IND;

// --- Handles FreeRTOS ---
SemaphoreHandle_t semZeroCross = NULL;
QueueHandle_t espNowQueue = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =================================================================
// 4. ISRs E CALLBACKS 
// =================================================================
void IRAM_ATTR detectaZeroCross() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(semZeroCross, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  struct_message data;
  if (len == sizeof(data)) {
    memcpy(&data, incomingData, sizeof(data));
    xQueueSend(espNowQueue, &data, 0); // Envia para a fila sem bloquear
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
  portENTER_CRITICAL(&mux);
  currentBrightnessDelay = newDelay;
  portEXIT_CRITICAL(&mux);
}

void inicializarMatrizModL() {
  // Configuração padrão: permanecer no mesmo estado sem ação
  for (int i = 0; i < NUM_ESTADOS_MOD_L; i++) {
    for (int j = 0; j < NUM_EVENTOS_MOD_L; j++) {
      MatrizModL[i][j] = TransicaoModL((EstadosModL)i, NA);
    }
  }
  // MODO MANUAL
  MatrizModL[LAMP_IND][AJUST_SWITCH] = { LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL };
  MatrizModL[LAMP_IND][BOT_SETP]     = { PROC_WIFI, A02_INICIAR_MODO_AUTO };
  // MODO DE TRANSIÇÃO (AGUARDANDO DADOS)
  MatrizModL[PROC_WIFI][AJUST_SWITCH]    = { LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL };
  MatrizModL[PROC_WIFI][DADOS_RECEBIDOS] = { RECEBENDO, A03_DADO_INICIAL_RECEBIDO };
  // MODO AUTOMÁTICO
  MatrizModL[RECEBENDO][AJUST_SWITCH] = { LAMP_IND, A01_AJUSTAR_BRILHO_MANUAL };
  MatrizModL[RECEBENDO][TIMEOUT_WIFI] = { PROC_WIFI, A02_INICIAR_MODO_AUTO };
  MatrizModL[RECEBENDO][AJUSTE_LUMINOSIDADE_AUTO] = { RECEBENDO, A04_AJUSTAR_BRILHO_AUTO };
}

// =================================================================
// 7. TAREFAS FreeRTOS
// =================================================================

// --- TAREFA 1: Controle do Dimmer (Alta Prioridade, Núcleo 1) ---
void taskDimmerControl(void *pvParameters) {
  Serial.println("[FreeRTOS] Tarefa do Dimmer iniciada no núcleo 1.");
  for (;;) {
    if (xSemaphoreTake(semZeroCross, portMAX_DELAY) == pdTRUE) {
      long atraso_us;
      portENTER_CRITICAL(&mux);
      atraso_us = currentBrightnessDelay;
      portEXIT_CRITICAL(&mux);

      if (atraso_us != IDLE_STATE) {
        delayMicroseconds(atraso_us);
        digitalWrite(PIN_DIMMER_TRIAC_GATE, HIGH);
        delayMicroseconds(20);
        digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
      }
    }
  }
}

// --- TAREFA 2: Máquina de Estados e Lógica (Prioridade Normal, Núcleo 0) ---
void taskStateMachine(void *pvParameters) {
  Serial.println("[FreeRTOS] Tarefa da Máquina de Estados iniciada no núcleo 0.");
  
  // Inicializa a matriz de transições dentro da própria tarefa
  inicializarMatrizModL();
  
  // Define o brilho inicial baseado no potenciômetro
  setLampBrightnessFromPercentage(map(rSwitch.getValue(), 0, 4095, 0, 100));

  for (;;) {
    // 1. GERAR EVENTO
    EventosModL evento = SEM_EVENTO;
    struct_message dataFromQueue;

    if (rSwitch.hasChanged()) {
      evento = AJUST_SWITCH;
    } else if (button.wasClicked()) {
      evento = BOT_SETP;
    } else if (estadoAtualModL == RECEBENDO && (millis() - ultimoDadoRecebido > WIFI_TIMEOUT_MS)) {
      evento = TIMEOUT_WIFI;
    } else if (xQueueReceive(espNowQueue, &dataFromQueue, 0) == pdTRUE) {
      receivedData = dataFromQueue;
      if (estadoAtualModL == PROC_WIFI) evento = DADOS_RECEBIDOS;
      else if (estadoAtualModL == RECEBENDO) evento = AJUSTE_LUMINOSIDADE_AUTO;
    }

    // 2. EXECUTAR MÁQUINA DE ESTADOS
    if (evento != SEM_EVENTO) {
      AcoesModL acao = MatrizModL[estadoAtualModL][evento].acao;

      // Executar Ação
      switch (acao) {
        case A01_AJUSTAR_BRILHO_MANUAL:
          setLampBrightnessFromPercentage(map(rSwitch.getValue(), 0, 4095, 0, 100));
          break;
        case A02_INICIAR_MODO_AUTO:
          delay(100);
          break;
        case A03_DADO_INICIAL_RECEBIDO:
          // Usa o primeiro valor recebido como setpoint, se desejado, ou mantém um fixo.
          // setpointLuminosidade = receivedData.luminosityValue;
          setLampBrightnessFromPercentage(50); // Inicia em 50%
          break;
        case A04_AJUSTAR_BRILHO_AUTO: {
          int erro = setpointLuminosidade - receivedData.luminosityValue;
          int ajuste = (int)(erro * Kp);
          int novoBrilho = lampBrightnessPercent + ajuste;
          setLampBrightnessFromPercentage(novoBrilho);
          break;
        }
      }
      
      // Mudar para o próximo estado
      estadoAtualModL = MatrizModL[estadoAtualModL][evento].proximoEstado;
    }
  }
}

// =================================================================
// 8. SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Módulo da Lâmpada (FreeRTOS) ---");

  // Criação dos objetos FreeRTOS
  semZeroCross = xSemaphoreCreateBinary();
  espNowQueue = xQueueCreate(5, sizeof(struct_message));

  if (semZeroCross == NULL || espNowQueue == NULL) {
    Serial.println("ERRO FATAL: Falha ao criar objetos FreeRTOS.");
    while(1); // Trava
  }

  // Configuração de Pinos
  pinMode(PIN_DIMMER_TRIAC_GATE, OUTPUT);
  digitalWrite(PIN_DIMMER_TRIAC_GATE, LOW);
  pinMode(PIN_DIMMER_ZERO_CROSS, INPUT_PULLUP);

  // Configuração Wi-Fi e ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Erro ESP-NOW"); return; }
  esp_now_register_recv_cb(OnDataRecv);

  // Anexa a interrupção de Zero Cross
  attachInterrupt(digitalPinToInterrupt(PIN_DIMMER_ZERO_CROSS), detectaZeroCross, RISING);

  // Criação das Tarefas
  // A tarefa da máquina de estados (lógica principal) roda no núcleo 0 com prioridade normal
  xTaskCreatePinnedToCore(taskStateMachine, "StateMachineTask", 10240, NULL, 1, NULL, 0);
  
  // A tarefa do dimmer (timing crítico) roda no núcleo 1 com prioridade alta
  xTaskCreatePinnedToCore(taskDimmerControl, "DimmerControlTask", 2048, NULL, 3, NULL, 1);

  Serial.println("Setup completo. Tarefas FreeRTOS iniciadas.");
}

void loop() {
  // O loop principal fica vazio. O FreeRTOS gerencia tudo.
  vTaskDelete(NULL);
}