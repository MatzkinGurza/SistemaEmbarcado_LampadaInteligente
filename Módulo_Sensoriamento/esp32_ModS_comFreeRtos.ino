// =================================================================
// Módulo de Sensoriamento 
// =================================================================
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <esp_now.h>

// --- CONFIGURAÇÕES ---
uint8_t macAddressEscravo[] = {0x1C, 0x69, 0x20, 0xA4, 0x88, 0x94};
const long READ_INTERVAL_MS = 2000; // Intervalo entre leituras no estado ENVIANDO
const long RETRY_INTERVAL_MS = 5000; // Intervalo para tentar reconectar no estado CONECTANDO

// --- DEFINIÇÕES DA MÁQUINA DE ESTADOS ---
enum EstadosModS { SENSOR_CONECTANDO, SENSOR_ENVIANDO, NUM_ESTADOS_MOD_S };
enum EventosModS { CONEXAO_OK, FALHA_CONEXAO, TEMPO_DE_ACAO, SEM_EVENTO_S, NUM_EVENTOS_MOD_S };
enum AcoesModS { NA_S, A01_TENTAR_CONECTAR, A02_LER_E_ENVIAR };
struct TransicaoModS { EstadosModS proximoEstado; AcoesModS acao; TransicaoModS(EstadosModS p = SENSOR_CONECTANDO, AcoesModS a = NA_S) : proximoEstado(p), acao(a) {} };

// --- OBJETOS E VARIÁVEIS GLOBAIS ---
BH1750 lightMeter;
esp_now_peer_info_t peerInfo;
long lastActionTime = 0;
EstadosModS estadoAtualModS = SENSOR_CONECTANDO; // O sistema inicia tentando conectar
TransicaoModS MatrizModS[NUM_ESTADOS_MOD_S][NUM_EVENTOS_MOD_S];
volatile bool ultimoEnvioOk = true;

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

typedef struct struct_message { int luminosityValue; } struct_message;
struct_message myData;

// --- FUNÇÕES DA MÁQUINA DE ESTADOS ---
void inicializarMatrizModS() {
  for (int i = 0; i < NUM_ESTADOS_MOD_S; i++) {
    for (int j = 0; j < NUM_EVENTOS_MOD_S; j++) {
      MatrizModS[i][j] = TransicaoModS((EstadosModS)i, NA_S);
    }
  }
  // ESTADO "CONECTANDO": Sua principal ação é tentar conectar quando o tempo de ação é atingido
  MatrizModS[SENSOR_CONECTANDO][CONEXAO_OK]     = { SENSOR_ENVIANDO,   A02_LER_E_ENVIAR };
  MatrizModS[SENSOR_CONECTANDO][TEMPO_DE_ACAO]  = { SENSOR_CONECTANDO, A01_TENTAR_CONECTAR };
  
  // ESTADO "ENVIANDO": Sua principal ação é ler e enviar quando o tempo de ação é atingido
  MatrizModS[SENSOR_ENVIANDO][TEMPO_DE_ACAO]  = { SENSOR_ENVIANDO,   A02_LER_E_ENVIAR };
  MatrizModS[SENSOR_ENVIANDO][FALHA_CONEXAO]   = { SENSOR_CONECTANDO, A01_TENTAR_CONECTAR };
}

// --- CALLBACK ESP-NOW ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ultimoEnvioOk = (status == ESP_NOW_SEND_SUCCESS);
}

// --- TAREFA FreeRTOS ---
void taskSensorStateMachine(void *pvParameters) {
  Serial.println("[FreeRTOS] Tarefa da Máquina de Estados do Sensor iniciada.");
  inicializarMatrizModS();

  for (;;) {
    // 1. GERAR EVENTO
    EventosModS evento = SEM_EVENTO_S;
    long currentTime = millis();
    long interval = (estadoAtualModS == SENSOR_ENVIANDO) ? READ_INTERVAL_MS : RETRY_INTERVAL_MS;

    if (!ultimoEnvioOk) {
      evento = FALHA_CONEXAO;
    } else if (currentTime - lastActionTime >= interval) {
      evento = TEMPO_DE_ACAO;
    } else if (estadoAtualModS == SENSOR_CONECTANDO && ultimoEnvioOk && lastActionTime != 0) {
      // Se estava conectando e o último envio (da tentativa) foi ok, transita
      evento = CONEXAO_OK;
    }

    // 2. EXECUTAR MÁQUINA DE ESTADOS
    if (evento != SEM_EVENTO_S) {
      EstadosModS estadoAnterior = estadoAtualModS;
      AcoesModS acao = MatrizModS[estadoAtualModS][evento].acao;
      estadoAtualModS = MatrizModS[estadoAtualModS][evento].proximoEstado;

      if(estadoAnterior != estadoAtualModS) {
        Serial.printf("\n[FSM] MUDANÇA DE ESTADO: %d -> %d\n", estadoAnterior, estadoAtualModS);
      }
      
      // Executar Ação
      switch (acao) {
        case A01_TENTAR_CONECTAR:
          Serial.print("[FSM] Ação: Tentando conectar...");
          lastActionTime = millis();
          if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println(" Peer adicionado. Conexão parece OK.");
            ultimoEnvioOk = true;
          } else {
            Serial.println(" Falha ao adicionar o peer.");
            ultimoEnvioOk = false;
          }
          break;

        case A02_LER_E_ENVIAR:
          if (lightMeter.measurementReady(true)) {
            lastActionTime = millis();
            float lux = lightMeter.readLightLevel();
            myData.luminosityValue = (int)lux;
            esp_err_t result = esp_now_send(macAddressEscravo, (uint8_t *) &myData, sizeof(myData));
            
            if (result == ESP_OK) {
                Serial.printf("[FSM] Lendo e enviando: %d Lux\n", myData.luminosityValue);
            } else {
                ultimoEnvioOk = false; // Dispara o evento de falha no próximo ciclo
            }
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Pausa cooperativa
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Módulo de Sensoriamento com FSM de 2 Estados ---");

  // Inicializa Hardware
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("ERRO: Sensor BH1750 nao encontrado."));
    while (1) vTaskDelay(1);
  }

  // Inicializa Wi-Fi e ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Erro ESP-NOW"); return; }
  esp_now_register_send_cb(OnDataSent);

  // Prepara informações do Peer, mas a conexão será gerenciada pela FSM
  memcpy(peerInfo.peer_addr, macAddressEscravo, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Cria a tarefa principal que gerencia a FSM
  xTaskCreatePinnedToCore(
      taskSensorStateMachine,
      "SensorFSM_Task",
      4096,
      NULL,
      1,
      NULL,
      0);
}

void loop() {
  vTaskDelete(NULL); // O loop principal não é usado
}
