// =================================================================
// Módulo de Sensoriamento (Emissor ESP-NOW) com FreeRTOS
// =================================================================
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <esp_now.h>

// --- CONFIGURAÇÕES ---
uint8_t macAddressEscravo[] = {0x1C, 0x69, 0x20, 0xA4, 0x88, 0x94};
const long readInterval = 2000;

// --- OBJETOS E VARIÁVEIS GLOBAIS ---
BH1750 lightMeter;
esp_now_peer_info_t peerInfo;

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

typedef struct struct_message {
  int luminosityValue; // Enviando o valor de lux
} struct_message;

struct_message myData;

// Função de callback 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n[ESP-NOW] Status do último envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

// [FreeRTOS] Tarefa para ler o sensor e enviar os dados
void taskReadAndSend(void *pvParameters) {
  Serial.println("[FreeRTOS] Tarefa de Leitura e Envio iniciada.");
  for (;;) { // Laço infinito da tarefa
    if (lightMeter.measurementReady(true)) {
      float lux = lightMeter.readLightLevel();
      myData.luminosityValue = (int)lux; // Envia o valor inteiro de lux

      esp_err_t result = esp_now_send(macAddressEscravo, (uint8_t *) &myData, sizeof(myData));

      if (result == ESP_OK) {
        Serial.printf("[SENSOR] Leitura: %.2f lx -> [ESP-NOW] Enviado: %d\n", lux, myData.luminosityValue);
      } else {
        Serial.println("[ESP-NOW] Erro ao enviar os dados.");
      }
    }
    // Aguarda o próximo ciclo 
    vTaskDelay(pdMS_TO_TICKS(readInterval));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Módulo de Sensoriamento (FreeRTOS) ---");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("ERRO: Sensor BH1750 nao encontrado."));
    while (1) vTaskDelay(1);
  }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Erro ao inicializar ESP-NOW"); return; }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, macAddressEscravo, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Falha ao adicionar o peer"); return; }

  // [FreeRTOS] Cria a tarefa, define a função, nome, tamanho da pilha, parâmetros, prioridade e em qual núcleo rodar
  xTaskCreatePinnedToCore(
      taskReadAndSend,    // Função da tarefa
      "ReadAndSendTask",  // Nome da tarefa
      4096,               // Tamanho da pilha (stack)
      NULL,               // Parâmetros da tarefa (nenhum)
      1,                  // Prioridade da tarefa (1 é normal)
      NULL,               // Handle da tarefa (não precisamos)
      0);                 // Roda no núcleo 0
}

void loop() {
  vTaskDelete(NULL); // Deleta a tarefa do loop() para liberar recursos.
}