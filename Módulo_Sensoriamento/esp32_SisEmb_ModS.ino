#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <esp_now.h>

// --- CONFIGURAÇÕES ---
// Endereço MAC do Módulo da Lâmpada (Escravo)
uint8_t macAddressEscravo[] = {0x1C, 0x69, 0x20, 0xA4, 0x88, 0x94};
// Intervalo entre leituras e envios (em milissegundos)
const long readInterval = 2000;

// --- OBJETOS E VARIÁVEIS GLOBAIS ---
BH1750 lightMeter;
esp_now_peer_info_t peerInfo;
long lastReadTime = 0;

// Pinos I2C
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// Estrutura de dados para envio.
typedef struct struct_message {
  int luminosityPercent;
} struct_message;

struct_message myData;

// Função de callback executada após o envio de uma mensagem
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n[ESP-NOW] Status do último envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=================================================");
  Serial.println("Módulo de Sensoriamento (Emissor ESP-NOW) - Iniciado");
  Serial.println("=================================================");

  // Inicia a comunicação I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Inicia o sensor BH1750
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("[SENSOR] Sensor BH1750 encontrado e configurado."));
  } else {
    Serial.println(F("[SENSOR] ERRO: Sensor BH1750 nao encontrado. Verifique as conexoes!"));
    while (1) { delay(10); } // Trava se não encontrar
  }

  // --- CONFIGURAÇÃO ESP-NOW ---
  // Coloca o dispositivo no modo Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("[WIFI] Endereço MAC deste dispositivo: ");
  Serial.println(WiFi.macAddress());

  // Inicializa o ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Erro ao inicializar o ESP-NOW");
    return;
  }
  
  // Registra o callback de envio
  esp_now_register_send_cb(OnDataSent);

  // Registra o peer (o Módulo da Lâmpada)
  memcpy(peerInfo.peer_addr, macAddressEscravo, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Adiciona o peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] Falha ao adicionar o peer");
    return;
  }
  Serial.println("[ESP-NOW] Peer adicionado. Pronto para enviar dados.");
}

void loop() {
  // Envia os dados periodicamente
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();

    if (lightMeter.measurementReady(true)) { 
      float lux = lightMeter.readLightLevel();
      myData.luminosityPercent = lux;

      // Envia a estrutura de dados via ESP-NOW
      esp_err_t result = esp_now_send(macAddressEscravo, (uint8_t *) &myData, sizeof(myData));

      if (result == ESP_OK) {
        Serial.print("[SENSOR] Leitura: ");
        Serial.print(lux);
        Serial.print(" lx -> [ESP-NOW] Enviado: ");
        Serial.print(myData.luminosityPercent);
        //Serial.println("%");
      } else {
        Serial.println("[ESP-NOW] Erro ao enviar os dados.");
      }
    }
  }
  delay(10); 
}