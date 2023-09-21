#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Configurações de rede Wi-Fi
const char* ssid = "nomeDaRede";
const char* password = "senhaDaRede";

// Configurações broker MQTT
const char* mqtt_server = "broker.hivemq.com";
const int port = 1883;

// Cliente MQTT
WiFiClient wifiClient;
PubSubClient mqttClient;

// Pino do LED
const int led1 = 23; // Pino do LED no ESP32

// Variável que vai indicar o estado do LED1 (false = Apagar LED | true = Ligar LED)
bool ledState = false;

// Declaração da task
TaskHandle_t task1;

// Função de callback para lidar com mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Callback");
  Serial.print("Topico: ");
  Serial.println(topic);
  // Converte o payload em uma string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Verifica qual tópico MQTT recebeu a mensagem
  if (strcmp(topic, "projetoifspbri/eng2023/ledstate") == 0) {
    if (message == "0") {
      // Se o payload for "0", o LED deve ser desligado
      ledState = false;
    } else if (message == "1") {
      // Se o payload for "1", o LED deve ser ligado
      ledState = true;
    }
  }
}

void controleLed(void *pvParameters) {
  while (1) {
    // Verifique o estado do LED e realize ação necessária
    if (ledState == true) {
      digitalWrite(led1, HIGH); // Liga o LED
    } else {
      digitalWrite(led1, LOW); // Desliga o LED
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Atraso de 100 ms entre verificações
  }
}

void setup () {
  Serial.begin(115200);
  pinMode(led1, OUTPUT);

  //Cria uma task que irá controlar o estado do LED1
  //xTaskCreatePinnedToCore(nome da função, "tarefa", tamanho da pilha, Parametros, Prioridade, alteração manual, núcleo (Principal = 1 e Secundário = 0))
  xTaskCreatePinnedToCore(controleLed, "task1", 10000, NULL, 1, NULL, 1);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado");
  Serial.println("Endereço ip: ");
  Serial.println(WiFi.localIP());
  
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(mqtt_server, port);
  
  while (!mqttClient.connected()) {
    Serial.print("********** Tentando estabelecer conexão MQTT...");
    if (mqttClient.connect("broker.hivemq.com", "", "")) { 
      Serial.println("-> Cliente MQTT conectado!");
    } else {
      Serial.print("Conexão falhou, rc=");
      Serial.print(mqttClient.state());
      Serial.println("-> Tente novamente em 3 segundos");
      delay(3000);
    }
  }
  
  mqttClient.setCallback(callback);
  mqttClient.subscribe("projetoifspbri/eng2023/ledstate");
}

void loop() {
  mqttClient.loop();
}