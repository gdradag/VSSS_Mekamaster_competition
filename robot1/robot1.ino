#include <AX12A.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Configuración para Dynamixel
#define DirectionPin 12   // Pin de dirección
#define BaudRate 1000000  // Tasa de baudios para AX-12A
#define ID1 15            // ID del primer motor
#define ID2 17            // ID del segundo motor

// Configuración de WiFi y MQTT
const char* ssid = "SEMILLERO_ROBOTICA";
const char* password = "Chimuelo2018";
const char* mqtt_server = "192.168.8.101";

WiFiClient espClient;
PubSubClient client(espClient);

void setupWiFi() {
  delay(10);
  Serial.println("📡 Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado");
  Serial.print("📶 IP asignada: ");
  Serial.println(WiFi.localIP());
}

void moveMotors(String direction) {
  Serial.print("🎮 Ejecutando comando: ");
  Serial.println(direction);

  if (direction == "forward") {
    ax12a.turn(ID1, 0, 300);
    delay(10);
    ax12a.turn(ID2, 1, 300);
  } else if (direction == "backward") {
    ax12a.turn(ID1, 1, 300);
    delay(10);
    ax12a.turn(ID2, 0, 300);
  } else if (direction == "left") {
    ax12a.turn(ID1, 1, 200);
    delay(10);
    ax12a.turn(ID2, 1, 200);
  } else if (direction == "right") {
    ax12a.turn(ID1, 0, 200);
    delay(10);
    ax12a.turn(ID2, 0, 200);
  } else if (direction == "stop") {
    ax12a.turn(ID1, 0, 0);
    delay(10);
    ax12a.turn(ID2, 0, 0);
  } else {
    Serial.println("⚠️ Comando no reconocido.");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📩 Mensaje recibido en el tópico [");
  Serial.print(topic);
  Serial.print("]: ");

  String command = "";
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    command += (char)payload[i];
  }
  Serial.println();

  command.trim(); // limpia espacios
  moveMotors(command);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("🔁 Intentando conexión MQTT...");
    if (client.connect("ESP32_robot1")) { //el ID de los robots debe ser diferente para que no exista interferencia entre las ESP32
      Serial.println("✅ Conectado al broker MQTT");
      client.subscribe("robot/move");
    } else {
      Serial.print("❌ Falló, rc=");
      Serial.print(client.state());
      Serial.println(" - Reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);  // Para depuración y para el AX-12A (compartido)
  ax12a.begin(BaudRate, DirectionPin, &Serial); // Comunicación con Dynamixel
  pinMode(DirectionPin, OUTPUT);                // Dirección del bus
  Serial.println("✅ Sistema listo: MQTT + Dynamixel");

  // Establecer motores en modo de rotación continua
  ax12a.setEndless(ID1, ON);
  ax12a.setEndless(ID2, ON);

  setupWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
