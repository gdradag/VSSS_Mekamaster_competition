#include <WiFi.h>
#include <PubSubClient.h>

// Pines de motor DC
const int IN1 = 13;    
const int IN2 = 14;    
const int IN3 = 12;    
const int IN4 = 25;    
const int ENA = 26;    
const int ENB = 27;    
const int STBY = 33;  

int velocidad = 60;

// Configuración de WiFi y MQTT
const char* ssid = "gabyd";
const char* password = "12345678";
const char* mqtt_server = "192.168.57.128";

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

// Función para ejecutar movimiento según el comando recibido
void moveMotors(String direction) {
  direction.trim();
  direction.toLowerCase();

  Serial.print("🎮 Ejecutando comando: ");
  Serial.println(direction);

  if (direction == "forward") {
    moveForward();
  } else if (direction == "backward") {
    moveBackward();
  } else if (direction == "left") {
    moveLeft();
  } else if (direction == "right") {
    moveRight();
  } else if (direction == "stop") {
    stopMotor();
  } else if (direction.startsWith("velocidad")) {
    // Comando ejemplo: "velocidad150"
    int vel = direction.substring(9).toInt();
    velocidad = constrain(vel, 0, 255);
    aplicarVelocidad();
    Serial.print("🚀 Velocidad actualizada: ");
    Serial.println(velocidad);
  } else {
    Serial.println("⚠️ Comando no reconocido.");
  }
}

// Función para manejar los mensajes recibidos por MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📩 Mensaje recibido en [");
  Serial.print(topic);
  Serial.print("]: ");

  String command = "";
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    command += (char)payload[i];
  }
  Serial.println();

  moveMotors(command);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("🔁 Intentando conexión MQTT...");
    if (client.connect("ESP32_robotDC")) {
      Serial.println("✅ Conectado al broker MQTT");
      client.subscribe("robot3/move");
    } else {
      Serial.print("❌ Falló, rc=");
      Serial.print(client.state());
      Serial.println(" - Reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configuración de pines de motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  setupWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("✅ Sistema listo: MQTT + Motor DC");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

// FUNCIONES DE CONTROL DE MOTORES

void aplicarVelocidad() {
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  aplicarVelocidad();
  Serial.println("➡️ Adelante");
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  aplicarVelocidad();
  Serial.println("⬅️ Atrás");
}

void moveLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  aplicarVelocidad();
  Serial.println("↖️ Izquierda");
}

void moveRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  aplicarVelocidad();
  Serial.println("↗️ Derecha");
}

void stopMotor() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  Serial.println("⛔ Detenido");
}
