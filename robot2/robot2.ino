#include <AX12A.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DirectionPin 12   // Pin de dirección de los motores
#define BaudRate 1000000  // Velocidad de comunicación Dynamixel
#define ID1 4             // ID del primer motor (puedes cambiar si el robot 2 usa otros IDs)
#define ID2 5             // ID del segundo motor

// Configuración WiFi y MQTT
const char* ssid = "SEMILLERO_ROBOTICA";
const char* password = "Chimuelo2018";
const char* mqtt_server = "192.168.8.100";

WiFiClient espClient;
PubSubClient client(espClient);

// Función para conectarse al WiFi
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

// Función de callback para mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("📩 Mensaje recibido en el tópico [");
    Serial.print(topic);
    Serial.print("]: ");

    String command = "";
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        command += (char)payload[i];
    }
    Serial.println();

    command.trim();  // Elimina espacios innecesarios

    Serial.print("🎮 Comando interpretado: ");
    Serial.println(command);

    if (command == "forward") {
        ax12a.turn(ID1, 0, 300);
        ax12a.turn(ID2, 1, 300);
    } else if (command == "backward") {
        ax12a.turn(ID1, 1, 300);
        ax12a.turn(ID2, 0, 300);
    } else if (command == "left") {
        ax12a.turn(ID1, 1, 200);
        ax12a.turn(ID2, 1, 200);
    } else if (command == "right") {
        ax12a.turn(ID1, 0, 200);
        ax12a.turn(ID2, 0, 200);
    } else if (command == "stop") {
        ax12a.turn(ID1, 0, 0);
        ax12a.turn(ID2, 0, 0);
    } else {
        Serial.println("⚠️ Comando no reconocido.");
    }
}

// Reintenta conexión al broker si se pierde
void reconnect() {
    while (!client.connected()) {
        Serial.print("🔁 Intentando conexión MQTT...");
        if (client.connect("ESP32Client2")) { // Cambio de nombre para diferenciar el robot 2
            Serial.println("✅ Conectado al broker MQTT");
            client.subscribe("robot2/move");  // Suscribirse al tópico del robot 2
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
    ax12a.begin(BaudRate, DirectionPin, &Serial);
    pinMode(DirectionPin, OUTPUT);

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
