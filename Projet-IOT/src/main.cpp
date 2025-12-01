#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 19

MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Message MQTT
String messageToDisplay = "";
bool newMessage = false;

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";

WiFiClient espClient;
PubSubClient client(espClient);


// =========================
// MQTT CALLBACK
// =========================
void callback(char* topic, byte* message, unsigned int length) {
  messageToDisplay = "";

  for (int i = 0; i < length; i++) {
    messageToDisplay += (char)message[i];
  }

  Serial.print("MQTT Message reçu: ");
  Serial.println(messageToDisplay);

  newMessage = true;
}


// =========================
// RECONNEXION MQTT
// =========================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connexion MQTT… ");

    if (client.connect("ESP32_Matrix")) {
      Serial.println("OK !");
      client.subscribe(topic_sub);
    } else {
      Serial.print("Erreur MQTT, code = ");
      Serial.print(client.state());
      Serial.println(" → retry dans 2s");
      delay(2000);
    }
  }
}


// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  // --------------------------
  //  MATRICE LED
  // --------------------------
  myDisplay.begin();
  myDisplay.setIntensity(5);
  myDisplay.displayClear();
  Serial.println("Matrice prête !");


  // --------------------------
  //  APPAIRAGE WiFi AUTOMATIQUE
  // --------------------------
  WiFiManager wm;

  wm.setAPCallback([](WiFiManager *myWM) {
    Serial.println("=== MODE CONFIGURATION ===");
    Serial.println("Connectez-vous au WiFi : ESP32_Setup");
  });

  bool res = wm.autoConnect("ESP32_Setup", "12345678");

  if (!res) {
    Serial.println("⚠️ ÉCHEC de l'appairage, reboot...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi connecté !");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());


  // --------------------------
  //  MQTT
  // --------------------------
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}


// =========================
// LOOP
// =========================
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  if (newMessage) {
    myDisplay.displayText(
      messageToDisplay.c_str(),
      PA_CENTER, 50, 0,
      PA_SCROLL_LEFT, PA_SCROLL_LEFT
    );
    newMessage = false;
  }

  myDisplay.displayAnimate();
}
