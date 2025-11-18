#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 19

MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

String messageToDisplay = "";
bool newMessage = false;


const char* ssid = "Noah";
const char* password = "Paultoutnu";


const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";

WiFiClient espClient;
PubSubClient client(espClient);


void callback(char* topic, byte* message, unsigned int length) {
  messageToDisplay = "";

  for (int i = 0; i < length; i++) {
    messageToDisplay += (char)message[i];
  }

  Serial.print("MQTT Message reçu: ");
  Serial.println(messageToDisplay);

  newMessage = true;
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Connexion MQTT… ");

    if (client.connect("ESP32_Matrix")) {
      Serial.println("OK");
      client.subscribe(topic_sub);
    } else {
      Serial.print("Erreur, code = ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  myDisplay.begin();
  myDisplay.setIntensity(5);
  myDisplay.displayClear();

  Serial.print("  Connexion au WiFi... ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

Serial.println("");

if (WiFi.status() == WL_CONNECTED) {
  Serial.println("WiFi connecté !");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
} else {
  Serial.println("ÉCHEC connexion WiFi !");
}


  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Matrice prête !");
}

void loop() {
  if (!client.connected()) {
    reconnect();
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
