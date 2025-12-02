#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include "arduinoFFT.h"

// Configuration de la matrice LED
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   18  // SCK (vert) - GPIO18
#define DATA_PIN  23  // DIN (cyan) - GPIO23  
#define CS_PIN    22  // CS (orange) - GPIO22

// Objets pour la matrice
MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Configuration du microphone
#define MIC_PIN 34

// Configuration FFT
#define SAMPLES 128
#define SAMPLING_FREQUENCY 10000

arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
double vReal[SAMPLES];
double vImag[SAMPLES];

// Variables pour l'affichage
#define NUM_BANDS 32
int bandValues[NUM_BANDS] = {0};

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";
const char* topic_mode = "iot/demo/mode"; // Nouveau topic pour changer de mode

WiFiClient espClient;
PubSubClient client(espClient);

String messageToDisplay = "";
bool newMessage = false;

// MODE: true = Spectre Audio, false = Affichage texte
bool spectreMode = true;
unsigned long lastModeSwitch = 0;

// =========================
// MQTT CALLBACK
// =========================
void callback(char* topic, byte* message, unsigned int length) {
  String topicStr = String(topic);
  String msg = "";
  
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  Serial.print("MQTT reçu sur ");
  Serial.print(topicStr);
  Serial.print(": ");
  Serial.println(msg);

  // Changement de mode
  if (topicStr == topic_mode) {
    if (msg == "spectre" || msg == "audio") {
      spectreMode = true;
      Serial.println("→ Mode SPECTRE AUDIO activé");
    } else if (msg == "texte" || msg == "message") {
      spectreMode = false;
      myDisplay.displayClear();
      Serial.println("→ Mode TEXTE activé");
    }
  }
  // Message à afficher
  else if (topicStr == topic_sub) {
    messageToDisplay = msg;
    newMessage = true;
    spectreMode = false; // Passe en mode texte automatiquement
  }
}

// =========================
// RECONNEXION MQTT
// =========================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connexion MQTT... ");
    
    String clientId = "ESP32_Matrix_";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println(" -> Connecté !");
      client.subscribe(topic_sub);
      client.subscribe(topic_mode);
    } else {
      Serial.print(" -> Echec, code = ");
      Serial.print(client.state());
      Serial.println(" -> retry dans 2s");
      delay(2000);
    }
  }
}

// =========================
// SPECTRE AUDIO
// =========================
void updateSpectrum() {
  // Échantillonnage
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long microseconds = micros();
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us)) {}
  }
  
  // FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  
  // Répartition sur 32 bandes
  for (int i = 0; i < NUM_BANDS; i++) {
    int startBin = (i * (SAMPLES / 4)) / NUM_BANDS + 1;
    int endBin = ((i + 1) * (SAMPLES / 4)) / NUM_BANDS + 1;
    
    double sum = 0;
    for (int j = startBin; j < endBin; j++) {
      sum += vReal[j];
    }
    
    int avg = sum / (endBin - startBin);
    bandValues[i] = map(avg, 0, 2000, 0, 8);
    bandValues[i] = constrain(bandValues[i], 0, 8);
  }
}

void displaySpectrum() {
  mx.clear();
  
  for (int band = 0; band < NUM_BANDS; band++) {
    int barHeight = bandValues[band];
    for (int y = 0; y < barHeight; y++) {
      mx.setPoint(7 - y, band, true);
    }
  }
  
  mx.update();
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Matrice LED
  myDisplay.begin();
  myDisplay.setIntensity(5);
  myDisplay.displayClear();
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 5);
  
  Serial.println("Matrice prête !");

  // Configuration ADC pour le microphone
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  // WiFi
  WiFi.mode(WIFI_STA);
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

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  Serial.println("\n=== COMMANDES MQTT ===");
  Serial.println("Topic: iot/demo/mode");
  Serial.println("  - 'spectre' ou 'audio' -> Mode spectre audio");
  Serial.println("  - 'texte' ou 'message' -> Mode texte");
  Serial.println("\nTopic: iot/demo");
  Serial.println("  - Envoyer un message à afficher");
  Serial.println("========================\n");
}

// =========================
// LOOP
// =========================
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  if (spectreMode) {
    // MODE SPECTRE AUDIO
    updateSpectrum();
    displaySpectrum();
  } else {
    // MODE TEXTE
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
}