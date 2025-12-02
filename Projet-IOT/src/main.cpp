#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <DNSServer.h>
#include <WebServer.h>

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Configuration de la matrice LED
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   18  // SCK (vert) - GPIO18
#define DATA_PIN  23  // DIN (cyan) - GPIO23  
#define CS_PIN    19  // CS (orange) - GPIO19

MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Configuration du microphone
#define MIC_PIN 34

// Configuration pour le spectre audio simplifié
#define SAMPLES 64
#define SAMPLING_FREQUENCY 8000
#define NUM_BANDS 32

unsigned int sampling_period_us;
int bandValues[NUM_BANDS] = {0};

// Message MQTT
String messageToDisplay = "";
bool newMessage = false;

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";
const char* topic_mode = "iot/demo/mode";

WiFiClient espClient;
PubSubClient client(espClient);

// MODE: true = Spectre Audio, false = Affichage texte
bool spectreMode = false;  // Démarrer en mode texte par défaut

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

  // Changement de mode via topic dédié
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
  // Message à afficher OU commande de mode sur topic principal
  else if (topicStr == topic_sub) {
    // Vérifier si c'est une commande de mode
    if (msg == "spectre" || msg == "audio") {
      spectreMode = true;
      Serial.println("→ Mode SPECTRE AUDIO activé");
    } else if (msg == "texte" || msg == "message") {
      spectreMode = false;
      myDisplay.displayClear();
      Serial.println("→ Mode TEXTE activé");
    } else {
      // C'est un message normal à afficher
      messageToDisplay = msg;
      newMessage = true;
      spectreMode = false; // Passe en mode texte automatiquement
      Serial.println("→ Message à afficher: " + msg);
    }
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

    Serial.print("ClientID: ");
    Serial.print(clientId);

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
// SPECTRE AUDIO SIMPLIFIÉ
// =========================
void updateSpectrum() {
  int samples[SAMPLES];
  
  // Échantillonnage rapide
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(MIC_PIN);
    delayMicroseconds(sampling_period_us);
  }
  
  // Analyse simple par bandes de fréquence
  int samplesPerBand = SAMPLES / NUM_BANDS;
  
  for (int band = 0; band < NUM_BANDS; band++) {
    int startIdx = band * samplesPerBand;
    int endIdx = startIdx + samplesPerBand;
    
    // Calculer l'amplitude moyenne de la bande
    int minVal = 4095;
    int maxVal = 0;
    
    for (int i = startIdx; i < endIdx && i < SAMPLES; i++) {
      if (samples[i] < minVal) minVal = samples[i];
      if (samples[i] > maxVal) maxVal = samples[i];
    }
    
    // Amplitude = différence max-min (détection de pics)
    int amplitude = maxVal - minVal;
    
    // Mapper sur 0-8 (hauteur de la matrice)
    bandValues[band] = map(amplitude, 0, 2000, 0, 8);
    bandValues[band] = constrain(bandValues[band], 0, 8);
  }
}

void displaySpectrum() {
  mx.clear();
  
  // Afficher les barres du spectre
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

  // --------------------------
  //  MATRICE LED
  // --------------------------
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

  // --------------------------
  //  APPAIRAGE WiFi AUTOMATIQUE
  // --------------------------
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

  if (spectreMode) {
    // =============================
    // MODE SPECTRE AUDIO
    // =============================
    updateSpectrum();
    displaySpectrum();
    delay(50); // ~20 FPS pour le spectre
    
  } else {
    // =============================
    // MODE TEXTE
    // =============================
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