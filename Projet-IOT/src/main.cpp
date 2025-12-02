#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

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

// Configuration FFT simplifiée (sans librairie externe)
#define SAMPLES 64  // Réduit pour ESP32
#define SAMPLING_FREQUENCY 8000
#define NUM_BANDS 32

unsigned int sampling_period_us;
int bandValues[NUM_BANDS] = {0};

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";
const char* topic_mode = "iot/demo/mode";

WiFiClient espClient;
PubSubClient client(espClient);

String messageToDisplay = "";
bool newMessage = false;

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
    
    if (client.connect(clientId.c_str())) {
      Serial.println(" -> Connecté !");
      client.subscribe(topic_sub);
      client.subscribe(topic_mode);
      Serial.println("Souscrit à: " + String(topic_sub) + " et " + String(topic_mode));
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
  // On divise les échantillons en bandes et on calcule l'amplitude moyenne
  int samplesPerBand = SAMPLES / NUM_BANDS;
  
  for (int band = 0; band < NUM_BANDS; band++) {
    int startIdx = band * samplesPerBand;
    int endIdx = startIdx + samplesPerBand;
    
    // Calculer l'amplitude moyenne de la bande
    long sum = 0;
    int minVal = 4095;
    int maxVal = 0;
    
    for (int i = startIdx; i < endIdx && i < SAMPLES; i++) {
      if (samples[i] < minVal) minVal = samples[i];
      if (samples[i] > maxVal) maxVal = samples[i];
      sum += abs(samples[i] - 2048); // Centrer autour de 2048 (milieu de 12 bits)
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
      mx.setPoint(7 - y, band, true);  // 7-y pour inverser (bas vers haut)
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
  
  Serial.println("\n\n=================================");
  Serial.println("ESP32 - Matrice LED + Spectre Audio");
  Serial.println("=================================\n");

  // Matrice LED
  myDisplay.begin();
  myDisplay.setIntensity(5);
  myDisplay.displayClear();
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 5);
  
  Serial.println("✓ Matrice LED prête !");

  // Configuration ADC pour le microphone
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  Serial.println("✓ Microphone configuré sur GPIO34");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  
  // Message de bienvenue sur la matrice
  myDisplay.displayText("WiFi...", PA_CENTER, 50, 0, PA_PRINT, PA_NO_EFFECT);
  myDisplay.displayAnimate();
  while (!myDisplay.displayAnimate()) {
    delay(10);
  }
  
  wm.setAPCallback([](WiFiManager *myWM) {
    Serial.println("\n=== MODE CONFIGURATION ===");
    Serial.println("Connectez-vous au WiFi : ESP32_Setup");
    Serial.println("Mot de passe : 12345678");
    Serial.println("==========================\n");
  });

  bool res = wm.autoConnect("ESP32_Setup", "12345678");
  
  if (!res) {
    Serial.println("⚠️ ÉCHEC de l'appairage WiFi");
    myDisplay.displayText("WiFi Error", PA_CENTER, 50, 0, PA_PRINT, PA_NO_EFFECT);
    delay(3000);
    ESP.restart();
  }

  Serial.println("✓ WiFi connecté !");
  Serial.print("  IP: ");
  Serial.println(WiFi.localIP());
  
  // Message de succès
  myDisplay.displayText("Connected!", PA_CENTER, 50, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  for (int i = 0; i < 50; i++) {
    myDisplay.displayAnimate();
    delay(50);
  }
  myDisplay.displayClear();

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  Serial.println("\n=== COMMANDES MQTT ===");
  Serial.println("Topic: " + String(topic_mode));
  Serial.println("  - 'spectre' ou 'audio' -> Mode spectre audio");
  Serial.println("  - 'texte' ou 'message' -> Mode texte");
  Serial.println("\nTopic: " + String(topic_sub));
  Serial.println("  - Envoyer un message à afficher");
  Serial.println("  - Ou 'spectre'/'texte' pour changer de mode");
  Serial.println("========================\n");
  
  Serial.println("✓ Système prêt !\n");
}

// =========================
// LOOP
// =========================
void loop() {
  // Maintenir la connexion MQTT
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
      Serial.println("[TEXTE] Affichage: " + messageToDisplay);
      myDisplay.displayText(
        messageToDisplay.c_str(),
        PA_CENTER, 50, 0,
        PA_SCROLL_LEFT, PA_SCROLL_LEFT
      );
      newMessage = false;
    }
    
    // Animation du texte
    if (myDisplay.displayAnimate()) {
      // L'animation est terminée, on peut réafficher le message
      delay(1000); // Pause de 1s avant de recommencer
      if (messageToDisplay.length() > 0) {
        myDisplay.displayReset();
      }
    }
    
    delay(10); // Petit délai pour ne pas saturer le CPU
  }
}