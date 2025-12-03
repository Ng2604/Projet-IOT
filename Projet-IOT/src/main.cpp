#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <DNSServer.h>
#include <WebServer.h>

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <arduinoFFT.h>

// Configuration matrice
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   18
#define DATA_PIN  23
#define CS_PIN    19

// PIN MICROPHONE
#define MIC_PIN 34

// MD_Parola pour le texte MQTT
MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// MD_MAX72XX pour le spectre audio
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";
WiFiClient espClient;
PubSubClient client(espClient);

// Message MQTT
String messageToDisplay = "";
bool newMessage = false;

// MODES DE FONCTIONNEMENT
enum Mode {
  MODE_AFFICHAGE,  // Mode affichage LED (par défaut)
  MODE_MICRO       // Mode spectre audio
};

Mode currentMode = MODE_AFFICHAGE;  // Mode par défaut
bool displayingText = false;
unsigned long textStartTime = 0;

// FFT AUDIO - Configuration
#define SAMPLES 128
#define SAMPLING_FREQ 10000
#define NUM_BANDS 32

// Variables FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long samplingPeriod;
int bandValues[NUM_BANDS];
unsigned long lastDebugTime = 0;
unsigned long lastMqttCheck = 0;

// Objet FFT
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);


// =========================
// MQTT CALLBACK - OPTIMISÉ
// =========================
void callback(char* topic, byte* message, unsigned int length) {
  messageToDisplay = "";
  messageToDisplay.reserve(length + 1);  // Pré-allouer la mémoire

  for (int i = 0; i < length; i++) {
    messageToDisplay += (char)message[i];
  }

  Serial.print("📥 MQTT reçu: ");
  Serial.println(messageToDisplay);

  // ========================================
  // GESTION DES COMMANDES MODE
  // ========================================
  if (messageToDisplay == "MODE:AFFICHAGE") {
    currentMode = MODE_AFFICHAGE;
    displayingText = false;
    myDisplay.displayClear();
    mx.clear();
    Serial.println("✅ Mode AFFICHAGE activé");
    return;
  }
  
  if (messageToDisplay == "MODE:MICRO") {
    currentMode = MODE_MICRO;
    displayingText = false;
    myDisplay.displayClear();
    mx.clear();
    Serial.println("✅ Mode MICRO activé");
    return;
  }

  // ========================================
  // MESSAGE TEXTE (seulement en mode AFFICHAGE)
  // ========================================
  if (currentMode == MODE_AFFICHAGE) {
    newMessage = true;
  } else {
    Serial.println("⚠️ Message ignoré (mode MICRO)");
  }
}


// =========================
// RECONNEXION MQTT - OPTIMISÉE
// =========================
void reconnectMQTT() {
  if (client.connected()) return;  // Déjà connecté
  
  Serial.print("Connexion MQTT... ");
  
  String clientId = "ESP32_";
  clientId += String(ESP.getEfuseMac(), HEX);  // ID unique basé sur MAC
  
  if (client.connect(clientId.c_str())) {
    Serial.println("✅ Connecté");
    client.subscribe(topic_sub);
  } else {
    Serial.print("❌ Échec (");
    Serial.print(client.state());
    Serial.println(")");
  }
}


// =========================
// LECTURE & ANALYSE AUDIO
// =========================
void sampleAndAnalyzeAudio() {
  // Échantillonnage audio
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long currentMicros = micros();
    
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    
    while (micros() < (currentMicros + samplingPeriod)) {
      // Attente précise
    }
  }

  // Calcul FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Forcer DC à 0
  vReal[0] = 0;
  vReal[1] = 0;

  // Répartir sur 32 bandes
  for (int i = 0; i < NUM_BANDS; i++) {
    int startBin = i * (SAMPLES / 2) / NUM_BANDS;
    int endBin = (i + 1) * (SAMPLES / 2) / NUM_BANDS;
    
    double maxVal = 0;
    for (int j = startBin; j < endBin; j++) {
      if (vReal[j] > maxVal) {
        maxVal = vReal[j];
      }
    }
    
    if (maxVal < 50) maxVal = 0;
    
    bandValues[i] = map(maxVal, 0, 1500, 0, 8);
    bandValues[i] = constrain(bandValues[i], 0, 8);
  }
}


// =========================
// AFFICHAGE SPECTRE
// =========================
void displaySpectrum() {
  mx.clear();
  
  for (int col = 0; col < NUM_BANDS; col++) {
    int height = bandValues[col];
    
    if (col == 0 && height >= 7) continue;
    
    for (int row = 0; row < height; row++) {
      mx.setPoint(7 - row, col, true);
    }
  }
}


// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n🚀 ESP32 IoT - DÉMARRAGE");

  // Configuration FFT
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQ));
  pinMode(MIC_PIN, INPUT);

  // Matrice LED
  myDisplay.begin();
  myDisplay.setIntensity(8);
  myDisplay.setInvert(false);
  myDisplay.displayClear();
  
  mx.begin();
  mx.clear();
  
  Serial.println("✅ Matrice prête");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Désactiver le sleep WiFi pour latence minimale
  
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);  // Timeout 3 minutes

  wm.setAPCallback([](WiFiManager *myWM) {
    Serial.println("\n📱 WiFi: ESP32_Setup / 12345678");
  });

  if (!wm.autoConnect("ESP32_Setup", "12345678")) {
    Serial.println("⚠️ Timeout WiFi, reboot...");
    delay(1000);
    ESP.restart();
  }

  Serial.println("✅ WiFi connecté");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());

  // MQTT avec optimisations
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Paramètres optimisés pour latence minimale
  client.setKeepAlive(10);
  client.setSocketTimeout(3);
  client.setBufferSize(512);
  
  Serial.println("✅ Configuration terminée");
  Serial.println("📋 Mode: AFFICHAGE par défaut\n");
}


// =========================
// LOOP - ULTRA OPTIMISÉE
// =========================
void loop() {
  // Vérifier MQTT fréquemment (toutes les 10ms max)
  unsigned long now = millis();
  if (now - lastMqttCheck >= 10) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
    lastMqttCheck = now;
  }

  // ========================================
  // MODE AFFICHAGE
  // ========================================
  if (currentMode == MODE_AFFICHAGE) {
    
    // Nouveau message : l'afficher immédiatement
    if (newMessage) {
      displayingText = true;
      textStartTime = millis();
      
      myDisplay.displayClear();
      myDisplay.setTextAlignment(PA_CENTER);
      myDisplay.setSpeed(40);
      myDisplay.setPause(0);
      myDisplay.setScrollSpacing(1);
      
      myDisplay.displayText(
        messageToDisplay.c_str(),
        PA_LEFT,
        50,
        0,
        PA_SCROLL_LEFT,
        PA_SCROLL_LEFT
      );
      
      newMessage = false;
      Serial.println("✅ Message affiché");
    }

    // Animer le texte
    if (displayingText) {
      bool animFinished = myDisplay.displayAnimate();
      
      if (animFinished || millis() - textStartTime > 10000) {
        displayingText = false;
        myDisplay.displayClear();
        mx.clear();
      }
    }
    
    // PAS DE DELAY en mode affichage = réactivité maximale
    yield();
  }
  
  // ========================================
  // MODE MICRO
  // ========================================
  else if (currentMode == MODE_MICRO) {
    
    sampleAndAnalyzeAudio();
    displaySpectrum();
    
    // Debug périodique
    if (millis() - lastDebugTime > 3000) {
      Serial.print("🎵 ");
      for (int i = 0; i < NUM_BANDS; i += 4) {
        Serial.printf("[%d]=%d ", i, bandValues[i]);
      }
      Serial.println();
      lastDebugTime = millis();
    }
    
    delay(30);
  }
}