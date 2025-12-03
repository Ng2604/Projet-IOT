#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <DNSServer.h>
#include <WebServer.h>

#include <MD_MAX72xx.h>
#include <SPI.h>
#include <arduinoFFT.h>

// Essaie ces types un par un jusqu'à ce que ça marche :
// #define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define HARDWARE_TYPE MD_MAX72XX::PAROLA_HW
// #define HARDWARE_TYPE MD_MAX72XX::GENERIC_HW
// #define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
// #define HARDWARE_TYPE MD_MAX72XX::DR0CR0RD1_HW
#define MAX_DEVICES 4
#define CLK_PIN   18
#define DATA_PIN  23
#define CS_PIN    19

// PIN MICROPHONE
#define MIC_PIN 34

// UNIQUEMENT MD_MAX72XX (plus de MD_Parola qui interfère)
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// MQTT
const char* mqtt_server = "test.mosquitto.org";
const char* topic_sub = "iot/demo";
WiFiClient espClient;
PubSubClient client(espClient);

// Message MQTT
String messageToDisplay = "";
bool newMessage = false;

// FFT AUDIO - Configuration
#define SAMPLES 128
#define SAMPLING_FREQ 10000
#define NUM_BANDS 32

// Variables FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long samplingPeriod;
int bandValues[NUM_BANDS];
unsigned long lastDebugTime = 0;  // Pour le debug périodique

// Objet FFT
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);


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
    Serial.print("Connexion MQTT... ");
    String clientId = "ESP32_Matrix_";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(" -> Connecté !");
      client.subscribe(topic_sub);
    } else {
      Serial.print(" -> Echec, code = ");
      Serial.print(client.state());
      Serial.println(" -> retry dans 2s");
      delay(2000);
    }
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

  // IMPORTANT : Forcer la première bin (DC/bruit) à 0
  vReal[0] = 0;
  vReal[1] = 0;  // Et la deuxième aussi pour être sûr

  // Répartir les fréquences sur 32 bandes
  for (int i = 0; i < NUM_BANDS; i++) {
    int startBin = i * (SAMPLES / 2) / NUM_BANDS;
    int endBin = (i + 1) * (SAMPLES / 2) / NUM_BANDS;
    
    double maxVal = 0;
    for (int j = startBin; j < endBin; j++) {
      if (vReal[j] > maxVal) {
        maxVal = vReal[j];
      }
    }
    
    // AJOUT D'UN SEUIL pour ignorer le bruit de fond
    if (maxVal < 50) {  // Seuil réduit de 100 à 50 pour plus de sensibilité
      maxVal = 0;
    }
    
    // Normaliser sur 8 niveaux avec MEILLEURE sensibilité
    bandValues[i] = map(maxVal, 0, 1500, 0, 8);  // Réduit de 3000 à 1500
    bandValues[i] = constrain(bandValues[i], 0, 8);
  }
}


// =========================
// AFFICHAGE SPECTRE SUR MATRICE
// =========================
void displaySpectrum() {
  // CLEAR COMPLET - éteint TOUT
  mx.clear();
  
  // Dessiner le spectre
  for (int col = 0; col < NUM_BANDS; col++) {
    int height = bandValues[col];
    
    for (int row = 0; row < height; row++) {
      mx.setPoint(7 - row, col, true);
    }
  }
}


// =========================
// AFFICHAGE TEXTE SIMPLE
// =========================
void displayText(String text) {
  mx.clear();
  
  // Affichage simple du texte (première ligne, tronqué si trop long)
  mx.setFont(nullptr);  // Police par défaut
  
  // Afficher les premiers caractères (max 4 modules × 8 pixels)
  int maxChars = min((int)text.length(), 16);
  
  for (int i = 0; i < maxChars && i < 4; i++) {
    // Affichage basique caractère par caractère
    // (Pour un vrai scroll, il faudrait une lib dédiée)
    mx.setChar(i * 8, text[i]);
  }
}


// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Configuration FFT
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQ));

  // Pin microphone
  pinMode(MIC_PIN, INPUT);

  // Matrice LED - UNIQUEMENT mx
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2);  // Intensité basse
  
  // CLEAR AGRESSIF - éteindre chaque pixel individuellement
  for (int device = 0; device < MAX_DEVICES; device++) {
    for (int col = 0; col < 8; col++) {
      for (int row = 0; row < 8; row++) {
        mx.setPoint(row, device * 8 + col, false);
      }
    }
  }
  
  // Double clear pour être sûr
  mx.clear();
  mx.clear();
  
  Serial.println("Matrice prête !");
  
  // DEBUG : Afficher si des pixels sont encore allumés
  Serial.println("Test pixels après clear:");
  for (int col = 0; col < 8; col++) {
    for (int row = 0; row < 8; row++) {
      if (mx.getPoint(row, col)) {
        Serial.printf("Pixel allumé détecté à row=%d, col=%d\n", row, col);
      }
    }
  }

  // WiFi Manager
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

  Serial.println("🎵 Mode Spectre Audio activé !");
}


// =========================
// LOOP
// =========================
void loop() {
  // Maintenir MQTT
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Si message MQTT reçu
  if (newMessage) {
    displayText(messageToDisplay);
    delay(3000);  // Afficher 3 secondes
    mx.clear();
    newMessage = false;
  }

  // Mode spectre audio
  sampleAndAnalyzeAudio();
  displaySpectrum();
  
  // DEBUG : afficher les valeurs toutes les 2 secondes
  if (millis() - lastDebugTime > 2000) {
    Serial.print("Valeurs spectre: ");
    for (int i = 0; i < NUM_BANDS; i += 4) {  // Afficher 1 sur 4 pour lisibilité
      Serial.printf("[%d]=%d ", i, bandValues[i]);
    }
    Serial.println();
    
    // Afficher aussi les valeurs RAW max pour calibration
    Serial.print("Valeurs RAW FFT (bins 10-20): ");
    for (int i = 10; i < 20; i++) {
      Serial.printf("%.0f ", vReal[i]);
    }
    Serial.println();
    
    lastDebugTime = millis();
  }
  
  delay(50);
}