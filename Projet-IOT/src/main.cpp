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

// Essaie ces types un par un si FC16_HW ne marche pas :
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
// #define HARDWARE_TYPE MD_MAX72XX::PAROLA_HW
// #define HARDWARE_TYPE MD_MAX72XX::GENERIC_HW
// #define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
#define MAX_DEVICES 4
#define CLK_PIN   18
#define DATA_PIN  23
#define CS_PIN    19

// PIN MICROPHONE (selon ton schéma, j'utilise un ADC libre)
#define MIC_PIN 34  // ADC1_CH6 - Change si ton micro est sur un autre pin

MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
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

// Variables FFT (DOIVENT être déclarées avant l'objet FFT)
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long samplingPeriod;
int bandValues[NUM_BANDS];

// Objet FFT (APRÈS les variables)
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
    
    vReal[i] = analogRead(MIC_PIN);  // Lecture du micro
    vImag[i] = 0;
    
    // Attendre la prochaine période d'échantillonnage
    while (micros() < (currentMicros + samplingPeriod)) {
      // Attente précise
    }
  }

  // Calcul FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

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
    
    // Normaliser sur 8 niveaux (hauteur de la matrice)
    bandValues[i] = map(maxVal, 0, 2000, 0, 8);
    bandValues[i] = constrain(bandValues[i], 0, 8);
  }
}


// =========================
// AFFICHAGE SPECTRE SUR MATRICE
// =========================
void displaySpectrum() {
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);  // Désactive rafraîchissement auto
  mx.clear();  // Efface TOUT (devrait éteindre la barre problématique)
  
  for (int col = 0; col < NUM_BANDS; col++) {
    int height = bandValues[col];
    
    // Dessiner une colonne verticale pour chaque bande
    for (int row = 0; row < height; row++) {
      // Inverser l'ordre si nécessaire (essaie avec et sans cette ligne)
      int adjustedCol = col;  // Ou essaie : int adjustedCol = NUM_BANDS - 1 - col;
      
      mx.setPoint(7 - row, adjustedCol, true);  // 7-row pour inverser (bas vers haut)
    }
  }
  
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);  // Réactive le rafraîchissement
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

  // Matrice LED
  mx.begin();  // Initialise MD_MAX72XX pour le dessin pixel
  mx.control(MD_MAX72XX::INTENSITY, 3);  // Réduit l'intensité (0-15)
  mx.clear();  // Efface tout
  
  myDisplay.begin();
  myDisplay.setIntensity(3);  // Cohérence avec mx
  myDisplay.displayClear();
  Serial.println("Matrice prête !");

  // WiFi Manager (ton appairage conservé)
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
  // Maintenir MQTT connecté
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Si message MQTT reçu, afficher en mode texte pendant 5 secondes
  if (newMessage) {
    myDisplay.displayText(
      messageToDisplay.c_str(),
      PA_CENTER, 50, 0,
      PA_SCROLL_LEFT, PA_SCROLL_LEFT
    );
    
    unsigned long textStartTime = millis();
    while (millis() - textStartTime < 5000) {  // Afficher 5 secondes
      myDisplay.displayAnimate();
      client.loop();  // Maintenir MQTT pendant l'affichage
    }
    
    newMessage = false;
  }

  // Mode spectre audio par défaut
  sampleAndAnalyzeAudio();
  
  // Clear agressif pour éviter les pixels fantômes
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (int i = 0; i < MAX_DEVICES * 8; i++) {
    for (int j = 0; j < 8; j++) {
      mx.setPoint(j, i, false);  // Éteint TOUT pixel par pixel
    }
  }
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  
  displaySpectrum();
  
  delay(50);  // Rafraîchissement ~20 FPS
}