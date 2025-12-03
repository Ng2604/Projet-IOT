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
// IMPORTANT : Utilise FC16_HW comme dans ton code original pour le texte
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
// Si le texte ne défile pas bien, essaie aussi :
// #define HARDWARE_TYPE MD_MAX72XX::PAROLA_HW
#define MAX_DEVICES 4
#define CLK_PIN   18
#define DATA_PIN  23
#define CS_PIN    19

// PIN MICROPHONE
#define MIC_PIN 34

// MD_Parola pour le texte MQTT (comme ton ancien code)
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
bool displayingText = false;  // Mode affichage : texte ou spectre

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
unsigned long textStartTime = 0;

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

    Serial.print("ClientID: ");
    Serial.print(clientId);

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

  // Forcer la première bin (DC/bruit) à 0
  vReal[0] = 0;
  vReal[1] = 0;

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
    
    // Seuil pour ignorer le bruit de fond
    if (maxVal < 50) {
      maxVal = 0;
    }
    
    // Normaliser sur 8 niveaux
    bandValues[i] = map(maxVal, 0, 1500, 0, 8);
    bandValues[i] = constrain(bandValues[i], 0, 8);
  }
}


// =========================
// AFFICHAGE SPECTRE SUR MATRICE
// =========================
void displaySpectrum() {
  // Clear AGRESSIF avant chaque frame
  for (int i = 0; i < MAX_DEVICES * 8; i++) {
    for (int j = 0; j < 8; j++) {
      mx.setPoint(j, i, false);
    }
  }
  
  // Dessiner le spectre sur TOUTES les colonnes (0 à 31)
  for (int col = 0; col < NUM_BANDS; col++) {
    int height = bandValues[col];
    
    // Ignorer bande 0 si saturée (le point fixe)
    if (col == 0 && height >= 7) {
      continue;
    }
    
    // Dessiner la barre de bas en haut
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

  // Configuration FFT
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQ));

  // Pin microphone
  pinMode(MIC_PIN, INPUT);

  // --------------------------
  //  MATRICE LED
  // --------------------------
  myDisplay.begin();  // MD_Parola en premier
  myDisplay.setIntensity(5);
  myDisplay.displayClear();
  
  mx.begin();  // MD_MAX72XX ensuite
  mx.clear();
  
  Serial.println("Matrice prête !");

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

  // Si message MQTT reçu, passer en mode texte
  if (newMessage) {
    displayingText = true;
    textStartTime = millis();
    
    // RESET complet avant passage en mode texte
    mx.clear();
    for (int i = 0; i < MAX_DEVICES * 8; i++) {
      for (int j = 0; j < 8; j++) {
        mx.setPoint(j, i, false);
      }
    }
    
    // Configurer MD_Parola pour utiliser TOUTE la matrice
    myDisplay.displayClear();
    myDisplay.setTextAlignment(PA_CENTER);
    myDisplay.setSpeed(50);
    
    // Afficher le texte avec MD_Parola sur TOUTE la matrice
    myDisplay.displayText(
      messageToDisplay.c_str(),
      PA_CENTER, 50, 0,
      PA_SCROLL_LEFT, PA_SCROLL_LEFT
    );
    
    newMessage = false;
    Serial.println("Mode TEXTE activé");
  }

  // Mode texte : animer le texte pendant 10 secondes
  if (displayingText) {
    bool animFinished = myDisplay.displayAnimate();
    
    // Vérifier si animation terminée OU timeout
    if (animFinished || millis() - textStartTime > 10000) {
      displayingText = false;
      
      // RESET COMPLET avant retour au spectre
      myDisplay.displayClear();
      delay(100);  // Petit délai pour laisser le clear se faire
      
      // Clear pixel par pixel pour être sûr
      mx.clear();
      for (int i = 0; i < MAX_DEVICES * 8; i++) {
        for (int j = 0; j < 8; j++) {
          mx.setPoint(j, i, false);
        }
      }
      
      Serial.println("Retour au spectre audio");
    }
  }
  // Mode spectre : afficher le spectre audio
  else {
    sampleAndAnalyzeAudio();
    displaySpectrum();
    
    // DEBUG : afficher les valeurs toutes les 2 secondes
    if (millis() - lastDebugTime > 2000) {
      Serial.print("Valeurs spectre: ");
      for (int i = 0; i < NUM_BANDS; i += 4) {
        Serial.printf("[%d]=%d ", i, bandValues[i]);
      }
      Serial.println();
      
      // Valeurs RAW pour calibration
      Serial.print("Valeurs RAW FFT (bins 10-20): ");
      for (int i = 10; i < 20; i++) {
        Serial.printf("%.0f ", vReal[i]);
      }
      Serial.println();
      
      lastDebugTime = millis();
    }
  }
  
  delay(50);
}