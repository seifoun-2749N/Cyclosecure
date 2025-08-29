// Configuration GSM
#define TINY_GSM_MODEM_SIM800

#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

const char FIREBASE_HOST[] = "cyclo-fef18-default-rtdb.europe-west1.firebasedatabase.app";
const char FIREBASE_AUTH[] = "uyuTjNPPVlddvEhG1rq11vdBN8tA6eJdgGJRHyGW";
const char FIREBASE_PATH[] = "/";
const int SSL_PORT = 443;
#define WIFI_SSID "Cyclosecure"
#define WIFI_PASSWORD "Cyclosecure2025"

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Objets Bluetooth et GPS
BluetoothSerial BTSerial;
#define gpsSerial Serial2
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

// Variables de données GPS
float lattitude = 0.0, longitude = 0.0;

// Variables de données capteurs
float ax, ay, az;
float gx, gy, gz;

// Configuration série matérielle pour GSM
#define MODEM_RX 16
#define MODEM_TX 17
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);

// Pins et variables
int mic;
const int boutonPin = 4;
const int buzz = 15;
const int ledg = 12;
const int ledr = 14;

// Variables for Firebase updates
unsigned long lastFirebaseUpdate = 0;
const unsigned long firebaseUpdateInterval = 10000; // Update every 10 seconds
bool wifiConnected = false;

// Déclarations de fonctions
bool AccelCrashDetected(float ax, float ay, float az);
bool GyroCrashDetected(float gx, float gy, float gz);
bool MicCrashDetected(float mic);
bool validation();
void setupWiFi();
void setupFirebase();
void updateGPSToFirebase();
void updateSensorDataToFirebase();
void sendCrashDataToFirebase();

// Seuils pour la détection d'accidents
const float accelThreshold = 10.0;
const float gyroThreshold = 400.0;
const float micThreshold = 150;

void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println("\nFailed to connect to WiFi");
    wifiConnected = false;
  }
}

void setupFirebase() {
  if (!wifiConnected) return;
  
  // Firebase configuration
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  
  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Firebase initialized");
}

void updateGPSToFirebase() {
  if (!wifiConnected || lattitude == 0.0 || longitude == 0.0) return;
  
  // Create JSON object for GPS data
  FirebaseJson gpsJson;
  gpsJson.set("latitude", lattitude);
  gpsJson.set("longitude", longitude);
  gpsJson.set("timestamp", millis());
  
  // Update GPS data in Firebase
  String gpsPath = "/cyclesecure/gps";
  if (Firebase.setJSON(firebaseData, gpsPath, gpsJson)) {
    Serial.println("GPS data updated to Firebase successfully");
  } else {
    Serial.print("Failed to update GPS data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void updateSensorDataToFirebase() {
  if (!wifiConnected) return;
  
  // Create JSON object for sensor data
  FirebaseJson sensorJson;
  sensorJson.set("accelerometer/x", ax);
  sensorJson.set("accelerometer/y", ay);
  sensorJson.set("accelerometer/z", az);
  sensorJson.set("gyroscope/x", gx);
  sensorJson.set("gyroscope/y", gy);
  sensorJson.set("gyroscope/z", gz);
  sensorJson.set("microphone", mic);
  sensorJson.set("timestamp", millis());
  
  // Update sensor data in Firebase
  String sensorPath = "/cyclesecure/sensors";
  if (Firebase.setJSON(firebaseData, sensorPath, sensorJson)) {
    Serial.println("Sensor data updated to Firebase successfully");
  } else {
    Serial.print("Failed to update sensor data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void sendCrashDataToFirebase() {
  if (!wifiConnected) return;
  
  // Create JSON object for crash data
  FirebaseJson crashJson;
  crashJson.set("alert", true);
  crashJson.set("latitude", lattitude);
  crashJson.set("longitude", longitude);
  crashJson.set("accelerometer/x", ax);
  crashJson.set("accelerometer/y", ay);
  crashJson.set("accelerometer/z", az);
  crashJson.set("gyroscope/x", gx);
  crashJson.set("gyroscope/y", gy);
  crashJson.set("gyroscope/z", gz);
  crashJson.set("microphone", mic);
  crashJson.set("timestamp", millis());
  
  // Send crash alert to Firebase
  String crashPath = "/cyclesecure/alerts/" + String(millis());
  if (Firebase.setJSON(firebaseData, crashPath, crashJson)) {
    Serial.println("Crash alert sent to Firebase successfully");
  } else {
    Serial.print("Failed to send crash alert: ");
    Serial.println(firebaseData.errorReason());
  }
}

void setupGSM() {
  SerialGSM.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  // Remplacez par votre APN réel
  modem.gprsConnect("internet.tn", "", ""); // APN pour Tunisie Telecom
  
  // Décommentez si un PIN est requis
  // modem.simUnlock("1234");
}

void sendSMS(String number, String message) {
  if (modem.sendSMS(number, message)) {
    Serial.println("✅ SMS sent successfully");
  } else {
    Serial.println("❌ SMS failed");
  }
}

void makeCall(String number) {
  Serial.println("📞 Dialing " + number);
  if (modem.callNumber(number)) {
    Serial.println("✅ Call started");
  } else {
    Serial.println("❌ Call failed");
  }
}

void hangUp() {
  modem.callHangup();
  Serial.println("📴 Call ended");
}

void setup() {
  Serial.begin(115200);
  BTSerial.begin("CycleSecure"); // Nom du dispositif Bluetooth
  delay(100);

  // Setup WiFi and Firebase first
  setupWiFi();
  setupFirebase();
  
  setupGSM();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  gpsSerial.begin(9600);
  pinMode(boutonPin, INPUT_PULLUP);
  pinMode(buzz, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledr, OUTPUT);

  digitalWrite(ledg, HIGH);
  digitalWrite(ledr, LOW);
  
  Serial.println("CycleSecure System Ready!");
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    setupWiFi();
  }

  // Get GPS data
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      lattitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.println(lattitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
    }
  }

  // Get sensor events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Convert acceleration from m/s² to g's
  ax = accel.acceleration.x / 9.80665;
  ay = accel.acceleration.y / 9.80665;
  az = accel.acceleration.z / 9.80665;

  gx = gyro.gyro.x * 57.2958;
  gy = gyro.gyro.y * 57.2958;
  gz = gyro.gyro.z * 57.2958;

  Serial.print("Accel (g): X = ");
  Serial.print(ax);
  Serial.print(" Y = ");
  Serial.print(ay);
  Serial.print(" Z = ");
  Serial.print(az);

  Serial.print(" / Gyro (g): X = ");
  Serial.print(gx);
  Serial.print(" Y = ");
  Serial.print(gy);
  Serial.print(" Z = ");
  Serial.print(gz);

  mic = analogRead(A3);
  Serial.print(" / Mic : ");
  Serial.println(mic);

  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    Serial.print("Button State: ");
    Serial.println(digitalRead(boutonPin));
    lastDebugTime = millis();
  }

  // Update Firebase data periodically
  if (millis() - lastFirebaseUpdate > firebaseUpdateInterval) {
    updateGPSToFirebase();
    updateSensorDataToFirebase();
    lastFirebaseUpdate = millis();
  }

  // Check for crash conditions
  if (AccelCrashDetected(ax, ay, az) || GyroCrashDetected(gx, gy, gz) || MicCrashDetected(mic)) {
    Serial.println("Crash detected!");
    BTSerial.println("A crash is Detected !");
    BTSerial.println("GPS Data :");
    BTSerial.print("Latitude: ");
    BTSerial.println(lattitude);
    BTSerial.print("Longitude: ");
    BTSerial.println(longitude);
    BTSerial.print("Accelerometer data :");
    BTSerial.print("Ax: ");
    BTSerial.print(ax);
    BTSerial.print(", Ay: ");
    BTSerial.print(ay);
    BTSerial.print(", Az: ");
    BTSerial.println(az);
    
    // Send crash data to Firebase
    sendCrashDataToFirebase();
    
    String sms = "🚨 Crash Detected!\n";
    sms += "Latitude: " + String(lattitude, 6) + "\n";
    sms += "Longitude: " + String(longitude, 6) + "\n";

    sendSMS("+216XXXXXXXX", sms);  // Replace with emergency number
    makeCall("+216XXXXXXXX");   // Emergency contact number
    delay(20000);  // 20s ringing
    hangUp();
  }

  delay(500);
}

// The following functions remain unchanged from original code
bool validation() {
  bool shouldExit = false;
  bool lastButtonState = HIGH; // Track previous state

  for (int i = 0; i < 5; i++) {
    digitalWrite(buzz, HIGH);
    digitalWrite(ledr, HIGH);
    digitalWrite(ledg, LOW);
    unsigned long startOnTime = millis();
    while (millis() - startOnTime < 500) {
      bool currentButtonState = digitalRead(boutonPin);
      // Detect falling edge (HIGH -> LOW)
      if (lastButtonState == HIGH && currentButtonState == LOW) {
        delay(50); // Debounce
        if (digitalRead(boutonPin) == LOW) {
          shouldExit = true;
          break;
        }
      }
      lastButtonState = currentButtonState;
    }
    digitalWrite(buzz, LOW);
    digitalWrite(ledr, LOW);

    if (shouldExit) {
      Serial.println(" / No crash (button pressed)");
      return false;
    }

    unsigned long startOffTime = millis();
    while (millis() - startOffTime < 500) {
      bool currentButtonState = digitalRead(boutonPin);
      if (lastButtonState == HIGH && currentButtonState == LOW) {
        delay(50);
        if (digitalRead(boutonPin) == LOW) {
          shouldExit = true;
          break;
        }
      }
      lastButtonState = currentButtonState;
    }

    if (shouldExit) {
      Serial.println(" / No crash (button pressed)");
      return false;
    }
  }

  Serial.println(" / Crash confirmed");
  return true;
}

bool AccelCrashDetected(float ax, float ay, float az) {
  // Calculate the magnitude of the acceleration vector
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);

  // Check if the acceleration exceeds the threshold
  if (accelMagnitude > accelThreshold) {
    Serial.println("Acceleration threshold exceeded!");
    return validation();
  }
  return false;
}

bool GyroCrashDetected(float gx, float gy, float gz) {
  // Calculate the magnitude of the acceleration vector
  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

  // Check if the acceleration exceeds the threshold
  if (gyroMagnitude > gyroThreshold) {
    Serial.println("Acceleration threshold exceeded!");
    return validation();
  }
  return false;
}

bool MicCrashDetected(float mic) {
  if (mic > micThreshold) {
    Serial.println("Microphone threshold exceeded!");
    return validation();
  }
  return false;
}