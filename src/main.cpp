// Configuration GSM
#define TINY_GSM_MODEM_SIM800
#define GSM_RST_PIN 2 

#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// Firebase Configuration
const char FIREBASE_HOST[] = "cyclo-fef18-default-rtdb.europe-west1.firebasedatabase.app";
const char FIREBASE_AUTH[] = "uyuTjNPPVlddvEhG1rq11vdBN8tA6eJdgGJRHyGW";
const char FIREBASE_PATH[] = "/";
const int SSL_PORT = 443;
#define WIFI_SSID "TOPNET_1488"
#define WIFI_PASSWORD "zky4hl242i"

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Bluetooth and GPS objects
BluetoothSerial BTSerial;
#define gpsSerial Serial2
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

// User Authentication Variables
struct UserProfile {
  char deviceId[32];
  char userEmail[64];
  char emergencyContacts[5][20]; // Up to 5 emergency contacts
  int numContacts;
  char emergencyMessage[200];
  bool isConfigured;
};

UserProfile userProfile;
bool deviceRegistered = false;
bool configMode = false;

// GPS data variables
float lattitude = 0.0, longitude = 0.0;

// Sensor data variables
float ax, ay, az;
float gx, gy, gz;

// GSM Serial Configuration
#define MODEM_RX 16
#define MODEM_TX 17
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);

// Pins and variables
int mic;
const int boutonPin = 4;
const int buzz = 15;
const int ledg = 12;
const int ledr = 14;
const int configPin = 13; // New pin for configuration mode

// Firebase update variables
unsigned long lastFirebaseUpdate = 0;
const unsigned long firebaseUpdateInterval = 10000; // Update every 10 seconds
bool wifiConnected = false;

// Crash detection thresholds
const float accelThreshold = 10.0;
const float gyroThreshold = 400.0;
const float micThreshold = 150;

// EEPROM addresses
const int EEPROM_SIZE = 512;
const int USER_PROFILE_ADDR = 0;

// Function declarations
bool AccelCrashDetected(float ax, float ay, float az);
bool GyroCrashDetected(float gx, float gy, float gz);
bool MicCrashDetected(float mic);
bool validation();
void setupWiFi();
void setupFirebase();
void updateGPSToFirebase();
void updateSensorDataToFirebase();
void sendCrashDataToFirebase();
void setupGSM();
void sendSMS(String number, String message);
void makeCall(String number);
void hangUp();
void saveUserProfile();
void loadUserProfile();
void configurationMode();
void registerDeviceWithUser();
void loadUserSettingsFromFirebase();
String generateDeviceId();

void saveUserProfile() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(USER_PROFILE_ADDR, userProfile);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("User profile saved to EEPROM");
}

void loadUserProfile() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(USER_PROFILE_ADDR, userProfile);
  EEPROM.end();
  
  // Check if profile is valid
  if (strlen(userProfile.deviceId) == 0 || !userProfile.isConfigured) {
    Serial.println("No valid user profile found");
    userProfile.isConfigured = false;
    deviceRegistered = false;
  } else {
    Serial.println("User profile loaded from EEPROM");
    Serial.println("Device ID: " + String(userProfile.deviceId));
    Serial.println("User Email: " + String(userProfile.userEmail));
    Serial.println("Emergency Contacts: " + String(userProfile.numContacts));
    deviceRegistered = true;
  }
}

String generateDeviceId() {
  String deviceId = "CYCLO_";
  uint64_t chipid = ESP.getEfuseMac();
  deviceId += String((uint32_t)(chipid >> 32), HEX);
  deviceId += String((uint32_t)chipid, HEX);
  deviceId.toUpperCase();
  return deviceId;
}

void configurationMode() {
  Serial.println("\n=== CONFIGURATION MODE ===");
  Serial.println("Device ID: " + generateDeviceId());
  Serial.println("Please use the mobile app or web interface to configure this device");
  Serial.println("Send device registration data via Bluetooth...");
  
  // Generate and store device ID
  String devId = generateDeviceId();
  devId.toCharArray(userProfile.deviceId, sizeof(userProfile.deviceId));
  
  // Wait for configuration via Bluetooth
  unsigned long configTimeout = millis() + 300000; // 5 minute timeout
  
  while (millis() < configTimeout && !deviceRegistered) {
    if (BTSerial.available()) {
      String receivedData = BTSerial.readString();
      receivedData.trim();
      
      Serial.println("Received: " + receivedData);
      
      // Parse JSON configuration
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, receivedData);
      
      if (!error) {
        if (doc["command"] == "configure") {
          // Extract user email
          String userEmail = doc["email"];
          userEmail.toCharArray(userProfile.userEmail, sizeof(userProfile.userEmail));
          
          // Extract emergency contacts
          JsonArray contacts = doc["emergencyContacts"];
          userProfile.numContacts = min((int)contacts.size(), 5);
          
          for (int i = 0; i < userProfile.numContacts; i++) {
            String contact = contacts[i];
            contact.toCharArray(userProfile.emergencyContacts[i], sizeof(userProfile.emergencyContacts[i]));
          }
          
          // Extract emergency message
          String message = doc["emergencyMessage"];
          message.toCharArray(userProfile.emergencyMessage, sizeof(userProfile.emergencyMessage));
          
          userProfile.isConfigured = true;
          deviceRegistered = true;
          
          saveUserProfile();
          registerDeviceWithUser();
          
          BTSerial.println("{\"status\":\"success\",\"message\":\"Device configured successfully\"}");
          Serial.println("Device configured successfully!");
          
          break;
        }
      } else {
        Serial.println("Failed to parse JSON: " + String(error.c_str()));
        BTSerial.println("{\"status\":\"error\",\"message\":\"Invalid JSON format\"}");
      }
    }
    
    // Blink LED to indicate config mode
    digitalWrite(ledr, !digitalRead(ledr));
    delay(500);
  }
  
  if (!deviceRegistered) {
    Serial.println("Configuration timeout. Device will use default settings.");
    // Set default emergency contact and message
    strcpy(userProfile.emergencyContacts[0], "+21693937213");
    userProfile.numContacts = 1;
    strcpy(userProfile.emergencyMessage, "Emergency Alert! Accident detected. Location:");
    userProfile.isConfigured = false;
  }
}

void registerDeviceWithUser() {
  if (!wifiConnected) return;
  
  // Register device with user in Firebase
  FirebaseJson deviceJson;
  deviceJson.set("deviceId", userProfile.deviceId);
  deviceJson.set("userEmail", userProfile.userEmail);
  deviceJson.set("registeredAt", millis());
  deviceJson.set("status", "active");
  
  String devicePath = "/cyclesecure/devices/" + String(userProfile.deviceId);
  if (Firebase.setJSON(firebaseData, devicePath, deviceJson)) {
    Serial.println("Device registered with user successfully");
  } else {
    Serial.print("Failed to register device: ");
    Serial.println(firebaseData.errorReason());
  }
}

void loadUserSettingsFromFirebase() {
  if (!wifiConnected || !deviceRegistered) return;
  
  // Load user settings from Firebase
  String userPath = "/cyclesecure/users/";
  String emailPath = String(userProfile.userEmail);
  emailPath.replace(".", "_");
  emailPath.replace("@", "_");
  userPath += emailPath;
  
  if (Firebase.getJSON(firebaseData, userPath)) {
    FirebaseJson &json = firebaseData.jsonObject();
    
    // Update emergency contacts if available
    FirebaseJsonData contactsData;
    if (json.get(contactsData, "emergencyContacts")) {
      FirebaseJsonArray contactsArray;
      contactsData.getArray(contactsArray);
      
      userProfile.numContacts = min((int)contactsArray.size(), 5);
      for (int i = 0; i < userProfile.numContacts; i++) {
        FirebaseJsonData contactData;
        contactsArray.get(contactData, i);
        String contact = contactData.stringValue;
        contact.toCharArray(userProfile.emergencyContacts[i], sizeof(userProfile.emergencyContacts[i]));
      }
    }
    
    // Update emergency message if available
    FirebaseJsonData messageData;
    if (json.get(messageData, "emergencyMessage")) {
      String message = messageData.stringValue;
      message.toCharArray(userProfile.emergencyMessage, sizeof(userProfile.emergencyMessage));
    }
    
    saveUserProfile();
    Serial.println("User settings updated from Firebase");
  }
}

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
  
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Firebase initialized");
}

void updateGPSToFirebase() {
  if (!wifiConnected || lattitude == 0.0 || longitude == 0.0) return;
  
  FirebaseJson gpsJson;
  gpsJson.set("deviceId", userProfile.deviceId);
  gpsJson.set("latitude", lattitude);
  gpsJson.set("longitude", longitude);
  gpsJson.set("timestamp", millis());
  
  String gpsPath = "/cyclesecure/gps/" + String(userProfile.deviceId);
  if (Firebase.setJSON(firebaseData, gpsPath, gpsJson)) {
    Serial.println("GPS data updated to Firebase successfully");
  } else {
    Serial.print("Failed to update GPS data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void updateSensorDataToFirebase() {
  if (!wifiConnected) return;
  
  FirebaseJson sensorJson;
  sensorJson.set("deviceId", userProfile.deviceId);
  sensorJson.set("userEmail", userProfile.userEmail);
  sensorJson.set("accelerometer/x", ax);
  sensorJson.set("accelerometer/y", ay);
  sensorJson.set("accelerometer/z", az);
  sensorJson.set("gyroscope/x", gx);
  sensorJson.set("gyroscope/y", gy);
  sensorJson.set("gyroscope/z", gz);
  sensorJson.set("microphone", mic);
  sensorJson.set("timestamp", millis());
  
  String sensorPath = "/cyclesecure/sensors/" + String(userProfile.deviceId);
  if (Firebase.setJSON(firebaseData, sensorPath, sensorJson)) {
    Serial.println("Sensor data updated to Firebase successfully");
  } else {
    Serial.print("Failed to update sensor data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void sendCrashDataToFirebase() {
  if (!wifiConnected) return;
  
  FirebaseJson crashJson;
  crashJson.set("deviceId", userProfile.deviceId);
  crashJson.set("userEmail", userProfile.userEmail);
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
  
  String crashPath = "/cyclesecure/alerts/" + String(userProfile.deviceId) + "_" + String(millis());
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

  int baudRates[] = {9600, 115200};
  int numRates = sizeof(baudRates) / sizeof(baudRates[0]);
  
  for (int i = 0; i < numRates; i++) {
    Serial.print("Testing baud rate: ");
    Serial.println(baudRates[i]);
    
    SerialGSM.begin(baudRates[i], SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(2000);
    
    while (SerialGSM.available()) {
      SerialGSM.read();
    }
    
    for (int attempt = 0; attempt < 3; attempt++) {
      Serial.print("  Attempt ");
      Serial.print(attempt + 1);
      Serial.print(": ");
      
      SerialGSM.println("AT");
      delay(1000);
      
      String response = "";
      unsigned long timeout = millis() + 2000;
      bool gotResponse = false;
      
      while (millis() < timeout) {
        if (SerialGSM.available()) {
          char c = SerialGSM.read();
          response += c;
          gotResponse = true;
        }
        delay(10);
      }
      
      if (gotResponse && response.indexOf("OK") >= 0) {
        Serial.println("SUCCESS! GSM modem is responding!");
        modem.gprsConnect("internet.ooredoo.tn", "", "");
        return;
      }
      
      delay(500);
    }
    
    SerialGSM.end();
    delay(1000);
  }
}

void sendSMS(String number, String message) {
  if (modem.sendSMS(number, message)) {
    Serial.println("SMS sent successfully to " + number);
  } else {
    Serial.println("SMS failed to " + number);
  }
}

void makeCall(String number) {
  Serial.println("Dialing " + number);
  if (modem.callNumber(number)) {
    Serial.println("Call started");
  } else {
    Serial.println("Call failed");
  }
}

void hangUp() {
  modem.callHangup();
  Serial.println("Call ended");
}

void setup() {
  Serial.begin(115200);
  BTSerial.begin("CycleSecure_" + generateDeviceId());
  delay(100);

  // Initialize pins
  pinMode(boutonPin, INPUT_PULLUP);
  pinMode(configPin, INPUT_PULLUP);
  pinMode(buzz, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledr, OUTPUT);
  pinMode(GSM_RST_PIN, OUTPUT);

  // Reset GSM module
  digitalWrite(GSM_RST_PIN, LOW);
  delay(100);
  digitalWrite(GSM_RST_PIN, HIGH);
  delay(3000);

  // Load user profile
  loadUserProfile();

  // Check if configuration button is pressed
  if (digitalRead(configPin) == LOW || !userProfile.isConfigured) {
    configMode = true;
    digitalWrite(ledr, HIGH);
    digitalWrite(ledg, LOW);
  }

  // Setup WiFi and Firebase
  setupWiFi();
  setupFirebase();
  
  if (configMode || !deviceRegistered) {
    configurationMode();
  }

  // Load updated settings from Firebase
  if (deviceRegistered) {
    loadUserSettingsFromFirebase();
  }

  setupGSM();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  digitalWrite(ledg, HIGH);
  digitalWrite(ledr, LOW);
  
  Serial.println("CycleSecure System Ready!");
  Serial.println("Device ID: " + String(userProfile.deviceId));
  if (deviceRegistered) {
    Serial.println("Registered to: " + String(userProfile.userEmail));
    Serial.println("Emergency Contacts: " + String(userProfile.numContacts));
    for (int i = 0; i < userProfile.numContacts; i++) {
      Serial.println("  Contact " + String(i+1) + ": " + String(userProfile.emergencyContacts[i]));
    }
  }
}

void loop() {
  // Check for configuration button press
  if (digitalRead(configPin) == LOW) {
    delay(50); // Debounce
    if (digitalRead(configPin) == LOW) {
      Serial.println("Configuration button pressed");
      configMode = true;
      configurationMode();
    }
  }

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

  Serial.print(" / Gyro (°/s): X = ");
  Serial.print(gx);
  Serial.print(" Y = ");
  Serial.print(gy);
  Serial.print(" Z = ");
  Serial.print(gz);

  mic = analogRead(A3);
  Serial.print(" / Mic : ");
  Serial.println(mic);

  // Update Firebase data periodically
  if (millis() - lastFirebaseUpdate > firebaseUpdateInterval) {
    if (deviceRegistered) {
      updateGPSToFirebase();
      updateSensorDataToFirebase();
      // Periodically check for updated settings
      if (millis() % 60000 == 0) { // Every minute
        loadUserSettingsFromFirebase();
      }
    }
    lastFirebaseUpdate = millis();
  }

  // Check for crash conditions
  if (AccelCrashDetected(ax, ay, az) || GyroCrashDetected(gx, gy, gz) || MicCrashDetected(mic)) {
    Serial.println("Crash detected!");
    BTSerial.println("CRASH DETECTED!");
    BTSerial.println("GPS Data :");
    BTSerial.print("Latitude: ");
    BTSerial.println(lattitude);
    BTSerial.print("Longitude: ");
    BTSerial.println(longitude);
    
    // Send crash data to Firebase
    sendCrashDataToFirebase();
    
    // Prepare emergency message with location
    String emergencyMsg = String(userProfile.emergencyMessage) + "\n";
    emergencyMsg += "Latitude: " + String(lattitude, 6) + "\n";
    emergencyMsg += "Longitude: " + String(longitude, 6) + "\n";
    emergencyMsg += "Device: " + String(userProfile.deviceId);
    
    // Send SMS and call all emergency contacts
    for (int i = 0; i < userProfile.numContacts; i++) {
      String contactNum = String(userProfile.emergencyContacts[i]);
      Serial.println("Alerting contact " + String(i+1) + ": " + contactNum);
      
      sendSMS(contactNum, emergencyMsg);
      delay(2000); // Wait between messages
      
      makeCall(contactNum);
      delay(20000); // 20s ringing
      hangUp();
      delay(5000); // Wait before next contact
    }
  }

  delay(500);
}

bool validation() {
  bool shouldExit = false;
  bool lastButtonState = HIGH;

  for (int i = 0; i < 5; i++) {
    digitalWrite(buzz, HIGH);
    digitalWrite(ledr, HIGH);
    digitalWrite(ledg, LOW);
    unsigned long startOnTime = millis();
    while (millis() - startOnTime < 500) {
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
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  if (accelMagnitude > accelThreshold) {
    Serial.println("Acceleration threshold exceeded!");
    return validation();
  }
  return false;
}

bool GyroCrashDetected(float gx, float gy, float gz) {
  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);
  if (gyroMagnitude > gyroThreshold) {
    Serial.println("Gyroscope threshold exceeded!");
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