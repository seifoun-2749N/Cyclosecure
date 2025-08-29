#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h> // ESP32 built-in Bluetooth
#include <TinyGsmClient.h>

BluetoothSerial BTSerial;
#define gpsSerial Serial2    // Use hardware serial port
#define TINY_GSM_MODEM_SIM800    

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

// GPS data variables
float lattitude, longitude;

// Sensor data variables
float ax, ay, az;
float gx, gy, gz;

// Hardware serial for GSM
#define MODEM_RX 16
#define MODEM_TX 17
HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);

int mic;
const int boutonPin = 17;
const int buzz = 15;
const int ledg = 12;
const int ledr = 14;

bool AccelCrashDetected(float ax, float ay, float az);
bool GyroCrashDetected(float gx, float gy, float gz);
bool MicCrashDetected(float mic);
bool validation();

// Thresholds for detecting a crash
const float accelThreshold = 10.0;
const float gyroThreshold = 400.0;
const float micThreshold = 150;

void setupGSM() {
  SerialGSM.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  // Unlock SIM if PIN is required
  // modem.simUnlock("1234");
}

void sendSMS(String number, String message) {
  if (modem.sendSMS(number, message)) {
    Serial.println("✅ SMS sent successfully");
  } else {
    Serial.println("❌ SMS failed");
  }
}

void setup() {
  Serial.begin(115200);
  BTSerial.begin("CycleSecure"); // Bluetooth device name
  delay(100);

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
}

void loop() {
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
    String sms = "🚨 Crash Detected!\n";
    sms += "Latitude: " + String(latitude, 6) + "\n";
    sms += "Longitude: " + String(longitude, 6) + "\n";

    sendSMS("+216XXXXXXXX", sms);  // Replace with emergency number
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