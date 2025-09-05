# 🚴‍♂️ CycleSecure - Intelligent Bicycle Crash Detection & Alert System

CycleSecure is an advanced safety solution designed to protect cyclists by intelligently detecting potential crashes and sending real-time alerts to emergency contacts. It leverages GPS, motion sensors, a microphone, and GSM/WiFi connectivity to ensure quick response and peace of mind.

---

## 🔧 Key Features

- **Crash Detection:** 
  - Monitors accelerometer, gyroscope, and microphone for abnormal activity.
  - Validates crash events through user input to reduce false positives.
  
- **Real-Time Location Sharing:**
  - Sends the cyclist's current GPS coordinates upon confirmed impact.

- **Emergency Alerts:**
  - Automatically sends SMS messages to pre-configured emergency contacts.
  - Places automated phone calls to contacts in critical scenarios.

- **Firebase Integration:**
  - Logs GPS data, sensor readings, and crash events to the cloud.
  - Allows remote access to real-time and historical data.

- **User Configuration via Bluetooth:**
  - Configure user email, emergency contacts, and messages from a mobile app or PC over Bluetooth.

- **Device Management Interface:**
  - Users can view live data and manage settings from a connected mobile device or web platform via Firebase.

- **WiFi + GSM Dual Communication:**
  - Uses WiFi for cloud synchronization and GSM for SMS/calls when offline.

---

## 📦 Hardware Requirements

- ESP32 microcontroller  
- SIM800 GSM Module  
- MPU6050 Accelerometer + Gyroscope  
- Microphone module  
- GPS Module (connected to Serial2)  
- Optional: Buzzer, LEDs, Push Button  
- EEPROM (for saving user settings)

---

## ⚙️ Software Requirements

- Arduino IDE (with ESP32 board support)  
- Required Libraries:
  - `TinyGPS++`
  - `Adafruit_MPU6050`
  - `BluetoothSerial`
  - `TinyGSM`
  - `FirebaseESP32`
  - `ArduinoJson`
  - `EEPROM`
