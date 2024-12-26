/**
 * This program is designed to read sensor data from a UART connection and send it 
 * to a Firebase Realtime Database using an ESP32. The program establishes a Wi-Fi 
 * connection, initializes Firebase, and reads data periodically from a connected device 
 * via UART. The data is sent to Firebase in JSON format.
 *
 * Features:
 * - Wi-Fi connection setup
 * - Firebase Realtime Database initialization and data sending
 * - UART communication to receive sensor data
 * - Periodic data transmission to Firebase (every 10 seconds)
 *
 * Ensure you replace placeholders like `WIFI_SSID`, `WIFI_PASSWORD`, and `DATABASE_URL` 
 * with your actual Firebase and Wi-Fi credentials.
 */

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HardwareSerial.h>

// Firebase configuration settings
#define WIFI_SSID "Replace with your Wi-Fi SSID" 
#define WIFI_PASSWORD "Replace with your Wi-Fi Password" 
#define API_KEY "Firebase API Key" 
#define DATABASE_URL "Firebase database URL"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Hardware setup
#define led 2 // LED is connected to GPIO 2
HardwareSerial SerialUART(1); // UART1, using GPIO16 (RX) and GPIO17 (TX)

// Struct definition for sensor data
typedef struct {
  float Ax; // Accelerometer X-axis
  float Ay; // Accelerometer Y-axis
  float Az; // Accelerometer Z-axis
  float Gx; // Gyroscope X-axis
  float Gy; // Gyroscope Y-axis
  float Gz; // Gyroscope Z-axis
  float Pressure; // Atmospheric pressure
  float Altitude; // Altitude in meters
  float Latitude; // GPS Latitude
  float Longtitude; // GPS Longitude
  float Temperature; // Temperature in degrees Celsius
} SensorData;

// Variable to track the time between data transmissions
unsigned long sendDataPrevMillis = 0;

void setup() {
  // Initialize Serial communication for debugging and UART communication
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, 16, 17); // Set up UART with RX and TX pins
  pinMode(led, OUTPUT); // Configure LED pin as output

  // Wi-Fi connection setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { // Wait for Wi-Fi connection
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWi-Fi connected.");
  Serial.println(WiFi.localIP()); // Print the device's IP address

  // Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Firebase authentication (if required)
  auth.user.email = "yusufkarabocekz@gmail.com"; // Replace with your email
  auth.user.password = "11ROCKETMANbbb11"; // Replace with your password

  Firebase.begin(&config, &auth); // Initialize Firebase
  Firebase.reconnectNetwork(true); // Reconnect automatically if the network disconnects

  // Check Firebase initialization
  if (!Firebase.ready()) {
    Serial.println("Firebase initialization failed!");
  } else {
    Serial.println("Firebase initialized and ready.");
  }
}

void loop() {
  static unsigned long previousMillis = 0;      // Tracks the last data transmission time
  static unsigned long previousLedMillis = 0;  // Tracks the last LED toggle time
  const unsigned long ledInterval = 250;       // LED toggle interval (in milliseconds)

  unsigned long currentMillis = millis();      // Get the current time

  // Check if it's time to toggle the LED
  if (currentMillis - previousLedMillis >= ledInterval) {
    previousLedMillis = currentMillis;        // Update the last toggle time
    digitalWrite(led, !digitalRead(led));     // Toggle the LED state
  }

  // Send data to Firebase every 10 seconds
  if (Firebase.ready() && currentMillis - previousMillis >= 10000) {
    previousMillis = currentMillis;          // Update the last data transmission time

    // Check if UART data is available and matches the expected size
    if (SerialUART.available() >= sizeof(SensorData)) {
      SensorData data;
      uint8_t buffer[sizeof(SensorData)];

      // Read data from UART and copy it into the SensorData struct
      SerialUART.readBytes(buffer, sizeof(SensorData));
      memcpy(&data, buffer, sizeof(SensorData)); // Convert buffer to struct

      // Create a JSON object and populate it with sensor data
      FirebaseJson json;
      json.set("/Accelerometer/X", data.Ax);
      json.set("/Accelerometer/Y", data.Ay);
      json.set("/Accelerometer/Z", data.Az);

      json.set("/Gyroscope/X", data.Gx);
      json.set("/Gyroscope/Y", data.Gy);
      json.set("/Gyroscope/Z", data.Gz);

      json.set("/Pressure", data.Pressure);
      json.set("/Altitude", data.Altitude);
      json.set("/Latitude", data.Latitude);
      json.set("/Longitude", data.Longtitude);
      json.set("/Temperature", data.Temperature);

      // Send JSON data to Firebase
      if (Firebase.RTDB.setJSON(&fbdo, "/sensorData", &json)) {
        Serial.println("Data sent to Firebase successfully!");
      } else {
        Serial.print("Firebase error: ");
        Serial.println(fbdo.errorReason());
      }
    } else {
      Serial.println("No data available from UART.");
    }
  }
}