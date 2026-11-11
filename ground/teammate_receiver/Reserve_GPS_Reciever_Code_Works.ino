#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

// Global Variables
String incomingString;

unsigned long previousMillis = 0;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);

// LoRa Configuration
#define LORA_RX 16
#define LORA_TX 17

void setup() {

  Serial.begin(115200);

  // Initialize LoRa UART
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  Serial2.setTimeout(250);

  // Initialize LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("LoRa Receiver Ready");

  // Print startup message
  Serial.println("----------------------");
  Serial.println("ESP32 RECEIVER STARTED");
  Serial.println("LoRa UART: 115200");
  Serial.println("LoRa RX: GPIO 16");
  Serial.println("LoRa TX: GPIO 17");
  Serial.println("----------------------");
  Serial.println("Waiting for LoRa data...");
}

void loop() {

  // Print waiting message every 2 seconds
  if (millis() - previousMillis >= 2000) {

    previousMillis = millis();

    if (!Serial2.available()) {
      Serial.println("Waiting for LoRa data...");
    }
  }

  if (Serial2.available()) {

    // Receive message
    incomingString = Serial2.readStringUntil('\n');
    incomingString.trim();

    if (incomingString.length() == 0) {
      return;
    }

    // Print raw received message
    Serial.println("RAW RECEIVED:");
    Serial.println(incomingString);
    Serial.println("----------------------");

    // Convert String to character array
    char dataArray[150];

    incomingString.toCharArray(dataArray, sizeof(dataArray));

    // Parse incoming message
    char* data = strtok(dataArray, ",");

    // Check sender address
    if (data == NULL || strncmp(data, "+RCV=", 5) != 0) {

      Serial.println("Invalid LoRa message");

      return;
    }

    // Extract payload length
    data = strtok(NULL, ",");

    if (data == NULL) {

      Serial.println("Invalid packet length");

      return;
    }

    // Extract GPS connection status
    char* connection = strtok(NULL, ",");

    if (connection == NULL) {

      Serial.println("Invalid packet received");

      return;
    }

    Serial.print("Connection: ");
    Serial.println(connection);

    // Clear LCD
    lcd.clear();

    // Check GPS connection status
    if (strcmp(connection, "NG") == 0) {

      // No GPS fix
      lcd.setCursor(0, 0);
      lcd.print("LAT 0");

      lcd.setCursor(0, 1);
      lcd.print("LON 0");

      Serial.println("No GPS Fix");

    }

    else if (strcmp(connection, "YG") == 0) {

      // Extract latitude and longitude
      char* latitude = strtok(NULL, ",");
      char* longitude = strtok(NULL, ",");

      if (latitude == NULL || longitude == NULL) {

        Serial.println("Invalid GPS coordinates");

        return;
      }

      // Display latitude
      lcd.setCursor(0, 0);
      lcd.print("LAT ");
      lcd.print(latitude);

      Serial.print("Lat: ");
      Serial.println(latitude);

      // Display longitude
      lcd.setCursor(0, 1);
      lcd.print("LON ");
      lcd.print(longitude);

      Serial.print("Long: ");
      Serial.println(longitude);

    }

    else {

      Serial.println("Unknown GPS status");

    }

    Serial.println("----------------------");
  }
}