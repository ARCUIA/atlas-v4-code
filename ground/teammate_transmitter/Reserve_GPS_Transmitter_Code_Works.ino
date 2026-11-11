#include <string.h>

// Global Variables
String incomingString;
String outgoingString;
String lora_RX_address = "0";

String gpsString;

double latitude = 0;
double longitude = 0;

bool gpsFix = false;

unsigned long previousMillis = 0;
unsigned long lastGPSFix = 0;

const unsigned long transmitInterval = 1000;

// LoRa Configuration
#define LORA_RX 16
#define LORA_TX 17

// GPS Configuration
#define GPS_RX 21
#define GPS_TX 22

void setup() {

  Serial.begin(115200);

  // Initialize LoRa UART
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  Serial2.setTimeout(250);

  // Initialize GPS UART
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("LoRa GPS Transmitter Ready");
}

void loop() {

  // Read GPS data
  while (Serial1.available()) {

    char gpsData = Serial1.read();

    if (gpsData == '\n') {

      gpsString.trim();

      // Parse completed GPS sentence
      if (gpsString.startsWith("$GPRMC") ||
          gpsString.startsWith("$GNRMC")) {

        char dataArray[150];

        gpsString.toCharArray(dataArray, sizeof(dataArray));

        // Parse GPS sentence
        char* data = strtok(dataArray, ",");

        data = strtok(NULL, ",");  // Time

        char* status = strtok(NULL, ",");  // GPS Status

        char* lat = strtok(NULL, ",");  // Latitude

        char* latDirection = strtok(NULL, ",");  // N/S

        char* lon = strtok(NULL, ",");  // Longitude

        char* lonDirection = strtok(NULL, ",");  // E/W

        if (status != NULL && strcmp(status, "A") == 0 &&
            lat != NULL && lon != NULL &&
            latDirection != NULL && lonDirection != NULL) {

          // Convert GPS coordinates from NMEA format
          double rawLatitude = atof(lat);
          double rawLongitude = atof(lon);

          latitude = (int)(rawLatitude / 100) +
                     (rawLatitude - ((int)(rawLatitude / 100) * 100)) / 60.0;

          longitude = (int)(rawLongitude / 100) +
                      (rawLongitude - ((int)(rawLongitude / 100) * 100)) / 60.0;

          // Apply coordinate directions
          if (strcmp(latDirection, "S") == 0) {
            latitude = -latitude;
          }

          if (strcmp(lonDirection, "W") == 0) {
            longitude = -longitude;
          }

          gpsFix = true;

          lastGPSFix = millis();
        }

        else {

          gpsFix = false;
        }
      }

      gpsString = "";
    }

    else if (gpsData != '\r') {

      gpsString += gpsData;
    }
  }

  // Transmit GPS data every second
  if (millis() - previousMillis >= transmitInterval) {

    previousMillis = millis();

    // Check if GPS fix is valid
    if (gpsFix && millis() - lastGPSFix < 5000) {

      // Create GPS payload
      outgoingString = "YG," +
                       String(latitude, 6) + "," +
                       String(longitude, 6);

      Serial.println("GPS FIX FOUND");

      Serial.print("Latitude: ");
      Serial.println(latitude, 6);

      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
    }

    else {

      // Create no GPS fix payload
      outgoingString = "NG,0,0";

      Serial.println("GPS FIX NOT FOUND");
    }

    // Calculate payload length
    int payloadLength = outgoingString.length();

    // Create LoRa transmission command
    String command = "AT+SEND=" + lora_RX_address + "," +
                     String(payloadLength) + "," +
                     outgoingString;

    // Transmit message through LoRa
    Serial.println("Sending:");
    Serial.println(command);

    Serial2.println(command);

    Serial.println("----------------------");
  }

  // Read LoRa responses
  while (Serial2.available()) {

    String response = Serial2.readStringUntil('\n');
    response.trim();

    if (response.length() > 0) {

      Serial.println("LoRa Response:");
      Serial.println(response);
    }
  }
}