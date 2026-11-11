/*
  RYLR998 settings reader (ESP32). Prints what the module is currently set to, then lets you type AT commands.

  Use it on the RECEIVER's module, wired exactly as in Reserve_GPS_Reciever_Code_Works.ino (module TXD -> GPIO 16,
  module RXD -> GPIO 17, 3V3, GND). Upload this instead of the receiver sketch, open the Serial Monitor at 115200,
  and compare with devices/radio/radio.h on the flight board: network id, band, spreading factor, bandwidth, coding
  rate and preamble must all match. (The receiver module was reported as NETWORK=6, ADDRESS=1, BAND=915000000.) If they differ, change the RADIO_* lines in radio.h (or change the module with
  an AT command typed here, e.g. AT+PARAMETER=9,7,1,12).
  The Serial Monitor's line ending can be anything: this sketch adds the \r\n the module needs.
*/
#define LORA_RX 16     // ESP32 RX  <- module TXD
#define LORA_TX 17     // ESP32 TX  -> module RXD

static void forwardModuleOutput(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms)
    while (Serial2.available()) Serial.write(Serial2.read());
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(1000);
  Serial.println("--- RYLR998 current settings ---");
  static const char *const queries[] = { "AT", "AT+VER?", "AT+ADDRESS?", "AT+NETWORKID?", "AT+BAND?", "AT+PARAMETER?", "AT+CRFOP?", "AT+IPR?" };
  for (uint8_t i = 0; i < sizeof queries / sizeof queries[0]; i++) {
    Serial.print("> "); Serial.println(queries[i]);
    Serial2.print(queries[i]); Serial2.print("\r\n");
    forwardModuleOutput(700);
  }
  Serial.println("--- no answer to any of these? check wiring and power ---");
  Serial.println("--- expected for this setup: NETWORK=6, ADDRESS=1, BAND=915000000, PARAMETER=9,7,1,12 (factory default), IPR=115200 ---");
  Serial.println("--- now type AT commands if you want to change something ---");
}

void loop() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') Serial2.print("\r\n"); else Serial2.write((uint8_t)c);
  }
  while (Serial2.available()) Serial.write(Serial2.read());
}
