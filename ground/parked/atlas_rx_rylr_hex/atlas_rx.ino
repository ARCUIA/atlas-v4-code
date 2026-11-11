/*
  Atlas ground station: ESP32 + REYAX RYLR998 receiving the flight board's 915 MHz GPS packets,
  shown on the Waveshare LCD1602 RGB display and on the serial port.

  Board: "ESP32 Dev Module". No extra libraries: only Wire and the files in this folder.
  NOT TESTED ON HARDWARE. The shared files (pkt, rylr, rylr_link, lcd_layout, ground_rx, ...) are copies of app/core/
  (ground/sync_shared.sh refreshes them) and are tested on the PC against simulated modules.

  Wiring:  module VCC -> 3V3, GND -> GND, module TXD -> ESP32 pin RYLR_RX_PIN, module RXD -> ESP32 pin RYLR_TX_PIN.
*/
#include <Wire.h>
#include "wslcd1602.h"
extern "C" {
#include "rylr_link.h"
#include "lcd_layout.h"
#include "ground_rx.h"
}

// ---- RYLR998 serial port (ESP32 UART2 on these pins) ----
#define RYLR_RX_PIN  26        // ESP32 RX  <- module TXD
#define RYLR_TX_PIN  27        // ESP32 TX  -> module RXD

// ---- LCD: Waveshare LCD1602 RGB Module (fixed I2C addresses 0x3E and 0x60) ----
#define PIN_SDA   21
#define PIN_SCL   22
#define LCD_COLS  16
#define LCD_ROWS  2
#define LCD_BRIGHT 255         // backlight brightness, 0..255

// ---- link settings: MUST match devices/radio/radio.h on the flight board (a PC test compares them) ----
#define OWN_ADDR       2       // this station
#define PEER_ADDR      1       // the flight board
#define NETWORK_ID     6
#define BAND_HZ        915000000UL
#define LORA_SF        11
#define LORA_BW_CODE   9       // 9 = 500 kHz
#define LORA_CR_CODE   1       // 1 = 4/5
#define LORA_PREAMBLE  12
#define TX_DBM         22

WsLcd1602 lcd;
static rylr_link_t radio;
static lcd_state_t rxs;
static char shown[LCD_MAX_ROWS][LCD_MAX_COLS + 1];     // what is on the LCD now, so only changed rows are rewritten
static lcd_rgb_t shownRgb = { 1, 2, 3 };               // an impossible colour, so the first draw sets it

// ---- serial port access for the link ----
static int ioTxStart(const uint8_t *p, uint16_t n) { return Serial2.write(p, n) == n ? 0 : -1; }
static int ioTxIdle(void) { return 1; }
static int ioReadByte(uint8_t *b) {
  if (Serial2.available() > 0) { *b = (uint8_t)Serial2.read(); return 1; }
  return 0;
}
static uint32_t ioNow(void) { return millis(); }
static void ioDelay(uint32_t ms) { delay(ms); }
static const rylr_io_t io = { ioTxStart, ioTxIdle, ioReadByte, ioNow, ioDelay };
static const rylr_settings_t cfg = { OWN_ADDR, PEER_ADDR, NETWORK_ID, BAND_HZ, LORA_SF, LORA_BW_CODE, LORA_CR_CODE, LORA_PREAMBLE, TX_DBM };

void drawDisplay(uint32_t now) {
  char rows[LCD_MAX_ROWS][LCD_MAX_COLS + 1];
  lcd_render(&rxs, now, LCD_COLS, LCD_ROWS, rows);
  for (uint8_t r = 0; r < LCD_ROWS; r++) {
    if (strcmp(rows[r], shown[r]) != 0) {              // rewrite only what changed: no flicker
      lcd.setCursor(0, r);
      lcd.print(rows[r]);
      strcpy(shown[r], rows[r]);
    }
  }
  lcd_rgb_t c = lcd_status_rgb(&rxs, now);             // backlight colour shows the link state
  if (c.r != shownRgb.r || c.g != shownRgb.g || c.b != shownRgb.b) {
    lcd.setRGB((uint8_t)((uint16_t)c.r * LCD_BRIGHT / 255), (uint8_t)((uint16_t)c.g * LCD_BRIGHT / 255),
               (uint8_t)((uint16_t)c.b * LCD_BRIGHT / 255));
    shownRgb = c;
  }
}

void refreshDisplay(uint32_t now) {                    // keeps the "age" and the page alternation moving
  static uint32_t last = 0;
  if (now - last < 250) return;
  last = now;
  drawDisplay(now);
}

void fatal(const char *line1, const char *line2) {
  Serial.print(line1); Serial.print(": "); Serial.println(line2);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
  lcd.setRGB(LCD_BRIGHT, 0, 0);                        // red
  while (true) { delay(10); }
}

// called by the link for every packet the module hands over: "+RCV=<addr>,<length>,<hex data>,<rssi>,<snr>"
void onReceive(void *ctx, uint16_t addr, const char *hex, uint16_t hexlen, int rssi, int snr) {
  (void)ctx; (void)addr;
  char line[220];
  int rc = ground_rx_handle(&rxs, hex, hexlen, rssi, snr, millis(), line, sizeof line);
  if (rc == GROUND_RX_OK) {
    Serial.println(line);
    drawDisplay(millis());
  } else if (rc == GROUND_RX_NOT_HEX) {
    Serial.println("received something that is not hex text");
  } else {
    Serial.println("received hex, but not an Atlas packet (another device on this network?)");
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!lcd.probe()) Serial.println("LCD not found at I2C address 0x3E: check SDA/SCL wiring and power");
  lcd.begin();
  lcd_state_init(&rxs);
  drawDisplay(0);                                      // "ATLAS GROUND RX / waiting for data"

  Serial2.begin(115200, SERIAL_8N1, RYLR_RX_PIN, RYLR_TX_PIN);
  int rc = rylr_link_init(&radio, &io, &cfg);          // sets address, network, band, RF parameters (only what differs)
  if (rc == RYLR_ERR_NO_RESPONSE) fatal("RYLR998 no reply", "check wiring/power");
  if (rc == RYLR_ERR_BAD_REPLY)   fatal("RYLR998 bad reply", "check baud/wiring");
  if (rc != RYLR_OK) {
    char code[17]; snprintf(code, sizeof code, "+ERR=%d", RYLR_ERR_REJECTED - rc);
    fatal("RYLR998 rejected", code);
  }
  rylr_link_set_rcv(&radio, onReceive, 0);
  Serial.println("Atlas ground station listening");
}

void loop() {
  uint32_t now = millis();
  rylr_link_poll(&radio, now);                         // reads the module; calls onReceive() for each packet
  refreshDisplay(now);

  static uint32_t lastStatus = 0;
  if (now - lastStatus >= 10000) {                     // a sign of life while no packets arrive
    lastStatus = now;
    Serial.print("[listening] packets="); Serial.print(rxs.rx_count);
    Serial.print(" module restarts="); Serial.println(radio.reconfigs);
  }
}
