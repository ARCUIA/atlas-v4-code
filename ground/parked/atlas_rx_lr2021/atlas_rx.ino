/*
  Atlas ground station: ESP32 + Wio-LR2021 receiving the flight board's 915 MHz telemetry,
  shown on the 16x2 RGB LCD and on the serial port.

  Needs RadioLib 7.8.1 exactly (Library Manager > RadioLib > choose version 7.8.1). Board: "ESP32 Dev Module".
  The display is the Waveshare LCD1602 RGB Module (I2C): its driver, wslcd1602.h, is in this folder and
  needs nothing but the standard Wire library. The backlight colour shows the link state.
  NOT TESTED ON HARDWARE. pkt.c, rx_format.c and lcd_layout.c are copies of app/core/
  (ground/sync_shared.sh refreshes them); the display logic is tested on the PC.
*/
#include <Wire.h>
#include "wslcd1602.h"
#include <RadioLib.h>
extern "C" {
#include "rx_format.h"
#include "lcd_layout.h"
}

// ---- radio wiring: EDIT to match how the Wio-LR2021 is connected to the ESP32 ----
#define PIN_CS    5        // module NSS
#define PIN_IRQ   4        // ESP32 pin wired to the module DIO pin used as interrupt
#define PIN_RST   16       // module RESET
#define PIN_BUSY  17       // module BUSY
#define IRQ_DIO   5        // which module DIO (5..11) that IRQ wire is on. DIO5 is the module's usual IRQ pin.
// The radio's SPI uses the ESP32's default (VSPI) pins: SCK 18, MISO 19, MOSI 23.

// ---- LCD: Waveshare LCD1602 RGB Module (fixed I2C addresses 0x3E and 0x60) ----
#define PIN_SDA   21
#define PIN_SCL   22
#define LCD_COLS  16
#define LCD_ROWS  2
#define LCD_BRIGHT 255     // backlight brightness, 0..255 (lower it if the colours are too bright)

// ---- link settings: MUST match the flight board (devices/radio/radio.h) ----
#define FREQ_MHZ    915.0      // MHz (RadioLib 7.8.1 wants MHz here)
#define BW_KHZ      500.0      // kHz (and kHz here, not Hz)
#define SF          12
#define CR_DEN      5          // coding rate 4/5
#define SYNC_WORD   0x12
#define PREAMBLE    8

LR2021 radio = new Module(PIN_CS, PIN_IRQ, PIN_RST, PIN_BUSY);
WsLcd1602 lcd;

volatile bool rxFlag = false;
static lcd_state_t rxs;
static char shown[LCD_MAX_ROWS][LCD_MAX_COLS + 1];     // what is on the LCD now, so only changed rows are rewritten
static lcd_rgb_t shownRgb = { 1, 2, 3 };               // an impossible colour, so the first draw sets it

// called by RadioLib when a packet has arrived. MUST be void with no arguments.
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) { rxFlag = true; }

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

void fatal(const char *what, int state) {
  Serial.print(what); Serial.print(" failed, code "); Serial.println(state);
  char code[17];
  snprintf(code, sizeof code, "code %d", state);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(what);
  lcd.setCursor(0, 1); lcd.print(code);
  lcd.setRGB(LCD_BRIGHT, 0, 0);                        // red
  while (true) { delay(10); }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!lcd.probe()) Serial.println("LCD not found at I2C address 0x3E: check SDA/SCL wiring and power");
  lcd.begin();
  lcd_state_init(&rxs);
  drawDisplay(0);                                      // "ATLAS GROUND RX / waiting for data"

  radio.tcxoVoltage = 3.0;        // the Wio-LR2021's TCXO is powered at 3.0 V (RadioLib's default is 1.6 V)
  radio.irqDioNum = IRQ_DIO;      // before begin()

  ConfigLoRa_t cfg;
  cfg.frequency       = FREQ_MHZ;
  cfg.bandwidth       = BW_KHZ;
  cfg.spreadingFactor = SF;
  cfg.codingRate      = CR_DEN;
  cfg.syncWord        = SYNC_WORD;
  cfg.preambleLength  = PREAMBLE;

  int state = radio.begin(cfg);
  if (state != RADIOLIB_ERR_NONE) fatal("RADIO begin", state);

  radio.setRxBoostedGainMode(7);  // a few dB more sensitivity for a little more current: fine on USB power
  radio.setIrqAction(setFlag);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) fatal("RADIO listen", state);
  Serial.println("Atlas ground station listening");
}

void loop() {
  uint32_t now = millis();

  if (rxFlag) {
    rxFlag = false;

    uint8_t buf[64];
    size_t len = radio.getPacketLength();
    if (len > sizeof buf) len = sizeof buf;
    int state = radio.readData(buf, len);

    if (state == RADIOLIB_ERR_NONE) {
      int rssi = (int)radio.getRSSI();
      int snr  = (int)radio.getSNR();
      if (lcd_state_update(&rxs, buf, len, rssi, snr, now) == 0) {
        char line[200];
        rx_format_line(buf, len, (float)rssi, (float)snr, line, sizeof line);
        Serial.print(line);
        Serial.print(" rx="); Serial.print(rxs.rx_count);
        Serial.print(" lost="); Serial.println(rxs.lost);
        drawDisplay(now);
      } else {
        Serial.println("packet received, but it is not an Atlas packet");   // another device on the same channel
      }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("packet received with a CRC error");
    } else {
      Serial.print("read failed, code "); Serial.println(state);
    }
    radio.startReceive();         // back to listening
  }

  refreshDisplay(now);
}
