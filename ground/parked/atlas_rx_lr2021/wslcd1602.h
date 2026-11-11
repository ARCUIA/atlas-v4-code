#ifndef WSLCD1602_H
#define WSLCD1602_H
/*
  Minimal driver for the Waveshare LCD1602 RGB Module (16x2, I2C, 3.3 V / 5 V).
  Needs only the Wire library: nothing extra to install.

  Chips: AiP31068L text controller at I2C address 0x3E, PCA9633 backlight driver at 0x60.
  Protocol (matches Waveshare's demos and the community drivers for this module):
    text controller: [0x3E][0x80][command]  or  [0x3E][0x40][character]
    backlight:       [0x60][register][value]; red = reg 0x04, green = 0x03, blue = 0x02
*/
#include <Wire.h>

class WsLcd1602 {
public:
  static const uint8_t LCD_ADDR = 0x3E;
  static const uint8_t RGB_ADDR = 0x60;

  bool probe() {                                  // true if the text controller answers on the bus
    Wire.beginTransmission(LCD_ADDR);
    return Wire.endTransmission() == 0;
  }

  void begin() {
    delay(50);                                    // power-up wait
    command(0x28); delay(5);                      // function set: 2 lines, 5x8 font (sent twice, as in the vendor code)
    command(0x28); delay(5);
    command(0x0C);                                // display on, cursor off, no blink
    clear();
    command(0x06);                                // entry mode: cursor moves right
    rgbWrite(0x00, 0x00);                         // PCA9633 MODE1: normal
    rgbWrite(0x08, 0xFF);                         // LEDOUT: all three channels on PWM
    rgbWrite(0x01, 0x20);                         // MODE2, as in the vendor code
  }

  void clear() { command(0x01); delay(2); }       // clear needs about 2 ms
  void setCursor(uint8_t col, uint8_t row) { command((uint8_t)((row == 0 ? 0x80 : 0xC0) | (col & 0x3F))); }
  void print(const char *s) { while (*s) data((uint8_t)*s++); }
  void setRGB(uint8_t r, uint8_t g, uint8_t b) { rgbWrite(0x04, r); rgbWrite(0x03, g); rgbWrite(0x02, b); }

private:
  void command(uint8_t c) { Wire.beginTransmission(LCD_ADDR); Wire.write(0x80); Wire.write(c); Wire.endTransmission(); delayMicroseconds(50); }
  void data(uint8_t d)    { Wire.beginTransmission(LCD_ADDR); Wire.write(0x40); Wire.write(d); Wire.endTransmission(); delayMicroseconds(50); }
  void rgbWrite(uint8_t reg, uint8_t v) { Wire.beginTransmission(RGB_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission(); }
};
#endif
