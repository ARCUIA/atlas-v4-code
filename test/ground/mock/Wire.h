#pragma once
#include "Arduino.h"
/* Simulated I2C bus: an AiP31068L text controller at 0x3E and a PCA9633 backlight driver at 0x60. */
struct SimLcd {
  char ddram[2][16]; unsigned addr; int inc; bool display_on; unsigned commands, datas;
  void reset() { memset(ddram, '#', sizeof ddram); addr = 0; inc = 1; display_on = false; commands = datas = 0; }
  void put(int rs, uint8_t b) {
    if (rs) {                                         /* data: a character at the cursor */
      datas++;
      if (addr <= 0x0F) ddram[0][addr] = (char)b; else if (addr >= 0x40 && addr <= 0x4F) ddram[1][addr - 0x40] = (char)b;
      addr += inc;
    } else {                                          /* command */
      commands++;
      if (b == 0x01) { memset(ddram, ' ', sizeof ddram); addr = 0; }
      else if (b == 0x02) addr = 0;
      else if ((b & 0xFC) == 0x04) inc = (b & 0x02) ? 1 : -1;      /* entry mode set */
      else if ((b & 0xF8) == 0x08) display_on = (b & 0x04) != 0;   /* display control */
      else if (b & 0x80) addr = b & 0x7F;                          /* set DDRAM address */
    }
  }
  void transaction(const uint8_t *p, size_t n) {      /* control byte: bit7 = Co (another control byte follows), bit6 = RS */
    size_t i = 0;
    while (i < n) {
      uint8_t ctl = p[i++]; int rs = (ctl >> 6) & 1;
      if (ctl & 0x80) { if (i < n) put(rs, p[i++]); }
      else { while (i < n) put(rs, p[i++]); }
    }
  }
};
struct SimPca { uint8_t reg[0x0A]; void reset() { memset(reg, 0, sizeof reg); }
  void transaction(const uint8_t *p, size_t n) {
    if (n < 1) return;
    unsigned r = p[0] & 0x1F; bool ai = (p[0] & 0x80) != 0;
    for (size_t i = 1; i < n; i++) { if (r < sizeof reg) reg[r] = p[i]; if (ai) r++; }
  } };
inline SimLcd sim_lcd; inline SimPca sim_pca;
class TwoWire {
  uint8_t addr_ = 0, buf_[64]; size_t n_ = 0;
public:
  bool begin(int = -1, int = -1) { return true; }
  void beginTransmission(uint8_t a) { addr_ = a; n_ = 0; }
  size_t write(uint8_t b) { if (n_ < sizeof buf_) buf_[n_++] = b; return 1; }
  uint8_t endTransmission(bool = true) {
    if (addr_ == 0x3E) { sim_lcd.transaction(buf_, n_); return 0; }
    if (addr_ == 0x60) { sim_pca.transaction(buf_, n_); return 0; }
    return 2;                                         /* no device at that address */
  }
};
inline TwoWire Wire;
