#pragma once
#include "Wire.h"
inline void sim_reset() { sim_lcd.reset(); sim_pca.reset(); }
inline void sim_report() {
  printf("ROW0=|%.16s|\nROW1=|%.16s|\nRGB=%u,%u,%u\nMODE1=%u LEDOUT=%u MODE2=%u\nDISPLAY_ON=%d\n",
         sim_lcd.ddram[0], sim_lcd.ddram[1], sim_pca.reg[4], sim_pca.reg[3], sim_pca.reg[2],
         sim_pca.reg[0], sim_pca.reg[8], sim_pca.reg[1], (int)sim_lcd.display_on);
}
