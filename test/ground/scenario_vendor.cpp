#include "report.h"
#include "LCD1602_RGB.h"
int main() {
  sim_reset();
  LCD1602 lcd(16, 2);
  lcd.init();
  lcd.setRGB(255, 160, 0);
  lcd.setCursor(0, 0); lcd.send_string("ATLAS GROUND RX ");
  lcd.setCursor(0, 1); lcd.send_string("waiting for data");
  lcd.setCursor(0, 1); lcd.send_string("LON -91.650000  ");
  lcd.setCursor(4, 0); lcd.send_string("42");
  sim_report();
  return 0;
}
