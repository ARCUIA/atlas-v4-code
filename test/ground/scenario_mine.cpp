#include "report.h"
#include "wslcd1602.h"
int main() {
  sim_reset();
  WsLcd1602 lcd;
  if (!lcd.probe()) { puts("probe failed"); return 1; }
  lcd.begin();
  lcd.setRGB(255, 160, 0);
  lcd.setCursor(0, 0); lcd.print("ATLAS GROUND RX ");
  lcd.setCursor(0, 1); lcd.print("waiting for data");
  lcd.setCursor(0, 1); lcd.print("LON -91.650000  ");      /* overwrite row 1 */
  lcd.setCursor(4, 0); lcd.print("42");                      /* partial overwrite inside row 0 */
  sim_report();
  return 0;
}
