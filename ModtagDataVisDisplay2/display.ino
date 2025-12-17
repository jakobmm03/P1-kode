void printLevel(int x) {

  tft.setCursor(0, 0);
  //int m = map(x, 1, 100, 1, 100);
  printLevelBar(x);

  //  tft.setCursor(0, 30);
  //  tft.setTextSize(2);
  //  tft.print(F("Water Level: "));
  tft.setCursor(150, 30);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(x);
  tft.print(" % ");
}

void printLevelBar(int x) {
  x /= 10;

  for (int i = 0; i < 10; i++) {
    if (i < x) {
      if (alarmGreen == true) {
        tft.fillRect(i * (tft.width() / 10), 0, tft.width() / 10, 25, TFT_GREEN);
      } else if (alarmYellow == true) {
        tft.fillRect(i * (tft.width() / 10), 0, tft.width() / 10, 25, TFT_YELLOW);
      } else if (alarmRed == true) {
        tft.fillRect(i * (tft.width() / 10), 0, tft.width() / 10, 25, TFT_RED);
      }
    } else {
      tft.fillRect(i * (tft.width() / 10), 0, tft.width() / 10, 25, TFT_BLACK);
      tft.drawRect(i * (tft.width() / 10), 0, tft.width() / 10, 25, TFT_WHITE);
    }
  }
}


void printDisplayInit() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 30);
  tft.setTextSize(2);
  tft.print(F("Water Level:"));


  tft.setCursor(0, 60);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  //tft.println("P-Delay:");
  tft.println("Afstand:");
  // tft.println("S-Afstand:");
  // tft.println("Height:");
  tft.println("Hum.:");
  //tft.println("ik mod: ");
  tft.println("Temp.:");
}
