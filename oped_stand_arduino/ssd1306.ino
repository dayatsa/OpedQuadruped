void oledInit() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);
  display.clearDisplay();

  // text display tests
  display.setTextSize(2.6);
  display.setTextColor(WHITE);
  display.setCursor(40, 4);
  display.println("OPED");
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(37, 24);
  display.println("QUADRUPED");
  display.display();
  delay(2000);
  display.clearDisplay();
}


void printImu(float _pitch, float _roll) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(15, 0);
  display.println("PITCH");
//  display.display();  
  display.setCursor(90, 0);
  display.println("ROLL");
//  display.display();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 17);
  display.println(_pitch,1);  
  display.setTextColor(WHITE);
  display.setCursor(75, 17);
  display.println(_roll,1);
  display.display();
}


void testdrawrect(void) {
  for (int16_t i = 0; i < display.height() / 2; i += 2) {
    display.drawRect(i, i, display.width() - 2 * i, display.height() - 2 * i, WHITE);
    display.display();
  }
}
