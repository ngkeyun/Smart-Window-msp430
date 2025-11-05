#include <Wire.h>
#include <TinyScreen.h>

// --- TinyScreen+ setup ---
TinyScreen display = TinyScreen(TinyScreenPlus);

// --- Inputs (you can change pins later) ---
const int PIN_RAIN_ANALOG = A0;   // analog rain/moisture sensor
const int PIN_WINDOW_SW   = 3;    // digital window state switch (to GND when CLOSED)

// --- Status variables (what you need to show) ---
bool isRaining  = false;
bool windowOpen = true;
bool autoMode   = true;

// --- For nicer UI refresh ---
bool last_isRaining  = !isRaining;
bool last_windowOpen = !windowOpen;
bool last_autoMode   = !autoMode;
unsigned long lastRefreshMs = 0;
const unsigned long REFRESH_MS = 250; // redraw guard

// --- Simple thresholds (tune later) ---
const int RAIN_THRESHOLD = 500;  // 0-1023; > threshold means "wet"

// --- Helpers for button edge detection ---
bool wasUpperRight = false;
bool wasLowerRight = false;

void setup() {
  pinMode(PIN_WINDOW_SW, INPUT_PULLUP);
  display.begin();
  display.setBrightness(12);
  display.setFlip(true);
  display.clearScreen();
  display.setFont(thinPixel7_10ptFontInfo);
  drawScreen(true);
}

void loop() {
  readInputs();
  handleButtons();

  bool changed = (isRaining  != last_isRaining) ||
                 (windowOpen != last_windowOpen) ||
                 (autoMode   != last_autoMode);

  if (changed || (millis() - lastRefreshMs >= REFRESH_MS)) {
    drawScreen(changed);
    lastRefreshMs = millis();
    last_isRaining  = isRaining;
    last_windowOpen = windowOpen;
    last_autoMode   = autoMode;
  }

  delay(30);
}

void readInputs() {
  int rainRaw = analogRead(PIN_RAIN_ANALOG);
  isRaining = (rainRaw > RAIN_THRESHOLD);
  int sw = digitalRead(PIN_WINDOW_SW);
  windowOpen = (sw == HIGH);
}

void handleButtons() {
  uint8_t b = display.getButtons();
  bool nowUpperRight = (b & TSButtonUpperRight);
  bool nowLowerRight = (b & TSButtonLowerRight);

  if (!wasUpperRight && nowUpperRight) autoMode = !autoMode;
  if (!wasLowerRight && nowLowerRight) windowOpen = !windowOpen;

  wasUpperRight = nowUpperRight;
  wasLowerRight = nowLowerRight;
}

void drawScreen(bool emphasizeChange) {
  display.clearScreen();
  centerPrint(48, 2, "SMART WINDOW", TS_8b_White);

  if (isRaining)
    linePrint(0, 18, "Rain: YES", TS_8b_Red);
  else
    linePrint(0, 18, "Rain: NO", TS_8b_Green);

  if (windowOpen)
    linePrint(0, 36, "Window: OPEN", TS_8b_Yellow);
  else
    linePrint(0, 36, "Window: CLOSED", TS_8b_Blue);

  if (autoMode)
    linePrint(0, 54, "Mode: AUTO", TS_8b_White);
  else
    linePrint(0, 54, "Mode: MANUAL", TS_8b_White);
}

void centerPrint(int xCenter, int y, const char* text, uint8_t color) {
  display.fontColor(color, TS_8b_Black);
  int w = display.getPrintWidth((char*)text);
  display.setCursor(xCenter - (w / 2), y);
  display.print(text);
}

void linePrint(int x, int y, const char* text, uint8_t color) {
  display.fontColor(color, TS_8b_Black);
  display.setCursor(x, y);
  display.print(text);
}
