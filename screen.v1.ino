#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// ==================== MICROPHONE ====================
#define MIC_PIN A0
#define SAMPLE_COUNT 50

// ==================== RGB LED =======================
#define RED_PIN   3
#define GREEN_PIN 4
#define BLUE_PIN  5

// Common cathode: analogWrite(255) = full brightness. Do not invert.
const bool COMMON_CATHODE = true;

// ==================== ROTARY ENCODER ================
#define ENCODER_CLK 7
#define ENCODER_DT  6
#define ENCODER_SW  2

// ==================== TFT SCREEN ====================
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST   8
#define TFT_MOSI 11
#define TFT_SCLK 12

Adafruit_ST7735 tft = Adafruit_ST7735(
  TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST
);

// ==================== SOUND BUFFER ==================
#define BUFFER_SIZE 5

float dBBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int readingsStored = 0;

// ==================== SYSTEM STATE ==================
int mode = 0;  // 0 = Sound Reactive, 1 = Ambient Lamp

float currentDB = 0.0;
float averageDB = 0.0;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 200;

// ==================== ENCODER STATE =================
int lastCLKState = HIGH;
int lastButtonReading = HIGH;
int stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 40;

unsigned long lastEncoderChange = 0;
const unsigned long ENCODER_DEBOUNCE = 2;

// ==================== COLOR PALETTE ================
// Same order and same RGB as updateLED().
const int COLOR_COUNT = 12;
int ambientColor = 0;

const uint8_t ambientColors[COLOR_COUNT][3] = {
  {  0,   0, 255},  // 0 Blue
  {  0, 100, 255},  // 1 Blue-cyan
  {  0, 255, 255},  // 2 Cyan
  {  0, 255, 100},  // 3 Turquoise
  {  0, 255,   0},  // 4 Green
  {100, 255,   0},  // 5 Yellow-green
  {255, 255,   0},  // 6 Yellow
  {255, 165,   0},  // 7 Orange
  {255,  50,   0},  // 8 Orange-red
  {255,   0,   0},  // 9 Red
  {255,   0,  20},  // 10 Red-pink
  {255,   0,  50}   // 11 Deep red-pink
};

const char* ambientColorNames[COLOR_COUNT] = {
  "BLUE",
  "BLUE-CYAN",
  "CYAN",
  "TURQUOISE",
  "GREEN",
  "YELLOW-GREEN",
  "YELLOW",
  "ORANGE",
  "ORANGE-RED",
  "RED",
  "RED-PINK",
  "DEEP RED-PINK"
};

int lastDrawnMode = -1;
int lastDrawnColor = -1;
int lastDrawnDBTenths = -9999;

// ==================== RGB LED FUNCTION ==============
void setColor(int red, int green, int blue) {
  red   = constrain(red, 0, 255);
  green = constrain(green, 0, 255);
  blue  = constrain(blue, 0, 255);

  if (!COMMON_CATHODE) {
    red   = 255 - red;
    green = 255 - green;
    blue  = 255 - blue;
  }

  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

// ==================== MICROPHONE ====================
float readMicrophone() {
  int samples[SAMPLE_COUNT];
  long sum = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = analogRead(MIC_PIN);
    sum += samples[i];
    delayMicroseconds(200);
  }

  float sampleMean = (float)sum / SAMPLE_COUNT;
  float sumSquares = 0.0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float difference = samples[i] - sampleMean;
    sumSquares += difference * difference;
  }

  float rms = sqrt(sumSquares / SAMPLE_COUNT);

  if (rms < 0.1) {
    rms = 0.1;
  }

  return 20.0 * log10(rms);
}

// ==================== WEIGHTED BUFFER ===============
void resetBuffer() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    dBBuffer[i] = 0.0;
  }

  bufferIndex = 0;
  readingsStored = 0;
  currentDB = 0.0;
  averageDB = 0.0;
}

float updateBuffer(float newReading) {
  if (readingsStored > 3 && fabs(newReading - averageDB) > 6.0) {
    newReading = averageDB;
  }

  dBBuffer[bufferIndex] = newReading;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  if (readingsStored < BUFFER_SIZE) {
    readingsStored++;
  }

  float weightedTotal = 0.0;
  float totalWeight = 0.0;

  int oldestIndex =
    (bufferIndex - readingsStored + BUFFER_SIZE) % BUFFER_SIZE;

  for (int i = 0; i < readingsStored; i++) {
    int index = (oldestIndex + i) % BUFFER_SIZE;
    float weight = i + 1;

    weightedTotal += dBBuffer[index] * weight;
    totalWeight += weight;
  }

  if (totalWeight > 0.0) {
    return weightedTotal / totalWeight;
  }

  return newReading;
}

// Same steps as updateLED: quiet = blue (0), loud = deep red-pink (11).
int getSoundColorIndex(float averageDB) {
  if (averageDB < 2)       return 0;
  else if (averageDB < 4)  return 1;
  else if (averageDB < 6)  return 2;
  else if (averageDB < 8)  return 3;
  else if (averageDB < 10) return 4;
  else if (averageDB < 12) return 5;
  else if (averageDB < 14) return 6;
  else if (averageDB < 16) return 7;
  else if (averageDB < 18) return 8;
  else if (averageDB < 20) return 9;
  else if (averageDB < 22) return 10;
  else if (averageDB >= 22) return 11;
}

// ==================== SOUND-REACTIVE COLORS ==========
void updateLED(float averageDB) {
  if (averageDB < 2) {
    setColor(0, 0, 255);        // Blue
  }
  else if (averageDB < 4) {
    setColor(0, 100, 255);      // Blue-cyan
  }
  else if (averageDB < 6) {
    setColor(0, 255, 255);      // Cyan
  }
  else if (averageDB < 8) {
    setColor(0, 255, 100);      // Turquoise
  }
  else if (averageDB < 10) {
    setColor(0, 255, 0);        // Green
  }
  else if (averageDB < 12) {
    setColor(100, 255, 0);      // Yellow-green
  }
  else if (averageDB < 14) {
    setColor(255, 255, 0);      // Yellow
  }
  else if (averageDB < 16) {
    setColor(255, 165, 0);      // Orange
  }
  else if (averageDB < 18) {
    setColor(255, 50, 0);       // Orange-red
  }
  else if (averageDB < 20) {
    setColor(255, 0, 0);        // Red
  }
  else if (averageDB < 22) {
    setColor(255, 0, 20);       // Red-pink
  }
  else if (averageDB >= 22) {
    setColor(255, 0, 50);       // Deep red-pink
  }
}

// ==================== AMBIENT LAMP ==================
void setAmbientColor() {
  Serial.print("Screen index: ");
  Serial.print(ambientColor);
  Serial.print(" | LED palette index: ");
  Serial.println(ambientColor);

  setColor(
    ambientColors[ambientColor][0],
    ambientColors[ambientColor][1],
    ambientColors[ambientColor][2]
  );
}

// ==================== TFT COLOR CONVERSION ===========
uint16_t getTFTColor(int red, int green, int blue) {
  return tft.color565(red, green, blue);
}

// ==================== TFT SCREEN ====================
void drawScreenBase() {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 5);
  tft.println("LIGHT MONITOR");

  tft.drawFastHLine(5, 19, 150, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 28);
  tft.print("MODE:");

  tft.setCursor(5, 48);
  tft.print("REL dB:");

  tft.setCursor(5, 68);
  tft.print("COLOR:");

  tft.setCursor(5, 88);
  tft.print("LIGHT LEVEL");

  tft.drawRect(5, 102, 150, 13, ST77XX_WHITE);

  lastDrawnMode = -1;
  lastDrawnColor = -1;
  lastDrawnDBTenths = -9999;
}

void updateScreen() {
  int colorIndex = (mode == 1) ? ambientColor : getSoundColorIndex(averageDB);
  int dbTenths = (int)(averageDB * 10.0);

  bool textDirty =
    (mode != lastDrawnMode) ||
    (mode == 1 && colorIndex != lastDrawnColor) ||
    (mode == 0 && dbTenths != lastDrawnDBTenths);

  if (textDirty) {
    tft.fillRect(45, 28, 110, 12, ST77XX_BLACK);
    tft.setCursor(45, 28);
    tft.setTextColor(ST77XX_GREEN);

    if (mode == 0) {
      tft.print("SOUND REACTIVE");
    } else if (mode == 1) {
      tft.print("AMBIENT LAMP");
    } else {
      tft.print("AMBIENT LAMP");
    }

    tft.fillRect(50, 48, 105, 12, ST77XX_BLACK);
    tft.setCursor(50, 48);
    tft.setTextColor(ST77XX_YELLOW);

    if (mode == 0) {
      tft.print(averageDB, 1);
    } else {
      tft.print("PAUSED");
    }

    tft.fillRect(45, 68, 110, 12, ST77XX_BLACK);
    tft.setCursor(45, 68);
    tft.setTextColor(ST77XX_WHITE);

    if (mode == 1) {
      tft.print(ambientColorNames[ambientColor]);
    } else {
      tft.print("AUTO");
    }

    lastDrawnMode = mode;
    lastDrawnColor = colorIndex;
    lastDrawnDBTenths = dbTenths;
  }

  tft.fillRect(7, 104, 146, 9, ST77XX_BLACK);

  uint16_t barColor = getTFTColor(
    ambientColors[colorIndex][0],
    ambientColors[colorIndex][1],
    ambientColors[colorIndex][2]
  );

  if (mode == 0) {
    float boundedDB = constrain(averageDB, 0.0, 22.0);
    int barWidth = (int)(boundedDB / 22.0 * 146.0);

    if (barWidth > 0) {
      tft.fillRect(7, 104, barWidth, 9, barColor);
    }
  } else {
    tft.fillRect(7, 104, 146, 9, barColor);
  }
}

// ==================== BUTTON =========================
void checkEncoderButton() {
  int reading = digitalRead(ENCODER_SW);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        if (mode == 0) {
          mode = 1;
          ambientColor = 0;
          setAmbientColor();

          Serial.println();
          Serial.println("MODE: AMBIENT LAMP");
          Serial.println("Ambient Color: 0 - BLUE");
        } else {
          mode = 0;
          resetBuffer();

          Serial.println();
          Serial.println("MODE: SOUND REACTIVE");
        }

        updateScreen();
      }
    }
  }

  lastButtonReading = reading;
}

// ==================== ROTARY ENCODER =================
void checkEncoder() {
  int currentCLK = digitalRead(ENCODER_CLK);

  if (lastCLKState == HIGH && currentCLK == LOW) {
    if (millis() - lastEncoderChange >= ENCODER_DEBOUNCE) {
      lastEncoderChange = millis();

      if (digitalRead(ENCODER_DT) != currentCLK) {
        ambientColor++;
      } else {
        ambientColor--;
      }

      if (ambientColor >= COLOR_COUNT) {
        ambientColor = 0;
      }

      if (ambientColor < 0) {
        ambientColor = COLOR_COUNT - 1;
      }

      setAmbientColor();
      updateScreen();

      Serial.print("Ambient Color: ");
      Serial.print(ambientColor);
      Serial.print(" - ");
      Serial.println(ambientColorNames[ambientColor]);
    }
  }

  lastCLKState = currentCLK;
}

// ==================== SETUP ==========================
void setup() {
  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  analogReadResolution(12);
  lastCLKState = digitalRead(ENCODER_CLK);

  resetBuffer();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawScreenBase();

  setColor(
    ambientColors[0][0],
    ambientColors[0][1],
    ambientColors[0][2]
  );

  updateScreen();

  Serial.println("LIGHT MONITOR READY");
  Serial.println("Mode: SOUND REACTIVE");
  Serial.println("Encoder CLK: D7 | DT: D6 | SW: D2");
  Serial.println("RGB LED: R=D3, G=D4, B=D5");
  Serial.println("Common-cathode LED: PWM not inverted.");
  Serial.println("Relative dB is not calibrated SPL.");
}

// this is a change

// ==================== MAIN LOOP ======================
void loop() {
  checkEncoderButton();

  if (mode == 0) {
    if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
      lastUpdateTime = millis();

      currentDB = readMicrophone();
      averageDB = updateBuffer(currentDB);

      updateLED(averageDB);
      updateScreen();

      Serial.print("Raw relative dB: ");
      Serial.print(currentDB, 2);
      Serial.print(" | Weighted average: ");
      Serial.println(averageDB, 2);
    }
  } else {
    checkEncoder();
    delay(2);
  }
}