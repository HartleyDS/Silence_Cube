#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>


// ==================== MICROPHONES ====================
#define MIC_A_PIN A0
#define MIC_B_PIN A1
#define SAMPLE_COUNT 50


// ==================== RGB LED ========================
#define RED_PIN   3
#define GREEN_PIN 4
#define BLUE_PIN  5

// Common cathode: analogWrite(255) = full brightness. Do not invert.
const bool COMMON_CATHODE = true;


// ==================== ROTARY ENCODER =================
#define ENCODER_CLK 7
#define ENCODER_DT  6
#define ENCODER_SW  2


// ==================== TFT SCREEN =====================
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST   8
#define TFT_MOSI 11
#define TFT_SCLK 12

Adafruit_ST7735 tft = Adafruit_ST7735(
  TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST
);


// ==================== SOUND BUFFER ===================
#define BUFFER_SIZE 5

float dBBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int readingsStored = 0;


// ==================== SYSTEM STATE ===================
#define MODE_SOUND   0
#define MODE_AMBIENT 1
#define MODE_CLOCK   2

int mode = MODE_SOUND;

float currentDB = 0.0;
float averageDB = 0.0;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 200;


// ==================== CLOCK ==========================
int clockHours   = 12;
int clockMinutes = 0;
int clockSeconds = 0;
unsigned long lastTick = 0;

bool isSettingTime = false;
int  setField = 0;          // 0 = hours, 1 = minutes


// ==================== ENCODER / BUTTON STATE =========
int lastCLKState = HIGH;
int lastButtonReading = HIGH;
int stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 40;

unsigned long lastEncoderChange = 0;
const unsigned long ENCODER_DEBOUNCE = 2;

unsigned long buttonPressStart = 0;
bool buttonHeld = false;
const unsigned long LONG_PRESS_MS = 800;


// ==================== COLOR PALETTE ==================
const int COLOR_COUNT = 12;
int ambientColor = 0;

const uint8_t ambientColors[COLOR_COUNT][3] = {
  {  0,   0, 255},  // 1. BLUE
  {  0, 100, 255},  // 2. BLUE-CYAN
  {  0, 255, 255},  // 3. CYAN
  {  0, 255, 100},  // 4. CYAN-GREEN
  {  0, 255,   0},  // 5. GREEN
  {100, 255,   0},  // 6. YELLOW-GREEN
  {255, 255,   0},  // 7. YELLOW
  {255, 150,   0},  // 8. ORANGE
  {255,  50,   0},  // 9. ORANGE-RED
  {255,   0,   0},  // 10. RED
  {180,   0, 150},  // 11. RED-PURPLE
  { 80,   0, 180}   // 12. DEEP PURPLE
};

const char* ambientColorNames[COLOR_COUNT] = {
  "BLUE",
  "BLUE-CYAN",
  "CYAN",
  "CYAN-GREEN",
  "GREEN",
  "YELLOW-GREEN",
  "YELLOW",
  "ORANGE",
  "ORANGE-RED",
  "RED",
  "RED-PURPLE",
  "DEEP PURPLE"
};

int lastDrawnMode = -1;
int lastDrawnColor = -1;
int lastDrawnDBTenths = -9999;
int lastDrawnHours = -1;
int lastDrawnMinutes = -1;
int lastDrawnSeconds = -1;
bool lastDrawnSetting = false;
int lastDrawnSetField = -1;


// ==================== RGB LED FUNCTION ===============
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


// ==================== TWO-MICROPHONE READER ==========
float readLouderMicrophone() {
  // MIC A
  long sumA = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sumA += analogRead(MIC_A_PIN);
    delayMicroseconds(200);
  }
  float meanA = (float)sumA / SAMPLE_COUNT;

  float sumSquaresA = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float diff = analogRead(MIC_A_PIN) - meanA;
    sumSquaresA += diff * diff;
    delayMicroseconds(200);
  }
  float rmsA = sqrt(sumSquaresA / SAMPLE_COUNT);
  if (rmsA < 0.1) rmsA = 0.1;
  float dBA = 20.0 * log10(rmsA);

  // MIC B
  long sumB = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sumB += analogRead(MIC_B_PIN);
    delayMicroseconds(200);
  }
  float meanB = (float)sumB / SAMPLE_COUNT;

  float sumSquaresB = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float diff = analogRead(MIC_B_PIN) - meanB;
    sumSquaresB += diff * diff;
    delayMicroseconds(200);
  }
  float rmsB = sqrt(sumSquaresB / SAMPLE_COUNT);
  if (rmsB < 0.1) rmsB = 0.1;
  float dBB = 20.0 * log10(rmsB);

  return (dBA > dBB) ? dBA : dBB;
}


// ==================== WEIGHTED BUFFER ================
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
  int oldestIndex = (bufferIndex - readingsStored + BUFFER_SIZE) % BUFFER_SIZE;

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


// --------------------------------
// Same thresholds as the LED colour steps
// --------------------------------
int getSoundColorIndex(float averageDB) {
  if      (averageDB <  2) return 0;
  else if (averageDB <  4) return 1;
  else if (averageDB <  6) return 2;
  else if (averageDB <  8) return 3;
  else if (averageDB < 10) return 4;
  else if (averageDB < 12) return 5;
  else if (averageDB < 14) return 6;
  else if (averageDB < 16) return 7;
  else if (averageDB < 18) return 8;
  else if (averageDB < 20) return 9;
  else if (averageDB < 22) return 10;
  else                     return 11;
}


// --------------------------------
// RGB LED  (Sound Reactive)
// --------------------------------
void updateLED(float averageDB) {
  int idx = getSoundColorIndex(averageDB);
  setColor(ambientColors[idx][0],
           ambientColors[idx][1],
           ambientColors[idx][2]);
}


// ==================== AMBIENT LAMP ===================
void setAmbientColor() {
  setColor(ambientColors[ambientColor][0],
           ambientColors[ambientColor][1],
           ambientColors[ambientColor][2]);
}


// ==================== TFT HELPERS ====================
uint16_t getTFTColor(int red, int green, int blue) {
  return tft.color565(red, green, blue);
}


void drawScreenBase() {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 5);
  tft.println("LIGHT MONITOR");

  tft.drawFastHLine(5, 19, 150, ST77XX_WHITE);

  // Labels that stay for all modes
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 28);
  tft.print("MODE:");

  lastDrawnMode = -1;
  lastDrawnColor = -1;
  lastDrawnDBTenths = -9999;
  lastDrawnHours = -1;
  lastDrawnMinutes = -1;
  lastDrawnSeconds = -1;
  lastDrawnSetting = false;
  lastDrawnSetField = -1;
}


void updateScreen() {
  int colorIndex = (mode == MODE_AMBIENT) ? ambientColor : getSoundColorIndex(averageDB);
  int dbTenths   = (int)(averageDB * 10.0);

  // ----- MODE line (all modes) -----
  if (mode != lastDrawnMode || isSettingTime != lastDrawnSetting) {
    tft.fillRect(45, 28, 110, 12, ST77XX_BLACK);
    tft.setCursor(45, 28);
    tft.setTextColor(ST77XX_GREEN);
    if (mode == MODE_SOUND) {
      tft.print("SOUND REACTIVE");
    } else if (mode == MODE_AMBIENT) {
      tft.print("AMBIENT LAMP");
    } else {
      tft.print(isSettingTime ? "CLOCK  [SET]" : "CLOCK");
    }
    lastDrawnMode = mode;
    lastDrawnSetting = isSettingTime;
  }

  // ========== CLOCK MODE ==========
  if (mode == MODE_CLOCK) {
    // Large time
    bool timeDirty = (clockHours != lastDrawnHours) ||
                     (clockMinutes != lastDrawnMinutes) ||
                     (clockSeconds != lastDrawnSeconds) ||
                     (isSettingTime != lastDrawnSetting) ||
                     (setField != lastDrawnSetField);

    if (timeDirty) {
      // Clear time area
      tft.fillRect(5, 48, 150, 40, ST77XX_BLACK);

      // HH:MM
      tft.setTextSize(3);
      tft.setCursor(18, 50);

      // Hours (highlight if setting hours)
      if (isSettingTime && setField == 0) {
        tft.setTextColor(ST77XX_YELLOW);
      } else {
        tft.setTextColor(ST77XX_WHITE);
      }
      if (clockHours < 10) tft.print('0');
      tft.print(clockHours);

      tft.setTextColor(ST77XX_WHITE);
      tft.print(':');

      // Minutes (highlight if setting minutes)
      if (isSettingTime && setField == 1) {
        tft.setTextColor(ST77XX_YELLOW);
      } else {
        tft.setTextColor(ST77XX_WHITE);
      }
      if (clockMinutes < 10) tft.print('0');
      tft.print(clockMinutes);

      // Seconds (smaller)
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(110, 58);
      if (clockSeconds < 10) tft.print('0');
      tft.print(clockSeconds);

      // SET indicator
      if (isSettingTime) {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(5, 90);
        tft.print(setField == 0 ? "SET HOURS  (press=min)" : "SET MINS   (press=hrs)");
      } else {
        tft.fillRect(5, 90, 150, 10, ST77XX_BLACK);
      }

      lastDrawnHours    = clockHours;
      lastDrawnMinutes  = clockMinutes;
      lastDrawnSeconds  = clockSeconds;
      lastDrawnSetField = setField;
    }

    // Small relative dB + bar (LED is still reacting)
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 105);
    tft.print("dB:");
    tft.fillRect(30, 105, 40, 10, ST77XX_BLACK);
    tft.setCursor(30, 105);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(averageDB, 1);

    // Mini level bar
    tft.fillRect(80, 105, 70, 8, ST77XX_BLACK);
    uint16_t barColor = getTFTColor(
      ambientColors[colorIndex][0],
      ambientColors[colorIndex][1],
      ambientColors[colorIndex][2]
    );
    float boundedDB = constrain(averageDB, 0.0, 22.0);
    int barWidth = (int)(boundedDB / 22.0 * 68.0);
    if (barWidth > 0) {
      tft.fillRect(81, 106, barWidth, 6, barColor);
    }

  // ========== SOUND / AMBIENT MODES ==========
  } else {
    // Clear any leftover clock graphics if we just left clock mode
    if (lastDrawnHours != -1) {
      tft.fillRect(5, 48, 150, 70, ST77XX_BLACK);
      lastDrawnHours = -1;
    }

    // dB value
    bool textDirty =
      (mode == MODE_AMBIENT && colorIndex != lastDrawnColor) ||
      (mode == MODE_SOUND && dbTenths != lastDrawnDBTenths) ||
      (mode != lastDrawnMode);

    if (textDirty || lastDrawnDBTenths == -9999) {
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(5, 48);
      tft.print("REL dB:");

      tft.fillRect(50, 48, 105, 12, ST77XX_BLACK);
      tft.setCursor(50, 48);
      tft.setTextColor(ST77XX_YELLOW);
      if (mode == MODE_SOUND) {
        tft.print(averageDB, 1);
      } else {
        tft.print("PAUSED");
      }

      // Color name
      tft.setCursor(5, 68);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("COLOR:");
      tft.fillRect(45, 68, 110, 12, ST77XX_BLACK);
      tft.setCursor(45, 68);
      if (mode == MODE_AMBIENT) {
        tft.print(ambientColorNames[ambientColor]);
      } else {
        tft.print("AUTO");
      }

      tft.setCursor(5, 88);
      tft.print("LIGHT LEVEL");
      tft.drawRect(5, 102, 150, 13, ST77XX_WHITE);

      lastDrawnColor    = colorIndex;
      lastDrawnDBTenths = dbTenths;
    }

    // Level bar
    tft.fillRect(7, 104, 146, 9, ST77XX_BLACK);
    uint16_t barColor = getTFTColor(
      ambientColors[colorIndex][0],
      ambientColors[colorIndex][1],
      ambientColors[colorIndex][2]
    );

    if (mode == MODE_SOUND) {
      float boundedDB = constrain(averageDB, 0.0, 22.0);
      int barWidth = (int)(boundedDB / 22.0 * 146.0);
      if (barWidth > 0) {
        tft.fillRect(7, 104, barWidth, 9, barColor);
      }
    } else {
      tft.fillRect(7, 104, 146, 9, barColor);
    }
  }
}


// ==================== BUTTON (MODE + SET) ============
void checkEncoderButton() {
  int reading = digitalRead(ENCODER_SW);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        // Button just pressed
        buttonPressStart = millis();
        buttonHeld = false;
      } else {
        // Button just released
        unsigned long pressDuration = millis() - buttonPressStart;

        if (pressDuration >= LONG_PRESS_MS) {
          // Long press
          if (mode == MODE_CLOCK) {
            isSettingTime = !isSettingTime;
            if (isSettingTime) {
              setField = 0;               // start with hours
              Serial.println(F("CLOCK: Entered SET mode"));
            } else {
              Serial.println(F("CLOCK: Left SET mode"));
            }
            updateScreen();
          }
        } else {
          // Short press
          if (mode == MODE_CLOCK && isSettingTime) {
            // Toggle hours <-> minutes while setting
            setField = 1 - setField;
            updateScreen();
            Serial.print(F("SET field: "));
            Serial.println(setField == 0 ? "HOURS" : "MINUTES");
          } else {
            // Cycle modes
            mode = (mode + 1) % 3;   // 0→1→2→0

            isSettingTime = false;   // always leave set mode when changing modes

            if (mode == MODE_AMBIENT) {
              ambientColor = 0;
              setAmbientColor();
              Serial.println(F("\nMODE: AMBIENT LAMP"));
            } else if (mode == MODE_SOUND) {
              resetBuffer();
              Serial.println(F("\nMODE: SOUND REACTIVE"));
            } else {
              Serial.println(F("\nMODE: CLOCK"));
            }
            drawScreenBase();        // clean redraw when mode changes
            updateScreen();
          }
        }
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

      bool clockwise = (digitalRead(ENCODER_DT) != currentCLK);

      if (mode == MODE_AMBIENT) {
        // Ambient colour change
        if (clockwise) ambientColor++;
        else           ambientColor--;

        if (ambientColor >= COLOR_COUNT) ambientColor = 0;
        if (ambientColor < 0)            ambientColor = COLOR_COUNT - 1;

        setAmbientColor();
        updateScreen();

        Serial.print(F("Ambient Color: "));
        Serial.print(ambientColor);
        Serial.print(F(" - "));
        Serial.println(ambientColorNames[ambientColor]);

      } else if (mode == MODE_CLOCK && isSettingTime) {
        // Adjust hours or minutes
        if (setField == 0) {
          // Hours
          if (clockwise) clockHours++;
          else           clockHours--;
          if (clockHours > 23) clockHours = 0;
          if (clockHours < 0)  clockHours = 23;
        } else {
          // Minutes
          if (clockwise) clockMinutes++;
          else           clockMinutes--;
          if (clockMinutes > 59) clockMinutes = 0;
          if (clockMinutes < 0)  clockMinutes = 59;
        }
        clockSeconds = 0;          // reset seconds when setting
        lastTick = millis();       // keep tick aligned
        updateScreen();

        Serial.print(F("Set time: "));
        if (clockHours < 10) Serial.print('0');
        Serial.print(clockHours);
        Serial.print(':');
        if (clockMinutes < 10) Serial.print('0');
        Serial.println(clockMinutes);
      }
    }
  }
  lastCLKState = currentCLK;
}


// ==================== CLOCK TICK =====================
void updateClock() {
  if (isSettingTime) return;          // freeze while setting

  if (millis() - lastTick >= 1000) {
    lastTick += 1000;
    clockSeconds++;

    if (clockSeconds >= 60) {
      clockSeconds = 0;
      clockMinutes++;
      if (clockMinutes >= 60) {
        clockMinutes = 0;
        clockHours++;
        if (clockHours >= 24) clockHours = 0;
      }
    }
    // Screen will be refreshed in the main loop
  }
}


// ==================== SETUP ==========================
void setup() {
  Serial.begin(115200);

  pinMode(RED_PIN,   OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN,  OUTPUT);

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);
  pinMode(ENCODER_SW,  INPUT_PULLUP);

  // Uncomment only if your board supports 12-bit ADC
  // analogReadResolution(12);

  lastCLKState = digitalRead(ENCODER_CLK);
  lastTick = millis();

  resetBuffer();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawScreenBase();

  setColor(0, 0, 255);
  updateScreen();

  Serial.println(F("LIGHT MONITOR READY (2 mics + screen + encoder + CLOCK)"));
  Serial.println(F("Modes: SOUND REACTIVE → AMBIENT LAMP → CLOCK"));
  Serial.println(F("Clock: Long-press = enter/exit SET, Short-press in SET = toggle H/M"));
  Serial.println(F("Mic A = A0 | Mic B = A1  (louder wins)"));
  Serial.println(F("Encoder CLK:D7  DT:D6  SW:D2"));
  Serial.println(F("RGB LED: R=D3 G=D4 B=D5"));
}


// ==================== MAIN LOOP ======================
void loop() {
  checkEncoderButton();
  checkEncoder();

  // Sound reading + LED update for both Sound Reactive AND Clock modes
  if (mode == MODE_SOUND || mode == MODE_CLOCK) {
    if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
      lastUpdateTime = millis();

      currentDB = readLouderMicrophone();
      averageDB = updateBuffer(currentDB);

      updateLED(averageDB);          // LED always reacts in these two modes

      if (mode == MODE_SOUND) {
        updateScreen();
      }
      // Clock mode screen is updated below after the tick
    }
  }

  // Clock-specific work
  if (mode == MODE_CLOCK) {
    updateClock();
    updateScreen();                  // keeps time + mini bar fresh
  }

  // Small yield so the encoder stays responsive
  delay(2);
}

//How are you today?