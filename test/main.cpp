#include <Arduino.h>
#include <U8g2lib.h>

/*
 * =====================================================================================
 * PROJECT: Turbidity Meter - Production / Release Firmware
 * TARGET : ESP32-C3 SuperMini
 * DISPLAY: OPEN-SMART ST7567 Monochrome LCD (128x64) via U8g2
 * =====================================================================================
 * 
 * LOCKED HARDWARE PIN MAPPING:
 * -------------------------------------------------------------------------------------
 *  Display SCK          -> GPIO 3  (Software SPI Clock)
 *  Display SDA (MOSI)   -> GPIO 6  (Software SPI Data)
 *  Display CS           -> GPIO 7  (Chip Select - Active Low)
 *  Display DC           -> GPIO 10 (Data / Command Select)
 *  Display RS (RST)     -> GPIO 5  (Hardware Reset - Active Low)
 *  Display LED          -> GND     (Backlight Active Low)
 *  Buzzer               -> GPIO 4  (PWM Tone / Audio Alerts)
 *  Turbidity Sensor A0  -> GPIO 1  (ADC1_CH1 with 1:1 voltage divider)
 *  Water Detector Wires -> GND & GPIO 0 (Digital Water Detection)
 *  User Switch / Button -> GPIO 20 & GND (Internal Pull-Up)
 *  Onboard Blue LED     -> GPIO 8  (Active LOW)
 * =====================================================================================
 */

// --- Display Pins ---
#define PIN_SCK      3
#define PIN_SDA      6
#define PIN_CS       7
#define PIN_DC       10
#define PIN_RST      5

// --- Peripheral Pins ---
#define PIN_BUZZER    4
#define PIN_TURBIDITY 1   // Analog Turbidity Sensor A0 (via 1:1 voltage divider)
#define PIN_WATER     0   // Water detector wire (other wire to GND)
#define PIN_SWITCH   20   // User interaction switch (connected to GND)
#define PIN_LED       8   // Onboard Blue Status LED

// --- Voltage Divider Multiplier ---
// 1:1 voltage divider halves the 0-5V sensor output to 0-2.5V for the ESP32 ADC.
// Multiply measured ESP32 voltage by 2.0 to reconstruct the actual 0-5V sensor voltage.
#define VOLTAGE_DIVIDER_FACTOR 2.0f

// --- Button Timing Constants ---
#define DEBOUNCE_MS     50
#define LONG_PRESS_MS 1000

// U8g2 4-Wire Software SPI Constructor
U8G2_ST7567_JLX12864_F_4W_SW_SPI u8g2(
    U8G2_R0, 
    /* clock=*/ PIN_SCK, 
    /* data=*/  PIN_SDA, 
    /* cs=*/    PIN_CS, 
    /* dc=*/    PIN_DC, 
    /* reset=*/ PIN_RST
);

// System State Machine
enum SystemState {
  STATE_IDLE_WAIT_WATER,   // Waiting for user to dip probe in water
  STATE_PROMPT_CHECK,      // Water detected: Ask user to test or cancel
  STATE_MEASURING,         // Reading and averaging sensor samples
  STATE_SHOW_RESULT,       // Displaying clean, easy-to-read water quality result
  STATE_REJECTED           // User chose to skip/cancel test
};

SystemState currentState = STATE_IDLE_WAIT_WATER;

// Button Events
enum ButtonEvent {
  BTN_NONE,
  BTN_SHORT_PRESS,
  BTN_LONG_PRESS
};

// Measurement Data
float measuredSensorVolts = 0.0f;
float calculatedNTU = 0.0f;
const char* clarityStatus = "Clean";
const char* qualityAdvice = "Safe for use";

// State Timing & Debounce Variables
unsigned long stateEnterTime = 0;
unsigned long lastWaterDebounceTime = 0;
bool lastRawWaterState = false;
bool debouncedWaterPresent = false;

// -----------------------------------------------------------------------------
// Audio Feedback Helpers
// -----------------------------------------------------------------------------
void beepAlert() {
  tone(PIN_BUZZER, 2400, 100);
  delay(110);
  tone(PIN_BUZZER, 3100, 160);
  delay(170);
  noTone(PIN_BUZZER);
}

void beepConfirm() {
  tone(PIN_BUZZER, 2800, 80);
  delay(90);
  noTone(PIN_BUZZER);
}

void beepReject() {
  tone(PIN_BUZZER, 1200, 220);
  delay(230);
  noTone(PIN_BUZZER);
}

void beepSuccess() {
  tone(PIN_BUZZER, 2400, 70);
  delay(80);
  tone(PIN_BUZZER, 3300, 140);
  delay(150);
  noTone(PIN_BUZZER);
}

// -----------------------------------------------------------------------------
// Non-Blocking Button Handler (Short Press vs Long Press)
// -----------------------------------------------------------------------------
ButtonEvent readButton() {
  static bool lastPinState = HIGH;
  static bool isPressed = false;
  static unsigned long pressStartTime = 0;
  static bool longPressTriggered = false;

  bool currentPinState = digitalRead(PIN_SWITCH);
  ButtonEvent event = BTN_NONE;

  // Button just pressed (Active LOW)
  if (currentPinState == LOW && lastPinState == HIGH) {
    pressStartTime = millis();
    isPressed = true;
    longPressTriggered = false;
  }
  // Button held down (Check for long press >= 1000ms)
  else if (currentPinState == LOW && isPressed) {
    if (!longPressTriggered && (millis() - pressStartTime >= LONG_PRESS_MS)) {
      longPressTriggered = true;
      event = BTN_LONG_PRESS;
      beepReject();
    }
  }
  // Button released
  else if (currentPinState == HIGH && lastPinState == LOW) {
    if (isPressed) {
      unsigned long duration = millis() - pressStartTime;
      if (!longPressTriggered && (duration >= DEBOUNCE_MS)) {
        event = BTN_SHORT_PRESS;
        beepConfirm();
      }
      isPressed = false;
    }
  }

  lastPinState = currentPinState;
  return event;
}

// -----------------------------------------------------------------------------
// Digital Water Presence Detection (GPIO 0 with 150ms Debounce)
// -----------------------------------------------------------------------------
bool checkWaterPresence() {
  bool rawPresent = (digitalRead(PIN_WATER) == LOW);

  if (rawPresent != lastRawWaterState) {
    lastWaterDebounceTime = millis();
    lastRawWaterState = rawPresent;
  }

  if ((millis() - lastWaterDebounceTime) > 150) {
    debouncedWaterPresent = rawPresent;
  }

  return debouncedWaterPresent;
}

// -----------------------------------------------------------------------------
// Turbidity Measurement & Novice-Friendly Quality Classification
// -----------------------------------------------------------------------------
void performTurbidityMeasurement() {
  uint32_t sumMillivolts = 0;
  const int SAMPLES = 40;

  for (int i = 0; i < SAMPLES; i++) {
    sumMillivolts += analogReadMilliVolts(PIN_TURBIDITY);
    delay(15);
  }

  float espAdcVolts = (sumMillivolts / (float)SAMPLES) / 1000.0f;
  measuredSensorVolts = espAdcVolts * VOLTAGE_DIVIDER_FACTOR;

  /*
   * Standard DFRobot Turbidity Sensor Characteristic Equation:
   * NTU = -1120.4 * V^2 + 5742.3 * V - 4352.9
   */
  if (measuredSensorVolts >= 4.12f) {
    calculatedNTU = 0.0f;
  } else if (measuredSensorVolts <= 2.50f) {
    calculatedNTU = 3000.0f;
  } else {
    calculatedNTU = (-1120.4f * measuredSensorVolts * measuredSensorVolts) + 
                    (5742.3f * measuredSensorVolts) - 4352.9f;
    if (calculatedNTU < 0.0f) calculatedNTU = 0.0f;
    if (calculatedNTU > 3000.0f) calculatedNTU = 3000.0f;
  }

  // Consumer-Friendly Water Quality Rating
  if (calculatedNTU < 25.0f) {
    clarityStatus = "CRYSTAL CLEAR";
    qualityAdvice = "Excellent quality";
  } else if (calculatedNTU < 100.0f) {
    clarityStatus = "CLEAR WATER";
    qualityAdvice = "Good clarity";
  } else if (calculatedNTU < 350.0f) {
    clarityStatus = "SLIGHTLY CLOUDY";
    qualityAdvice = "Visible particles";
  } else if (calculatedNTU < 800.0f) {
    clarityStatus = "CLOUDY WATER";
    qualityAdvice = "Filtration needed";
  } else {
    clarityStatus = "VERY DIRTY";
    qualityAdvice = "Do not consume";
  }
}

// -----------------------------------------------------------------------------
// Consumer-Facing UI Screens (128x64 Monochrome)
// -----------------------------------------------------------------------------

// 1. Standby / Idle Screen
void renderIdleScreen() {
  u8g2.clearBuffer();

  // Clean Header Banner
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(18, 9, "WATER TESTER");
  u8g2.setDrawColor(1);

  // Main Status Instruction
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(8, 29, "Ready to Test");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(12, 42, "Submerge the probe");

  // Visual Standby Frame with Animated Dot
  static uint8_t animTick = 0;
  animTick++;
  u8g2.drawFrame(10, 48, 108, 14);
  u8g2.drawStr(22, 58, "Waiting for water");
  if ((animTick / 3) % 2 == 0) {
    u8g2.drawDisc(104, 55, 3);
  }

  u8g2.sendBuffer();
}

// 2. Water Detected - Confirmation Prompt Screen
void renderPromptScreen() {
  u8g2.clearBuffer();

  // Inverted Alert Header
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(14, 9, "WATER DETECTED");
  u8g2.setDrawColor(1);

  // Question
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(8, 25, "Test this water?");

  // Divider
  u8g2.drawHLine(4, 29, 120);

  // Simple, Novice-Friendly Button Instructions
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(8, 42, "Press : Start Test");
  u8g2.drawStr(8, 56, "Hold  : Cancel");

  u8g2.sendBuffer();
}

// 3. Testing / Analyzing Screen
void renderMeasuringScreen(int progressPct) {
  u8g2.clearBuffer();

  // Inverted Header
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(20, 9, "TESTING WATER");
  u8g2.setDrawColor(1);

  // Status
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(14, 27, "Analyzing clarity...");

  // Progress Bar
  u8g2.drawFrame(12, 33, 104, 12);
  int fillWidth = (progressPct * 100) / 100;
  if (fillWidth > 100) fillWidth = 100;
  u8g2.drawBox(14, 35, fillWidth, 8);

  // User Advice
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(8, 58, "Please keep probe still");

  u8g2.sendBuffer();
}

// 4. Test Result Screen
void renderResultScreen() {
  u8g2.clearBuffer();

  // Header
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(26, 9, "WATER QUALITY");
  u8g2.setDrawColor(1);

  // Main Clarity Status
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(4, 27, clarityStatus);

  // Turbidity NTU Value
  char ntuBuf[24];
  snprintf(ntuBuf, sizeof(ntuBuf), "Clarity: %.0f NTU", calculatedNTU);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(4, 40, ntuBuf);

  // Quality Advice Subtext
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(4, 51, qualityAdvice);

  // Footer Actions
  u8g2.drawHLine(0, 54, 128);
  u8g2.drawStr(4, 63, "Press: Retest | Hold: Done");

  u8g2.sendBuffer();
}

// 5. Test Cancelled Screen
void renderRejectedScreen() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(18, 9, "TEST CANCELLED");
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(12, 32, "Measurement skipped");

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(8, 48, "Remove probe to reset");

  u8g2.sendBuffer();
}

// -----------------------------------------------------------------------------
// Arduino Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(800);

  // Initialize Input / Output Pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // Off initially (Active LOW)

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_WATER, INPUT_PULLUP);  // Digital water detection wire
  pinMode(PIN_SWITCH, INPUT_PULLUP); // Locked to GPIO 20

  // Quick startup confirmation beep
  tone(PIN_BUZZER, 2200, 80);
  delay(90);
  noTone(PIN_BUZZER);

  // Initialize U8g2 Display
  u8g2.begin();
  u8g2.setContrast(120);

  renderIdleScreen();
  stateEnterTime = millis();
}

// -----------------------------------------------------------------------------
// Main Loop - State Machine
// -----------------------------------------------------------------------------
void loop() {
  bool waterPresent = checkWaterPresence();
  ButtonEvent btn = readButton();

  switch (currentState) {

    // -------------------------------------------------------------------------
    // 1. STANDBY - WAITING FOR WATER
    // -------------------------------------------------------------------------
    case STATE_IDLE_WAIT_WATER: {
      digitalWrite(PIN_LED, HIGH);

      // Smooth idle animation
      static unsigned long lastIdleRender = 0;
      if (millis() - lastIdleRender >= 250) {
        lastIdleRender = millis();
        renderIdleScreen();
      }

      // Automatically transition when water is touched
      if (waterPresent) {
        beepAlert();
        currentState = STATE_PROMPT_CHECK;
        stateEnterTime = millis();
        renderPromptScreen();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 2. WATER DETECTED - PROMPT
    // -------------------------------------------------------------------------
    case STATE_PROMPT_CHECK: {
      digitalWrite(PIN_LED, LOW); // LED ON while awaiting user decision

      // If probe was withdrawn before user decided, auto-return to standby
      if (!waterPresent && (millis() - stateEnterTime > 2000)) {
        currentState = STATE_IDLE_WAIT_WATER;
        break;
      }

      // User presses button -> Start Test
      if (btn == BTN_SHORT_PRESS) {
        currentState = STATE_MEASURING;
        stateEnterTime = millis();
      } 
      // User holds button -> Cancel Test
      else if (btn == BTN_LONG_PRESS) {
        currentState = STATE_REJECTED;
        stateEnterTime = millis();
        renderRejectedScreen();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 3. ANALYZING / MEASURING
    // -------------------------------------------------------------------------
    case STATE_MEASURING: {
      digitalWrite(PIN_LED, (millis() % 200 < 100) ? LOW : HIGH);

      // Smooth progress animation across ~1 second
      for (int pct = 0; pct <= 100; pct += 20) {
        renderMeasuringScreen(pct);
        delay(90);
      }

      // Take 40 oversampled ADC readings from GPIO 1 and compute NTU
      performTurbidityMeasurement();
      beepSuccess();

      currentState = STATE_SHOW_RESULT;
      stateEnterTime = millis();
      renderResultScreen();
      break;
    }

    // -------------------------------------------------------------------------
    // 4. DISPLAY RESULT
    // -------------------------------------------------------------------------
    case STATE_SHOW_RESULT: {
      digitalWrite(PIN_LED, LOW); // Solid ON

      // User presses button to re-test
      if (btn == BTN_SHORT_PRESS) {
        currentState = STATE_MEASURING;
        stateEnterTime = millis();
      }
      // User holds button to finish
      else if (btn == BTN_LONG_PRESS) {
        currentState = STATE_IDLE_WAIT_WATER;
        stateEnterTime = millis();
      }
      // Auto-reset when probe is taken out of water after 3.5 seconds
      else if (!waterPresent && (millis() - stateEnterTime > 3500)) {
        currentState = STATE_IDLE_WAIT_WATER;
      }
      break;
    }

    // -------------------------------------------------------------------------
    // 5. TEST CANCELLED
    // -------------------------------------------------------------------------
    case STATE_REJECTED: {
      digitalWrite(PIN_LED, HIGH);

      // Return to standby after 2.5 seconds or when probe is withdrawn
      if (!waterPresent || (millis() - stateEnterTime > 2500)) {
        currentState = STATE_IDLE_WAIT_WATER;
      }
      break;
    }
  }

  delay(10);
}
