#include <Arduino.h>
#include <U8g2lib.h>

/*
 * =====================================================================================
 * PROJECT: Turbidity Meter with Digital Water Detection & Confirmation Switch
 * TARGET : ESP32-C3 SuperMini
 * DISPLAY: OPEN-SMART ST7567 Monochrome LCD (128x64) via U8g2
 * =====================================================================================
 * 
 * FINAL PIN ASSIGNMENTS:
 * -------------------------------------------------------------------------------------
 *  Display SCK          -> GPIO 3  (Software SPI Clock)
 *  Display SDA (MOSI)   -> GPIO 6  (Software SPI Data)
 *  Display CS           -> GPIO 7  (Chip Select - Active Low)
 *  Display DC           -> GPIO 10 (Data / Command select)
 *  Display RS (RST)     -> GPIO 5  (Hardware Reset - Active Low)
 *  Display LED          -> GND     (Backlight ON - Active Low on Open-Smart board)
 *  Buzzer               -> GPIO 4  (PWM Tone / Digital Output)
 *  Turbidity Sensor A0  -> GPIO 1  (ADC1_CH1 with 1:1 voltage divider for 5V sensor)
 *  Water Detector Wires -> GND & GPIO 0 (Pure Digital INPUT_PULLUP - No ADC disconnection)
 *  Switch / Button      -> GPIO 20 & GND (Internal Pull-Up)
 *  Onboard Blue LED     -> GPIO 8  (Active LOW)
 * =====================================================================================
 * 
 * WATER PROBE SETUP TIP:
 *  Strip about 1 to 2 cm of insulation from the two probe wires (GND and GPIO 0).
 *  Hold them parallel about 1 to 2 mm apart (making sure the metal tips do not touch).
 *  When submerged, the water bridges the small gap and pulls GPIO 0 to digital LOW.
 * =====================================================================================
 */

// --- Display Pins ---
#define PIN_SCK      3
#define PIN_SDA      6
#define PIN_CS       7
#define PIN_DC       10
#define PIN_RST      5

// --- Sensor & Interface Pins ---
#define PIN_BUZZER    4
#define PIN_TURBIDITY 1   // Analog input from Turbidity Sensor A0 (via 1:1 divider)
#define PIN_WATER     0   // Pure digital water detector probe (other wire to GND)
#define PIN_SWITCH   20   // Confirmation switch (connected to GND)
#define PIN_LED       8   // Onboard Blue LED

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
  STATE_IDLE_WAIT_WATER,   // Turbidity sensor idle, waiting for water probe
  STATE_PROMPT_CHECK,      // Water detected! Prompt: Short press = Confirm, Long = Reject
  STATE_MEASURING,         // Reading turbidity sensor samples
  STATE_SHOW_RESULT,       // Displaying turbidity (Volts, NTU, clarity status)
  STATE_REJECTED           // User rejected reading (wait for reset)
};

SystemState currentState = STATE_IDLE_WAIT_WATER;

// Button Events
enum ButtonEvent {
  BTN_NONE,
  BTN_SHORT_PRESS,
  BTN_LONG_PRESS
};

// Global Measurement Data
float measuredSensorVolts = 0.0f;
float calculatedNTU = 0.0f;
const char* clarityStatus = "Unknown";
const char* qualityAdvice = "Unknown";

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
  tone(PIN_BUZZER, 1200, 250);
  delay(260);
  noTone(PIN_BUZZER);
}

void beepSuccess() {
  tone(PIN_BUZZER, 2400, 70);
  delay(80);
  tone(PIN_BUZZER, 3300, 130);
  delay(140);
  noTone(PIN_BUZZER);
}

// -----------------------------------------------------------------------------
// Button Handler (Non-Blocking Short vs Long Press)
// -----------------------------------------------------------------------------
ButtonEvent readButton() {
  static bool lastPinState = HIGH;
  static bool isPressed = false;
  static unsigned long pressStartTime = 0;
  static bool longPressTriggered = false;

  bool currentPinState = digitalRead(PIN_SWITCH);
  ButtonEvent event = BTN_NONE;

  // Button pressed (Active LOW)
  if (currentPinState == LOW && lastPinState == HIGH) {
    pressStartTime = millis();
    isPressed = true;
    longPressTriggered = false;
  }
  // Button held down
  else if (currentPinState == LOW && isPressed) {
    if (!longPressTriggered && (millis() - pressStartTime >= LONG_PRESS_MS)) {
      longPressTriggered = true;
      event = BTN_LONG_PRESS; // Trigger immediately once threshold is crossed
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
// Water Presence Detection (Pure Digital INPUT_PULLUP with 150ms Debounce)
// -----------------------------------------------------------------------------
bool checkWaterPresence() {
  // Pure digital read: Internal pull-up stays permanently connected!
  // In air: Pin is held firmly at 3.3V (HIGH) -> rawPresent = false (no false triggers)
  // In water: Liquid conduction pulls pin to LOW -> rawPresent = true
  bool rawPresent = (digitalRead(PIN_WATER) == LOW);

  if (rawPresent != lastRawWaterState) {
    lastWaterDebounceTime = millis();
    lastRawWaterState = rawPresent;
  }

  // 150ms stable reading required to confirm state change
  if ((millis() - lastWaterDebounceTime) > 150) {
    debouncedWaterPresent = rawPresent;
  }

  return debouncedWaterPresent;
}

// -----------------------------------------------------------------------------
// Turbidity Measurement & NTU Computation (GPIO 1)
// -----------------------------------------------------------------------------
void performTurbidityMeasurement() {
  Serial.println("[TURBIDITY] Starting multi-sample measurement on GPIO 1...");

  uint32_t sumMillivolts = 0;
  const int SAMPLES = 40;

  for (int i = 0; i < SAMPLES; i++) {
    sumMillivolts += analogReadMilliVolts(PIN_TURBIDITY);
    delay(15);
  }

  float espAdcVolts = (sumMillivolts / (float)SAMPLES) / 1000.0f;
  // Multiply by 2.0 to account for 1:1 voltage divider
  measuredSensorVolts = espAdcVolts * VOLTAGE_DIVIDER_FACTOR;

  /*
   * Standard Analog Turbidity Sensor characteristic curve (DFRobot style):
   * Pure/clear water: ~4.10V - 4.30V
   * Turbid/cloudy water: voltage drops
   * Polynomial formula: NTU = -1120.4 * V^2 + 5742.3 * V - 4352.9
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

  Serial.printf("[TURBIDITY] ADC: %.2fV | Sensor (x2): %.2fV | NTU: %.1f | Status: %s | Advice: %s\n",
                espAdcVolts, measuredSensorVolts, calculatedNTU, clarityStatus, qualityAdvice);
}

// -----------------------------------------------------------------------------
// UI Rendering Functions (128x64 Monochrome)
// -----------------------------------------------------------------------------
void renderIdleScreen() {
  u8g2.clearBuffer();

  // Title Bar
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(10, 9, "JAMES WATER TESTER");
  u8g2.setDrawColor(1);

  // Status prompt
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(12, 28, "Waiting for Water");

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(8, 46, "Water would be detected");

  // Animated probe frame
  static uint8_t animTick = 0;
  animTick++;
  u8g2.drawStr(20, 55, "  automatically");
  if ((animTick / 3) % 2 == 0) {
    u8g2.drawDisc(102, 55, 3);
  }

  u8g2.sendBuffer();
}

void renderPromptScreen() {
  u8g2.clearBuffer();

  // Inverted header banner
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(14, 9, "WATER DETECTED!");
  u8g2.setDrawColor(1);

  // Question Prompt
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 28, "Check Turbidity?");

  // Divider
  u8g2.drawHLine(4, 32, 120);

  // Switch Instructions
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(4, 44, "Press -> YES");
  u8g2.drawStr(4, 54, "Hold -> NO");

  u8g2.sendBuffer();
}

void renderMeasuringScreen(int progressPct) {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(24, 9, "MEASURING...");
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(10, 28, "Sampling Turbidity A0");

  // Progress Bar
  u8g2.drawFrame(10, 36, 108, 12);
  int fillWidth = (progressPct * 104) / 100;
  if (fillWidth > 104) fillWidth = 104;
  u8g2.drawBox(12, 38, fillWidth, 8);

  char pctStr[16];
  snprintf(pctStr, sizeof(pctStr), "%d%%", progressPct);
  u8g2.drawStr(54, 58, pctStr);

  u8g2.sendBuffer();
}

void renderResultScreen() {
  u8g2.clearBuffer();

  // Title Bar
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(18, 9, "TURBIDITY RESULT");
  u8g2.setDrawColor(1);

  // Measured Values
  char ntuBuf[22];
  snprintf(ntuBuf, sizeof(ntuBuf), "NTU : %.0f NTU", calculatedNTU);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 24, ntuBuf);

  // Clarity Status Label
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 36, clarityStatus);

  // Quality Advice
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(2, 47, qualityAdvice);

  // Footer / Next actions
  u8g2.drawHLine(0, 52, 128);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(2, 62, "Press -> Retest   Hold -> Exit");

  u8g2.sendBuffer();
}

void renderRejectedScreen() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(14, 9, "CHECK CANCELLED");
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 32, "Measurement cancelled");

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(8, 48, "Remove detector to reset");

  u8g2.sendBuffer();
}

// -----------------------------------------------------------------------------
// Arduino Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1200); // Allow USB CDC to initialize

  Serial.println("\n========================================================");
  Serial.println("  ESP32-C3 Turbidity Meter - Digital Water Mode Active  ");
  Serial.println("========================================================");

  // Pin Modes
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // Off initially (Active LOW)

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Configure Water Detector with pure INPUT_PULLUP (never disconnected by ADC)
  pinMode(PIN_WATER, INPUT_PULLUP);
  pinMode(PIN_SWITCH, INPUT_PULLUP); // Momentary push button on GPIO 21

  // Start Buzzer Check
  tone(PIN_BUZZER, 2200, 80);
  delay(100);
  noTone(PIN_BUZZER);

  // Initialize U8g2 Monochrome LCD
  Serial.println("[U8G2] Initializing ST7567 Monochrome LCD...");
  u8g2.begin();
  u8g2.setContrast(120);

  // Startup Screen - Display Project Name
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setDrawColor(0);
  u8g2.drawStr(10, 9, "JAMES WATER TESTER");
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 38, "James Water Tester");
  u8g2.sendBuffer();
  delay(2000);

  renderIdleScreen();
  stateEnterTime = millis();

  Serial.println("[SETUP] System ready. Water pin locked HIGH by pull-up.");
}

// -----------------------------------------------------------------------------
// Main Loop - State Machine
// -----------------------------------------------------------------------------
void loop() {
  bool waterPresent = checkWaterPresence();
  ButtonEvent btn = readButton();

  switch (currentState) {

    // -------------------------------------------------------------------------
    // STATE 1: IDLE - WAITING FOR WATER
    // -------------------------------------------------------------------------
    case STATE_IDLE_WAIT_WATER: {
      digitalWrite(PIN_LED, HIGH);

      // Refresh idle animation every 250ms
      static unsigned long lastIdleRender = 0;
      if (millis() - lastIdleRender >= 250) {
        lastIdleRender = millis();
        renderIdleScreen();
      }

      // Transition when water is detected
      if (waterPresent) {
        Serial.println("[STATE] Water detected (GPIO 0 pulled LOW)! Prompting user.");
        beepAlert();
        currentState = STATE_PROMPT_CHECK;
        stateEnterTime = millis();
        renderPromptScreen();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // STATE 2: WATER DETECTED - CONFIRMATION PROMPT
    // -------------------------------------------------------------------------
    case STATE_PROMPT_CHECK: {
      digitalWrite(PIN_LED, LOW); // LED ON while awaiting user decision

      // Screen only clears if water has stopped being detected
      if (!waterPresent) {
        Serial.println("[STATE] Water no longer detected. Returning to IDLE.");
        currentState = STATE_IDLE_WAIT_WATER;
        break;
      }

      // User short press -> Confirm and read turbidity
      if (btn == BTN_SHORT_PRESS) {
        Serial.println("[ACTION] User CONFIRMED turbidity check (Short press).");
        currentState = STATE_MEASURING;
        stateEnterTime = millis();
      } 
      // User long press -> Reject reading
      else if (btn == BTN_LONG_PRESS) {
        Serial.println("[ACTION] User REJECTED turbidity check (Long press).");
        currentState = STATE_REJECTED;
        stateEnterTime = millis();
        renderRejectedScreen();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // STATE 3: MEASURING TURBIDITY
    // -------------------------------------------------------------------------
    case STATE_MEASURING: {
      digitalWrite(PIN_LED, (millis() % 200 < 100) ? LOW : HIGH); // Fast blinking

      // Progress animation across ~1 second
      for (int pct = 0; pct <= 100; pct += 20) {
        renderMeasuringScreen(pct);
        delay(100);
      }

      // Read 40 oversampled ADC measurements on GPIO 1 and calculate NTU
      performTurbidityMeasurement();
      beepSuccess();

      currentState = STATE_SHOW_RESULT;
      stateEnterTime = millis();
      renderResultScreen();
      break;
    }

    // -------------------------------------------------------------------------
    // STATE 4: SHOW RESULT
    // -------------------------------------------------------------------------
    case STATE_SHOW_RESULT: {
      digitalWrite(PIN_LED, LOW); // Solid ON

      // User short press to re-measure
      if (btn == BTN_SHORT_PRESS) {
        Serial.println("[ACTION] Re-measuring turbidity...");
        currentState = STATE_MEASURING;
        stateEnterTime = millis();
      }
      // User long press to reset / exit back to idle
      else if (btn == BTN_LONG_PRESS) {
        Serial.println("[ACTION] Exiting result screen to IDLE.");
        currentState = STATE_IDLE_WAIT_WATER;
        stateEnterTime = millis();
      }
      break;
    }

    // -------------------------------------------------------------------------
    // STATE 5: REJECTED / CANCELLED
    // -------------------------------------------------------------------------
    case STATE_REJECTED: {
      digitalWrite(PIN_LED, HIGH);

      // Return to IDLE automatically after 2.5 seconds or when probe is withdrawn
      if (!waterPresent || (millis() - stateEnterTime > 2500)) {
        currentState = STATE_IDLE_WAIT_WATER;
      }
      break;
    }
  }

  delay(10);
}
