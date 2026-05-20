// ===================================================
// MASTER CONTROLLER — TTGO T-Display
// Controls state machine, sensors, display, and sends
// sync pulses to slave (Motion 2350 Pro) via wire.
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

// ===================================================
// PIN DEFINITIONS
// ===================================================
#define TRIG_PIN       2    // HC-SR04 trigger
#define ECHO_PIN       3    // HC-SR04 echo
#define MPU_SDA        17   // MPU-6050 I2C data
#define MPU_SCL        18   // MPU-6050 I2C clock
#define MPU_ADDR       0x68
#define BUTTON_PIN     43   // Manual advance button
#define STATE_OUT_PIN  1    // Pulse out to Motion 2350 Pro
#define SLAVE_PIN      10   // Pulse out to slave TTGO

// Encoder motor — second motor driver IN3/IN4/EN
#define EM_In1         11   // IN3 on second motor driver
#define EM_In2         12   // IN4 on second motor driver
#define EM_En          13   // EN on second motor driver

// ===================================================
// GLOBAL TIMING CONSTANTS (milliseconds)
// Adjust these to tune robot behaviour without
// hunting through the state machine logic.
// ===================================================
#define ROCK_COLLECTION_TIME  1500  // ms — rock forward, back, forward each
#define RAISE_ARM_TIME        8000  // ms — time to fully raise the arm
#define TURN_SETTLE_TIME       500  // ms — extra settle after turn completes
#define RAMP_APPROACH_TIME    2000  // ms — initial fast burst onto ramp (fallback)
#define DOWN_RAMP_MIN_TIME    1000  // ms — minimum time before ultrasonic check starts

// Ultrasonic thresholds
#define DRIVE_STOP_DIST_CM    6     // cm — stop driving when object detected
#define RAMP_STOP_DIST_CM     7     // cm — stop ramp when wall detected
#define DOWN_RAMP_STOP_CM     7     // cm — stop down-ramp when bottom detected

// Encoder motor extend duration — timer based
#define EXTEND_TIME_MS         400  // ms — run encoder motor for this long
#define EXTEND_SPEED           255  // PWM 0-255 — full speed
#define PWM_FREQ              1000  // ledc PWM frequency (Hz)
#define PWM_RESOLUTION           8  // 8-bit = 0-255

// ===================================================
// COLOUR — TFT_eSPI has no light grey built-in
// ===================================================
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618

// ===================================================
// DISPLAY
// ===================================================
TFT_eSPI tft = TFT_eSPI();

// ===================================================
// STATE MACHINE
// ===================================================
enum Level {
  WAITING,       // 0  — idle, show sensor data, wait for button
  CALIBRATION,   // 1  — MPU-6050 calibration, then auto-advance
  STOPA,         // 2  — wait for button press after calibration
  ROCKFORWARD,   // 3  — drive forward for ROCK_COLLECTION_TIME ms
  ROCKBACK,      // 4  — drive back for ROCK_COLLECTION_TIME ms
  ROCKFORWARD2,  // 5  — drive forward again for ROCK_COLLECTION_TIME ms
  STOPB,         // 6  — wait for button press after rocking
  EXTEND,        // 7  — run encoder motor 3 revolutions at 255 PWM
  STOPI,         // 8  — wait for button after extend
  RAISEARM,      // 9  — raise linear actuator for RAISE_ARM_TIME ms
  STOPC,         // 10 — wait for button after arm raised
  DRIVE,         // 11 — drive forward until ultrasonic > DRIVE_STOP_DIST_CM
  STOPD,         // 12 — wait for button after drive
  TURN,          // 13 — turn until yaw <= -90 degrees
  STOPE,         // 14 — wait for button after turn
  RAMP,          // 15 — drive onto ramp until ultrasonic > RAMP_STOP_DIST_CM
  STOPF,         // 16 — wait for button at top of ramp
  STOPG,         // 17 — wait for button after (future use)
  DEPOSIT,       // 18 — deposit rocks (TODO skeleton)
  STOPH,         // 19 — wait for button after deposit
  DOWNRAMP,      // 20 — drive down ramp slowly until ultrasonic > DOWN_RAMP_STOP_CM
  FINISH         // 21 — mission complete
};

Level currentState = WAITING;

// ===================================================
// MPU-6050 VARIABLES
// ===================================================
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;

float accOffX = 0, accOffY = 0, accOffZ = 0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
bool  calibrated = false;

float yawAngle    = 0;
unsigned long lastGyroTime = 0;

// ===================================================
// SENSOR & COMMS VARIABLES
// ===================================================
long distance   = 0;
int  pulseCount = 0;

// ===================================================
// STATE TIMER — used by timed states
// ===================================================
unsigned long stateStartTime = 0;

// ===================================================
// MPU-6050 FUNCTIONS
// ===================================================
void initMPU6050() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // wake up
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); // accel ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); // gyro ±250 deg/s
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

  int16_t rawAccX  = Wire.read() << 8 | Wire.read();
  int16_t rawAccY  = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ  = Wire.read() << 8 | Wire.read();
  int16_t rawTemp  = Wire.read() << 8 | Wire.read();
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  accX  = rawAccX  / 16384.0;
  accY  = rawAccY  / 16384.0;
  accZ  = rawAccZ  / 16384.0;
  temperature = (rawTemp / 340.0) + 36.53;
  gyroX = rawGyroX / 131.0;
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;

  if (calibrated) {
    accX  -= accOffX;
    accY  -= accOffY;
    accZ  -= accOffZ;
    gyroX -= gyroOffX;
    gyroY -= gyroOffY;
    gyroZ -= gyroOffZ;
  }

  if (lastGyroTime > 0 && calibrated) {
    unsigned long now = millis();
    float dt = (now - lastGyroTime) / 1000.0;
    yawAngle += gyroZ * dt;
    lastGyroTime = now;
  } else {
    lastGyroTime = millis();
  }
}

// ===================================================
// ULTRASONIC
// ===================================================
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ===================================================
// COMMS — send a pulse to both slave boards
// ===================================================
void sendPulse() {
  digitalWrite(STATE_OUT_PIN, HIGH);
  digitalWrite(SLAVE_PIN,     HIGH);
  delay(100);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_PIN,     LOW);
  pulseCount++;
  Serial.print("PULSE SENT #");
  Serial.println(pulseCount);
}

// ===================================================
// BUTTON HELPER
// ===================================================
bool buttonPressed() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
      delay(50);
      return true;
    }
  }
  return false;
}

// ===================================================
// ADVANCE STATE — sends pulse and updates state index
// ===================================================
void advanceState() {
  sendPulse();
  currentState  = (Level)(currentState + 1);
  stateStartTime = millis();
  Serial.print("state advanced -> ");
  Serial.println(currentState);
}

// ===================================================
// CALIBRATION ROUTINE (blocking, with progress bar)
// ===================================================
void runCalibration() {
  int samples = 200;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  float sumGX = 0, sumGY = 0, sumGZ = 0;

  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATING", tft.width() / 2, 30);
  tft.setTextSize(1);
  tft.drawString("KEEP ROBOT STILL", tft.width() / 2, 55);

  int barX = 20, barY = 75, barW = tft.width() - 40, barH = 16;
  tft.drawRect(barX, barY, barW, barH, TFT_BLACK);

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

    int16_t rAX = Wire.read() << 8 | Wire.read();
    int16_t rAY = Wire.read() << 8 | Wire.read();
    int16_t rAZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // discard temp
    int16_t rGX = Wire.read() << 8 | Wire.read();
    int16_t rGY = Wire.read() << 8 | Wire.read();
    int16_t rGZ = Wire.read() << 8 | Wire.read();

    sumAX += rAX / 16384.0;
    sumAY += rAY / 16384.0;
    sumAZ += rAZ / 16384.0;
    sumGX += rGX / 131.0;
    sumGY += rGY / 131.0;
    sumGZ += rGZ / 131.0;

    int fillW = map(i, 0, samples - 1, 0, barW - 4);
    tft.fillRect(barX + 2, barY + 2, fillW, barH - 4, TFT_DARKGREEN);
    delay(10);
  }

  accOffX  = sumAX / samples;
  accOffY  = sumAY / samples;
  accOffZ  = (sumAZ / samples) - 1.0;
  gyroOffX = sumGX / samples;
  gyroOffY = sumGY / samples;
  gyroOffZ = sumGZ / samples;

  calibrated    = true;
  yawAngle      = 0;
  lastGyroTime  = millis();

  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 10;
  tft.drawString("CALIBRATED OK", 5, y);   y += 18;
  tft.drawString("AccX: " + String(accOffX, 4), 5, y); y += 14;
  tft.drawString("AccY: " + String(accOffY, 4), 5, y); y += 14;
  tft.drawString("AccZ: " + String(accOffZ, 4), 5, y); y += 14;
  tft.drawString("GyrX: " + String(gyroOffX, 4), 5, y); y += 14;
  tft.drawString("GyrY: " + String(gyroOffY, 4), 5, y); y += 14;
  tft.drawString("GyrZ: " + String(gyroOffZ, 4), 5, y); y += 18;
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PRESS BUTTON TO CONTINUE", tft.width() / 2, tft.height() - 10);
}

// ===================================================
// DISPLAY HELPERS — shared layout components
// ===================================================
void drawTopBar(String label, uint32_t colour) {
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString(label + "  |  PULSES: " + String(pulseCount), tft.width() / 2, 10);
  tft.drawLine(0, 22, tft.width(), 22, TFT_BLACK);
}

void drawBigLabel(String label, uint32_t colour) {
  tft.fillRect(0, 25, tft.width(), 60, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(4);
  tft.drawString(label, tft.width() / 2, 55);
  tft.drawLine(0, 85, tft.width(), 85, TFT_BLACK);
}

void drawInfoLines(String line1, String line2, String line3 = "") {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString(line1, tft.width() / 2, 103);
  tft.drawString(line2, tft.width() / 2, 120);
  if (line3.length()) tft.drawString(line3, tft.width() / 2, 137);
}

// ===================================================
// PER-STATE DISPLAY FUNCTIONS
// ===================================================
void displayWaiting() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("WAITING", TFT_DARKGREY);
  drawBigLabel("WAIT", TFT_DARKGREY);
  drawInfoLines("Robot idle — sensors live", "PRESS BUTTON to calibrate", "");
}

void displayStopA() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP A", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Calibration complete", "PRESS BUTTON to begin rock collect", "");
}

void displayRockForward(int pass) {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("ROCK FWD " + String(pass), TFT_NAVY);
  drawBigLabel("FWD", TFT_NAVY);
  drawInfoLines("Driving forward to collect rocks",
                "Timer: " + String(ROCK_COLLECTION_TIME) + " ms", "");
}

void displayRockBack() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("ROCK BACK", TFT_NAVY);
  drawBigLabel("BACK", TFT_NAVY);
  drawInfoLines("Reversing to consolidate rocks",
                "Timer: " + String(ROCK_COLLECTION_TIME) + " ms", "");
}

void displayStopB() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP B", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Rock collection complete", "PRESS BUTTON to extend arm", "");
}

void displayExtend() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("EXTEND", TFT_PURPLE);
  drawBigLabel("EXTND", TFT_PURPLE);
  drawInfoLines("Extending encoder motor arm",
                "3 revolutions at full speed", "");
}

void displayStopI() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP I", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Arm extended", "PRESS BUTTON to raise arm", "");
}

void displayRaiseArm() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("RAISE ARM", TFT_PURPLE);
  drawBigLabel("RAISE", TFT_PURPLE);
  drawInfoLines("Lifting actuator",
                "Timer: " + String(RAISE_ARM_TIME / 1000) + " s", "");
}

void displayStopC() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP C", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Arm raised", "PRESS BUTTON to drive", "");
}

void displayDrive() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("DRIVE", TFT_NAVY);
  drawBigLabel("DRIVE", TFT_NAVY);
}

void displayStopD() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP D", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Object detected — stopped", "PRESS BUTTON to turn", "");
}

void displayTurn() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("TURN", TFT_NAVY);
  drawBigLabel("TURN", TFT_NAVY);
}

void displayStopE() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP E", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Turn complete", "PRESS BUTTON to begin ramp", "");
}

void displayRamp() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("RAMP", TFT_NAVY);
  drawBigLabel("RAMP", TFT_NAVY);
}

void displayStopF() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP F", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Top of ramp reached", "PRESS BUTTON to deposit", "");
}

void displayStopG() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP G", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Pre-deposit pause", "PRESS BUTTON to deposit", "");
}

void displayDeposit() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("DEPOSIT", TFT_ORANGE);
  drawBigLabel("DUMP", TFT_ORANGE);
  drawInfoLines("Depositing rocks", "TODO: deposit mechanism", "");
}

void displayStopH() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("STOP H", TFT_RED);
  drawBigLabel("STOP", TFT_RED);
  drawInfoLines("Deposit complete", "PRESS BUTTON to descend ramp", "");
}

void displayDownRamp() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("DOWN RAMP", TFT_NAVY);
  drawBigLabel("DOWN", TFT_NAVY);
}

void displayFinish() {
  tft.fillScreen(TFT_LIGHTGREY);
  drawTopBar("FINISH", TFT_DARKGREEN);
  drawBigLabel("DONE!", TFT_DARKGREEN);
  drawInfoLines("All states complete", "Mission success!", "");
}

// ===================================================
// DYNAMIC DATA REFRESH
// ===================================================
void refreshDriveData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  if (distance > 0 && distance < 400) {
    tft.drawString("DIST: " + String(distance) + " cm", tft.width() / 2, 105);
  } else {
    tft.drawString("DIST: NO ECHO", tft.width() / 2, 105);
  }
  tft.setTextSize(1);
  tft.drawString("Stop threshold: " + String(DRIVE_STOP_DIST_CM) + " cm",
                 tft.width() / 2, 130);
}

void refreshTurnData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("YAW: " + String(yawAngle, 1) + " deg", tft.width() / 2, 105);
  tft.setTextSize(1);
  tft.drawString("Target: <= -90 deg", tft.width() / 2, 130);
  float progress = constrain(abs(yawAngle) / 90.0, 0.0, 1.0);
  int barW = tft.width() - 40;
  tft.drawRect(20, 142, barW, 10, TFT_BLACK);
  tft.fillRect(22, 144, (int)((barW - 4) * progress), 6, TFT_DARKGREEN);
}

void refreshRampData(String label, int threshold) {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  if (distance > 0 && distance < 400) {
    tft.drawString("DIST: " + String(distance) + " cm", tft.width() / 2, 105);
  } else {
    tft.drawString("DIST: NO ECHO", tft.width() / 2, 105);
  }
  tft.setTextSize(1);
  tft.drawString(label + " threshold: " + String(threshold) + " cm",
                 tft.width() / 2, 130);
}

void refreshTimerBar(unsigned long duration) {
  unsigned long elapsed = millis() - stateStartTime;
  float progress = constrain((float)elapsed / duration, 0.0, 1.0);
  int barW = tft.width() - 40;
  tft.fillRect(20, 142, barW, 10, TFT_LIGHTGREY);
  tft.drawRect(20, 142, barW, 10, TFT_BLACK);
  tft.fillRect(22, 144, (int)((barW - 4) * progress), 6, TFT_NAVY);
}

void refreshWaitingData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 92;
  if (distance > 0 && distance < 400)
    tft.drawString("DIST: " + String(distance) + " cm", 5, y);
  else
    tft.drawString("DIST: NO ECHO", 5, y);
  y += 16;
  tft.drawString("Acc  X:" + String(accX, 2) + " Y:" + String(accY, 2) +
                 " Z:" + String(accZ, 2), 5, y);
  y += 16;
  tft.drawString("Gyr  X:" + String(gyroX, 1) + " Y:" + String(gyroY, 1) +
                 " Z:" + String(gyroZ, 1), 5, y);
  y += 16;
  tft.drawString("Temp: " + String(temperature, 1) + " C", 5, y);
}

// ===================================================
// ENCODER MOTOR FUNCTIONS
// ===================================================
void encoderMotorSetup() {
  pinMode(EM_In1,      OUTPUT);
  pinMode(EM_In2,      OUTPUT);
  ledcSetup(4, PWM_FREQ, PWM_RESOLUTION); // channel 4 (0,2 used by drive motors)
  ledcAttachPin(EM_En, 4);
  ledcWrite(4, 0);
}

// direction: 1 = extend forward, -1 = retract, 0 = stop
void setEncoderMotor(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(EM_In1, HIGH);
    digitalWrite(EM_In2, LOW);
    ledcWrite(4, speed);
  } else if (direction == -1) {
    digitalWrite(EM_In1, LOW);
    digitalWrite(EM_In2, HIGH);
    ledcWrite(4, speed);
  } else {
    digitalWrite(EM_In1, LOW);
    digitalWrite(EM_In2, LOW);
    ledcWrite(4, 0);
  }
}

void stopEncoderMotor() {
  setEncoderMotor(0, 0);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);  // power to display

  Serial.begin(115200);

  pinMode(TRIG_PIN,      OUTPUT);
  pinMode(ECHO_PIN,      INPUT);
  pinMode(BUTTON_PIN,    INPUT_PULLUP);
  pinMode(STATE_OUT_PIN, OUTPUT);
  pinMode(SLAVE_PIN,     OUTPUT);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_PIN,     LOW);

  initMPU6050();
  encoderMotorSetup();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_LIGHTGREY);

  displayWaiting();

  stateStartTime = millis();
  Serial.println("MASTER ready — WAITING");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  distance = readDistanceCM();
  readMPU6050();

  // ------------------------------------------------
  // WAITING — show live data, button starts sequence
  // ------------------------------------------------
  if (currentState == WAITING) {
    refreshWaitingData();

    if (buttonPressed()) {
      advanceState(); // -> CALIBRATION
      runCalibration();
      advanceState(); // -> STOPA
      displayStopA();
    }
  }

  // ------------------------------------------------
  // STOP A — wait for button after calibration
  // ------------------------------------------------
  else if (currentState == STOPA) {
    if (buttonPressed()) {
      advanceState(); // -> ROCKFORWARD
      displayRockForward(1);
    }
  }

  // ------------------------------------------------
  // ROCK FORWARD (pass 1) — auto-advance after timer
  // ------------------------------------------------
  else if (currentState == ROCKFORWARD) {
    refreshTimerBar(ROCK_COLLECTION_TIME);
    if (millis() - stateStartTime >= ROCK_COLLECTION_TIME) {
      advanceState(); // -> ROCKBACK
      displayRockBack();
    }
  }

  // ------------------------------------------------
  // ROCK BACK — auto-advance after timer
  // ------------------------------------------------
  else if (currentState == ROCKBACK) {
    refreshTimerBar(ROCK_COLLECTION_TIME);
    if (millis() - stateStartTime >= ROCK_COLLECTION_TIME) {
      advanceState(); // -> ROCKFORWARD2
      displayRockForward(2);
    }
  }

  // ------------------------------------------------
  // ROCK FORWARD (pass 2) — auto-advance after timer
  // ------------------------------------------------
  else if (currentState == ROCKFORWARD2) {
    refreshTimerBar(ROCK_COLLECTION_TIME);
    if (millis() - stateStartTime >= ROCK_COLLECTION_TIME) {
      advanceState(); // -> STOPB
      displayStopB();
    }
  }

  // ------------------------------------------------
  // STOP B — wait for button after rocking
  // ------------------------------------------------
  else if (currentState == STOPB) {
    if (buttonPressed()) {
      advanceState(); // -> EXTEND
      displayExtend();
    }
  }

  // ------------------------------------------------
  // EXTEND — run encoder motor for EXTEND_TIME_MS
  // ------------------------------------------------
  else if (currentState == EXTEND) {
    setEncoderMotor(1, EXTEND_SPEED);

    unsigned long elapsed = millis() - stateStartTime;
    float progress = constrain((float)elapsed / EXTEND_TIME_MS, 0.0, 1.0);
    tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
    tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.drawString(String(elapsed) + " / " + String(EXTEND_TIME_MS) + " ms",
                   tft.width() / 2, 105);
    int barW = tft.width() - 40;
    tft.fillRect(20, 142, barW, 10, TFT_LIGHTGREY);
    tft.drawRect(20, 142, barW, 10, TFT_BLACK);
    tft.fillRect(22, 144, (int)((barW - 4) * progress), 6, TFT_PURPLE);

    if (elapsed >= 500) {
      stopEncoderMotor();
      advanceState(); // -> STOPI
      displayStopI();
    }
  }

  // ------------------------------------------------
  // STOP I — wait for button after extend
  // ------------------------------------------------
  else if (currentState == STOPI) {
    if (buttonPressed()) {
      advanceState(); // -> RAISEARM
      displayRaiseArm();
    }
  }

  // ------------------------------------------------
  // RAISE ARM — auto-advance after RAISE_ARM_TIME ms
  // ------------------------------------------------
  else if (currentState == RAISEARM) {
    refreshTimerBar(RAISE_ARM_TIME);
    if (millis() - stateStartTime >= RAISE_ARM_TIME) {
      advanceState(); // -> STOPC
      displayStopC();
    }
  }

  // ------------------------------------------------
  // STOP C — wait for button after arm raised
  // ------------------------------------------------
  else if (currentState == STOPC) {
    if (buttonPressed()) {
      advanceState(); // -> DRIVE
      displayDrive();
    }
  }

  // ------------------------------------------------
  // DRIVE — auto-stop on ultrasonic threshold
  // ------------------------------------------------
  else if (currentState == DRIVE) {
    refreshDriveData();

    if (distance > DRIVE_STOP_DIST_CM && distance < 400) {
      advanceState(); // -> STOPD
      displayStopD();
    }
  }

  // ------------------------------------------------
  // STOP D — wait for button after drive
  // ------------------------------------------------
  else if (currentState == STOPD) {
    if (buttonPressed()) {
      yawAngle     = 0;
      lastGyroTime = millis();
      advanceState(); // -> TURN
      displayTurn();
    }
  }

  // ------------------------------------------------
  // TURN — auto-stop when yaw reaches -90 degrees
  // ------------------------------------------------
  else if (currentState == TURN) {
    refreshTurnData();

    if (yawAngle <= -90.0) {
      delay(TURN_SETTLE_TIME);
      advanceState(); // -> STOPE
      displayStopE();
    }
  }

  // ------------------------------------------------
  // STOP E — wait for button after turn
  // ------------------------------------------------
  else if (currentState == STOPE) {
    if (buttonPressed()) {
      advanceState(); // -> RAMP
      displayRamp();
    }
  }

  // ------------------------------------------------
  // RAMP — drive until ultrasonic detects wall at top
  // ------------------------------------------------
  else if (currentState == RAMP) {
    refreshRampData("Ramp stop", RAMP_STOP_DIST_CM);

    if (distance > RAMP_STOP_DIST_CM && distance < 400) {
      advanceState(); // -> STOPF
      displayStopF();
    }
  }

  // ------------------------------------------------
  // STOP F — wait for button at top of ramp
  // ------------------------------------------------
  else if (currentState == STOPF) {
    if (buttonPressed()) {
      advanceState(); // -> STOPG
      displayStopG();
    }
  }

  // ------------------------------------------------
  // STOP G — wait for button (pre-deposit pause)
  // ------------------------------------------------
  else if (currentState == STOPG) {
    if (buttonPressed()) {
      advanceState(); // -> DEPOSIT
      displayDeposit();
    }
  }

  // ------------------------------------------------
  // DEPOSIT — TODO: implement deposit mechanism
  // ------------------------------------------------
  else if (currentState == DEPOSIT) {
    advanceState(); // -> STOPH  (remove when deposit logic added)
    displayStopH();
  }

  // ------------------------------------------------
  // STOP H — wait for button after deposit
  // ------------------------------------------------
  else if (currentState == STOPH) {
    if (buttonPressed()) {
      advanceState(); // -> DOWNRAMP
      displayDownRamp();
    }
  }

  // ------------------------------------------------
  // DOWN RAMP — drive slowly until clear of ramp
  // ------------------------------------------------
  else if (currentState == DOWNRAMP) {
    refreshRampData("Down-ramp stop", DOWN_RAMP_STOP_CM);

    bool minTimePassed = (millis() - stateStartTime) > DOWN_RAMP_MIN_TIME;
    if (minTimePassed && distance > DOWN_RAMP_STOP_CM && distance < 400) {
      advanceState(); // -> FINISH
      displayFinish();
    }
  }

  // ------------------------------------------------
  // FINISH — mission complete, sit idle
  // ------------------------------------------------
  else if (currentState == FINISH) {
    // done
  }

  delay(50);
}
