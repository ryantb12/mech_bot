// ===================================================
// MASTER CONTROLLER v2 — TTGO T-Display
//
// State order:
//  0  WAITING     — idle, show sensors, button to start
//  1  CALIBRATION — IMU calibration (auto-advance)
//  2  FORWARD1    — drive forward (pass 1), timer
//  3  BACKWARD    — drive backward, timer
//  4  FORWARD2    — drive forward (pass 2), timer
//  5  EXTEND      — extend arm phase one (encoder motor), timer
//  6  RAISEARM    — raise linear actuator, timer
//  7  TURN        — turn until yaw <= -90 deg (gyro)
//  8  FORWARD3    — drive forward to deposit, ultrasonic
//  9  DEPOSIT     — deposit rocks (TODO)
//  10 GOHOME      — return home, timer
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

// ===================================================
// PIN DEFINITIONS
// ===================================================
// Ultrasonic A (front)
#define TRIG_A         2
#define ECHO_A         3

// Ultrasonic B (second)
#define TRIG_B         43
#define ECHO_B         44

// MPU-6050
#define MPU_SDA        16
#define MPU_SCL        21
#define MPU_ADDR       0x68

// Button
#define BUTTON_PIN     10

// Comms out
#define STATE_OUT_PIN  1    // Pulse out to Motion 2350 Pro
#define SLAVE_TX       17   // Master TX → Slave RX
#define SLAVE_RX       18   // Master RX ← Slave TX (future use)

// Encoder motor — arm extend
#define EM_In1         11
#define EM_In2         12
#define EM_En          13

// ===================================================
// TIMING CONSTANTS — adjust to tune behaviour
// ===================================================
#define FORWARD1_TIME      2000   // ms — first forward pass
#define BACKWARD_TIME      2000   // ms — backward pass
#define FORWARD2_TIME      2000   // ms — second forward pass
#define EXTEND_TIME        1500   // ms — encoder motor arm extend
#define RAISE_ARM_TIME     8000   // ms — linear actuator raise
#define TURN_SETTLE_TIME    500   // ms — settle after turn completes
#define GOHOME_TIME        4000   // ms — drive back to home position

// Ultrasonic threshold for FORWARD3 (deposit approach)
#define DEPOSIT_STOP_CM    6      // cm — stop when this close to target

// Encoder motor
#define EXTEND_SPEED       255
#define PWM_FREQ          1000
#define PWM_RESOLUTION       8

// ===================================================
// COLOUR — TFT_eSPI defines TFT_LIGHTGREY as 0xD69A.
// Override to match the working grey used in this project.
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
  WAITING,      // 0
  CALIBRATION,  // 1
  FORWARD1,     // 2
  BACKWARD,     // 3
  FORWARD2,     // 4
  EXTEND,       // 5
  RAISEARM,     // 6
  TURN,         // 7
  FORWARD3,     // 8
  DEPOSIT,      // 9
  GOHOME        // 10
};

Level currentState = WAITING;

// ===================================================
// MPU-6050
// ===================================================
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;

float accOffX = 0, accOffY = 0, accOffZ = 0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
bool  calibrated = false;

float yawAngle = 0;
unsigned long lastGyroTime = 0;

// ===================================================
// SENSOR & COMMS
// ===================================================
long distA      = 0;   // Ultrasonic A (front) — TRIG 2 / ECHO 3
long distB      = 0;   // Ultrasonic B (second) — TRIG 43 / ECHO 44
int  pulseCount = 0;
unsigned long stateStartTime = 0;

// ===================================================
// MPU-6050 FUNCTIONS
// ===================================================
void initMPU6050() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
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
    accX  -= accOffX;  accY  -= accOffY;  accZ  -= accOffZ;
    gyroX -= gyroOffX; gyroY -= gyroOffY; gyroZ -= gyroOffZ;
  }

  if (lastGyroTime > 0 && calibrated) {
    float dt = (millis() - lastGyroTime) / 1000.0;
    yawAngle += gyroZ * dt;
  }
  lastGyroTime = millis();
}

// ===================================================
// ULTRASONIC
// ===================================================
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  return duration * 0.034 / 2;
}

long readDistanceACM() { return readUltrasonic(TRIG_A, ECHO_A); }
long readDistanceBCM() { return readUltrasonic(TRIG_B, ECHO_B); }

// ===================================================
// COMMS
// ===================================================
void sendPulse() {
  digitalWrite(STATE_OUT_PIN, HIGH);
  digitalWrite(SLAVE_TX,      HIGH);
  delay(100);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_TX,      LOW);
  pulseCount++;
  Serial.print("PULSE SENT #");
  Serial.println(pulseCount);
}

// ===================================================
// BUTTON
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
// ADVANCE STATE
// ===================================================
void advanceState() {
  sendPulse();
  currentState   = (Level)(currentState + 1);
  stateStartTime = millis();
  Serial.print("-> STATE ");
  Serial.println(currentState);
}

// ===================================================
// CALIBRATION (blocking)
// ===================================================
void runCalibration() {
  const int SAMPLES = 200;
  float sAX = 0, sAY = 0, sAZ = 0;
  float sGX = 0, sGY = 0, sGZ = 0;

  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATING", tft.width() / 2, 30);
  tft.setTextSize(1);
  tft.drawString("KEEP ROBOT STILL", tft.width() / 2, 55);

  int bx = 20, by = 75, bw = tft.width() - 40, bh = 16;
  tft.drawRect(bx, by, bw, bh, TFT_BLACK);

  for (int i = 0; i < SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

    int16_t rAX = Wire.read() << 8 | Wire.read();
    int16_t rAY = Wire.read() << 8 | Wire.read();
    int16_t rAZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t rGX = Wire.read() << 8 | Wire.read();
    int16_t rGY = Wire.read() << 8 | Wire.read();
    int16_t rGZ = Wire.read() << 8 | Wire.read();

    sAX += rAX / 16384.0; sAY += rAY / 16384.0; sAZ += rAZ / 16384.0;
    sGX += rGX / 131.0;   sGY += rGY / 131.0;   sGZ += rGZ / 131.0;

    tft.fillRect(bx + 2, by + 2, map(i, 0, SAMPLES - 1, 0, bw - 4), bh - 4, TFT_DARKGREEN);
    delay(10);
  }

  accOffX  = sAX / SAMPLES;     accOffY  = sAY / SAMPLES;
  accOffZ  = sAZ / SAMPLES - 1.0;
  gyroOffX = sGX / SAMPLES;     gyroOffY = sGY / SAMPLES;
  gyroOffZ = sGZ / SAMPLES;

  calibrated   = true;
  yawAngle     = 0;
  lastGyroTime = millis();

  tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 10;
  tft.drawString("CALIBRATED OK", 5, y);       y += 18;
  tft.drawString("AccX: " + String(accOffX, 4), 5, y); y += 14;
  tft.drawString("AccY: " + String(accOffY, 4), 5, y); y += 14;
  tft.drawString("AccZ: " + String(accOffZ, 4), 5, y); y += 14;
  tft.drawString("GyrX: " + String(gyroOffX, 4), 5, y); y += 14;
  tft.drawString("GyrY: " + String(gyroOffY, 4), 5, y); y += 14;
  tft.drawString("GyrZ: " + String(gyroOffZ, 4), 5, y);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PRESS BUTTON TO START", tft.width() / 2, tft.height() - 10);

  while (!buttonPressed()) delay(10);
}

// ===================================================
// DISPLAY HELPERS
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

void drawInfoLines(String line1, String line2) {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString(line1, tft.width() / 2, 108);
  tft.drawString(line2, tft.width() / 2, 128);
}

// ===================================================
// DYNAMIC REFRESHES
// ===================================================
void refreshTimerBar(unsigned long duration) {
  float progress = constrain((float)(millis() - stateStartTime) / duration, 0.0, 1.0);
  int bw = tft.width() - 40;
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  unsigned long remaining = duration - min((unsigned long)(millis() - stateStartTime), duration);
  tft.drawString(String(remaining / 1000.0, 1) + " s remaining", tft.width() / 2, 108);
  tft.drawRect(20, 130, bw, 12, TFT_BLACK);
  tft.fillRect(22, 132, (int)((bw - 4) * progress), 8, TFT_NAVY);
}

void refreshDistanceData(int threshold) {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  String dsA = (distA > 0 && distA < 400) ? "A: " + String(distA) + "cm" : "A: NO ECHO";
  String dsB = (distB > 0 && distB < 400) ? "B: " + String(distB) + "cm" : "B: NO ECHO";
  tft.drawString(dsA, tft.width() / 2, 100);
  tft.drawString(dsB, tft.width() / 2, 118);
  tft.setTextSize(1);
  tft.drawString("Stop (A) at: < " + String(threshold) + " cm", tft.width() / 2, 138);
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
  int bw = tft.width() - 40;
  tft.drawRect(20, 142, bw, 10, TFT_BLACK);
  tft.fillRect(22, 144, (int)((bw - 4) * progress), 6, TFT_DARKGREEN);
}

void refreshWaitingData() {
  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 92;
  tft.drawString("A: " + ((distA > 0 && distA < 400) ? String(distA) + " cm" : String("NO ECHO")), 5, y);
  tft.drawString("B: " + ((distB > 0 && distB < 400) ? String(distB) + " cm" : String("NO ECHO")), tft.width() / 2, y);
  y += 16;
  tft.drawString("Acc X:" + String(accX,2) + " Y:" + String(accY,2) + " Z:" + String(accZ,2), 5, y);
  y += 16;
  tft.drawString("Gyr X:" + String(gyroX,1) + " Y:" + String(gyroY,1) + " Z:" + String(gyroZ,1), 5, y);
  y += 16;
  tft.drawString("Temp: " + String(temperature, 1) + " C", 5, y);
}

// ===================================================
// ENCODER MOTOR
// ===================================================
void encoderMotorSetup() {
  pinMode(EM_In1, OUTPUT);
  pinMode(EM_In2, OUTPUT);
  ledcSetup(4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(EM_En, 4);
  ledcWrite(4, 0);
}

void setEncoderMotor(int dir, int spd) {
  if (dir == 1) {
    digitalWrite(EM_In1, HIGH); digitalWrite(EM_In2, LOW);
  } else if (dir == -1) {
    digitalWrite(EM_In1, LOW);  digitalWrite(EM_In2, HIGH);
  } else {
    digitalWrite(EM_In1, LOW);  digitalWrite(EM_In2, LOW); spd = 0;
  }
  ledcWrite(4, spd);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  Serial.begin(115200);

  pinMode(TRIG_A,        OUTPUT);
  pinMode(ECHO_A,        INPUT);
  pinMode(TRIG_B,        OUTPUT);
  pinMode(ECHO_B,        INPUT);
  pinMode(BUTTON_PIN,    INPUT_PULLUP);
  pinMode(STATE_OUT_PIN, OUTPUT);
  pinMode(SLAVE_TX,      OUTPUT);
  pinMode(SLAVE_RX,      INPUT);
  digitalWrite(STATE_OUT_PIN, LOW);
  digitalWrite(SLAVE_TX,      LOW);

  initMPU6050();
  encoderMotorSetup();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_LIGHTGREY);

  drawTopBar("WAITING", TFT_DARKGREY);
  drawBigLabel("WAIT", TFT_DARKGREY);
  drawInfoLines("Sensors live", "PRESS BUTTON to calibrate");

  stateStartTime = millis();
  Serial.println("MASTER v2 ready — WAITING");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  distA = readDistanceACM();
  distB = readDistanceBCM();
  readMPU6050();

  // ------------------------------------------------
  // 0: WAITING — sensors live, button starts
  // ------------------------------------------------
  if (currentState == WAITING) {
    refreshWaitingData();
    if (buttonPressed()) {
      advanceState();           // -> CALIBRATION
      runCalibration();
      advanceState();           // -> FORWARD1
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("FORWARD 1", TFT_NAVY);
      drawBigLabel("FWD", TFT_NAVY);
    }
  }

  // ------------------------------------------------
  // 2: FORWARD1 — drive forward, timer
  // ------------------------------------------------
  else if (currentState == FORWARD1) {
    refreshTimerBar(FORWARD1_TIME);
    if (millis() - stateStartTime >= FORWARD1_TIME) {
      advanceState();           // -> BACKWARD
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("BACKWARD", TFT_NAVY);
      drawBigLabel("BACK", TFT_NAVY);
    }
  }

  // ------------------------------------------------
  // 3: BACKWARD — drive backward, timer
  // ------------------------------------------------
  else if (currentState == BACKWARD) {
    refreshTimerBar(BACKWARD_TIME);
    if (millis() - stateStartTime >= BACKWARD_TIME) {
      advanceState();           // -> FORWARD2
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("FORWARD 2", TFT_NAVY);
      drawBigLabel("FWD", TFT_NAVY);
    }
  }

  // ------------------------------------------------
  // 4: FORWARD2 — drive forward, timer
  // ------------------------------------------------
  else if (currentState == FORWARD2) {
    refreshTimerBar(FORWARD2_TIME);
    if (millis() - stateStartTime >= FORWARD2_TIME) {
      advanceState();           // -> EXTEND
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("EXTEND ARM", TFT_PURPLE);
      drawBigLabel("EXTND", TFT_PURPLE);
    }
  }

  // ------------------------------------------------
  // 5: EXTEND — encoder motor arm extend, timer
  // ------------------------------------------------
  else if (currentState == EXTEND) {
    setEncoderMotor(1, EXTEND_SPEED);
    refreshTimerBar(EXTEND_TIME);
    if (millis() - stateStartTime >= EXTEND_TIME) {
      setEncoderMotor(0, 0);
      advanceState();           // -> RAISEARM
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("RAISE ARM", TFT_PURPLE);
      drawBigLabel("RAISE", TFT_PURPLE);
      drawInfoLines("Raising arm", String(RAISE_ARM_TIME / 1000) + " s");
    }
  }

  // ------------------------------------------------
  // 6: RAISEARM — linear actuator raise, timer
  // ------------------------------------------------
  else if (currentState == RAISEARM) {
    refreshTimerBar(RAISE_ARM_TIME);
    if (millis() - stateStartTime >= RAISE_ARM_TIME) {
      advanceState();           // -> TURN
      yawAngle     = 0;
      lastGyroTime = millis();
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("TURN", TFT_NAVY);
      drawBigLabel("TURN", TFT_NAVY);
    }
  }

  // ------------------------------------------------
  // 7: TURN — gyro yaw, auto at -90 deg
  // ------------------------------------------------
  else if (currentState == TURN) {
    refreshTurnData();
    if (yawAngle <= -90.0) {
      delay(TURN_SETTLE_TIME);
      advanceState();           // -> FORWARD3
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("FORWARD 3", TFT_NAVY);
      drawBigLabel("FWD", TFT_NAVY);
    }
  }

  // ------------------------------------------------
  // 8: FORWARD3 — drive to deposit, ultrasonic
  // ------------------------------------------------
  else if (currentState == FORWARD3) {
    refreshDistanceData(DEPOSIT_STOP_CM);
    if (distA > 0 && distA <= DEPOSIT_STOP_CM) {
      advanceState();           // -> DEPOSIT
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("DEPOSIT", TFT_ORANGE);
      drawBigLabel("DUMP", TFT_ORANGE);
      drawInfoLines("Depositing", "TODO: mechanism");
    }
  }

  // ------------------------------------------------
  // 9: DEPOSIT — TODO: actuate deposit mechanism
  // Immediately advances for now; add timing/logic here
  // ------------------------------------------------
  else if (currentState == DEPOSIT) {
    // TODO: run deposit servo/motor here
    delay(500);
    advanceState();             // -> GOHOME
    tft.fillScreen(TFT_LIGHTGREY);
    drawTopBar("GO HOME", TFT_DARKGREEN);
    drawBigLabel("HOME", TFT_DARKGREEN);
  }

  // ------------------------------------------------
  // 10: GOHOME — drive back, timer, then idle
  // ------------------------------------------------
  else if (currentState == GOHOME) {
    refreshTimerBar(GOHOME_TIME);
    if (millis() - stateStartTime >= GOHOME_TIME) {
      sendPulse();  // stop signal to slave/motion boards
      tft.fillScreen(TFT_LIGHTGREY);
      drawTopBar("COMPLETE", TFT_DARKGREEN);
      drawBigLabel("DONE", TFT_DARKGREEN);
      drawInfoLines("Mission complete", "Pulses sent: " + String(pulseCount));
      Serial.println("MISSION COMPLETE");
      while (true) delay(1000);  // hold final screen
    }
  }

  delay(50);
}
