// ===================================================
// SLAVE MOTOR + ACTUATOR TEST
// Runs a simple sequence on loop:
//  1. Drive BACK  2 seconds
//  2. Drive FWD   2 seconds
//  3. Retract actuator 4 seconds
//  4. Extend actuator  4 seconds
// Repeats forever. No comms, no state machine.
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>

// ===================================================
// PIN DEFINITIONS — same as slave12
// ===================================================
// Left wheel motor — MD1 Secondary (U4), A1/2 side
#define LM_In1  43   // S_In1_SlavePin_43 — TTGO pin 3
#define LM_In2  44   // S_In2_SlavePin_44 — TTGO pin 4
#define LM_En   10   // MD1_S_EN_A_SlavePin_10 — TTGO pin 20

// Right wheel motor — MD2 Primary (U3), A1/2 side
#define RM_In1   7   // IN1_SlavePin_16 — TTGO pin 8
#define RM_In2   8   // IN2_SlavePin_21 — TTGO pin 7
#define RM_En    3   // MD2_EN_A_SlavePin_3 — TTGO pin 21

// Linear actuator — MD1 Secondary (U4), B1/2 side
#define LA_In1   1   // S_In3_SlavePin_1 — TTGO pin 23
#define LA_In2   2   // S_In4_SlavePin_2 — TTGO pin 22
#define LA_En   11   // MD1_S_EN_B_SlavePin_11 — TTGO pin 19

// ===================================================
// PWM
// ===================================================
#define PWM_FREQ        1000
#define PWM_RESOLUTION     8

#define DRIVE_SPEED     240
#define ACTUATOR_SPEED  255

// ===================================================
// COLOUR
// ===================================================
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618

TFT_eSPI tft = TFT_eSPI();

// ===================================================
// MOTOR STRUCTS
// ===================================================
struct Motor {
  int in1, in2, en, channel;
};

struct LinearAct {
  int in1, in2, en;
};

Motor     leftMotor      = { LM_In1, LM_In2, LM_En, 0 };
Motor     rightMotor     = { RM_In1, RM_In2, RM_En, 2 };
LinearAct linearActuator = { LA_In1, LA_In2, LA_En };

// ===================================================
// MOTOR FUNCTIONS
// ===================================================
void motorSetup(Motor m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  ledcSetup(m.channel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(m.en, m.channel);
}

void setMotor(Motor m, int dir, int spd) {
  if (dir == 1) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (dir == -1) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    spd = 0;
  }
  ledcWrite(m.channel, spd);
}

void setLinearAct(LinearAct l, int dir) {
  if (dir == 1) {        // extend up
    digitalWrite(l.in1, HIGH);
    digitalWrite(l.in2, LOW);
    digitalWrite(l.en,  HIGH);
  } else if (dir == -1) { // retract down
    digitalWrite(l.in1, LOW);
    digitalWrite(l.in2, HIGH);
    digitalWrite(l.en,  HIGH);
  } else {
    digitalWrite(l.in1, LOW);
    digitalWrite(l.in2, LOW);
    digitalWrite(l.en,  LOW);
  }
}

void driveForward(int spd) {
  setMotor(leftMotor,  -1, spd);
  setMotor(rightMotor, -1, spd);
}

void driveBack(int spd) {
  setMotor(leftMotor,   1, spd);
  setMotor(rightMotor, -1, spd);
}

void stopMotors() {
  setMotor(leftMotor,  0, 0);
  setMotor(rightMotor, 0, 0);
}

// ===================================================
// DISPLAY
// ===================================================
void showScreen(String label, String line1, String line2, uint32_t colour) {
  tft.fillScreen(TFT_LIGHTGREY);
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString("MOTOR TEST", tft.width() / 2, 12);

  tft.fillRect(0, 25, tft.width(), 60, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY);
  tft.setTextSize(4);
  tft.drawString(label, tft.width() / 2, 55);

  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.drawString(line1, tft.width() / 2, 108);
  tft.drawString(line2, tft.width() / 2, 128);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  Serial.begin(115200);

  motorSetup(leftMotor);
  motorSetup(rightMotor);

  pinMode(LA_In1, OUTPUT);
  pinMode(LA_In2, OUTPUT);
  pinMode(LA_En,  OUTPUT);
  digitalWrite(LA_En, LOW);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_LIGHTGREY);

  Serial.println("Motor test starting...");
}

// ===================================================
// LOOP — repeating sequence
// ===================================================
void loop() {

  // ---- 1: DRIVE BACK 2s --------------------------
  showScreen("BACK", "Driving backwards", "2 seconds", TFT_NAVY);
  Serial.println("DRIVE BACK");
  driveBack(DRIVE_SPEED);
  delay(2000);
  stopMotors();
  delay(300);

  // ---- 2: DRIVE FORWARD 2s -----------------------
  showScreen("FWD", "Driving forward", "2 seconds", TFT_NAVY);
  Serial.println("DRIVE FORWARD");
  driveForward(DRIVE_SPEED);
  delay(2000);
  stopMotors();
  delay(300);

  // ---- 3: RETRACT ACTUATOR 4s --------------------
  showScreen("DOWN", "Retracting actuator", "4 seconds", TFT_PURPLE);
  Serial.println("ACTUATOR RETRACT");
  setLinearAct(linearActuator, -1);
  delay(4000);
  setLinearAct(linearActuator, 0);
  delay(300);

  // ---- 4: EXTEND ACTUATOR 4s ---------------------
  showScreen("UP", "Extending actuator", "4 seconds", TFT_PURPLE);
  Serial.println("ACTUATOR EXTEND");
  setLinearAct(linearActuator, 1);
  delay(13000);
  setLinearAct(linearActuator, 0);
  delay(300);
}
