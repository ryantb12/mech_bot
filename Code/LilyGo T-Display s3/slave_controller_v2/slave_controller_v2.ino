// ===================================================
// SLAVE CONTROLLER v2 — TTGO T-Display S3 #2
//
// Receives rising-edge pulses from master on COMMS_PIN.
// Each pulse advances one state, matching master_controller_v2.
//
// WIRING: master GPIO 17 (SLAVE_TX) --> slave GPIO 18 (COMMS_PIN)
//
// State | Master state | Slave action
// ------+--------------+------------------------------
//   0   | WAITING      | All stop
//   1   | CALIBRATION  | All stop (master calibrating)
//   2   | FORWARD1     | Drive forward
//   3   | BACKWARD     | Drive backward
//   4   | FORWARD2     | Drive forward
//   5   | EXTEND       | All stop (arm extend on master)
//   6   | RAISEARM     | Raise linear actuator
//   7   | TURN         | Stop actuator, turn in place
//   8   | FORWARD3     | Drive forward (to deposit)
//   9   | DEPOSIT      | All stop
//  10   | GOHOME       | Drive backward
//  11+  | Done         | All stop
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>

// ===================================================
// PIN DEFINITIONS — from schematic SCHLIB_TTGO-T-Display
// ===================================================
// Comms in — master GPIO 17 → slave GPIO 18 (TTGO pin 5)
#define COMMS_PIN  18

// Left wheel motor — MD1 Secondary (U4), A1/2 side
#define LM_In1  43   // TTGO pin 3
#define LM_In2  44   // TTGO pin 4
#define LM_En   10   // TTGO pin 20 — LEDC channel 0

// Right wheel motor — MD2 Primary (U3)
#define RM_In1  16   // TTGO pin 8  (was wrongly set to GPIO 7)
#define RM_In2  21   // TTGO pin 7  (was wrongly set to GPIO 8)
#define RM_En    3   // TTGO pin 21 — LEDC channel 2

// Linear actuator — MD1 Secondary (U4), B1/2 side
#define LA_In1   1   // TTGO pin 23
#define LA_In2   2   // TTGO pin 22
#define LA_En   11   // TTGO pin 19

// ===================================================
// SPEEDS (0–1023 for 10-bit PWM)
// ===================================================
#define DRIVE_SPEED      700
#define TURN_SPEED       500
#define ACT_SPEED       1023

// Right motor correction — mechanical slip compensation
// Reduces right motor speed by 25% to drive straight
#define RM_CORRECTION  0.75f

// ===================================================
// PWM
// ===================================================
#define PWM_FREQ       1000
#define PWM_RESOLUTION   10   // 10-bit: 0–1023

// ===================================================
// COLOUR
// ===================================================
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618

TFT_eSPI tft = TFT_eSPI();

// ===================================================
// STATE
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
  GOHOME,       // 10
  DONE          // 11
};

Level currentState = WAITING;
bool  lastPinState = LOW;

// ===================================================
// MOTOR STRUCTS
// ===================================================
struct Motor { int in1, in2, en, ch; };
struct LinearAct { int in1, in2, en; };

Motor     leftMotor  = { LM_In1, LM_In2, LM_En, 0 };
Motor     rightMotor = { RM_In1, RM_In2, RM_En, 2 };
LinearAct actuator   = { LA_In1, LA_In2, LA_En    };

// ===================================================
// MOTOR FUNCTIONS
// ===================================================
void motorSetup(Motor m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  ledcSetup(m.ch, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(m.en, m.ch);
  ledcWrite(m.ch, 0);
}

// direction: 1=fwd, -1=rev, 0=stop
void setMotor(Motor m, int dir, int spd) {
  if (dir == 1)       { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
  else if (dir == -1) { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
  else                { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW); spd = 0; }
  ledcWrite(m.ch, spd);
}

// direction: 1=raise, -1=lower, 0=stop
void setActuator(LinearAct l, int dir) {
  if (dir == 1)       { digitalWrite(l.in1, HIGH); digitalWrite(l.in2, LOW);  digitalWrite(l.en, HIGH); }
  else if (dir == -1) { digitalWrite(l.in1, LOW);  digitalWrite(l.in2, HIGH); digitalWrite(l.en, HIGH); }
  else                { digitalWrite(l.in1, LOW);  digitalWrite(l.in2, LOW);  digitalWrite(l.en, LOW);  }
}

void stopMotors()   { setMotor(leftMotor, 0, 0); setMotor(rightMotor, 0, 0); }
void stopActuator() { setActuator(actuator, 0); }
void stopAll()      { stopMotors(); stopActuator(); }

// Right motor physically inverted — direction signals are always opposite to left
// RM_CORRECTION reduces right motor speed to compensate mechanical slip
void driveForward(int spd)  { setMotor(leftMotor, -1, spd); setMotor(rightMotor,  1, (int)(spd * RM_CORRECTION)); }
void driveBackward(int spd) { setMotor(leftMotor,  1, spd); setMotor(rightMotor, -1, (int)(spd * RM_CORRECTION)); }
void driveTurn(int spd)     { setMotor(leftMotor,  1, (int)(spd * 0.5)); setMotor(rightMotor,  1, (int)(spd * RM_CORRECTION)); } // L back half-speed, R fwd

// ===================================================
// DISPLAY
// ===================================================
void showState(String label, uint32_t colour) {
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString("SLAVE  STATE: " + String((int)currentState), tft.width() / 2, 12);
  tft.fillRect(0, 25, tft.width(), 145, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY);
  tft.setTextSize(4);
  tft.drawString(label, tft.width() / 2, 95);
}

// ===================================================
// STATE ENTRY
// ===================================================
void enterState(Level s) {
  Serial.print("STATE -> "); Serial.println((int)s);
  switch (s) {
    case WAITING:     showState("WAIT",  TFT_DARKGREY);  stopAll();                                  break;
    case CALIBRATION: showState("CAL",   TFT_DARKCYAN);  stopAll();                                  break;
    case FORWARD1:    showState("FWD1",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case BACKWARD:    showState("BACK",  TFT_NAVY);      stopActuator(); driveBackward(DRIVE_SPEED); break;
    case FORWARD2:    showState("FWD2",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case EXTEND:      showState("EXTND", TFT_PURPLE);    stopAll();                                  break;
    case RAISEARM:    showState("RAISE", TFT_PURPLE);    stopMotors(); setActuator(actuator, 1);     break;
    case TURN:        showState("TURN",  TFT_NAVY);      stopActuator(); driveTurn(TURN_SPEED);      break;
    case FORWARD3:    showState("FWD3",  TFT_NAVY);      stopActuator(); driveForward(DRIVE_SPEED);  break;
    case DEPOSIT:     showState("DUMP",  TFT_ORANGE);    stopAll();                                  break;
    case GOHOME:      showState("HOME",  TFT_DARKGREEN); stopActuator(); driveBackward(DRIVE_SPEED); break;
    case DONE:        showState("DONE",  TFT_DARKGREEN); stopAll();                                  break;
  }
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);  // power enable — same as testing_movement_v1

  Serial.begin(115200);

  pinMode(COMMS_PIN, INPUT_PULLDOWN);

  motorSetup(leftMotor);
  motorSetup(rightMotor);
  pinMode(LA_In1, OUTPUT); pinMode(LA_In2, OUTPUT); pinMode(LA_En, OUTPUT);
  stopAll();

  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_LIGHTGREY);
  enterState(WAITING);

  Serial.println("SLAVE v2 ready — pulses on GPIO 18");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  bool pinNow = digitalRead(COMMS_PIN);

  if (pinNow == HIGH && lastPinState == LOW) {
    delay(20);
    if (currentState < DONE) {
      currentState = (Level)(currentState + 1);
      enterState(currentState);
    }
  }

  lastPinState = pinNow;
  delay(5);
}
