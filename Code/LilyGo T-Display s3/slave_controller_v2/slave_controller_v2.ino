// ===================================================
// SLAVE CONTROLLER v2 — TTGO T-Display S3 #2
//
// Receives rising-edge pulses from master on COMMS_PIN.
// Each pulse advances one state, matching master_controller_v2.
//
// WIRING: master GPIO 10 (SLAVE_PIN) --> slave GPIO 16 (COMMS_PIN)
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
// PIN DEFINITIONS (confirmed working from testing_movement_v1)
// ===================================================
// Comms in — wire from master GPIO 10
#define COMMS_PIN  16

// Left wheel motor — MD1 Secondary (U4), A1/2 side
#define LM_In1  43
#define LM_In2  44
#define LM_En   10   // LEDC channel 0

// Right wheel motor — MD2 Primary (U3), A1/2 side
#define RM_In1   7
#define RM_In2   8
#define RM_En    3   // LEDC channel 2

// Linear actuator — MD1 Secondary (U4), B1/2 side
#define LA_In1   1
#define LA_In2   2
#define LA_En   11   // digital enable

// ===================================================
// SPEEDS (0–255)
// ===================================================
#define DRIVE_SPEED   200   // forward / backward
#define TURN_SPEED    160   // in-place turn
#define ACT_SPEED     255   // linear actuator (digital, speed unused)

// ===================================================
// PWM
// ===================================================
#define PWM_FREQ       1000
#define PWM_RESOLUTION    10

// ===================================================
// COLOUR
// ===================================================
#define TFT_LIGHTGREY 0xC618

// ===================================================
// DISPLAY
// ===================================================
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
struct Motor {
  int in1, in2, en, ch;
};

struct LinearAct {
  int in1, in2, en;
};

Motor     leftMotor  = { LM_In1, LM_In2, LM_En,  0 };
Motor     rightMotor = { RM_In1, RM_In2, RM_En,  2 };
LinearAct actuator   = { LA_In1, LA_In2, LA_En     };

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

// direction: 1=forward, -1=reverse, 0=stop
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
  ledcWrite(m.ch, spd);
}

// direction: 1=raise(extend), -1=lower(retract), 0=stop
void setActuator(LinearAct l, int dir) {
  if (dir == 1) {
    digitalWrite(l.in1, HIGH);
    digitalWrite(l.in2, LOW);
    digitalWrite(l.en,  HIGH);
  } else if (dir == -1) {
    digitalWrite(l.in1, LOW);
    digitalWrite(l.in2, HIGH);
    digitalWrite(l.en,  HIGH);
  } else {
    digitalWrite(l.in1, LOW);
    digitalWrite(l.in2, LOW);
    digitalWrite(l.en,  LOW);
  }
}

void stopMotors() {
  setMotor(leftMotor,  0, 0);
  setMotor(rightMotor, 0, 0);
}

void stopActuator() {
  setActuator(actuator, 0);
}

void stopAll() {
  stopMotors();
  stopActuator();
}

// Drive helpers — directions confirmed from testing_movement_v1
void driveForward(int spd) {
  setMotor(leftMotor,  -1, spd);
  setMotor(rightMotor, -1, spd);
}

void driveBackward(int spd) {
  setMotor(leftMotor,   1, spd);
  setMotor(rightMotor, -1, spd);
}

// Turn in place — left wheel back, right wheel forward.
// If robot turns the wrong way, swap the directions below.
void driveTurn(int spd) {
  setMotor(leftMotor,   1, spd);   // left backward
  setMotor(rightMotor, -1, spd);   // right forward
}

// ===================================================
// DISPLAY
// ===================================================
void showState(String label, String detail, uint32_t colour) {
  tft.fillRect(0, 0, tft.width(), 25, colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.drawString("SLAVE  |  STATE: " + String(currentState), tft.width() / 2, 12);

  tft.fillRect(0, 25, tft.width(), 60, TFT_LIGHTGREY);
  tft.setTextColor(colour, TFT_LIGHTGREY);
  tft.setTextSize(4);
  tft.drawString(label, tft.width() / 2, 55);

  tft.drawLine(0, 85, tft.width(), 85, TFT_BLACK);

  tft.fillRect(0, 88, tft.width(), 80, TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.drawString(detail, tft.width() / 2, 115);
}

// ===================================================
// STATE ENTRY — called once on each state change
// ===================================================
void enterState(Level s) {
  Serial.print("STATE -> ");
  Serial.println(s);

  switch (s) {

    case WAITING:
      stopAll();
      showState("WAIT", "Waiting for master", TFT_DARKGREY);
      break;

    case CALIBRATION:
      stopAll();
      showState("CAL", "Master calibrating...", TFT_DARKCYAN);
      break;

    case FORWARD1:
      stopActuator();
      driveForward(DRIVE_SPEED);
      showState("FWD1", "Driving forward (pass 1)", TFT_NAVY);
      break;

    case BACKWARD:
      stopActuator();
      driveBackward(DRIVE_SPEED);
      showState("BACK", "Driving backward", TFT_NAVY);
      break;

    case FORWARD2:
      stopActuator();
      driveForward(DRIVE_SPEED);
      showState("FWD2", "Driving forward (pass 2)", TFT_NAVY);
      break;

    case EXTEND:
      stopAll();
      showState("EXTND", "Arm extend (master)", TFT_PURPLE);
      break;

    case RAISEARM:
      stopMotors();
      setActuator(actuator, 1);   // raise arm
      showState("RAISE", "Raising arm", TFT_PURPLE);
      break;

    case TURN:
      stopActuator();
      driveTurn(TURN_SPEED);
      showState("TURN", "Turning in place", TFT_NAVY);
      break;

    case FORWARD3:
      stopActuator();
      driveForward(DRIVE_SPEED);
      showState("FWD3", "Driving to deposit", TFT_NAVY);
      break;

    case DEPOSIT:
      stopAll();
      showState("DUMP", "Depositing...", TFT_ORANGE);
      break;

    case GOHOME:
      stopActuator();
      driveBackward(DRIVE_SPEED);
      showState("HOME", "Returning home", TFT_DARKGREEN);
      break;

    case DONE:
      stopAll();
      showState("DONE", "Mission complete!", TFT_DARKGREEN);
      break;
  }
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);  // backlight on

  Serial.begin(115200);

  pinMode(COMMS_PIN, INPUT_PULLDOWN);

  motorSetup(leftMotor);
  motorSetup(rightMotor);

  pinMode(LA_In1, OUTPUT);
  pinMode(LA_In2, OUTPUT);
  pinMode(LA_En,  OUTPUT);
  stopAll();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_LIGHTGREY);

  enterState(WAITING);

  Serial.println("SLAVE v2 ready — waiting for pulses on GPIO 16");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  bool pinNow = digitalRead(COMMS_PIN);

  // Detect rising edge
  if (pinNow == HIGH && lastPinState == LOW) {
    delay(20);  // debounce

    if (currentState < DONE) {
      currentState = (Level)(currentState + 1);
      enterState(currentState);
    }
  }

  lastPinState = pinNow;
  delay(5);
}
