// ===================================================
// SLAVE CONTROLLER v2 — TTGO T-Display S3 #2
//
// Receives commands from master via ESP-NOW (wireless, no wire).
// MAC addresses defined in Context/mac_addresses.h and platformio.ini.
//
// ESP-NOW command byte:
//   0x00 = state advance
//   0x02–0x09 = direct motor commands
//
// MOTION board still uses GPIO pulse: master GPIO 1 → MOTION GP26
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
#include <WiFi.h>
#include <esp_now.h>

// ===================================================
// MAC ADDRESS — from Context/mac_addresses.h
// ===================================================
#ifndef MASTER_MAC
  #define MASTER_MAC {0x30, 0x30, 0xF9, 0x59, 0x31, 0x78}
#endif

// ===================================================
// ESP-NOW receive buffer (written by ISR, read by loop)
// ===================================================
volatile uint8_t espNowCmd    = 0xFF;  // 0xFF = empty
volatile bool    espNowPending = false;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len >= 1) {
    espNowCmd     = data[0];
    espNowPending = true;
  }
}

// ===================================================
// PIN DEFINITIONS — from schematic SCHLIB_TTGO-T-Display
// ===================================================

// Left wheel motor — MD1 Secondary (U4), A1/2 side
#define LM_In1  43   // TTGO pin 3
#define LM_In2  44   // TTGO pin 4
#define LM_En   10   // TTGO pin 20 — LEDC channel 0

// Right wheel motor — MD2 Primary (U3)
#define RM_In1  16   // TTGO pin 8  (was wrongly set to GPIO 7)
#define RM_In2  21   // TTGO pin 7  (was wrongly set to GPIO 8)
#define RM_En    3   // TTGO pin 21 — LEDC channel 2

// Linear actuator 1 — MD1 Secondary (U4), B1/2 side
#define LA_In1   1   // TTGO pin 23
#define LA_In2   2   // TTGO pin 22
#define LA_En   11   // TTGO pin 19

// Linear actuator 2 — runs in sync with actuator 1
#define LA2_In1 13   // TTGO spare
#define LA2_En  12   // TTGO spare

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
// Actuator 2 (GPIO 13/12) runs in sync — same direction, shared In2 (GPIO 2)
void setActuator(LinearAct l, int dir) {
  if (dir == 1) {
    digitalWrite(l.in1, HIGH); digitalWrite(l.in2, LOW);  digitalWrite(l.en, HIGH);
    digitalWrite(LA2_In1, HIGH); digitalWrite(LA2_En, HIGH);
  } else if (dir == -1) {
    digitalWrite(l.in1, LOW);  digitalWrite(l.in2, HIGH); digitalWrite(l.en, HIGH);
    digitalWrite(LA2_In1, LOW);  digitalWrite(LA2_En, HIGH);
  } else {
    digitalWrite(l.in1, LOW);  digitalWrite(l.in2, LOW);  digitalWrite(l.en, LOW);
    digitalWrite(LA2_In1, LOW);  digitalWrite(LA2_En, LOW);
  }
}

void stopMotors()   { setMotor(leftMotor, 0, 0); setMotor(rightMotor, 0, 0); }
void stopActuator() { setActuator(actuator, 0); }
void stopAll()      { stopMotors(); stopActuator(); }

// Right motor physically inverted — direction signals are always opposite to left
// RM_CORRECTION reduces right motor speed to compensate mechanical slip
void driveForward(int spd)  { setMotor(leftMotor, -1, spd); setMotor(rightMotor,  1, (int)(spd * RM_CORRECTION)); }
void driveBackward(int spd) { setMotor(leftMotor,  1, spd); setMotor(rightMotor, -1, (int)(spd * RM_CORRECTION)); }
// Turn left: left wheel back at +20% speed, right stopped
// Turn right: right wheel back at +20% speed, left stopped
void driveTurn(int spd)     { setMotor(leftMotor, 1, (int)(spd * 1.2)); setMotor(rightMotor, 0, 0); }
void driveTurnRight(int spd){ setMotor(leftMotor, 0, 0); setMotor(rightMotor, -1, (int)(spd * RM_CORRECTION * 1.2)); }

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
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);

  Serial.begin(115200);

  // ESP-NOW — wireless comms from master
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
  } else {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.print("ESP-NOW ready. Slave MAC: ");
    Serial.println(WiFi.macAddress());
  }

  motorSetup(leftMotor);
  motorSetup(rightMotor);
  pinMode(LA_In1, OUTPUT); pinMode(LA_In2, OUTPUT); pinMode(LA_En, OUTPUT);
  pinMode(LA2_In1, OUTPUT); pinMode(LA2_En, OUTPUT);
  digitalWrite(LA2_In1, LOW); digitalWrite(LA2_En, LOW);
  stopAll();

  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_LIGHTGREY);
  enterState(WAITING);

  Serial.println("SLAVE v2 ready — ESP-NOW");
}

void executeDirectCmd(uint8_t n) {
  Serial.print("CMD: "); Serial.println(n);
  switch (n) {
    case 2: stopActuator(); driveForward(DRIVE_SPEED);   break;
    case 3: stopActuator(); driveBackward(DRIVE_SPEED);  break;
    case 4: stopActuator(); driveTurn(TURN_SPEED);       break;
    case 5: stopActuator(); driveTurnRight(TURN_SPEED);  break;
    case 6: stopAll();                                   break;
    case 7: stopMotors(); setActuator(actuator,  1);     break;
    case 8: stopMotors(); setActuator(actuator, -1);     break;
    case 9: stopActuator();                              break;
  }
}

void loop() {
  if (espNowPending) {
    uint8_t cmd   = espNowCmd;
    espNowPending = false;
    espNowCmd     = 0xFF;
    if (cmd == 0x00) {
      Serial.println("STATE ADVANCE via ESP-NOW");
      if (currentState < DONE) { currentState = (Level)(currentState+1); enterState(currentState); }
    } else {
      executeDirectCmd(cmd);
    }
  }

  if (burstCount > 0 && (millis() - lastShortPulse) > 400) {
  delay(5);
}
