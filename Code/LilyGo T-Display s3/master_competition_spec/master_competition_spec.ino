// ===================================================
// MASTER COMPETITION SPEC — T-Display S3
//
// Button-driven state machine. No web server / WiFi AP.
// ESP-NOW → slave | GPIO 1 pulse bursts → MOTION GP26
//
// FLOW:
//   [WAIT]   → btn → Jig setup → [JIG]
//   [JIG]    → btn → Jig retract → [READY]
//   [READY]  → btn → Phase 1 → Phase 2 → [FINISHED]
//   Button during competition = EMERGENCY STOP
//   [FINISHED] → btn → Retract arm 6× → [DONE]
//
// SYNC: Phase 1 + Phase 2 sequences must match
//       master_control_panel_v2 handleCompetition() /
//       handleCompetition2() — update both together.
//
// ── WHERE THINGS ARE ──────────────────────────────────
//   Line  18  Includes
//   Line  23  Pin defines (STATE_OUT, BUTTON_PIN)
//   Line  27  Slave MAC address
//   Line  33  TFT display setup + showStep()
//   Line  54  motionFlipDeg — tracks claw angle
//   Line  57  sendToSlave()  — ESP-NOW to slave board
//   Line  68  sendMotionBurst() — GPIO pulse to MOTION board
//   Line  75  sendFlipAbsolute() — set claw angle by pulse burst
//   Line  82  stopAll() — emergency stop all boards
//   Line  89  safeDelay() / SD() macro — delay with e-stop check
//   Line 111  runJigSetup()   — jig preparation sequence
//   Line 129  runJigRetract() — jig teardown sequence
//   Line 143  runPhase1()     — competition phase 1 sequence
//   Line 197  runPhase2()     — competition phase 2 sequence
//   Line 228  State machine enum + readButton()
//   Line 244  setup()
//   Line 267  loop() — state machine switch
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

// ── Pins ──────────────────────────────────────────────
// STATE_OUT: sends pulse bursts to MOTION board GP26
// BUTTON_PIN: onboard button, active LOW (pulled up internally)
#define STATE_OUT    1
#define BUTTON_PIN  10

// ── Slave MAC ─────────────────────────────────────────
// MAC of the slave T-Display S3 — verified from Serial Monitor
#ifndef SLAVE_MAC
  #define SLAVE_MAC {0x30, 0x30, 0xF9, 0x59, 0x31, 0x78}
#endif
uint8_t slaveMac[] = SLAVE_MAC;

// ── Display ───────────────────────────────────────────
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// showStep: fills screen with colour, prints line1 large + line2 small below
void showStep(const char* line1, const char* line2 = "", uint32_t colour = TFT_DARKGREY) {
  tft.fillScreen(colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(line1, tft.width() / 2, line2[0] ? tft.height() / 2 - 18 : tft.height() / 2);
  if (line2[0]) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, colour);
    tft.drawString(line2, tft.width() / 2, tft.height() / 2 + 14);
  }
  Serial.println(line1);
}

// ── Flipper tracking ──────────────────────────────────
// Tracks the claw servo angle so sendFlipAbsolute knows what to send
int motionFlipDeg = 90;

// ── Comms ─────────────────────────────────────────────

// sendToSlave: sends 1-byte command (+ optional speed byte) to slave via ESP-NOW
//   cmd 2=FWD  3=BACK  4=TURN_L  5=TURN_R  6=STOP
//   cmd 7=ACT_UP  8=ACT_DN  9=ACT_STOP
void sendToSlave(uint8_t cmd, int speed = -1) {
  if (speed >= 0) {
    uint8_t data[2] = {cmd, (uint8_t)(constrain(speed, 0, 1023) / 4)};
    esp_now_send(slaveMac, data, 2);
  } else {
    esp_now_send(slaveMac, &cmd, 1);
  }
  Serial.print("SLV:"); Serial.println(cmd);
}

// sendMotionBurst: sends n short pulses (15ms HIGH / 15ms LOW) to MOTION GP26
//   MOTION board counts pulses and runs the matching servo command:
//   2=EXTEND  3=EXT_STOP  4=RETRACT  5=FLIP_RAISE  6=FLIP_MID
//   7=FLIP_DUMP  8=ALL_NEUTRAL  21-57=absolute claw angle (0-180° in 5° steps)
void sendMotionBurst(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(STATE_OUT, HIGH); delay(15);
    digitalWrite(STATE_OUT, LOW);  delay(15);
  }
}

// sendFlipAbsolute: moves claw to exact angle (rounds to nearest 5°)
//   Encodes as burst count: burst = 21 + (angle / 5)
//   e.g. 52° → rounds to 50° → burst 31 → 31 pulses sent
void sendFlipAbsolute(int angle) {
  motionFlipDeg = ((constrain(angle, 0, 180) + 2) / 5) * 5;
  int burst = 21 + (motionFlipDeg / 5);  // 21–57
  sendMotionBurst(burst);
  Serial.print("FLIP->"); Serial.println(motionFlipDeg);
}

// stopAll: halts everything on both boards immediately
void stopAll() {
  sendToSlave(6);      // slave: stop drive motors
  sendToSlave(9);      // slave: stop linear actuators
  sendMotionBurst(3);  // motion: stop extender
  sendMotionBurst(8);  // motion: all servos to neutral
}

// ── Safe delay (e-stop aware) ─────────────────────────
// Used ONLY inside runPhase1() and runPhase2().
// Polls the button every 5ms during the wait.
// Returns true  → time elapsed normally, continue sequence.
// Returns false → button was pressed, trigger emergency stop.
bool safeDelay(unsigned long ms) {
  unsigned long end = millis() + ms;
  while (millis() < end) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(30);
      if (digitalRead(BUTTON_PIN) == LOW) {
        while (digitalRead(BUTTON_PIN) == LOW) delay(5);  // wait for release
        return false;
      }
    }
    delay(5);
  }
  return true;
}

// SD macro: drop-in replacement for delay() inside phase functions.
// If button pressed mid-wait: stops all, shows E-STOP screen, exits the phase function.
#define SD(ms) do { if (!safeDelay(ms)) { stopAll(); showStep("E-STOP", "Press reset to restart", TFT_RED); return false; } } while(0)

// ── JIG SETUP ─────────────────────────────────────────
// Runs when button pressed from WAIT state.
// Prepares the robot for competition — blocking, no e-stop.
// Sequence: claw 52° → actuator UP 10s → extend arm 3s → actuator DOWN 7s
void runJigSetup() {
  showStep("JIG SETUP", "Flipper 52deg", TFT_PURPLE);
  sendFlipAbsolute(52);   // claw to 52° start position
  delay(1000);

  showStep("JIG SETUP", "Actuator up 10s", TFT_NAVY);
  sendToSlave(7); delay(10000); sendToSlave(9);  // raise linear actuators 10s
  delay(500);

  showStep("JIG SETUP", "Extend arm 3s", TFT_PURPLE);
  sendMotionBurst(2); delay(3000); sendMotionBurst(3);  // extend arm 3s
  delay(500);

  showStep("JIG SETUP", "Actuator down 7s", TFT_NAVY);
  sendToSlave(8); delay(7000); sendToSlave(9);  // lower actuators 7s into jig
}

// ── JIG RETRACT ───────────────────────────────────────
// Runs when button pressed from JIG state.
// Lifts robot out of jig and retracts arm — blocking, no e-stop.
// Sequence: actuator UP 12s → claw 52° → retract arm 3s
void runJigRetract() {
  showStep("RETRACT", "Actuator up 12s", TFT_NAVY);
  sendToSlave(7); delay(12000); sendToSlave(9);  // raise actuators 12s (full travel)
  delay(500);

  showStep("RETRACT", "Flipper 52deg", TFT_PURPLE);
  sendFlipAbsolute(52);   // reset claw to start position
  delay(800);

  showStep("RETRACT", "Retract arm 3s", TFT_PURPLE);
  sendMotionBurst(4); delay(3000); sendMotionBurst(3);  // retract arm 3s
}

// ── COMPETITION PHASE 1 ───────────────────────────────
// Main scoring sequence. Runs after jig retract.
// Button during this phase = EMERGENCY STOP (via SD macro).
// Keep in sync with handleCompetition() in master_control_panel_v2.
// Returns true on completion, false if e-stopped.
bool runPhase1() {
  showStep("PHASE 1", "Running...", TFT_DARKCYAN);
  Serial.println("=== PHASE 1 START ===");

  // Claw to 52°, extend arm 3s
  sendFlipAbsolute(52);                                     SD(800);
  sendMotionBurst(2); SD(3000); sendMotionBurst(3);         SD(500);

  // Actuator down: 5s + 3s + 1s (three staged drops to dig in)
  sendToSlave(8); SD(5000); sendToSlave(9);                 SD(300);
  sendToSlave(8); SD(3000); sendToSlave(9);                 SD(300);
  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);

  // Open claw: sweep 60°→100° (2s between each step)
  sendFlipAbsolute(60);  SD(2000);
  sendFlipAbsolute(70);  SD(2000);
  sendFlipAbsolute(80);  SD(2000);
  sendFlipAbsolute(90);  SD(2000);
  sendFlipAbsolute(100); SD(500);

  // Actuator down 1.1s, then continue opening claw 105°→130°
  sendToSlave(8); SD(1100); sendToSlave(9);                 SD(500);
  sendFlipAbsolute(105); SD(2000);
  sendFlipAbsolute(110); SD(2000);
  sendFlipAbsolute(115); SD(2000);
  sendFlipAbsolute(120); SD(2000);
  sendFlipAbsolute(125); SD(2000);
  sendFlipAbsolute(130);                                    SD(500);

  // Actuator down 1s, extend arm 3s + 1s (two extension pushes)
  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);
  sendMotionBurst(2); SD(3000); sendMotionBurst(3);         SD(1000);
  sendMotionBurst(2); SD(1000); sendMotionBurst(3);         SD(500);

  // Actuator down 1s, then close claw to 135°→140°
  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);
  sendFlipAbsolute(135); SD(2000);
  sendFlipAbsolute(140);                                    SD(500);

  // Retract arm 1.3s
  sendMotionBurst(4); SD(1300); sendMotionBurst(3);         SD(1000);

  // Dump sweep: close claw from 145° to 170° (1.5s per step)
  sendFlipAbsolute(145); SD(1500);
  sendFlipAbsolute(150); SD(1500);
  sendFlipAbsolute(155); SD(1500);
  sendFlipAbsolute(160); SD(1500);
  sendFlipAbsolute(165); SD(1500);
  sendFlipAbsolute(170);                                    SD(500);

  // Raise actuators: 3s + 3s
  sendToSlave(7); SD(3000); sendToSlave(9);                 SD(1000);
  sendToSlave(7); SD(3000); sendToSlave(9);                 SD(1000);

  sendFlipAbsolute(160);  // hold claw at 160° ready for phase 2

  Serial.println("=== PHASE 1 DONE ===");
  return true;
}

// ── COMPETITION PHASE 2 ───────────────────────────────
// Robot drives to deposit zone and deposits rocks.
// Button during this phase = EMERGENCY STOP (via SD macro).
// Keep in sync with handleCompetition2() in master_control_panel_v2.
// Returns true on completion, false if e-stopped.
bool runPhase2() {
  showStep("PHASE 2", "Running...", TFT_DARKCYAN);
  Serial.println("=== PHASE 2 START ===");

  // Raise actuators 3s (clear ground)
  sendToSlave(7); SD(3000); sendToSlave(9);                 SD(500);

  // Drive to deposit: back 0.9s, turn left 0.8s
  sendToSlave(3, 472); SD(900); sendToSlave(6);             SD(500);
  sendToSlave(4, 500); SD(800); sendToSlave(6);             SD(500);

  // Forward 1s × 2 (approach deposit zone)
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);

  // Turn left 0.9s (final alignment)
  sendToSlave(4, 500); SD(900); sendToSlave(6);             SD(500);

  // Extend arm 6× 1s pulses (push rocks into deposit zone)
  for (int i = 0; i < 6; i++) {
    sendMotionBurst(2); SD(1000); sendMotionBurst(3);       SD(500);
  }

  // Drive forward 2s + 1s (into zone)
  sendToSlave(2, 472); SD(2000); sendToSlave(6);            SD(500);
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);

  // Open claw: pulse to 70° three times (ensure rocks drop), then to 80°
  sendFlipAbsolute(70);                                     SD(1000);
  for (int i = 0; i < 3; i++) { sendFlipAbsolute(70);      SD(200); }
  sendFlipAbsolute(80);                                     SD(1000);

  // Nudge back 100ms to clear deposit zone
  sendToSlave(3, 472); SD(100); sendToSlave(6);

  Serial.println("=== PHASE 2 DONE ===");
  return true;
}

// ── State machine ─────────────────────────────────────
// WAIT       → press btn → runJigSetup()  → JIG
// JIG        → press btn → runJigRetract() → READY
// READY      → press btn → runPhase1() + runPhase2() → FINISHED_STATE
// FINISHED_STATE → press btn → retract arm 6× → DONE display
// ESTOPPED   → stuck (press hardware reset to restart)
enum Phase { WAIT, JIG, READY, FINISHED_STATE, ESTOPPED };
Phase phase = WAIT;

// readButton: returns true once per press (debounced, waits for release)
bool readButton() {
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

// ── Setup ─────────────────────────────────────────────
void setup() {
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);  // TFT backlight enable
  Serial.begin(115200);
  pinMode(STATE_OUT, OUTPUT); digitalWrite(STATE_OUT, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // WiFi STA mode required for ESP-NOW (no AP needed here)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_now_init();
  {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, slaveMac, 6);
    p.channel = 0; p.encrypt = false;
    esp_now_add_peer(&p);
  }

  tft.init(); tft.setRotation(1);
  showStep("PRESS BTN", "to start jig setup", TFT_DARKGREY);
  Serial.println("Competition spec ready");
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  bool btn = readButton();

  switch (phase) {

    case WAIT:  // waiting for button to begin jig setup
      if (btn) {
        runJigSetup();
        phase = JIG;
        showStep("JIG READY", "Press btn to retract", TFT_DARKGREEN);
      }
      break;

    case JIG:  // jig is set — waiting for button to retract and prepare for comp
      if (btn) {
        runJigRetract();
        phase = READY;
        showStep("PRESS BTN", "to start competition", TFT_DARKGREY);
      }
      break;

    case READY:  // ready to run — button starts the full competition sequence
      if (btn) {
        bool ok = runPhase1();
        if (ok) ok = runPhase2();
        if (ok) {
          phase = FINISHED_STATE;
          showStep("FINISHED", "", TFT_DARKGREEN);
        } else {
          phase = ESTOPPED;  // button was pressed mid-run = e-stop
        }
      }
      break;

    case FINISHED_STATE:  // competition done — button runs post-run arm retract
      if (btn) {
        showStep("RETRACTING", "6x 1s pulses", TFT_ORANGE);
        for (int i = 0; i < 6; i++) {
          sendMotionBurst(4); delay(1000); sendMotionBurst(3);  // retract 1s × 6
          delay(300);
        }
        showStep("DONE", "All retracted", TFT_DARKGREEN);
      }
      break;

    case ESTOPPED:  // e-stopped — press hardware reset button to restart
      break;
  }

  delay(10);
}
