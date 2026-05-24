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
//
// SYNC: Phase 1 + Phase 2 sequences must match
//       master_control_panel_v2 handleCompetition() /
//       handleCompetition2() — update both together.
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

// ── Pins ──────────────────────────────────────────────
#define STATE_OUT    1    // → MOTION GP26
#define BUTTON_PIN  10    // onboard button (active LOW)

// ── Slave MAC ─────────────────────────────────────────
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
int motionFlipDeg = 90;

// ── Comms ─────────────────────────────────────────────
void sendToSlave(uint8_t cmd, int speed = -1) {
  if (speed >= 0) {
    uint8_t data[2] = {cmd, (uint8_t)(constrain(speed, 0, 1023) / 4)};
    esp_now_send(slaveMac, data, 2);
  } else {
    esp_now_send(slaveMac, &cmd, 1);
  }
  Serial.print("SLV:"); Serial.println(cmd);
}

void sendMotionBurst(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(STATE_OUT, HIGH); delay(15);
    digitalWrite(STATE_OUT, LOW);  delay(15);
  }
}

void sendFlipAbsolute(int angle) {
  motionFlipDeg = ((constrain(angle, 0, 180) + 2) / 5) * 5;
  int burst = 21 + (motionFlipDeg / 5);  // 21–57
  sendMotionBurst(burst);
  Serial.print("FLIP->"); Serial.println(motionFlipDeg);
}

void stopAll() {
  sendToSlave(6);      // slave motors stop
  sendToSlave(9);      // slave actuator stop
  sendMotionBurst(3);  // extender stop
  sendMotionBurst(8);  // motion all neutral
}

// ── Safe delay (e-stop aware) ─────────────────────────
// Returns true if time elapsed normally.
// Returns false immediately if button pressed (e-stop).
bool safeDelay(unsigned long ms) {
  unsigned long end = millis() + ms;
  while (millis() < end) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(30);
      if (digitalRead(BUTTON_PIN) == LOW) {
        while (digitalRead(BUTTON_PIN) == LOW) delay(5);
        return false;
      }
    }
    delay(5);
  }
  return true;
}

// SD: safe delay used inside competition phase functions.
// On button press: stops everything, shows E-STOP, returns false from caller.
#define SD(ms) do { if (!safeDelay(ms)) { stopAll(); showStep("E-STOP", "Press reset to restart", TFT_RED); return false; } } while(0)

// ── JIG SETUP (blocking, no e-stop) ──────────────────
void runJigSetup() {
  showStep("JIG SETUP", "Flipper 52deg", TFT_PURPLE);
  sendFlipAbsolute(52);
  delay(1000);

  showStep("JIG SETUP", "Actuator up 10s", TFT_NAVY);
  sendToSlave(7); delay(10000); sendToSlave(9);
  delay(500);

  showStep("JIG SETUP", "Extend arm 3s", TFT_PURPLE);
  sendMotionBurst(2); delay(3000); sendMotionBurst(3);
  delay(500);

  showStep("JIG SETUP", "Actuator down 7s", TFT_NAVY);
  sendToSlave(8); delay(7000); sendToSlave(9);
}

// ── JIG RETRACT (blocking, no e-stop) ────────────────
void runJigRetract() {
  showStep("RETRACT", "Actuator up 12s", TFT_NAVY);
  sendToSlave(7); delay(12000); sendToSlave(9);
  delay(500);

  showStep("RETRACT", "Flipper 52deg", TFT_PURPLE);
  sendFlipAbsolute(52);
  delay(800);

  showStep("RETRACT", "Retract arm 3s", TFT_PURPLE);
  sendMotionBurst(4); delay(3000); sendMotionBurst(3);
}

// ── COMPETITION PHASE 1 ───────────────────────────────
// Keep in sync with handleCompetition() in master_control_panel_v2.
// Returns false if e-stopped.
bool runPhase1() {
  showStep("PHASE 1", "Running...", TFT_DARKCYAN);
  Serial.println("=== PHASE 1 START ===");

  sendFlipAbsolute(52);                                     SD(800);
  sendMotionBurst(2); SD(3000); sendMotionBurst(3);         SD(500);

  sendToSlave(8); SD(5000); sendToSlave(9);                 SD(300);
  sendToSlave(8); SD(3000); sendToSlave(9);                 SD(300);
  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);

  sendFlipAbsolute(60);  SD(700);
  sendFlipAbsolute(70);  SD(700);
  sendFlipAbsolute(80);  SD(700);
  sendFlipAbsolute(90);  SD(700);
  sendFlipAbsolute(100); SD(500);

  sendToSlave(8); SD(1100); sendToSlave(9);                 SD(500);
  sendFlipAbsolute(105); SD(1000);
  sendFlipAbsolute(110); SD(1000);
  sendFlipAbsolute(115); SD(1000);
  sendFlipAbsolute(120); SD(1000);
  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);
  sendFlipAbsolute(125); SD(1000);
  sendFlipAbsolute(130);                                    SD(500);

  sendToSlave(8); SD(300); sendToSlave(9);                 SD(500);
  sendMotionBurst(2); SD(3000); sendMotionBurst(3);         SD(1000);
  sendMotionBurst(2); SD(1000); sendMotionBurst(3);         SD(500);

  sendToSlave(8); SD(1000); sendToSlave(9);                 SD(500);
  sendFlipAbsolute(135); SD(2000);
  sendFlipAbsolute(140);                                    SD(500);

  sendMotionBurst(4); SD(1550); sendMotionBurst(3);         SD(1000);

  sendFlipAbsolute(145); SD(800);
  sendFlipAbsolute(150); SD(800);
  sendFlipAbsolute(155); SD(800);
  sendFlipAbsolute(160); SD(800);
  sendFlipAbsolute(165); SD(800);
  sendFlipAbsolute(170);                                    SD(500);

  sendToSlave(7); SD(3000); sendToSlave(9);                 SD(1000);
  sendToSlave(7); SD(8000); sendToSlave(9);                 SD(1000);

  sendFlipAbsolute(160);

  Serial.println("=== PHASE 1 DONE ===");
  return true;
}

// ── COMPETITION PHASE 2 ───────────────────────────────
// Keep in sync with handleCompetition2() in master_control_panel_v2.
// Returns false if e-stopped.
bool runPhase2() {
  showStep("PHASE 2", "Running...", TFT_DARKCYAN);
  Serial.println("=== PHASE 2 START ===");

  sendToSlave(7); SD(3000); sendToSlave(9);                 SD(500);
  sendToSlave(3, 472); SD(900); sendToSlave(6);             SD(500);
  sendToSlave(4, 500); SD(800); sendToSlave(6);             SD(500);
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);
  sendToSlave(4, 500); SD(800); sendToSlave(6);             SD(500);

  for (int i = 0; i < 6; i++) {
    sendMotionBurst(2); SD(1000); sendMotionBurst(3);       SD(500);
  }

  sendToSlave(2, 472); SD(2000); sendToSlave(6);            SD(500);
  sendToSlave(2, 472); SD(1000); sendToSlave(6);            SD(500);

  sendFlipAbsolute(70);                                     SD(1000);
  for (int i = 0; i < 3; i++) { sendFlipAbsolute(70);      SD(200); }
  sendFlipAbsolute(80);                                     SD(1000);

  sendToSlave(3, 472); SD(100); sendToSlave(6);

  Serial.println("=== PHASE 2 DONE ===");
  return true;
}

// ── State machine ─────────────────────────────────────
enum Phase { WAIT, JIG, READY, FINISHED_STATE, ESTOPPED };
Phase phase = WAIT;

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
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);
  Serial.begin(115200);
  pinMode(STATE_OUT, OUTPUT); digitalWrite(STATE_OUT, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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

    case WAIT:
      if (btn) {
        runJigSetup();
        phase = JIG;
        showStep("JIG READY", "Press btn to retract", TFT_DARKGREEN);
      }
      break;

    case JIG:
      if (btn) {
        runJigRetract();
        phase = READY;
        showStep("PRESS BTN", "to start competition", TFT_DARKGREY);
      }
      break;

    case READY:
      if (btn) {
        bool ok = runPhase1();
        if (ok) ok = runPhase2();
        if (ok) {
          phase = FINISHED_STATE;
          showStep("FINISHED", "", TFT_DARKGREEN);
        } else {
          phase = ESTOPPED;
        }
      }
      break;

    case FINISHED_STATE:
      if (btn) {
        showStep("RETRACTING", "6x 1s pulses", TFT_ORANGE);
        for (int i = 0; i < 6; i++) {
          sendMotionBurst(4); delay(1000); sendMotionBurst(3);
          delay(300);
        }
        showStep("DONE", "All retracted", TFT_DARKGREEN);
      }
      break;

    case ESTOPPED:
      break;
  }

  delay(10);
}
