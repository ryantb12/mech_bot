// ===================================================
// MASTER COMPETITION — T-Display S3
//
// Standalone competition routine. No web server.
// Press the onboard button (GPIO 10) to start.
//
// WIRING (same as master_control_panel_v2):
//   GPIO  1 → MOTION GP26  (pulse bursts for servos)
//   ESP-NOW  → Slave T-Display S3  (motor + actuator)
//
// SLAVE MAC:  30:30:F9:59:31:78
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

void showStep(const char* label, uint32_t colour = TFT_DARKGREY) {
  tft.fillScreen(colour);
  tft.setTextColor(TFT_WHITE, colour);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(label, tft.width() / 2, tft.height() / 2);
  Serial.println(label);
}

// ── Flipper tracking ──────────────────────────────────
#define FLIP_R_RAISE_DEG  45
int motionFlipDeg = 90;

// ── Comms ─────────────────────────────────────────────
void sendToSlave(uint8_t cmd) {
  esp_now_send(slaveMac, &cmd, 1);
  Serial.print("SLAVE cmd:"); Serial.println(cmd);
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
  Serial.print("FLIP -> "); Serial.println(motionFlipDeg);
}

// ── Competition sequence ──────────────────────────────
void runCompetition() {
  // 1. Claw to 52° (rounds to 50°)
  showStep("CLAW 52", TFT_PURPLE);
  sendFlipAbsolute(52);
  delay(800);

  // 2. Extend arm 3s
  showStep("EXTEND 3s", TFT_PURPLE);
  sendMotionBurst(2); delay(3000); sendMotionBurst(3);
  delay(500);

  // 3. Three actuator drops: 5s, 3s, 1s
  showStep("ACT DN 5s", TFT_NAVY);
  sendToSlave(8); delay(5000); sendToSlave(9); delay(300);

  showStep("ACT DN 3s", TFT_NAVY);
  sendToSlave(8); delay(3000); sendToSlave(9); delay(300);

  showStep("ACT DN 1s", TFT_NAVY);
  sendToSlave(8); delay(1000); sendToSlave(9);
  delay(500);

  // 4. Open claw: sweep 60°→100° in 10° steps
  showStep("FLIP 60", TFT_DARKCYAN);
  sendFlipAbsolute(60);  delay(2000);

  showStep("FLIP 70", TFT_DARKCYAN);
  sendFlipAbsolute(70);  delay(2000);

  showStep("FLIP 80", TFT_DARKCYAN);
  sendFlipAbsolute(80);  delay(2000);

  showStep("FLIP 90", TFT_DARKCYAN);
  sendFlipAbsolute(90);  delay(2000);

  showStep("FLIP 100", TFT_DARKCYAN);
  sendFlipAbsolute(100); delay(500);

  // 5. Actuator down 1s, raise claw to 45°
  showStep("ACT DN 1s", TFT_NAVY);
  sendToSlave(8); delay(1000); sendToSlave(9);
  delay(500);

  showStep("RAISE 45", TFT_PURPLE);
  sendMotionBurst(5); motionFlipDeg = FLIP_R_RAISE_DEG;
  delay(2000);

  // 6. Sweep claw 120° → 130°
  showStep("FLIP 120", TFT_DARKCYAN);
  sendFlipAbsolute(120); delay(2000);

  showStep("FLIP 130", TFT_DARKCYAN);
  sendFlipAbsolute(130); delay(500);

  // 7. Actuator down 1s, extend 3s, extend 1s
  showStep("ACT DN 1s", TFT_NAVY);
  sendToSlave(8); delay(1000); sendToSlave(9); delay(500);

  showStep("EXTEND 3s", TFT_PURPLE);
  sendMotionBurst(2); delay(3000); sendMotionBurst(3); delay(1000);

  showStep("EXTEND 1s", TFT_PURPLE);
  sendMotionBurst(2); delay(1000); sendMotionBurst(3);
  delay(500);

  // 8. Actuator down 1s, sweep claw 135° → 140°
  showStep("ACT DN 1s", TFT_NAVY);
  sendToSlave(8); delay(1000); sendToSlave(9); delay(500);

  showStep("FLIP 135", TFT_DARKCYAN);
  sendFlipAbsolute(135); delay(2000);

  showStep("FLIP 140", TFT_DARKCYAN);
  sendFlipAbsolute(140); delay(500);

  // 9. Retract 1s
  showStep("RETRACT 1s", TFT_ORANGE);
  sendMotionBurst(4); delay(1000); sendMotionBurst(3);
  delay(1000);

  // 10. Sweep claw to dump: 145°→170°
  int dumpAngles[] = {145, 150, 155, 160, 165, 170};
  for (int i = 0; i < 6; i++) {
    char buf[12]; sprintf(buf, "FLIP %d", dumpAngles[i]);
    showStep(buf, TFT_ORANGE);
    sendFlipAbsolute(dumpAngles[i]);
    delay(i < 5 ? 1500 : 500);
  }

  // 11. Actuator up × 2 (3s each)
  showStep("ACT UP 3s", TFT_DARKGREEN);
  sendToSlave(7); delay(3000); sendToSlave(9); delay(1000);

  showStep("ACT UP 3s", TFT_DARKGREEN);
  sendToSlave(7); delay(3000); sendToSlave(9); delay(1000);

  // 12. Hold claw at 160°
  showStep("HOLD 160", TFT_DARKGREEN);
  sendFlipAbsolute(160);

  showStep("DONE!", TFT_DARKGREEN);
  Serial.println("=== COMPETITION DONE ===");
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  pinMode(15, OUTPUT); digitalWrite(15, HIGH);
  Serial.begin(115200);

  pinMode(STATE_OUT, OUTPUT); digitalWrite(STATE_OUT, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // WiFi STA mode for ESP-NOW (no AP needed here)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_now_init();
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, slaveMac, 6);
  p.channel = 0; p.encrypt = false;
  esp_now_add_peer(&p);

  tft.init(); tft.setRotation(1);
  showStep("PRESS BTN", TFT_DARKGREY);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
  tft.drawString("to start competition", tft.width() / 2, tft.height() / 2 + 25);

  Serial.println("Competition ready — press GPIO10 button");
}

// ── Loop ──────────────────────────────────────────────
void loop() {
  // Wait for button press (active LOW, debounce 50ms)
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      // Wait for release before starting
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
      runCompetition();
      // After completion — show ready again
      delay(2000);
      showStep("PRESS BTN", TFT_DARKGREY);
      tft.setTextSize(1);
      tft.setTextColor(TFT_YELLOW, TFT_DARKGREY);
      tft.drawString("to start again", tft.width() / 2, tft.height() / 2 + 25);
    }
  }
}
