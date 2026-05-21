// ===================================================
// MOTION 2350 PRO — SERVO + MUSIC + RGB CONTROLLER v2
//
// Receives rising-edge pulses from master on GP26.
// Each pulse advances one state, matching master_controller_v2.
//
// WIRING: master GPIO 1 (STATE_OUT_PIN) --> MOTION GP26
//
// State | Master state | MOTION action
// ------+--------------+------------------------------
//   0   | WAITING      | All servos neutral, white LED
//   1   | CALIBRATION  | All servos neutral, cyan LED
//   2   | FORWARD1     | All stop, green LED
//   3   | BACKWARD     | All stop, yellow LED
//   4   | FORWARD2     | All stop, green LED
//   5   | EXTEND       | Extender runs, purple LED
//   6   | RAISEARM     | Flippers raise claw, magenta LED
//   7   | TURN         | All neutral, blue LED
//   8   | FORWARD3     | All neutral, green LED
//   9   | DEPOSIT      | Flippers dump, orange LED
//  10   | GOHOME       | All neutral, teal LED
//  11+  | Done         | All neutral, white flash
//
// SERVOS (all continuous rotation):
//   GP0 — Extender       : 1500=stop, 1800=extend, 1200=retract
//   GP1 — Flipper right  : 1500=stop, 1800=raise,  1200=dump
//   GP3 — Flipper left   : INVERTED  1500=stop, 1200=raise, 1800=dump
//
// MUSIC: "Samplab_Again" — plays on loop from GP22 buzzer
// RGB LED: Adafruit NeoPixel — verify GP pin below
// ===================================================

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ===================================================
// PIN DEFINITIONS
// ===================================================
const int STATE_PIN    = 26;
const int EXTENDER_PIN = 0;
const int FLIPPER_R    = 1;
const int FLIPPER_L    = 3;
const int SERVO5_PIN   = 5;   // GP5 — range-check servo
const int BUZZER_PIN   = 22;

// Range check limits for GP5 servo (tune to your servo's safe range)
#define RANGE_MIN  30   // minimum safe angle
#define RANGE_MAX 150   // maximum safe angle

// *** Verify this pin matches your board's NeoPixel ***
#define RGB_PIN        28
#define RGB_COUNT       1   // number of onboard NeoPixels

// ===================================================
// EXTENDER (continuous rotation) — pulse widths µs
// ===================================================
const int NEUTRAL      = 1500;
const int EXT_EXTEND   = 1800;
const int EXT_RETRACT  = 1200;

// FLIPPERS (180° servo) — angles in degrees
// Back-to-back mount: right and left are mirrored.
// Tune FLIP_RAISE and FLIP_DUMP to match physical range.
const int FLIP_MID     =  90;   // midpoint — set on startup
const int FLIP_R_RAISE =  45;   // raise claw   (tune as needed)
const int FLIP_L_RAISE = 135;   // mirrored
const int FLIP_R_DUMP  = 135;   // dump rocks   (tune as needed)
const int FLIP_L_DUMP  =  45;   // mirrored

// ===================================================
// MELODY — "Samplab_Again" INSTRUMENTAL (Track 0)
// Played sequentially, highest pitch kept per start tick,
// 0-duration notes removed. ~10 second loop.
// {frequency Hz, duration ms}
// ===================================================
struct Note { int freq; int dur; };

const Note melody[] = {
  {  185,  510 },  // F#3
  {  440,  319 },  // A4   (kept over B3)
  {  554,  190 },  // C#5  (kept over C#4)
  {  293,  127 },  // D4
  {  185,  127 },  // F#3
  {  494,  510 },  // B4
  {  587,  383 },  // D5
  {  247,  127 },  // B3
  {  277,  319 },  // C#4
  {  659,  159 },  // E5
  {  740,  287 },  // F#5  (kept over B4)
  {  196,   30 },  // G3
  {  659,  927 },  // E5   (kept over B4)
  {  740,  383 },  // F#5  (kept over D4)
  {  329,   95 },  // E4
  {  587,  383 },  // D5
  {  659,  510 },  // E5   (kept over B4)
  {  659,  319 },  // E5
  {  740,  190 },  // F#5
  {  587,  479 },  // D5
  {  494,  319 },  // B4
  {  247,  127 },  // B3
  {  740,  319 },  // F#5  (kept over D5)
  {  659,  190 },  // E5   (kept over E4)
  {  740,  127 },  // F#5
  {  587,  415 },  // D5
  {  494,   30 },  // B4
  {  659,  319 },  // E5   (kept over D5)
  {  740,  159 },  // F#5
  {  659,  287 },  // E5   (kept over E4)
  {  740,  319 },  // F#5
  {  880,  350 },  // A5
  {  659,  670 },  // E5   (kept over F#5 30ms)
  {  740,  223 },  // F#5
  {  587,  383 },  // D5
  {  659,  895 },  // E5   (kept over B4)
  {  329,  415 },  // E4
  {  740,  190 },  // F#5
};

const int MELODY_LEN = sizeof(melody) / sizeof(melody[0]);
int  currentNote    = 0;
unsigned long noteStart = 0;

// ===================================================
// RGB LED
// ===================================================
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

// ===================================================
// SERVOS
// ===================================================
Servo extender;
Servo flipperR;
Servo flipperL;
Servo servo5;

void allNeutral() {
  extender.writeMicroseconds(NEUTRAL);  // continuous — µs
  flipperR.write(FLIP_MID);             // 180° — degrees
  flipperL.write(FLIP_MID);
}

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

// Current servo positions — tracked for panel UART commands
int usExt          = 1500;
int usFlipR        = 1500;
int usFlipL        = 1500;
int currentFlipDeg = FLIP_MID;  // tracks live flipper angle for nudge commands

void enterState(Level s);  // forward declaration — fixes Arduino enum scope error

// ===================================================
// STATE ENTRY
// ===================================================
void enterState(Level s) {
  Serial.print("MOTION STATE -> ");
  Serial.println((int)s);

  switch (s) {
    case WAITING:
      allNeutral();
      setLED(40, 40, 40);           // dim white
      break;

    case CALIBRATION:
      allNeutral();
      setLED(0, 80, 80);            // cyan
      break;

    case FORWARD1:
      allNeutral();
      setLED(0, 120, 0);            // green
      break;

    case BACKWARD:
      allNeutral();
      setLED(120, 100, 0);          // yellow
      break;

    case FORWARD2:
      allNeutral();
      setLED(0, 120, 0);            // green
      break;

    case EXTEND:
      // GP0=1800, GP1=1200 — confirmed working from servo test
      extender.writeMicroseconds(EXT_EXTEND);   // GP0 → 1800
      flipperR.writeMicroseconds(EXT_RETRACT);  // GP1 → 1200 (opposite, both drive extension)
      flipperL.writeMicroseconds(NEUTRAL);      // GP3 → stop
      setLED(80, 0, 120);           // purple
      break;

    case RAISEARM:
      extender.writeMicroseconds(NEUTRAL);
      flipperR.write(FLIP_R_RAISE);
      flipperL.write(FLIP_L_RAISE);
      setLED(120, 0, 80);           // magenta
      break;

    case TURN:
      allNeutral();
      setLED(0, 0, 120);            // blue
      break;

    case FORWARD3:
      allNeutral();
      setLED(0, 120, 40);           // green-teal
      break;

    case DEPOSIT:
      extender.writeMicroseconds(NEUTRAL);
      flipperR.write(FLIP_R_DUMP);
      flipperL.write(FLIP_L_DUMP);
      setLED(180, 60, 0);           // orange
      break;

    case GOHOME:
      allNeutral();
      setLED(0, 80, 60);            // teal
      break;

    case DONE:
    default:
      allNeutral();
      setLED(0, 0, 0);
      // Flash white 3 times
      for (int i = 0; i < 3; i++) {
        setLED(200, 200, 200); delay(200);
        setLED(0, 0, 0);       delay(200);
      }
      setLED(60, 60, 60);
      break;
  }

  // Short beep on every state change
  tone(BUZZER_PIN, 1200, 80);
}

// ===================================================
// NON-BLOCKING MUSIC
// Checks if the current note has finished and
// advances to the next one — call every loop.
// ===================================================
void updateMusic() {
  if (millis() - noteStart >= (unsigned long)melody[currentNote].dur) {
    currentNote = (currentNote + 1) % MELODY_LEN;
    noteStart   = millis();
    if (melody[currentNote].freq > 0) {
      tone(BUZZER_PIN, melody[currentNote].freq, melody[currentNote].dur);
    } else {
      noTone(BUZZER_PIN);
    }
  }
}

// ===================================================
// SERVO COMMAND EXECUTION
// Called with burst count from pulse decoder below.
// 2=extend  3=ext stop  4=retract
// 5=flip raise  6=flip stop  7=flip dump  8=all neutral
// ===================================================
void executeServoCmd(int n) {
  Serial.print("SERVO CMD: "); Serial.println(n);
  switch(n) {
    case 2: usExt=EXT_EXTEND;   extender.writeMicroseconds(usExt);  break;
    case 3: usExt=NEUTRAL;      extender.writeMicroseconds(usExt);  break;
    case 4: usExt=EXT_RETRACT;  extender.writeMicroseconds(usExt);  break;
    case 5: currentFlipDeg=FLIP_R_RAISE;
            flipperR.write(FLIP_R_RAISE); flipperL.write(FLIP_L_RAISE); break;
    case 6: currentFlipDeg=FLIP_MID;
            flipperR.write(FLIP_MID);     flipperL.write(FLIP_MID); break;
    case 7: currentFlipDeg=FLIP_R_DUMP;
            flipperR.write(FLIP_R_DUMP);  flipperL.write(FLIP_L_DUMP); break;
    case 8: currentFlipDeg=FLIP_MID; allNeutral(); break;
    // 9 = FLIP +5°  |  10 = FLIP -5°
    case 9:
      currentFlipDeg = constrain(currentFlipDeg + 5, 0, 180);
      flipperR.write(currentFlipDeg);
      flipperL.write(180 - currentFlipDeg);  // mirrored
      Serial.print("FLIP +5 -> "); Serial.println(currentFlipDeg);
      break;
    case 10:
      currentFlipDeg = constrain(currentFlipDeg - 5, 0, 180);
      flipperR.write(currentFlipDeg);
      flipperL.write(180 - currentFlipDeg);
      Serial.print("FLIP -5 -> "); Serial.println(currentFlipDeg);
      break;
    // 21–57 = absolute angle: (n-21)*5 degrees (0°–180° in 5° steps)
    default:
      if (n >= 21 && n <= 57) {
        currentFlipDeg = (n - 21) * 5;
        flipperR.write(currentFlipDeg);
        flipperL.write(180 - currentFlipDeg);
        Serial.print("FLIP ABS -> "); Serial.println(currentFlipDeg);
      }
      break;
    // 11 = Range check on GP5 — slow sweep, safe limits
    case 11: {
      Serial.println("RANGE CHECK GP5 start");
      servo5.write(90); delay(500);
      for(int p=95; p<=RANGE_MAX; p+=5){ servo5.write(p); delay(150); }
      for(int p=RANGE_MAX-5; p>=90; p-=5){ servo5.write(p); delay(150); }
      for(int p=85; p>=RANGE_MIN; p-=5){ servo5.write(p); delay(150); }
      for(int p=RANGE_MIN+5; p<=90; p+=5){ servo5.write(p); delay(150); }
      Serial.println("RANGE CHECK GP5 done");
      break;
    }
  }
  // Report current flipper positions after any servo command
  Serial.print("POS GP1:"); Serial.print(currentFlipDeg);
  Serial.print(" GP3:"); Serial.println(180 - currentFlipDeg);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(115200);

  pinMode(STATE_PIN, INPUT_PULLDOWN);
  pinMode(BUZZER_PIN, OUTPUT);

  extender.attach(EXTENDER_PIN);
  // 500–2500µs maps write(0–180) to the full physical range
  // If servo still doesn't reach endpoints, tune min/max below
  flipperR.attach(FLIPPER_R, 500, 2500);
  flipperL.attach(FLIPPER_L, 500, 2500);
  servo5.attach(SERVO5_PIN);
  servo5.write(90);  // start at midpoint
  allNeutral();

  rgb.begin();
  rgb.setBrightness(120);
  setLED(40, 40, 40);  // dim white = ready

  // Startup melody (first two notes)
  tone(BUZZER_PIN, 185, 200); delay(220);
  tone(BUZZER_PIN, 440, 200); delay(220);

  // Kick off looping music
  noteStart = millis();
  tone(BUZZER_PIN, melody[0].freq, melody[0].dur);

  Serial.println("MOTION v2 ready — GP26, music running");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  // ---- Pulse detection (duration-based) ----
  // Long pulse ≥60ms  → state advance
  // Short pulse <60ms → servo command (count bursts)
  static unsigned long risingTime   = 0;
  static int           burstCount   = 0;
  static unsigned long lastShortPulse = 0;

  bool pinNow = digitalRead(STATE_PIN);

  if (pinNow == HIGH && lastPinState == LOW) {
    risingTime = millis();  // record rising edge
  }

  if (pinNow == LOW && lastPinState == HIGH) {
    unsigned long dur = millis() - risingTime;
    if (dur >= 60) {
      // Long pulse — state advance
      if (currentState < DONE) {
        currentState = (Level)(currentState + 1);
        enterState(currentState);
        currentNote = 0; noteStart = millis();
        tone(BUZZER_PIN, melody[0].freq, melody[0].dur);
      }
      burstCount = 0;
    } else if (dur >= 10) {
      // Short pulse — part of servo command burst
      burstCount++;
      lastShortPulse = millis();
    }
  }

  // Process burst after 400ms of silence
  if (burstCount > 0 && (millis() - lastShortPulse) > 400) {
    executeServoCmd(burstCount);
    burstCount = 0;
  }

  lastPinState = pinNow;

  // ---- Keep music playing ----
  updateMusic();

  delay(5);
}
