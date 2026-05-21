// ===================================================
// MASTER UNIFIED PANEL
// Flash to master T-Display S3.
// Creates WiFi AP "Robot-Master" / password "robot1234"
// Connect phone → 192.168.4.1
//
// Controls ALL robot systems from one panel:
//   • DC motor drive → UART string protocol → slave
//   • Per-motor speed correction forwarded to slave
//   • Live sensors: ultrasonic A/B, IMU 6-axis, yaw, temp
//   • IMU calibration, yaw reset
//   • Navigation — program position, autonomous return
//   • MOTION board state-advance pulse
//   • MOTION board servo control (extender + flippers)
//   • Action log — downloadable as unified_actions.txt
//
// WIRING:
//   master GPIO  1  (STATE_OUT) → MOTION GP26
//   master GPIO 17  (SLAVE_TX)  → slave  GPIO 16
//   master GPIO  4  (MOTOR_TX)  → slave  GPIO 45 (UART)
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ─────────────────────────────────────────────
const char* SSID = "Robot-Master";
const char* PASS = "robot1234";
WebServer server(80);

// ── Pins ─────────────────────────────────────────────
#define TRIG_A        2
#define ECHO_A        3
#define TRIG_B       43
#define ECHO_B       44
#define MPU_SDA      16
#define MPU_SCL      21
#define MPU_ADDR   0x68
#define BUTTON_PIN   10
#define STATE_OUT     1    // state-advance pulse → MOTION GP26
#define SLAVE_TX     17    // state-advance pulse → slave GPIO 16
#define MOTOR_TX_PIN  4    // UART TX → slave GPIO 45

// ── Motor UART constants ──────────────────────────────
#define NAV_MOTOR_SPEED 500   // speed used during autonomous navigation

// ── TFT ──────────────────────────────────────────────
#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// ── Sensor state ──────────────────────────────────────
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;
float accOffX  = 0, accOffY  = 0, accOffZ  = 0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
bool  calibrated   = false;
float yawAngle     = 0;
unsigned long lastGyroTime = 0;
long distA = 0, distB = 0;
int  pulseCount = 0;

// ── Action log ───────────────────────────────────────
#define LOG_MAX_BYTES 6144
String actionLog = "";

// ── Navigation state ─────────────────────────────────
float prog_yaw      = 0;
long  prog_distA    = 0;
bool  hasProgrammed = false;
bool  navigating    = false;
char  lastNavCmd    = 'S';
unsigned long lastNavTick  = 0;
unsigned long navStartTime = 0;

const float NAV_ALIGN_THRESH = 30.0f;
const long  NAV_DIST_THRESH  =  8;
const int   NAV_TICK_MS      = 150;
const int   NAV_TIMEOUT_MS   = 20000;

// ── MPU-6050 ─────────────────────────────────────────

void initMPU() {
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

    int16_t rAX = Wire.read() << 8 | Wire.read();
    int16_t rAY = Wire.read() << 8 | Wire.read();
    int16_t rAZ = Wire.read() << 8 | Wire.read();
    int16_t rT  = Wire.read() << 8 | Wire.read();
    int16_t rGX = Wire.read() << 8 | Wire.read();
    int16_t rGY = Wire.read() << 8 | Wire.read();
    int16_t rGZ = Wire.read() << 8 | Wire.read();

    accX        = rAX / 16384.0f - accOffX;
    accY        = rAY / 16384.0f - accOffY;
    accZ        = rAZ / 16384.0f - accOffZ;
    gyroX       = rGX / 131.0f   - gyroOffX;
    gyroY       = rGY / 131.0f   - gyroOffY;
    gyroZ       = rGZ / 131.0f   - gyroOffZ;
    temperature = (rT  / 340.0f) + 36.53f;

    if (calibrated && lastGyroTime > 0) {
        float dt = (millis() - lastGyroTime) / 1000.0f;
        yawAngle += gyroZ * dt;
    }
    lastGyroTime = millis();
}

void runCalibration() {
    const int SAMPLES = 200;
    float sAX = 0, sAY = 0, sAZ = 0;
    float sGX = 0, sGY = 0, sGZ = 0;

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

        sAX += rAX / 16384.0f; sAY += rAY / 16384.0f; sAZ += rAZ / 16384.0f;
        sGX += rGX / 131.0f;   sGY += rGY / 131.0f;   sGZ += rGZ / 131.0f;
        delay(10);
    }

    accOffX  = sAX / SAMPLES;
    accOffY  = sAY / SAMPLES;
    accOffZ  = sAZ / SAMPLES - 1.0f;
    gyroOffX = sGX / SAMPLES;
    gyroOffY = sGY / SAMPLES;
    gyroOffZ = sGZ / SAMPLES;
    calibrated   = true;
    yawAngle     = 0;
    lastGyroTime = millis();
}

// ── Ultrasonic ───────────────────────────────────────
long ping(int trig, int echo) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    return pulseIn(echo, HIGH, 30000) * 0.034 / 2;
}

// ── State / servo pulses ──────────────────────────────
void sendPulse() {
    digitalWrite(STATE_OUT, HIGH);
    digitalWrite(SLAVE_TX,  HIGH);
    delay(100);
    digitalWrite(STATE_OUT, LOW);
    digitalWrite(SLAVE_TX,  LOW);
    pulseCount++;
}

void sendServoPulses(int count) {
    for (int i = 0; i < count; i++) {
        digitalWrite(STATE_OUT, HIGH); delay(25);
        digitalWrite(STATE_OUT, LOW);  delay(25);
    }
}

// ── Motor UART ────────────────────────────────────────
// Transmit a string command to the slave. Callers are responsible for logging.
void sendMotorCmd(const String& cmd) {
    Serial2.println(cmd);
    Serial.print("[UART] "); Serial.println(cmd);
}

// ── Logging helpers ───────────────────────────────────

void logAction(const String& description) {
    unsigned long ms   = millis();
    unsigned long secs = ms / 1000;
    unsigned long frac = ms % 1000;

    char timestamp[18];
    snprintf(timestamp, sizeof(timestamp), "[%4lus.%03lu] ", secs, frac);

    String entry = String(timestamp) + description + "\n";

    while (actionLog.length() + entry.length() > LOG_MAX_BYTES) {
        int cut = actionLog.indexOf('\n');
        if (cut < 0) { actionLog = ""; break; }
        actionLog = actionLog.substring(cut + 1);
    }

    actionLog += entry;
    Serial.print("[LOG] "); Serial.print(entry);
}

void logIMU(const char* label) {
    readMPU();
    char buf[160];
    snprintf(buf, sizeof(buf),
        "  %s IMU  — Accel: X=%+5.2fg Y=%+5.2fg Z=%+5.2fg"
        "  |  Gyro: X=%+6.1f Y=%+6.1f Z=%+6.1f °/s"
        "  |  Yaw: %+6.1f°  Temp: %.1f°C"
        "  |  Dist A: %ldcm  B: %ldcm",
        label, accX, accY, accZ, gyroX, gyroY, gyroZ,
        yawAngle, temperature, distA, distB);
    logAction(String(buf));
}

// ── Navigation helpers ────────────────────────────────

float normaliseAngle(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void sendNavCmd(char cmd) {
    lastNavCmd = cmd;

    String uartCmd;
    const char* name;
    if      (cmd == 'F') { uartCmd = "FWD:" + String(NAV_MOTOR_SPEED); name = "FORWARD";    }
    else if (cmd == 'B') { uartCmd = "BCK:" + String(NAV_MOTOR_SPEED); name = "BACKWARD";   }
    else if (cmd == 'L') { uartCmd = "TL:"  + String(NAV_MOTOR_SPEED); name = "TURN LEFT";  }
    else if (cmd == 'R') { uartCmd = "TR:"  + String(NAV_MOTOR_SPEED); name = "TURN RIGHT"; }
    else                 { uartCmd = "STP";                              name = "STOP";       }

    sendMotorCmd(uartCmd);

    float diff = normaliseAngle(prog_yaw - yawAngle);
    logAction("NAV  → " + String(name) +
              "  (diff: " + String(diff, 1) + "°"
              "  distA: " + String(distA) + "cm)");
}

void navTick() {
    if (millis() - navStartTime > NAV_TIMEOUT_MS) {
        sendNavCmd('S');
        navigating = false;
        logAction("NAV  timeout   — stopped after " + String(NAV_TIMEOUT_MS / 1000) + "s");
        return;
    }

    float diff = normaliseAngle(prog_yaw - yawAngle);

    bool headingOk = abs(diff) < 10.0f;
    bool distOk    = abs(distA - prog_distA) <= NAV_DIST_THRESH;
    if (headingOk && distOk) {
        sendNavCmd('S');
        navigating = false;
        logAction("NAV  arrived   — heading ~0°, distA within " +
                  String(NAV_DIST_THRESH) + "cm of programmed");
        return;
    }

    char cmd;
    if      (abs(diff) < NAV_ALIGN_THRESH)            cmd = 'F';
    else if (abs(diff) > (180.0f - NAV_ALIGN_THRESH)) cmd = 'B';
    else if (diff > 0)                                 cmd = 'R';
    else                                               cmd = 'L';

    if (cmd != lastNavCmd) sendNavCmd(cmd);
}

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Unified Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: Arial, sans-serif; background: #0d0d1a; color: #e0e0e0; padding: 10px; }
    h1 { color: #00d4ff; text-align: center; margin-bottom: 10px; font-size: 1.3em; }
    .card { background: #16213e; border-radius: 12px; padding: 12px; margin-bottom: 8px; }
    .card h2 { color: #00d4ff; font-size: 0.85em; text-transform: uppercase; margin-bottom: 8px; letter-spacing: 1px; }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
    .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; }
    .sensor-cell { background: #0a0a1a; border-radius: 8px; padding: 8px; text-align: center; }
    .cell-label  { font-size: 0.65em; color: #888; margin-bottom: 2px; }
    .cell-value  { font-size: 1.1em; color: #fdcb6e; font-weight: bold; }
    .row { display: flex; gap: 6px; margin-bottom: 6px; }
    .btn {
      border: none; border-radius: 8px; padding: 13px 8px;
      cursor: pointer; font-size: 0.88em; font-weight: bold; flex: 1;
    }
    .blue   { background: #0984e3; color: #fff; }
    .green  { background: #00b894; color: #fff; }
    .red    { background: #d63031; color: #fff; }
    .purple { background: #6c5ce7; color: #fff; }
    .grey   { background: #636e72; color: #fff; }
    .orange { background: #e17055; color: #fff; }
    .teal   { background: #00cec9; color: #fff; }
    .yellow { background: #fdcb6e; color: #222; }
    .full-width { width: 100%; padding: 14px; font-size: 1em; }
    label { font-size: 0.8em; color: #aaa; margin-bottom: 3px; display: block; }
    input[type=range] { width: 100%; height: 30px; accent-color: #00d4ff; margin-bottom: 4px; }
    .cal-badge { display: inline-block; padding: 2px 7px; border-radius: 20px; font-size: 0.7em; font-weight: bold; }
    .cal-yes { background: #00b894; color: #fff; }
    .cal-no  { background: #d63031; color: #fff; }
    .status-bar { background: #0a0a1a; border-radius: 8px; padding: 7px; text-align: center; font-size: 0.8em; color: #00d4ff; margin-top: 6px; }
    .info-box { background: #0a0a1a; border-radius: 8px; padding: 8px; font-size: 0.8em; color: #aaa; margin: 6px 0; text-align: center; }

    /* D-pad */
    .dpad {
      display: grid;
      grid-template-areas: '. up .' 'left center right' '. down .';
      grid-template-columns: 1fr 1fr 1fr;
      grid-template-rows: auto auto auto;
      gap: 6px;
      max-width: 300px;
      margin: 10px auto 4px;
    }
    .dpad-up     { grid-area: up; }
    .dpad-left   { grid-area: left; }
    .dpad-center { grid-area: center; }
    .dpad-right  { grid-area: right; }
    .dpad-down   { grid-area: down; }
    .dpad .btn   { padding: 20px 4px; font-size: 0.95em; flex: none; width: 100%; }

    /* Navigation indicator */
    .nav-indicator { background: #0a0a1a; border-radius: 8px; padding: 12px; text-align: center; margin: 8px 0; font-size: 1.6em; min-height: 56px; }
    .nav-label { font-size: 0.72em; color: #636e72; margin-top: 4px; text-align: center; margin-bottom: 8px; }

    /* Action log */
    #log_box {
      background: #0a0a1a; border-radius: 8px; padding: 10px;
      height: 260px; overflow-y: auto; font-family: monospace;
      font-size: 0.72em; color: #a8e6cf; white-space: pre-wrap;
      word-break: break-all; margin-bottom: 10px; border: 1px solid #2d3561;
    }
    .log-controls { display: flex; gap: 8px; }
    .log-note { font-size: 0.72em; color: #636e72; margin-top: 6px; text-align: center; }
  </style>
</head>
<body>

<h1>&#129302; Unified Control</h1>

<!-- ═══ MOTOR DRIVE ══════════════════════════════════ -->
<div class='card'>
  <h2>&#128663; Motor Drive</h2>
  <label>Speed: <span id='spd_label'>700</span> / 1023</label>
  <input type='range' min='0' max='1023' value='700' id='motor_spd'
    oninput="document.getElementById('spd_label').innerText=this.value">
  <div class='dpad'>
    <button class='btn green  dpad-up'     onclick='motor("fwd")'>&#9650; FWD</button>
    <button class='btn blue   dpad-left'   onclick='motor("turn_l")'>&#9664; L</button>
    <button class='btn red    dpad-center' onclick='motor("stop")'>&#9632; STOP</button>
    <button class='btn blue   dpad-right'  onclick='motor("turn_r")'>R &#9654;</button>
    <button class='btn orange dpad-down'   onclick='motor("back")'>&#9660; BCK</button>
  </div>
  <div class='status-bar' id='motor_status'>Ready</div>
</div>

<!-- Individual motor control -->
<div class='card'>
  <h2>Individual Motors</h2>
  <div class='row'>
    <button class='btn green'  onclick='motor("l_fwd")'>L FWD</button>
    <button class='btn orange' onclick='motor("l_back")'>L BACK</button>
    <button class='btn grey'   onclick='motor("l_stop")'>L STOP</button>
  </div>
  <div class='row'>
    <button class='btn green'  onclick='motor("r_fwd")'>R FWD</button>
    <button class='btn orange' onclick='motor("r_back")'>R BACK</button>
    <button class='btn grey'   onclick='motor("r_stop")'>R STOP</button>
  </div>
</div>

<!-- Actuator -->
<div class='card'>
  <h2>Linear Actuator</h2>
  <div class='row'>
    <button class='btn purple' onclick='motor("act_up")'>&#9650; UP</button>
    <button class='btn purple' onclick='motor("act_down")'>&#9660; DOWN</button>
    <button class='btn grey'   onclick='motor("act_stop")'>&#9632; STOP</button>
  </div>
</div>

<!-- Stop all -->
<div class='card'>
  <button class='btn red full-width' onclick='motor("stop_all")'>&#9888; STOP ALL MOTORS</button>
</div>

<!-- Motor correction sliders -->
<div class='card'>
  <h2>Motor Speed Correction</h2>
  <label>Left: <span id='lm_label'>1.00</span> &nbsp;(<span id='lm_pct'>100</span>%)</label>
  <input type='range' min='50' max='100' value='100' id='lm_corr'
    oninput='updateLmLabel(this.value)' onchange='setLm(this.value)'>
  <label style='margin-top:8px'>Right: <span id='rm_label'>0.75</span> &nbsp;(<span id='rm_pct'>75</span>%)</label>
  <input type='range' min='50' max='100' value='75' id='rm_corr'
    oninput='updateRmLabel(this.value)' onchange='setRm(this.value)'>
</div>

<!-- ═══ SENSORS ══════════════════════════════════════ -->

<!-- Ultrasonic -->
<div class='card'>
  <h2>Ultrasonic &nbsp;<span id='cal' class='cal-badge cal-no'>NOT CAL</span></h2>
  <div class='grid-2'>
    <div class='sensor-cell'><div class='cell-label'>Sensor A (cm)</div><div class='cell-value' id='dA'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Sensor B (cm)</div><div class='cell-value' id='dB'>--</div></div>
  </div>
</div>

<!-- Accelerometer -->
<div class='card'>
  <h2>Accelerometer (g)</h2>
  <div class='grid-3'>
    <div class='sensor-cell'><div class='cell-label'>X</div><div class='cell-value' id='aX'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Y</div><div class='cell-value' id='aY'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Z</div><div class='cell-value' id='aZ'>--</div></div>
  </div>
</div>

<!-- Gyro + misc -->
<div class='card'>
  <h2>Gyro (°/s) &nbsp;&nbsp; Yaw: <b id='yaw' style='color:#00d4ff'>--</b>°</h2>
  <div class='grid-3'>
    <div class='sensor-cell'><div class='cell-label'>X</div><div class='cell-value' id='gX'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Y</div><div class='cell-value' id='gY'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Z</div><div class='cell-value' id='gZ'>--</div></div>
  </div>
  <div class='grid-2' style='margin-top:6px'>
    <div class='sensor-cell'><div class='cell-label'>Temp (°C)</div><div class='cell-value' id='tmp'>--</div></div>
    <div class='sensor-cell'><div class='cell-label'>Pulses sent</div><div class='cell-value' id='pls'>--</div></div>
  </div>
</div>

<!-- ═══ IMU CONTROLS ════════════════════════════════ -->
<div class='card'>
  <h2>IMU Controls</h2>
  <div class='row'>
    <button class='btn purple' onclick='post("/calibrate")'>CALIBRATE</button>
    <button class='btn blue'   onclick='post("/reset_yaw")'>RESET YAW</button>
  </div>
</div>

<!-- ═══ NAVIGATION ══════════════════════════════════ -->
<div class='card'>
  <h2>&#127987; Navigation</h2>
  <button class='btn teal full-width' onclick='post("/program")'>&#128204; PROGRAM POSITION</button>
  <div class='info-box' id='prog_info'>No position programmed yet</div>
  <div class='nav-indicator' id='nav_arrow'>&#9711;</div>
  <div class='nav-label' id='nav_label'>Navigation idle</div>
  <div class='row'>
    <button class='btn green' onclick='post("/navigate")'>&#9654; NAVIGATE HOME</button>
    <button class='btn red'   onclick='post("/navstop")'>&#9632; STOP NAV</button>
  </div>
  <div class='status-bar' id='nav_status'>Press PROGRAM to save a position, then NAVIGATE HOME.</div>
</div>

<!-- ═══ STATE MACHINE ═══════════════════════════════ -->
<div class='card'>
  <h2>State Machine (MOTION Board)</h2>
  <button class='btn green full-width' onclick='post("/pulse")'>&#9654; ADVANCE STATE (pulse)</button>
</div>

<!-- ═══ MOTION SERVOS ═══════════════════════════════ -->
<div class='card'>
  <h2>Extender — GP0</h2>
  <div class='row'>
    <button class='btn purple' onclick='srv(2)'>EXTEND</button>
    <button class='btn grey'   onclick='srv(3)'>STOP</button>
    <button class='btn orange' onclick='srv(4)'>RETRACT</button>
  </div>
</div>

<div class='card'>
  <h2>Flippers — GP1 + GP3</h2>
  <div class='row'>
    <button class='btn purple' onclick='srv(5)'>RAISE</button>
    <button class='btn grey'   onclick='srv(6)'>STOP</button>
    <button class='btn orange' onclick='srv(7)'>DUMP</button>
  </div>
</div>

<div class='card'>
  <button class='btn red full-width' onclick='srv(8)'>ALL SERVOS NEUTRAL</button>
  <div class='status-bar' id='status'>Ready</div>
</div>

<!-- ═══ ACTION LOG ═══════════════════════════════════ -->
<div class='card'>
  <h2>&#128221; Action Log</h2>
  <div id='log_box'>(no actions yet)</div>
  <div class='log-controls'>
    <button class='btn blue' onclick='downloadLog()'>&#11015; Download .txt</button>
    <button class='btn grey' onclick='clearLog()'>Clear Log</button>
  </div>
  <p class='log-note'>Auto-refreshes every 3 s &bull; IMU snapshot before/after servo commands &bull; timestamps = seconds since boot</p>
</div>

<script>
  // ── Sensor refresh ─────────────────────────────────
  function refreshSensors() {
    fetch('/sensors')
      .then(r => r.json())
      .then(d => {
        document.getElementById('dA').innerText  = (d.distA > 0 && d.distA < 400) ? d.distA + ' cm' : 'n/a';
        document.getElementById('dB').innerText  = (d.distB > 0 && d.distB < 400) ? d.distB + ' cm' : 'n/a';
        document.getElementById('aX').innerText  = d.accX;
        document.getElementById('aY').innerText  = d.accY;
        document.getElementById('aZ').innerText  = d.accZ;
        document.getElementById('gX').innerText  = d.gyroX;
        document.getElementById('gY').innerText  = d.gyroY;
        document.getElementById('gZ').innerText  = d.gyroZ;
        document.getElementById('yaw').innerText = d.yaw;
        document.getElementById('tmp').innerText = d.temp;
        document.getElementById('pls').innerText = d.pulses;
        const badge = document.getElementById('cal');
        badge.innerText   = d.calibrated ? 'CALIBRATED' : 'NOT CAL';
        badge.className   = 'cal-badge ' + (d.calibrated ? 'cal-yes' : 'cal-no');
      })
      .catch(() => {});
  }
  setInterval(refreshSensors, 500);
  refreshSensors();

  // ── Motor commands ─────────────────────────────────
  function motor(action) {
    const s = document.getElementById('motor_spd').value;
    fetch('/motor?a=' + action + '&s=' + s)
      .then(r => r.text())
      .then(t => { document.getElementById('motor_status').innerText = t; })
      .catch(() => {});
  }

  // ── Motor correction sliders ───────────────────────
  function updateLmLabel(pct) {
    document.getElementById('lm_label').innerText = (pct / 100).toFixed(2);
    document.getElementById('lm_pct').innerText   = pct;
  }
  function updateRmLabel(pct) {
    document.getElementById('rm_label').innerText = (pct / 100).toFixed(2);
    document.getElementById('rm_pct').innerText   = pct;
  }
  function setLm(pct) {
    updateLmLabel(pct);
    fetch('/motorfix?m=l&v=' + (pct / 100))
      .then(r => r.text())
      .then(t => { document.getElementById('motor_status').innerText = t; })
      .catch(() => {});
  }
  function setRm(pct) {
    updateRmLabel(pct);
    fetch('/motorfix?m=r&v=' + (pct / 100))
      .then(r => r.text())
      .then(t => { document.getElementById('motor_status').innerText = t; })
      .catch(() => {});
  }

  // ── General commands (servos, IMU, nav) ───────────
  function post(url) {
    fetch(url)
      .then(r => r.text())
      .then(t => { document.getElementById('status').innerText = t; })
      .catch(() => {});
  }
  function srv(n) {
    fetch('/servo?n=' + n)
      .then(r => r.text())
      .then(t => { document.getElementById('status').innerText = t; });
  }

  // ── Navigation status ──────────────────────────────
  const NAV_ARROWS = { F: '&#9650;', B: '&#9660;', L: '&#9664;', R: '&#9654;', S: '&#9711;', idle: '&#9711;' };
  const NAV_NAMES  = { F: 'DRIVE FORWARD', B: 'DRIVE BACKWARD', L: 'TURNING LEFT', R: 'TURNING RIGHT', S: 'STOPPED', idle: 'Navigation idle' };

  function refreshNav() {
    fetch('/navstatus')
      .then(r => r.json())
      .then(d => {
        if (d.programmed) {
          document.getElementById('prog_info').innerText =
            'Programmed — yaw: ' + d.prog_yaw + '°   distA: ' + d.prog_distA + ' cm';
        }
        const key = d.navigating ? d.cmd : 'idle';
        document.getElementById('nav_arrow').innerHTML = NAV_ARROWS[key] || '?';
        document.getElementById('nav_label').innerText = NAV_NAMES[key] || key;
        if (d.navigating) {
          document.getElementById('nav_status').innerText =
            'Heading diff: ' + d.heading_diff + '°   distA now: ' + d.distA + ' cm   target: ' + d.prog_distA + ' cm';
        } else {
          document.getElementById('nav_status').innerText = d.programmed
            ? 'Press NAVIGATE HOME to return to programmed position.'
            : 'Press PROGRAM to save a position, then NAVIGATE HOME.';
        }
      })
      .catch(() => {});
  }
  setInterval(refreshNav, 400);
  refreshNav();

  // ── Action log ─────────────────────────────────────
  function refreshLog() {
    fetch('/log')
      .then(r => r.text())
      .then(text => {
        const box = document.getElementById('log_box');
        const wasAtBottom = box.scrollHeight - box.clientHeight <= box.scrollTop + 5;
        box.innerText = text;
        if (wasAtBottom) box.scrollTop = box.scrollHeight;
      });
  }
  function downloadLog() {
    fetch('/log')
      .then(r => r.text())
      .then(text => {
        const blob = new Blob([text], { type: 'text/plain' });
        const url  = URL.createObjectURL(blob);
        const a    = document.createElement('a');
        a.href     = url;
        a.download = 'unified_actions.txt';
        a.click();
        URL.revokeObjectURL(url);
      });
  }
  function clearLog() {
    fetch('/clearlog')
      .then(r => r.text())
      .then(t => {
        document.getElementById('log_box').innerText = '(log cleared)';
        document.getElementById('status').innerText = t;
      });
  }
  setInterval(refreshLog, 3000);
  refreshLog();
</script>

</body>
</html>
)rawliteral";

// ── HTTP handlers ─────────────────────────────────────

void handleRoot() { server.send(200, "text/html", HTML); }

void handleSensors() {
    String j = "{";
    j += "\"distA\":"      + String(distA)             + ",";
    j += "\"distB\":"      + String(distB)             + ",";
    j += "\"accX\":"       + String(accX,  2)          + ",";
    j += "\"accY\":"       + String(accY,  2)          + ",";
    j += "\"accZ\":"       + String(accZ,  2)          + ",";
    j += "\"gyroX\":"      + String(gyroX, 1)          + ",";
    j += "\"gyroY\":"      + String(gyroY, 1)          + ",";
    j += "\"gyroZ\":"      + String(gyroZ, 1)          + ",";
    j += "\"yaw\":"        + String(yawAngle,    1)    + ",";
    j += "\"temp\":"       + String(temperature, 1)   + ",";
    j += "\"pulses\":"     + String(pulseCount)        + ",";
    j += "\"calibrated\":" + String(calibrated ? "true" : "false") + "}";
    server.send(200, "application/json", j);
}

// GET /motor?a=fwd&s=700
// Forwards a motor command to the slave via UART.
// Any active navigation is cancelled when a manual command arrives.
void handleMotor() {
    String action = server.arg("a");
    int    speed  = constrain(server.arg("s").toInt(), 0, 1023);

    String cmd, label;
    if      (action == "fwd")      { cmd = "FWD:" + String(speed); label = "Drive fwd"; }
    else if (action == "back")     { cmd = "BCK:" + String(speed); label = "Drive back"; }
    else if (action == "turn_l")   { cmd = "TL:"  + String(speed); label = "Turn left"; }
    else if (action == "turn_r")   { cmd = "TR:"  + String(speed); label = "Turn right"; }
    else if (action == "stop")     { cmd = "STP";                   label = "Stop motors"; }
    else if (action == "l_fwd")    { cmd = "LF:"  + String(speed); label = "L fwd"; }
    else if (action == "l_back")   { cmd = "LB:"  + String(speed); label = "L back"; }
    else if (action == "l_stop")   { cmd = "LS";                    label = "L stop"; }
    else if (action == "r_fwd")    { cmd = "RF:"  + String(speed); label = "R fwd"; }
    else if (action == "r_back")   { cmd = "RB:"  + String(speed); label = "R back"; }
    else if (action == "r_stop")   { cmd = "RS";                    label = "R stop"; }
    else if (action == "act_up")   { cmd = "AUP";                   label = "Actuator up"; }
    else if (action == "act_down") { cmd = "ADN";                   label = "Actuator down"; }
    else if (action == "act_stop") { cmd = "AST";                   label = "Actuator stop"; }
    else if (action == "stop_all") { cmd = "STA";                   label = "Stop all"; }
    else { server.send(400, "text/plain", "Unknown: " + action); return; }

    if (navigating) {
        navigating = false;
        logAction("NAV  cancelled — manual motor override (" + action + ")");
    }

    logAction("MOTOR " + label + " @ " + String(speed));
    sendMotorCmd(cmd);
    server.send(200, "text/plain", label + " @ " + String(speed));
}

// GET /motorfix?m=l&v=0.85  →  update per-motor correction on slave
void handleMotorFix() {
    String motor = server.arg("m");
    float  value = constrain(server.arg("v").toFloat(), 0.5f, 1.0f);

    String cmd;
    if (motor == "l") {
        cmd = "LMC:" + String(value, 2);
        logAction("MOTOR L correction → " + String(value, 2));
    } else {
        cmd = "RMC:" + String(value, 2);
        logAction("MOTOR R correction → " + String(value, 2));
    }
    sendMotorCmd(cmd);
    server.send(200, "text/plain", motor + " correction → " + String(value, 2));
}

void handleServo() {
    int n = server.arg("n").toInt();
    if (n < 2 || n > 8) { server.send(400, "text/plain", "Bad cmd"); return; }

    const char* labels[] = { "", "", "EXTEND", "EXT STOP", "RETRACT",
                              "FLIP RAISE", "FLIP STOP", "FLIP DUMP", "ALL NEUTRAL" };
    logIMU("PRE ");
    logAction("CMD  servo    — " + String(labels[n]) + " (burst " + String(n) + ")");
    sendServoPulses(n);
    delay(300);
    logIMU("POST");
    server.send(200, "text/plain", String(labels[n]) + " (" + String(n) + " pulses)");
}

void handlePulse() {
    logIMU("PRE ");
    logAction("CMD  pulse    — state advance #" + String(pulseCount + 1));
    sendPulse();
    delay(100);
    logIMU("POST");
    server.send(200, "text/plain", "Pulse #" + String(pulseCount) + " sent");
}

void handleCalibrate() {
    logAction("CMD  calibrate — 200-sample IMU calibration");
    runCalibration();
    logIMU("POST");
    server.send(200, "text/plain", "IMU calibrated");
}

void handleResetYaw() {
    logAction("CMD  reset_yaw — yaw zeroed (was " + String(yawAngle, 1) + "°)");
    yawAngle     = 0;
    lastGyroTime = millis();
    server.send(200, "text/plain", "Yaw → 0");
}

void handleProgram() {
    if (!calibrated) { server.send(400, "text/plain", "Calibrate IMU first"); return; }
    prog_yaw      = yawAngle;
    prog_distA    = distA;
    hasProgrammed = true;
    logIMU("PROG");
    logAction("PROG saved     — yaw: " + String(prog_yaw, 1) +
              "°  distA: " + String(prog_distA) + "cm");
    server.send(200, "text/plain",
        "Programmed — yaw: " + String(prog_yaw, 1) +
        "°  distA: " + String(prog_distA) + "cm");
}

void handleNavigate() {
    if (!calibrated)    { server.send(400, "text/plain", "Calibrate IMU first"); return; }
    if (!hasProgrammed) { server.send(400, "text/plain", "Program a position first"); return; }
    if (navigating)     { server.send(200, "text/plain", "Already navigating"); return; }

    navigating     = true;
    lastNavCmd     = 'S';
    navStartTime   = millis();
    logAction("NAV  started   — home yaw: " + String(prog_yaw, 1) +
              "°  home distA: " + String(prog_distA) + "cm");
    server.send(200, "text/plain", "Navigation started");
}

void handleNavStop() {
    if (navigating) {
        navigating = false;
        sendNavCmd('S');
        logAction("NAV  stopped   — user cancelled");
    }
    server.send(200, "text/plain", "Navigation stopped");
}

void handleNavStatus() {
    float diff = normaliseAngle(prog_yaw - yawAngle);

    String cmd_str;
    if (!navigating)                                       cmd_str = "idle";
    else if (abs(diff) < NAV_ALIGN_THRESH)                 cmd_str = "F";
    else if (abs(diff) > (180.0f - NAV_ALIGN_THRESH))      cmd_str = "B";
    else if (diff > 0)                                     cmd_str = "R";
    else                                                   cmd_str = "L";

    String j = "{";
    j += "\"navigating\":"   + String(navigating    ? "true" : "false") + ",";
    j += "\"programmed\":"   + String(hasProgrammed ? "true" : "false") + ",";
    j += "\"prog_yaw\":"     + String(prog_yaw,   1) + ",";
    j += "\"prog_distA\":"   + String(prog_distA)    + ",";
    j += "\"heading_diff\":" + String(diff, 1)        + ",";
    j += "\"distA\":"        + String(distA)          + ",";
    j += "\"cmd\":\""        + cmd_str + "\"}";
    server.send(200, "application/json", j);
}

void handleLog() {
    server.send(200, "text/plain",
        actionLog.length() ? actionLog : "(no actions recorded yet)");
}

void handleClearLog() {
    actionLog = "";
    server.send(200, "text/plain", "Log cleared");
}

// ── Setup ─────────────────────────────────────────────
void setup() {
    pinMode(15, OUTPUT); digitalWrite(15, HIGH);
    Serial.begin(115200);

    Serial2.begin(115200, SERIAL_8N1, -1, MOTOR_TX_PIN);

    pinMode(TRIG_A,     OUTPUT);
    pinMode(ECHO_A,     INPUT);
    pinMode(TRIG_B,     OUTPUT);
    pinMode(ECHO_B,     INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(STATE_OUT,  OUTPUT); digitalWrite(STATE_OUT, LOW);
    pinMode(SLAVE_TX,   OUTPUT); digitalWrite(SLAVE_TX,  LOW);

    initMPU();
    readMPU();

    WiFi.softAP(SSID, PASS);
    IPAddress ip = WiFi.softAPIP();

    server.on("/",          handleRoot);
    server.on("/sensors",   handleSensors);
    server.on("/motor",     handleMotor);
    server.on("/motorfix",  handleMotorFix);
    server.on("/servo",     handleServo);
    server.on("/pulse",     handlePulse);
    server.on("/calibrate", handleCalibrate);
    server.on("/reset_yaw", handleResetYaw);
    server.on("/program",   handleProgram);
    server.on("/navigate",  handleNavigate);
    server.on("/navstop",   handleNavStop);
    server.on("/navstatus", handleNavStatus);
    server.on("/log",       handleLog);
    server.on("/clearlog",  handleClearLog);
    server.begin();

    tft.init(); tft.setRotation(1); tft.fillScreen(0xC618);
    tft.setTextColor(TFT_BLACK, 0xC618); tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2); tft.drawString("UNIFIED PANEL", tft.width() / 2, 25);
    tft.setTextSize(1);
    tft.drawString("WiFi: " + String(SSID), tft.width() / 2, 55);
    tft.drawString("Pass: " + String(PASS), tft.width() / 2, 72);
    tft.drawString(ip.toString(),           tft.width() / 2, 90);
    tft.drawString("Motor UART: GPIO4 -> slave GPIO45", tft.width() / 2, 115);

    logAction("Board booted    — unified panel ready at " + ip.toString());
    logIMU("BOOT");

    Serial.print("AP IP: "); Serial.println(ip);
}

// ── Loop ──────────────────────────────────────────────
void loop() {
    server.handleClient();
    distA = ping(TRIG_A, ECHO_A);
    distB = ping(TRIG_B, ECHO_B);
    readMPU();

    if (navigating && (millis() - lastNavTick > NAV_TICK_MS)) {
        lastNavTick = millis();
        navTick();
    }
}
