// ===================================================
// SLAVE CONTROL PANEL
// Flash to slave T-Display S3 for debug/tuning.
// Creates WiFi AP "Robot-Slave" / password "robot1234"
// Connect phone, open browser → 192.168.4.1
// Screen stays blank — status shown in phone browser.
//
// Features:
//   • Drive: forward, backward, turn L/R, stop
//   • Individual left/right motor control
//   • Linear actuator up/down/stop
//   • Per-motor speed correction sliders
//   • Action log — timestamped record of every command,
//                   downloadable as slave_actions.txt
// ===================================================

#include <WiFi.h>
#include <WebServer.h>

// ── WiFi access point ────────────────────────────────
const char* SSID = "Robot-Slave";
const char* PASS = "robot1234";
WebServer server(80);

// ── Motor UART ────────────────────────────────────────
// Receives string commands from the master board (GPIO 4 TX).
// Wire: master GPIO 4 → slave GPIO 45.
// Protocol: each command is a string terminated with '\n', e.g. "FWD:500\n"
#define NAV_RX_PIN  45

// ── Pin assignments — matches slave_controller_v2 ────
#define LM_In1  43
#define LM_In2  44
#define LM_En   10   // LEDC channel 0
#define RM_In1  16
#define RM_In2  21
#define RM_En    3   // LEDC channel 2
#define LA_In1   1
#define LA_In2   2
#define LA_En   11

// ── PWM — matches slave_controller_v2 ────────────────
#define PWM_FREQ        1000
#define PWM_RESOLUTION    10   // 10-bit: 0–1023
#define DEFAULT_SPEED    700

// ── Per-motor correction factors ─────────────────────
// Adjustable from the phone sliders to compensate for mechanical differences.
float lmCorrection = 1.00f;   // left  motor (default 100%)
float rmCorrection = 0.75f;   // right motor (default 75%)

// ── Motor structs ────────────────────────────────────
struct Motor { int in1, in2, en, ch; };
Motor left  = { LM_In1, LM_In2, LM_En, 0 };
Motor right = { RM_In1, RM_In2, RM_En, 2 };

void motorSetup(Motor m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    ledcSetup(m.ch, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(m.en, m.ch);
    ledcWrite(m.ch, 0);
}

// direction: 1=fwd, -1=rev, 0=stop
void setMotor(Motor m, int dir, int spd) {
    if      (dir ==  1) { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
    else if (dir == -1) { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
    else                { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);  spd = 0; }
    ledcWrite(m.ch, spd);
}

void stopMotors() {
    setMotor(left,  0, 0);
    setMotor(right, 0, 0);
}

// Right motor is physically inverted — direction signals always opposite to left.
// rmCorrection reduces right speed to compensate for mechanical slip.
void driveForward(int s)  { setMotor(left, -1, (int)(s * lmCorrection)); setMotor(right,  1, (int)(s * rmCorrection)); }
void driveBackward(int s) { setMotor(left,  1, (int)(s * lmCorrection)); setMotor(right, -1, (int)(s * rmCorrection)); }
void turnLeft(int s)      { setMotor(left,  1, (int)(s * lmCorrection * 0.5)); setMotor(right,  1, (int)(s * rmCorrection)); }
void turnRight(int s)     { setMotor(left, -1, (int)(s * lmCorrection));       setMotor(right, -1, (int)(s * rmCorrection * 0.5)); }

void setActuator(int dir) {
    if      (dir ==  1) { digitalWrite(LA_In1, HIGH); digitalWrite(LA_In2, LOW);  digitalWrite(LA_En, HIGH); }
    else if (dir == -1) { digitalWrite(LA_In1, LOW);  digitalWrite(LA_In2, HIGH); digitalWrite(LA_En, HIGH); }
    else                { digitalWrite(LA_In1, LOW);  digitalWrite(LA_In2, LOW);  digitalWrite(LA_En, LOW);  }
}

void stopAll() { stopMotors(); setActuator(0); }

// ── Action log ───────────────────────────────────────
// Timestamped record of every command sent from the UI.
// Capped at 6 KB — oldest lines dropped when full.
#define LOG_MAX_BYTES 6144
String actionLog = "";

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

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Slave Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: Arial, sans-serif;
      background: #0d0d1a;
      color: #e0e0e0;
      padding: 12px;
    }
    h1 {
      color: #00d4ff;
      text-align: center;
      margin-bottom: 12px;
      font-size: 1.4em;
    }
    .card {
      background: #16213e;
      border-radius: 12px;
      padding: 14px;
      margin-bottom: 10px;
    }
    .card h2 {
      color: #00d4ff;
      font-size: 0.9em;
      text-transform: uppercase;
      margin-bottom: 10px;
      letter-spacing: 1px;
    }
    .row {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-bottom: 8px;
    }
    .btn {
      border: none;
      border-radius: 8px;
      padding: 16px 10px;
      cursor: pointer;
      font-size: 0.95em;
      font-weight: bold;
      flex: 1;
      min-width: 70px;
    }
    .green  { background: #00b894; color: #fff; }
    .blue   { background: #0984e3; color: #fff; }
    .orange { background: #e17055; color: #fff; }
    .purple { background: #6c5ce7; color: #fff; }
    .grey   { background: #636e72; color: #fff; }
    .red    { background: #d63031; color: #fff; width: 100%; padding: 18px; font-size: 1.15em; }

    label { font-size: 0.85em; color: #aaa; display: block; margin-bottom: 4px; }
    input[type=range] { width: 100%; height: 32px; accent-color: #00d4ff; margin-bottom: 6px; }
    .status-bar {
      background: #0a0a1a;
      border-radius: 8px;
      padding: 8px;
      text-align: center;
      font-size: 0.85em;
      color: #00d4ff;
      margin-top: 6px;
    }

    /* Action log */
    #log_box {
      background: #0a0a1a;
      border-radius: 8px;
      padding: 10px;
      height: 240px;
      overflow-y: auto;
      font-family: monospace;
      font-size: 0.78em;
      color: #a8e6cf;
      white-space: pre-wrap;
      word-break: break-all;
      margin-bottom: 10px;
      border: 1px solid #2d3561;
    }
    .log-controls { display: flex; gap: 8px; }
    .log-note { font-size: 0.72em; color: #636e72; margin-top: 6px; text-align: center; }
  </style>
</head>
<body>

<h1>&#129302; Slave Control</h1>

<!-- Drive speed -->
<div class='card'>
  <h2>Speed</h2>
  <label>Drive speed: <span id='spd_label'>700</span> / 1023</label>
  <input type='range' min='0' max='1023' value='700' id='spd'
    oninput="document.getElementById('spd_label').innerText = this.value">
</div>

<!-- Motor correction sliders -->
<div class='card'>
  <h2>Motor Speed Correction</h2>
  <label>Left: <span id='lm_label'>1.00</span> &nbsp;(<span id='lm_pct'>100</span>%)</label>
  <input type='range' min='50' max='100' value='100' id='lm'
    oninput='updateLm(this.value)'
    onchange='setLm(this.value)'>
  <label style='margin-top:8px'>Right: <span id='rm_label'>0.75</span> &nbsp;(<span id='rm_pct'>75</span>%)</label>
  <input type='range' min='50' max='100' value='75' id='rm'
    oninput='updateRm(this.value)'
    onchange='setRm(this.value)'>
</div>

<!-- Drive buttons -->
<div class='card'>
  <h2>Drive</h2>
  <div class='row'>
    <button class='btn green'  onclick='cmd("fwd")'>&#9650; FWD</button>
    <button class='btn orange' onclick='cmd("back")'>&#9660; BACK</button>
    <button class='btn grey'   onclick='cmd("stop")'>&#9632; STOP</button>
  </div>
  <div class='row'>
    <button class='btn blue' onclick='cmd("turn_l")'>&#9668; TURN L</button>
    <button class='btn blue' onclick='cmd("turn_r")'>TURN R &#9658;</button>
  </div>
</div>

<!-- Individual motor control -->
<div class='card'>
  <h2>Individual Motors</h2>
  <div class='row'>
    <button class='btn green'  onclick='cmd("l_fwd")'>L FWD</button>
    <button class='btn orange' onclick='cmd("l_back")'>L BACK</button>
    <button class='btn grey'   onclick='cmd("l_stop")'>L STOP</button>
  </div>
  <div class='row'>
    <button class='btn green'  onclick='cmd("r_fwd")'>R FWD</button>
    <button class='btn orange' onclick='cmd("r_back")'>R BACK</button>
    <button class='btn grey'   onclick='cmd("r_stop")'>R STOP</button>
  </div>
</div>

<!-- Linear actuator -->
<div class='card'>
  <h2>Actuator</h2>
  <div class='row'>
    <button class='btn purple' onclick='cmd("act_up")'>&#9650; UP</button>
    <button class='btn purple' onclick='cmd("act_down")'>&#9660; DOWN</button>
    <button class='btn grey'   onclick='cmd("act_stop")'>&#9632; STOP</button>
  </div>
</div>

<!-- Stop all + status -->
<div class='card'>
  <button class='btn red' onclick='cmd("stop_all")'>&#9888; STOP ALL</button>
  <div class='status-bar' id='status'>Ready</div>
</div>

<!-- Action log -->
<div class='card'>
  <h2>&#128221; Action Log</h2>
  <div id='log_box'>(no actions yet)</div>
  <div class='log-controls'>
    <button class='btn blue' onclick='downloadLog()'>&#11015; Download .txt</button>
    <button class='btn grey' onclick='clearLog()'>Clear Log</button>
  </div>
  <p class='log-note'>
    Auto-refreshes every 3 s &bull; timestamps = seconds since boot
  </p>
</div>

<script>
  // ── Drive commands ──────────────────────────────────

  function cmd(action) {
    const s = document.getElementById('spd').value;
    fetch('/cmd?a=' + action + '&s=' + s)
      .then(r => r.text())
      .then(t => { document.getElementById('status').innerText = t; })
      .catch(() => {});
  }

  // ── Correction sliders ──────────────────────────────

  function updateLm(pct) {
    document.getElementById('lm_label').innerText = (pct / 100).toFixed(2);
    document.getElementById('lm_pct').innerText   = pct;
  }

  function setLm(pct) {
    updateLm(pct);
    fetch('/correction?m=l&v=' + (pct / 100))
      .then(r => r.text())
      .then(t => { document.getElementById('status').innerText = t; })
      .catch(() => {});
  }

  function updateRm(pct) {
    document.getElementById('rm_label').innerText = (pct / 100).toFixed(2);
    document.getElementById('rm_pct').innerText   = pct;
  }

  function setRm(pct) {
    updateRm(pct);
    fetch('/correction?m=r&v=' + (pct / 100))
      .then(r => r.text())
      .then(t => { document.getElementById('status').innerText = t; })
      .catch(() => {});
  }

  // ── Action log ──────────────────────────────────────

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
        a.download = 'slave_actions.txt';
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

void handleRoot() {
    server.send(200, "text/html", HTML);
}

// GET /cmd?a=fwd&s=700  →  execute a drive or actuator command
void handleCmd() {
    String action = server.arg("a");
    int    speed  = server.arg("s").toInt();

    if      (action == "fwd")      { driveForward(speed);  }
    else if (action == "back")     { driveBackward(speed); }
    else if (action == "stop")     { stopMotors();         }
    else if (action == "turn_l")   { turnLeft(speed);      }
    else if (action == "turn_r")   { turnRight(speed);     }
    else if (action == "l_fwd")  { setMotor(left,  -1, speed); }
    else if (action == "l_back") { setMotor(left,   1, speed); }
    else if (action == "l_stop") { setMotor(left,   0, 0);     }
    else if (action == "r_fwd")  { setMotor(right,  1, speed); }   // +1 = fwd (inverted motor)
    else if (action == "r_back") { setMotor(right, -1, speed); }   // -1 = back (inverted motor)
    else if (action == "r_stop") { setMotor(right,  0, 0);     }
    else if (action == "act_up")   { setActuator( 1); }
    else if (action == "act_down") { setActuator(-1); }
    else if (action == "act_stop") { setActuator( 0); }
    else if (action == "stop_all") { stopAll();       }

    logAction("CMD  " + action + " @ speed " + String(speed) +
              "  (L corr: " + String(lmCorrection, 2) +
              "  R corr: "  + String(rmCorrection, 2) + ")");

    server.send(200, "text/plain", action + " @ " + String(speed));
}

// GET /correction?m=l&v=0.85  →  update a per-motor correction factor
void handleCorrection() {
    String motor = server.arg("m");
    float  value = constrain(server.arg("v").toFloat(), 0.5f, 1.0f);

    if (motor == "l") {
        lmCorrection = value;
        logAction("SET  L motor correction → " + String(lmCorrection, 2));
        server.send(200, "text/plain", "L motor = " + String(lmCorrection, 2));
    } else {
        rmCorrection = value;
        logAction("SET  R motor correction → " + String(rmCorrection, 2));
        server.send(200, "text/plain", "R motor = " + String(rmCorrection, 2));
    }
}

// GET /log  →  return the full action log as plain text
void handleLog() {
    server.send(200, "text/plain",
        actionLog.length() ? actionLog : "(no actions recorded yet)");
}

// GET /clearlog  →  wipe the in-memory log buffer
void handleClearLog() {
    actionLog = "";
    server.send(200, "text/plain", "Log cleared");
}

// ── Setup ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    motorSetup(left);
    motorSetup(right);
    pinMode(LA_In1, OUTPUT);
    pinMode(LA_In2, OUTPUT);
    pinMode(LA_En,  OUTPUT);
    stopAll();

    // Nav UART from master: RX only, 115200 baud, GPIO 45
    Serial2.begin(115200, SERIAL_8N1, NAV_RX_PIN, -1);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASS);
    delay(100);
    IPAddress ip = WiFi.softAPIP();

    server.on("/",           handleRoot);
    server.on("/cmd",        handleCmd);
    server.on("/correction", handleCorrection);
    server.on("/log",        handleLog);
    server.on("/clearlog",   handleClearLog);
    server.begin();

    logAction("Board booted    — slave panel ready at " + ip.toString());

    Serial.println("=== SLAVE PANEL READY ===");
    Serial.print("WiFi: "); Serial.println(SSID);
    Serial.print("Pass: "); Serial.println(PASS);
    Serial.print("IP:   "); Serial.println(ip);
}

// ── UART command processor ────────────────────────────
// Called with a complete line (no '\n') received from the master.
void processUartCmd(const String& cmd) {
    if      (cmd.startsWith("FWD:")) { int s = cmd.substring(4).toInt(); driveForward(s);          logAction("NAV  FWD  @ " + String(s)); }
    else if (cmd.startsWith("BCK:")) { int s = cmd.substring(4).toInt(); driveBackward(s);         logAction("NAV  BCK  @ " + String(s)); }
    else if (cmd.startsWith("TL:"))  { int s = cmd.substring(3).toInt(); turnLeft(s);              logAction("NAV  TL   @ " + String(s)); }
    else if (cmd.startsWith("TR:"))  { int s = cmd.substring(3).toInt(); turnRight(s);             logAction("NAV  TR   @ " + String(s)); }
    else if (cmd == "STP")           {                                    stopMotors();              logAction("NAV  STP"); }
    else if (cmd.startsWith("LF:"))  { int s = cmd.substring(3).toInt(); setMotor(left,  -1, s);   logAction("NAV  LF   @ " + String(s)); }
    else if (cmd.startsWith("LB:"))  { int s = cmd.substring(3).toInt(); setMotor(left,   1, s);   logAction("NAV  LB   @ " + String(s)); }
    else if (cmd == "LS")            {                                    setMotor(left,   0, 0);   logAction("NAV  LS"); }
    else if (cmd.startsWith("RF:"))  { int s = cmd.substring(3).toInt(); setMotor(right,  1, s);   logAction("NAV  RF   @ " + String(s)); }
    else if (cmd.startsWith("RB:"))  { int s = cmd.substring(3).toInt(); setMotor(right, -1, s);   logAction("NAV  RB   @ " + String(s)); }
    else if (cmd == "RS")            {                                    setMotor(right,  0, 0);   logAction("NAV  RS"); }
    else if (cmd == "AUP")           {                                    setActuator( 1);          logAction("NAV  AUP"); }
    else if (cmd == "ADN")           {                                    setActuator(-1);          logAction("NAV  ADN"); }
    else if (cmd == "AST")           {                                    setActuator( 0);          logAction("NAV  AST"); }
    else if (cmd == "STA")           {                                    stopAll();                logAction("NAV  STA"); }
    else if (cmd.startsWith("LMC:")) { float v = constrain(cmd.substring(4).toFloat(), 0.5f, 1.0f); lmCorrection = v; logAction("NAV  LMC → " + String(v, 2)); }
    else if (cmd.startsWith("RMC:")) { float v = constrain(cmd.substring(4).toFloat(), 0.5f, 1.0f); rmCorrection = v; logAction("NAV  RMC → " + String(v, 2)); }
}

// ── Loop ──────────────────────────────────────────────
void loop() {
    server.handleClient();

    // Receive string commands from master over Serial2 (GPIO 45 RX).
    // Accumulate chars into uartBuf; process each complete '\n'-terminated line.
    static String uartBuf = "";
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            uartBuf.trim();
            if (uartBuf.length() > 0) processUartCmd(uartBuf);
            uartBuf = "";
        } else if (c != '\r') {
            if (uartBuf.length() < 32) uartBuf += c;
        }
    }
}
