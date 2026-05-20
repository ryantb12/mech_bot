// ===================================================
// SLAVE CONTROL PANEL
// Flash to slave T-Display S3 for debug/tuning.
// Creates WiFi AP "Robot-Slave" / password "robot1234"
// Connect phone, open browser → 192.168.4.1
// Screen stays blank — status shown in phone browser.
// ===================================================

#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ─────────────────────────────────────────────
const char* SSID = "Robot-Slave";
const char* PASS = "robot1234";
WebServer server(80);

// ── Pins — matches slave_controller_v2 exactly ───────
#define LM_In1  43
#define LM_In2  44
#define LM_En   10   // LEDC ch0
#define RM_In1  16   // TTGO pin 8
#define RM_In2  21   // TTGO pin 7
#define RM_En    3   // LEDC ch2
#define LA_In1   1
#define LA_In2   2
#define LA_En   11

// ── PWM — matches slave_controller_v2 ────────────────
#define PWM_FREQ       1000
#define PWM_RESOLUTION   10   // 10-bit: 0–1023

// ── Speeds ───────────────────────────────────────────
#define DEFAULT_SPEED  700

// Per-motor correction factors — adjustable from phone sliders
float lmCorrection = 1.00f;   // left motor  (default 100%)
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
  if (dir == 1)       { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
  else if (dir == -1) { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
  else                { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);  spd = 0; }
  ledcWrite(m.ch, spd);
}

void stopMotors() { setMotor(left, 0, 0); setMotor(right, 0, 0); }

// Right motor physically inverted — direction signals always opposite to left
// rmCorrection reduces right speed to compensate mechanical slip
void driveForward(int s)  { setMotor(left, -1, (int)(s*lmCorrection)); setMotor(right,  1, (int)(s*rmCorrection)); }
void driveBackward(int s) { setMotor(left,  1, (int)(s*lmCorrection)); setMotor(right, -1, (int)(s*rmCorrection)); }
void turnLeft(int s)  { setMotor(left,  1, (int)(s*lmCorrection*0.5)); setMotor(right,  1, (int)(s*rmCorrection)); }
void turnRight(int s) { setMotor(left, -1, (int)(s*lmCorrection));     setMotor(right, -1, (int)(s*rmCorrection*0.5)); }

void setActuator(int dir) {
  if (dir == 1)       { digitalWrite(LA_In1, HIGH); digitalWrite(LA_In2, LOW);  digitalWrite(LA_En, HIGH); }
  else if (dir == -1) { digitalWrite(LA_In1, LOW);  digitalWrite(LA_In2, HIGH); digitalWrite(LA_En, HIGH); }
  else                { digitalWrite(LA_In1, LOW);  digitalWrite(LA_In2, LOW);  digitalWrite(LA_En, LOW);  }
}

void stopAll() { stopMotors(); setActuator(0); }

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Robot Slave</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:12px}
  h1{color:#00d4ff;text-align:center;margin-bottom:12px;font-size:1.4em}
  .card{background:#16213e;border-radius:12px;padding:14px;margin-bottom:10px}
  .card h2{color:#00d4ff;font-size:.9em;text-transform:uppercase;margin-bottom:10px;letter-spacing:1px}
  .row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
  .btn{border:none;border-radius:8px;padding:16px 10px;cursor:pointer;font-size:.95em;font-weight:bold;flex:1;min-width:70px}
  .g{background:#00b894;color:#fff}
  .b{background:#0984e3;color:#fff}
  .o{background:#e17055;color:#fff}
  .p{background:#6c5ce7;color:#fff}
  .gr{background:#636e72;color:#fff}
  .red{background:#d63031;color:#fff;width:100%;padding:18px;font-size:1.15em}
  label{font-size:.85em;color:#aaa;display:block;margin-bottom:4px}
  input[type=range]{width:100%;height:32px;accent-color:#00d4ff;margin-bottom:6px}
  .status-bar{background:#0a0a1a;border-radius:8px;padding:8px;text-align:center;font-size:.85em;color:#00d4ff;margin-top:6px}
</style>
</head><body>
<h1>&#129302; Slave Control</h1>

<div class='card'>
  <h2>Speed</h2>
  <label>Drive speed: <span id='spd_lbl'>700</span> / 1023</label>
  <input type='range' min='0' max='1023' value='700' id='spd'
    oninput="document.getElementById('spd_lbl').innerText=this.value">
</div>

<div class='card'>
  <h2>Motor Speed Correction</h2>
  <label>Left: <span id='lm_lbl'>1.00</span> &nbsp;(<span id='lm_pct'>100</span>%)</label>
  <input type='range' min='50' max='100' value='100' id='lm'
    oninput='updateLm(this.value)'
    onchange='setLm(this.value)'>
  <label style='margin-top:8px'>Right: <span id='cor_lbl'>0.75</span> &nbsp;(<span id='cor_pct'>75</span>%)</label>
  <input type='range' min='50' max='100' value='75' id='cor'
    oninput='updateCor(this.value)'
    onchange='setCor(this.value)'>
</div>

<div class='card'>
  <h2>Drive</h2>
  <div class='row'>
    <button class='btn g' onclick='cmd("fwd")'>&#9650; FWD</button>
    <button class='btn o' onclick='cmd("back")'>&#9660; BACK</button>
    <button class='btn gr' onclick='cmd("stop")'>&#9632; STOP</button>
  </div>
  <div class='row'>
    <button class='btn b' onclick='cmd("turn_l")'>&#9668; TURN L</button>
    <button class='btn b' onclick='cmd("turn_r")'>TURN R &#9658;</button>
  </div>
</div>

<div class='card'>
  <h2>Individual Motors</h2>
  <div class='row'>
    <button class='btn g' onclick='cmd("l_fwd")'>L FWD</button>
    <button class='btn o' onclick='cmd("l_back")'>L BACK</button>
    <button class='btn gr' onclick='cmd("l_stop")'>L STOP</button>
  </div>
  <div class='row'>
    <button class='btn g' onclick='cmd("r_fwd")'>R FWD</button>
    <button class='btn o' onclick='cmd("r_back")'>R BACK</button>
    <button class='btn gr' onclick='cmd("r_stop")'>R STOP</button>
  </div>
</div>

<div class='card'>
  <h2>Actuator</h2>
  <div class='row'>
    <button class='btn p' onclick='cmd("act_up")'>&#9650; UP</button>
    <button class='btn p' onclick='cmd("act_down")'>&#9660; DOWN</button>
    <button class='btn gr' onclick='cmd("act_stop")'>&#9632; STOP</button>
  </div>
</div>

<div class='card'>
  <button class='btn red' onclick='cmd("stop_all")'>&#9888; STOP ALL</button>
  <div class='status-bar' id='status'>Ready</div>
</div>

<script>
function cmd(action){
  const s = document.getElementById('spd').value;
  fetch('/cmd?a='+action+'&s='+s)
    .then(r=>r.text())
    .then(t=>{ document.getElementById('status').innerText=t; })
    .catch(()=>{});
}
function updateLm(pct){
  document.getElementById('lm_lbl').innerText=(pct/100).toFixed(2);
  document.getElementById('lm_pct').innerText=pct;
}
function setLm(pct){
  updateLm(pct);
  fetch('/correction?m=l&v='+(pct/100))
    .then(r=>r.text())
    .then(t=>{ document.getElementById('status').innerText=t; })
    .catch(()=>{});
}
function updateCor(pct){
  document.getElementById('cor_lbl').innerText=(pct/100).toFixed(2);
  document.getElementById('cor_pct').innerText=pct;
}
function setCor(pct){
  updateCor(pct);
  fetch('/correction?m=r&v='+(pct/100))
    .then(r=>r.text())
    .then(t=>{ document.getElementById('status').innerText=t; })
    .catch(()=>{});
}
</script>
</body></html>
)rawliteral";

// ── Handlers ─────────────────────────────────────────
void handleRoot() { server.send(200, "text/html", HTML); }

void handleCorrection() {
  String m = server.arg("m");   // "l" or "r"
  float  v = constrain(server.arg("v").toFloat(), 0.5f, 1.0f);
  if (m == "l") {
    lmCorrection = v;
    Serial.print("LM correction: "); Serial.println(lmCorrection);
    server.send(200, "text/plain", "L motor = " + String(lmCorrection, 2));
  } else {
    rmCorrection = v;
    Serial.print("RM correction: "); Serial.println(rmCorrection);
    server.send(200, "text/plain", "R motor = " + String(rmCorrection, 2));
  }
}

void handleCmd() {
  String a = server.arg("a");
  int    s = server.arg("s").toInt();

  if      (a == "fwd")      driveForward(s);
  else if (a == "back")     driveBackward(s);
  else if (a == "stop")     stopMotors();
  else if (a == "turn_l")   turnLeft(s);
  else if (a == "turn_r")   turnRight(s);
  else if (a == "l_fwd")  { setMotor(left,  -1, s); }
  else if (a == "l_back") { setMotor(left,   1, s); }
  else if (a == "l_stop") { setMotor(left,   0, 0); }
  else if (a == "r_fwd")  { setMotor(right,  1, s); }  // +1 = fwd (inverted motor)
  else if (a == "r_back") { setMotor(right, -1, s); }  // -1 = back (inverted motor)
  else if (a == "r_stop") { setMotor(right,  0, 0); }
  else if (a == "act_up")   setActuator( 1);
  else if (a == "act_down") setActuator(-1);
  else if (a == "act_stop") setActuator( 0);
  else if (a == "stop_all") stopAll();

  server.send(200, "text/plain", a + " @ " + String(s));
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  motorSetup(left);
  motorSetup(right);
  pinMode(LA_In1, OUTPUT); pinMode(LA_In2, OUTPUT); pinMode(LA_En, OUTPUT);
  stopAll();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASS);
  delay(100);
  IPAddress ip = WiFi.softAPIP();

  server.on("/",           handleRoot);
  server.on("/cmd",        handleCmd);
  server.on("/correction", handleCorrection);
  server.begin();

  Serial.println("=== SLAVE PANEL READY ===");
  Serial.print("WiFi: "); Serial.println(SSID);
  Serial.print("Pass: "); Serial.println(PASS);
  Serial.print("IP:   "); Serial.println(ip);
}

void loop() { server.handleClient(); }
