// ===================================================
// MOTION 2350 PRO — SERVO CONTROL PANEL
// Flash to MOTION board for debug/servo tuning.
//
// *** Requires WiFi — only works on Pico 2W variant ***
// *** (RP2350 + CYW43439 wireless chip)             ***
// *** If your board has no WiFi, skip this file     ***
//
// Creates WiFi AP "Robot-Motion" / password "robot1234"
// Connect phone, open browser → 192.168.4.1
//
// Features:
//   • Extender servo (GP0) slider + buttons
//   • Flipper R (GP1) + Flipper L (GP3) sliders
//     (GP3 auto-inverts: same command, opposite signal)
//   • All-neutral button
//   • Raw µs input for fine-tuning
//   • RGB LED colour picker
// ===================================================

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ────────────────────────────────────────────
const char* SSID = "Robot-Motion";
const char* PASS = "robot1234";
WebServer server(80);

// ── Pins ────────────────────────────────────────────
const int EXTENDER_PIN = 0;
const int FLIPPER_R    = 1;
const int FLIPPER_L    = 3;
const int BUZZER_PIN   = 22;
#define   RGB_PIN       28
#define   RGB_COUNT      1

// ── Servo limits ─────────────────────────────────────
const int US_MIN  = 1000;
const int US_MID  = 1500;
const int US_MAX  = 2000;

Servo extender, flipR, flipL;
Adafruit_NeoPixel rgb(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ── Current values ───────────────────────────────────
int usExt  = 1500;
int usFlipR= 1500;
int usFlipL= 1500;

void applyServos(){
  extender.writeMicroseconds(usExt);
  flipR.writeMicroseconds(usFlipR);
  flipL.writeMicroseconds(usFlipL);
}

// Auto-invert: flipper L is always mirrored around 1500
int invertFlip(int us){ return 3000 - us; }

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Motion Control</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:12px}
  h1{color:#a29bfe;text-align:center;margin-bottom:12px;font-size:1.4em}
  .card{background:#16213e;border-radius:12px;padding:14px;margin-bottom:10px}
  .card h2{color:#a29bfe;font-size:.9em;text-transform:uppercase;margin-bottom:10px;letter-spacing:1px}
  .row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
  .btn{border:none;border-radius:8px;padding:14px 10px;cursor:pointer;font-size:.9em;font-weight:bold;flex:1}
  .p{background:#6c5ce7;color:#fff}
  .g{background:#00b894;color:#fff}
  .o{background:#e17055;color:#fff}
  .gr{background:#636e72;color:#fff}
  .red{background:#d63031;color:#fff;width:100%;padding:16px;font-size:1.1em}
  label{font-size:.85em;color:#aaa;margin-bottom:3px;display:block}
  input[type=range]{width:100%;height:32px;accent-color:#a29bfe;margin-bottom:4px}
  .us-val{text-align:center;font-size:1.2em;color:#fdcb6e;font-weight:bold;margin-bottom:6px}
  .status-bar{background:#0a0a1a;border-radius:8px;padding:8px;text-align:center;font-size:.85em;color:#a29bfe;margin-top:6px}
  .note{font-size:.75em;color:#636e72;margin-top:4px;text-align:center}
</style>
</head><body>
<h1>&#9881; Motion Control</h1>

<div class='card'>
  <h2>Extender — GP0</h2>
  <label>Position: <span id='ext_lbl'>1500</span> µs</label>
  <input type='range' min='1000' max='2000' value='1500' id='ext_sl'
    oninput="document.getElementById('ext_lbl').innerText=this.value"
    onchange='setServo("ext",this.value)'>
  <div class='row'>
    <button class='btn p' onclick='setUs("ext",1800)'>EXTEND</button>
    <button class='btn gr' onclick='setUs("ext",1500)'>STOP</button>
    <button class='btn o' onclick='setUs("ext",1200)'>RETRACT</button>
  </div>
</div>

<div class='card'>
  <h2>Flipper GP1 + GP3 (auto-inverted)</h2>
  <p class='note'>GP3 always mirrors GP1 — send one signal, both flip correctly.</p>
  <br>
  <label>Flipper: <span id='flip_lbl'>1500</span> µs (GP1) | <span id='flip_inv_lbl'>1500</span> µs (GP3)</label>
  <input type='range' min='1000' max='2000' value='1500' id='flip_sl'
    oninput="updateFlipLabel(this.value)"
    onchange='setServo("flip",this.value)'>
  <div class='row'>
    <button class='btn p' onclick='setUs("flip",1800)'>RAISE</button>
    <button class='btn gr' onclick='setUs("flip",1500)'>STOP</button>
    <button class='btn o' onclick='setUs("flip",1200)'>DUMP</button>
  </div>
</div>

<div class='card'>
  <h2>Individual Override (raw µs)</h2>
  <div class='row'>
    <div style='flex:1'>
      <label>GP0 Extender</label>
      <input type='number' id='us0' value='1500' min='1000' max='2000' style='width:100%;padding:8px;background:#0a0a1a;color:#fff;border:1px solid #444;border-radius:6px'>
    </div>
    <div style='flex:1'>
      <label>GP1 Flip R</label>
      <input type='number' id='us1' value='1500' min='1000' max='2000' style='width:100%;padding:8px;background:#0a0a1a;color:#fff;border:1px solid #444;border-radius:6px'>
    </div>
    <div style='flex:1'>
      <label>GP3 Flip L</label>
      <input type='number' id='us3' value='1500' min='1000' max='2000' style='width:100%;padding:8px;background:#0a0a1a;color:#fff;border:1px solid #444;border-radius:6px'>
    </div>
  </div>
  <button class='btn p' style='width:100%;margin-top:4px' onclick='sendRaw()'>SEND RAW</button>
</div>

<div class='card'>
  <h2>LED Colour</h2>
  <input type='color' id='led_col' value='#282828' style='width:100%;height:40px;border:none;border-radius:8px;cursor:pointer'
    onchange='setLED(this.value)'>
</div>

<div class='card'>
  <button class='btn red' onclick='neutral()'>ALL NEUTRAL (1500µs)</button>
  <div class='status-bar' id='status'>Ready</div>
</div>

<script>
function setServo(servo,us){
  us=parseInt(us);
  if(servo==='ext') { document.getElementById('ext_sl').value=us; document.getElementById('ext_lbl').innerText=us; }
  if(servo==='flip'){ document.getElementById('flip_sl').value=us; updateFlipLabel(us); }
  fetch('/servo?s='+servo+'&us='+us).then(r=>r.text()).then(t=>status(t));
}
function setUs(servo,us){ setServo(servo,us); }
function updateFlipLabel(us){
  document.getElementById('flip_lbl').innerText=us;
  document.getElementById('flip_inv_lbl').innerText=(3000-parseInt(us));
}
function sendRaw(){
  const e=document.getElementById('us0').value;
  const r=document.getElementById('us1').value;
  const l=document.getElementById('us3').value;
  fetch('/raw?e='+e+'&r='+r+'&l='+l).then(t=>status('Raw sent'));
}
function neutral(){ fetch('/neutral').then(r=>r.text()).then(t=>status(t)); }
function setLED(hex){
  const r=parseInt(hex.slice(1,3),16);
  const g=parseInt(hex.slice(3,5),16);
  const b=parseInt(hex.slice(5,7),16);
  fetch('/led?r='+r+'&g='+g+'&b='+b);
}
function status(t){ document.getElementById('status').innerText=t; }
</script>
</body></html>
)rawliteral";

// ── Server handlers ───────────────────────────────────
void handleRoot() { server.send(200,"text/html",HTML); }

void handleServo() {
  String s  = server.arg("s");
  int    us = server.arg("us").toInt();
  us = constrain(us, US_MIN, US_MAX);

  if(s=="ext")  { usExt=us; }
  else if(s=="flip"){
    usFlipR=us;
    usFlipL=invertFlip(us);  // auto-invert GP3
  }
  applyServos();
  server.send(200,"text/plain",s+"="+String(us)+"µs  GP3="+String(usFlipL)+"µs");
}

void handleRaw() {
  usExt  = constrain(server.arg("e").toInt(),US_MIN,US_MAX);
  usFlipR= constrain(server.arg("r").toInt(),US_MIN,US_MAX);
  usFlipL= constrain(server.arg("l").toInt(),US_MIN,US_MAX);
  applyServos();
  server.send(200,"text/plain","Raw: ext="+String(usExt)+" R="+String(usFlipR)+" L="+String(usFlipL));
}

void handleNeutral() {
  usExt=usFlipR=usFlipL=US_MID;
  applyServos();
  server.send(200,"text/plain","All servos → 1500µs");
}

void handleLED() {
  int r=server.arg("r").toInt(), g=server.arg("g").toInt(), b=server.arg("b").toInt();
  rgb.setPixelColor(0,rgb.Color(r,g,b)); rgb.show();
  server.send(200,"text/plain","LED OK");
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN,OUTPUT);

  extender.attach(EXTENDER_PIN);
  flipR.attach(FLIPPER_R);
  flipL.attach(FLIPPER_L);
  applyServos();

  rgb.begin(); rgb.setBrightness(100);
  rgb.setPixelColor(0,rgb.Color(10,10,40)); rgb.show();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID,PASS);
  IPAddress ip = WiFi.softAPIP();

  server.on("/",       handleRoot);
  server.on("/servo",  handleServo);
  server.on("/raw",    handleRaw);
  server.on("/neutral",handleNeutral);
  server.on("/led",    handleLED);
  server.begin();

  tone(BUZZER_PIN,880,150); delay(200); tone(BUZZER_PIN,1100,150);

  Serial.print("AP: "); Serial.println(SSID);
  Serial.print("IP: "); Serial.println(ip);
}

void loop() { server.handleClient(); }
