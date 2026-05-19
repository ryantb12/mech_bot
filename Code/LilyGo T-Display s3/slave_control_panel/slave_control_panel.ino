// ===================================================
// SLAVE CONTROL PANEL
// Flash to slave T-Display S3 for debug/tuning.
// Creates WiFi AP "Robot-Slave" / password "robot1234"
// Connect phone, open browser → 192.168.4.1
//
// Features:
//   • Left / right motor speed sliders (0–1023)
//   • Drive direction buttons (fwd/back/turn/stop)
//   • Linear actuator UP / DOWN / STOP
//   • Individual L/R motor overrides
//   • STOP ALL emergency button
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ────────────────────────────────────────────
const char* SSID = "Robot-Slave";
const char* PASS = "robot1234";
WebServer server(80);

// ── Motor pins (confirmed from testing_movement_v1) ─
#define LM_In1  43
#define LM_In2  44
#define LM_En   10   // LEDC ch0
#define RM_In1   7
#define RM_In2   8
#define RM_En    3   // LEDC ch2
#define LA_In1   1
#define LA_In2   2
#define LA_En   11

#define PWM_FREQ       1000
#define PWM_RESOLUTION   10   // 10-bit: 0–1023

#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// ── Motor state (for status endpoint) ───────────────
String lastCmd = "stop_all";
int    lastSpeed = 500;

// ── Motor control ────────────────────────────────────
struct Motor { int in1,in2,en,ch; };
Motor left  = {LM_In1,LM_In2,LM_En,0};
Motor right = {RM_In1,RM_In2,RM_En,2};

void motorSetup(Motor m){
  pinMode(m.in1,OUTPUT); pinMode(m.in2,OUTPUT);
  ledcSetup(m.ch,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(m.en,m.ch); ledcWrite(m.ch,0);
}

// dir: 1=fwd, -1=rev, 0=stop
void setMotor(Motor m, int dir, int spd){
  if(dir==1){  digitalWrite(m.in1,HIGH); digitalWrite(m.in2,LOW);  }
  else if(dir==-1){ digitalWrite(m.in1,LOW); digitalWrite(m.in2,HIGH); }
  else{ digitalWrite(m.in1,LOW); digitalWrite(m.in2,LOW); spd=0; }
  ledcWrite(m.ch,spd);
}

void stopMotors(){ setMotor(left,0,0); setMotor(right,0,0); }

void setActuator(int dir){
  if(dir==1){  digitalWrite(LA_In1,HIGH); digitalWrite(LA_In2,LOW);  digitalWrite(LA_En,HIGH); }
  else if(dir==-1){ digitalWrite(LA_In1,LOW); digitalWrite(LA_In2,HIGH); digitalWrite(LA_En,HIGH); }
  else{ digitalWrite(LA_In1,LOW); digitalWrite(LA_In2,LOW); digitalWrite(LA_En,LOW); }
}

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
  .spd-val{text-align:center;font-size:1.3em;color:#fdcb6e;font-weight:bold;margin-bottom:8px}
  .status-bar{background:#0a0a1a;border-radius:8px;padding:8px;text-align:center;font-size:.85em;color:#00d4ff;margin-top:6px}
</style>
</head><body>
<h1>&#129302; Slave Control</h1>

<div class='card'>
  <h2>Speed</h2>
  <label>Drive speed: <span id='spd_lbl'>500</span> / 1023</label>
  <input type='range' min='0' max='1023' value='500' id='spd'
    oninput="document.getElementById('spd_lbl').innerText=this.value">
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
  const spd = document.getElementById('spd').value;
  fetch('/cmd?a='+action+'&s='+spd)
    .then(r=>r.text())
    .then(t=>{
      document.getElementById('status').innerText=t;
    }).catch(()=>{});
}
</script>
</body></html>
)rawliteral";

// ── Server handlers ───────────────────────────────────
void handleRoot() { server.send(200,"text/html",HTML); }

void handleCmd() {
  String a = server.arg("a");
  int s    = server.arg("s").toInt();
  lastCmd  = a; lastSpeed = s;

  if(a=="fwd")      { setMotor(left,-1,s); setMotor(right,-1,s); }
  else if(a=="back"){ setMotor(left, 1,s); setMotor(right,-1,s); }
  else if(a=="stop"){ stopMotors(); }
  else if(a=="turn_l"){ setMotor(left, 1,s); setMotor(right,-1,s); }
  else if(a=="turn_r"){ setMotor(left,-1,s); setMotor(right, 1,s); }
  else if(a=="l_fwd") { setMotor(left,-1,s); }
  else if(a=="l_back"){ setMotor(left, 1,s); }
  else if(a=="l_stop"){ setMotor(left, 0,0); }
  else if(a=="r_fwd") { setMotor(right,-1,s); }
  else if(a=="r_back"){ setMotor(right, 1,s); }
  else if(a=="r_stop"){ setMotor(right, 0,0); }
  else if(a=="act_up")  { setActuator( 1); }
  else if(a=="act_down"){ setActuator(-1); }
  else if(a=="act_stop"){ setActuator( 0); }
  else if(a=="stop_all"){ stopMotors(); setActuator(0); }

  server.send(200,"text/plain",a+" @ spd="+String(s));

  // Update TFT
  tft.fillRect(0,88,tft.width(),50,0xC618);
  tft.setTextColor(TFT_BLACK,0xC618);
  tft.setTextDatum(MC_DATUM); tft.setTextSize(2);
  tft.drawString(a,tft.width()/2,108);
  tft.setTextSize(1);
  tft.drawString("spd:"+String(s),tft.width()/2,130);
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  pinMode(15,OUTPUT); digitalWrite(15,HIGH);
  Serial.begin(115200);

  motorSetup(left); motorSetup(right);
  pinMode(LA_In1,OUTPUT); pinMode(LA_In2,OUTPUT); pinMode(LA_En,OUTPUT);
  stopMotors(); setActuator(0);

  WiFi.softAP(SSID,PASS);
  IPAddress ip = WiFi.softAPIP();

  server.on("/",    handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();

  tft.init(); tft.setRotation(1); tft.fillScreen(0xC618);
  tft.setTextColor(TFT_BLACK,0xC618); tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); tft.drawString("SLAVE PANEL",tft.width()/2,30);
  tft.setTextSize(1); tft.drawString("WiFi: "+String(SSID),tft.width()/2,60);
  tft.drawString("Pass: "+String(PASS),tft.width()/2,78);
  tft.drawString(ip.toString(),tft.width()/2,100);
  tft.drawString("All motors stopped",tft.width()/2,130);

  Serial.print("AP: "); Serial.println(ip);
}

void loop() { server.handleClient(); }
