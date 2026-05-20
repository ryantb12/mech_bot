// ===================================================
// MASTER CONTROL PANEL
// Flash to master T-Display S3 for debug/tuning.
// Creates WiFi AP "Robot-Master" / password "robot1234"
// Connect phone, open browser → 192.168.4.1
//
// Features:
//   • Live ultrasonic A+B, IMU, yaw, temp (auto-refresh)
//   • Calibrate IMU, reset yaw
//   • Send state-advance pulse to slave + motion boards
//   • Servo control forwarded to MOTION board via UART
//       Extender (GP0) slider + buttons
//       Flipper (GP1+GP3 auto-inverted) slider + buttons
//
// WIRING (new): master GPIO 14 TX → MOTION GP8 RX
// UART protocol sent to MOTION: "EXT:xxxx\n" or "FLIP:xxxx\n"
//   where xxxx is pulse width in µs (1000–2000)
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ──────────────────────────────────────────────
const char* SSID = "Robot-Master";
const char* PASS = "robot1234";
WebServer server(80);

// ── Sensor / comms pins ───────────────────────────────
#define TRIG_A         2
#define ECHO_A         3
#define TRIG_B        43
#define ECHO_B        44
#define MPU_SDA       16
#define MPU_SCL       21
#define MPU_ADDR    0x68
#define BUTTON_PIN    10
#define STATE_OUT      1   // pulse → MOTION GP26
#define SLAVE_TX      17   // pulse → slave GPIO 16

// ── Servo commands use existing GPIO 1 → GP26 wire ───
// Pulse encoding:
//   Long  pulse 100ms = state advance (existing)
//   Burst of N×25ms   = servo command (N = command code)
//   2=extend  3=ext stop  4=retract
//   5=flip raise  6=flip stop  7=flip dump  8=all neutral

#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// ── Sensor state ──────────────────────────────────────
float accX,accY,accZ,gyroX,gyroY,gyroZ,temperature;
float accOffX=0,accOffY=0,accOffZ=0;
float gyroOffX=0,gyroOffY=0,gyroOffZ=0;
bool  calibrated = false;
float yawAngle   = 0;
unsigned long lastGyroTime = 0;
long  distA = 0, distB = 0;
int   pulseCount = 0;

// ── Servo pulse burst — STATE_OUT only (slave not involved) ──
void sendServoPulses(int count) {
  for(int i = 0; i < count; i++) {
    digitalWrite(STATE_OUT, HIGH); delay(25);
    digitalWrite(STATE_OUT, LOW);  delay(25);
  }
  Serial.print("SERVO BURST: "); Serial.println(count);
}

// ── MPU-6050 ─────────────────────────────────────────
void initMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(bool)true);
  int16_t rAX=Wire.read()<<8|Wire.read(), rAY=Wire.read()<<8|Wire.read(), rAZ=Wire.read()<<8|Wire.read();
  int16_t rT =Wire.read()<<8|Wire.read();
  int16_t rGX=Wire.read()<<8|Wire.read(), rGY=Wire.read()<<8|Wire.read(), rGZ=Wire.read()<<8|Wire.read();
  accX =rAX/16384.0-accOffX; accY =rAY/16384.0-accOffY; accZ =rAZ/16384.0-accOffZ;
  gyroX=rGX/131.0-gyroOffX;  gyroY=rGY/131.0-gyroOffY;  gyroZ=rGZ/131.0-gyroOffZ;
  temperature=(rT/340.0)+36.53;
  if(calibrated&&lastGyroTime>0){ float dt=(millis()-lastGyroTime)/1000.0; yawAngle+=gyroZ*dt; }
  lastGyroTime=millis();
}

void runCalibration() {
  const int S=200;
  float sAX=0,sAY=0,sAZ=0,sGX=0,sGY=0,sGZ=0;
  for(int i=0;i<S;i++){
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(bool)true);
    int16_t rAX=Wire.read()<<8|Wire.read(),rAY=Wire.read()<<8|Wire.read(),rAZ=Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read();
    int16_t rGX=Wire.read()<<8|Wire.read(),rGY=Wire.read()<<8|Wire.read(),rGZ=Wire.read()<<8|Wire.read();
    sAX+=rAX/16384.0; sAY+=rAY/16384.0; sAZ+=rAZ/16384.0;
    sGX+=rGX/131.0;   sGY+=rGY/131.0;   sGZ+=rGZ/131.0;
    delay(10);
  }
  accOffX=sAX/S; accOffY=sAY/S; accOffZ=sAZ/S-1.0;
  gyroOffX=sGX/S; gyroOffY=sGY/S; gyroOffZ=sGZ/S;
  calibrated=true; yawAngle=0; lastGyroTime=millis();
}

// ── Ultrasonic ───────────────────────────────────────
long ping(int trig,int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10); digitalWrite(trig,LOW);
  return pulseIn(echo,HIGH,30000)*0.034/2;
}

// ── State pulse ───────────────────────────────────────
void sendPulse(){
  digitalWrite(STATE_OUT,HIGH); digitalWrite(SLAVE_TX,HIGH); delay(100);
  digitalWrite(STATE_OUT,LOW);  digitalWrite(SLAVE_TX,LOW);
  pulseCount++;
}

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Master Control</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:10px}
  h1{color:#00d4ff;text-align:center;margin-bottom:10px;font-size:1.3em}
  .card{background:#16213e;border-radius:12px;padding:12px;margin-bottom:8px}
  .card h2{color:#00d4ff;font-size:.85em;text-transform:uppercase;margin-bottom:8px;letter-spacing:1px}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
  .grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
  .val{background:#0a0a1a;border-radius:8px;padding:8px;text-align:center}
  .val .lbl{font-size:.65em;color:#888;margin-bottom:2px}
  .val .num{font-size:1.1em;color:#fdcb6e;font-weight:bold}
  .row{display:flex;gap:6px;margin-bottom:6px}
  .btn{border:none;border-radius:8px;padding:13px 8px;cursor:pointer;font-size:.88em;font-weight:bold;flex:1}
  .blue{background:#0984e3;color:#fff}
  .green{background:#00b894;color:#fff}
  .red{background:#d63031;color:#fff}
  .purple{background:#6c5ce7;color:#fff}
  .grey{background:#636e72;color:#fff}
  .orange{background:#e17055;color:#fff}
  .full{width:100%;padding:14px;font-size:1em}
  label{font-size:.8em;color:#aaa;margin-bottom:3px;display:block}
  input[type=range]{width:100%;height:30px;accent-color:#00d4ff;margin-bottom:4px}
  .us{text-align:center;font-size:1.1em;color:#fdcb6e;font-weight:bold;margin-bottom:6px}
  .inv{font-size:.75em;color:#888;text-align:center;margin-bottom:6px}
  .cal{display:inline-block;padding:2px 7px;border-radius:20px;font-size:.7em;font-weight:bold}
  .cy{background:#00b894;color:#fff} .cn{background:#d63031;color:#fff}
  .status{background:#0a0a1a;border-radius:8px;padding:7px;text-align:center;font-size:.8em;color:#00d4ff;margin-top:6px}
</style>
</head><body>
<h1>&#129302; Master Control</h1>

<!-- SENSORS -->
<div class='card'>
  <h2>Ultrasonic &nbsp;<span id='cal' class='cal cn'>NOT CAL</span></h2>
  <div class='grid2'>
    <div class='val'><div class='lbl'>Sensor A (cm)</div><div class='num' id='dA'>--</div></div>
    <div class='val'><div class='lbl'>Sensor B (cm)</div><div class='num' id='dB'>--</div></div>
  </div>
</div>
<div class='card'>
  <h2>Accelerometer (g)</h2>
  <div class='grid3'>
    <div class='val'><div class='lbl'>X</div><div class='num' id='aX'>--</div></div>
    <div class='val'><div class='lbl'>Y</div><div class='num' id='aY'>--</div></div>
    <div class='val'><div class='lbl'>Z</div><div class='num' id='aZ'>--</div></div>
  </div>
</div>
<div class='card'>
  <h2>Gyro (deg/s) &nbsp;&nbsp; Yaw: <b id='yaw' style='color:#00d4ff'>--</b>&deg;</h2>
  <div class='grid3'>
    <div class='val'><div class='lbl'>X</div><div class='num' id='gX'>--</div></div>
    <div class='val'><div class='lbl'>Y</div><div class='num' id='gY'>--</div></div>
    <div class='val'><div class='lbl'>Z</div><div class='num' id='gZ'>--</div></div>
  </div>
  <div class='grid2' style='margin-top:6px'>
    <div class='val'><div class='lbl'>Temp (&deg;C)</div><div class='num' id='tmp'>--</div></div>
    <div class='val'><div class='lbl'>Pulses</div><div class='num' id='pls'>--</div></div>
  </div>
</div>

<!-- IMU CONTROLS -->
<div class='card'>
  <h2>IMU Controls</h2>
  <div class='row'>
    <button class='btn purple' onclick='post("/calibrate")'>CALIBRATE</button>
    <button class='btn blue'   onclick='post("/reset_yaw")'>RESET YAW</button>
  </div>
</div>

<!-- STATE MACHINE -->
<div class='card'>
  <h2>State Machine</h2>
  <button class='btn green full' onclick='post("/pulse")'>&#9654; ADVANCE STATE (pulse)</button>
</div>

<!-- EXTENDER SERVO -->
<div class='card'>
  <h2>Extender — GP0</h2>
  <div class='row'>
    <button class='btn purple' onclick='srv(2)'>EXTEND</button>
    <button class='btn grey'   onclick='srv(3)'>STOP</button>
    <button class='btn orange' onclick='srv(4)'>RETRACT</button>
  </div>
</div>

<!-- FLIPPER SERVOS -->
<div class='card'>
  <h2>Flipper — GP1 + GP3</h2>
  <div class='row'>
    <button class='btn purple' onclick='srv(5)'>RAISE</button>
    <button class='btn grey'   onclick='srv(6)'>STOP</button>
    <button class='btn orange' onclick='srv(7)'>DUMP</button>
  </div>
</div>

<!-- NEUTRAL -->
<div class='card'>
  <button class='btn red full' onclick='srv(8)'>ALL SERVOS NEUTRAL</button>
  <div class='status' id='status'>Ready</div>
</div>

<script>
function post(url){
  fetch(url).then(r=>r.text()).then(t=>{
    document.getElementById('status').innerText=t;
  }).catch(()=>{});
}
function srv(n){
  fetch('/servo?n='+n).then(r=>r.text()).then(t=>{
    document.getElementById('status').innerText=t;
  });
}
function refresh(){
  fetch('/sensors').then(r=>r.json()).then(d=>{
    document.getElementById('dA').innerText=d.distA>0&&d.distA<400?d.distA+'cm':'n/a';
    document.getElementById('dB').innerText=d.distB>0&&d.distB<400?d.distB+'cm':'n/a';
    document.getElementById('aX').innerText=d.accX;
    document.getElementById('aY').innerText=d.accY;
    document.getElementById('aZ').innerText=d.accZ;
    document.getElementById('gX').innerText=d.gyroX;
    document.getElementById('gY').innerText=d.gyroY;
    document.getElementById('gZ').innerText=d.gyroZ;
    document.getElementById('yaw').innerText=d.yaw;
    document.getElementById('tmp').innerText=d.temp;
    document.getElementById('pls').innerText=d.pulses;
    const c=document.getElementById('cal');
    c.innerText=d.calibrated?'CALIBRATED':'NOT CAL';
    c.className='cal '+(d.calibrated?'cy':'cn');
  }).catch(()=>{});
}
setInterval(refresh,500); refresh();
</script>
</body></html>
)rawliteral";

// ── Server handlers ───────────────────────────────────
void handleRoot()     { server.send(200,"text/html",HTML); }

void handleSensors() {
  String j="{";
  j+="\"distA\":"+String(distA)+",\"distB\":"+String(distB)+",";
  j+="\"accX\":"+String(accX,2)+",\"accY\":"+String(accY,2)+",\"accZ\":"+String(accZ,2)+",";
  j+="\"gyroX\":"+String(gyroX,1)+",\"gyroY\":"+String(gyroY,1)+",\"gyroZ\":"+String(gyroZ,1)+",";
  j+="\"yaw\":"+String(yawAngle,1)+",\"temp\":"+String(temperature,1)+",";
  j+="\"pulses\":"+String(pulseCount)+",\"calibrated\":"+(calibrated?"true":"false")+"}";
  server.send(200,"application/json",j);
}

void handleServo() {
  int n = server.arg("n").toInt();
  if(n < 2 || n > 8){ server.send(400,"text/plain","Bad cmd"); return; }
  sendServoPulses(n);
  const char* labels[] = {"","","EXTEND","EXT STOP","RETRACT","FLIP RAISE","FLIP STOP","FLIP DUMP","ALL NEUTRAL"};
  server.send(200,"text/plain",String(labels[n])+" ("+String(n)+" pulses)");
}

void handlePulse()     { sendPulse(); server.send(200,"text/plain","Pulse #"+String(pulseCount)+" sent"); }
void handleCalibrate() { runCalibration(); server.send(200,"text/plain","IMU calibrated"); }
void handleResetYaw()  { yawAngle=0; lastGyroTime=millis(); server.send(200,"text/plain","Yaw → 0"); }

// ── Setup ─────────────────────────────────────────────
void setup() {
  pinMode(15,OUTPUT); digitalWrite(15,HIGH);
  Serial.begin(115200);

  pinMode(TRIG_A,OUTPUT); pinMode(ECHO_A,INPUT);
  pinMode(TRIG_B,OUTPUT); pinMode(ECHO_B,INPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(STATE_OUT,OUTPUT); digitalWrite(STATE_OUT,LOW);
  pinMode(SLAVE_TX,OUTPUT);  digitalWrite(SLAVE_TX,LOW);

  initMPU();

  WiFi.softAP(SSID,PASS);
  IPAddress ip = WiFi.softAPIP();

  server.on("/",         handleRoot);
  server.on("/sensors",  handleSensors);
  server.on("/servo",    handleServo);
  server.on("/pulse",    handlePulse);
  server.on("/calibrate",handleCalibrate);
  server.on("/reset_yaw",handleResetYaw);
  server.begin();

  tft.init(); tft.setRotation(1); tft.fillScreen(0xC618);
  tft.setTextColor(TFT_BLACK,0xC618); tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); tft.drawString("MASTER PANEL",tft.width()/2,25);
  tft.setTextSize(1);
  tft.drawString("WiFi: "+String(SSID),tft.width()/2,55);
  tft.drawString("Pass: "+String(PASS),tft.width()/2,72);
  tft.drawString(ip.toString(),tft.width()/2,90);
  tft.drawString("Servo: pulse bursts via GPIO1",tft.width()/2,115);

  Serial.print("AP IP: "); Serial.println(ip);
}

void loop() {
  server.handleClient();
  distA = ping(TRIG_A,ECHO_A);
  distB = ping(TRIG_B,ECHO_B);
  readMPU();
}
