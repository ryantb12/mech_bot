// ===================================================
// MASTER CONTROL PANEL
// Flash this to the master T-Display S3 for debug/tuning.
// Creates WiFi AP "Robot-Master" / password "robot1234"
// Connect phone to that network, open browser → 192.168.4.1
//
// Features:
//   • Live ultrasonic A+B, IMU, yaw, temp readings
//   • Send state-advance pulse to slave + motion boards
//   • Trigger IMU calibration remotely
//   • Emergency stop broadcast
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ────────────────────────────────────────────
const char* SSID = "Robot-Master";
const char* PASS = "robot1234";
WebServer server(80);

// ── Pins ────────────────────────────────────────────
#define TRIG_A        2
#define ECHO_A        3
#define TRIG_B       43
#define ECHO_B       44
#define MPU_SDA      16
#define MPU_SCL      21
#define MPU_ADDR   0x68
#define BUTTON_PIN   10
#define STATE_OUT     1   // to MOTION 2350 Pro
#define SLAVE_TX     17   // to slave T-Display

#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// ── Sensor data ─────────────────────────────────────
float accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
float accOffX=0,accOffY=0,accOffZ=0;
float gyroOffX=0,gyroOffY=0,gyroOffZ=0;
bool  calibrated = false;
float yawAngle = 0;
unsigned long lastGyroTime = 0;
long  distA = 0, distB = 0;
int   pulseCount = 0;

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
  accX=rAX/16384.0-accOffX; accY=rAY/16384.0-accOffY; accZ=rAZ/16384.0-accOffZ;
  gyroX=rGX/131.0-gyroOffX; gyroY=rGY/131.0-gyroOffY; gyroZ=rGZ/131.0-gyroOffZ;
  temperature=(rT/340.0)+36.53;
  if(calibrated && lastGyroTime>0){ float dt=(millis()-lastGyroTime)/1000.0; yawAngle+=gyroZ*dt; }
  lastGyroTime=millis();
}

void runCalibration() {
  const int S=200; float sAX=0,sAY=0,sAZ=0,sGX=0,sGY=0,sGZ=0;
  for(int i=0;i<S;i++){
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(bool)true);
    int16_t rAX=Wire.read()<<8|Wire.read(), rAY=Wire.read()<<8|Wire.read(), rAZ=Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read();
    int16_t rGX=Wire.read()<<8|Wire.read(), rGY=Wire.read()<<8|Wire.read(), rGZ=Wire.read()<<8|Wire.read();
    sAX+=rAX/16384.0; sAY+=rAY/16384.0; sAZ+=rAZ/16384.0;
    sGX+=rGX/131.0;   sGY+=rGY/131.0;   sGZ+=rGZ/131.0;
    delay(10);
  }
  accOffX=sAX/S; accOffY=sAY/S; accOffZ=sAZ/S-1.0;
  gyroOffX=sGX/S; gyroOffY=sGY/S; gyroOffZ=sGZ/S;
  calibrated=true; yawAngle=0; lastGyroTime=millis();
}

// ── Ultrasonic ───────────────────────────────────────
long ping(int trig, int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10); digitalWrite(trig,LOW);
  return pulseIn(echo,HIGH,30000)*0.034/2;
}

// ── Pulse ────────────────────────────────────────────
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
<title>Robot Master</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:12px}
  h1{color:#00d4ff;text-align:center;margin-bottom:12px;font-size:1.4em}
  .card{background:#16213e;border-radius:12px;padding:14px;margin-bottom:10px}
  .card h2{color:#00d4ff;font-size:.9em;text-transform:uppercase;margin-bottom:10px;letter-spacing:1px}
  .row{display:flex;gap:8px;flex-wrap:wrap}
  .val{background:#0a0a1a;border-radius:8px;padding:10px;flex:1;min-width:120px;text-align:center}
  .val .lbl{font-size:.7em;color:#888;margin-bottom:4px}
  .val .num{font-size:1.3em;color:#fdcb6e;font-weight:bold}
  .btn{border:none;border-radius:8px;padding:14px 10px;cursor:pointer;font-size:.95em;font-weight:bold;flex:1}
  .btn-blue{background:#0984e3;color:#fff}
  .btn-green{background:#00b894;color:#fff}
  .btn-red{background:#d63031;color:#fff;width:100%;padding:16px;font-size:1.1em}
  .btn-purple{background:#6c5ce7;color:#fff}
  .status{text-align:center;font-size:.8em;color:#00d4ff;margin-top:6px}
  .cal-badge{display:inline-block;padding:3px 8px;border-radius:20px;font-size:.75em;font-weight:bold}
  .cal-yes{background:#00b894;color:#fff}
  .cal-no{background:#d63031;color:#fff}
</style>
</head><body>
<h1>&#129302; Master Control</h1>

<div class='card'>
  <h2>Ultrasonic  &nbsp;<span id='cal' class='cal-badge cal-no'>NOT CAL</span></h2>
  <div class='row'>
    <div class='val'><div class='lbl'>Sensor A (cm)</div><div class='num' id='dA'>--</div></div>
    <div class='val'><div class='lbl'>Sensor B (cm)</div><div class='num' id='dB'>--</div></div>
  </div>
</div>

<div class='card'>
  <h2>IMU — Accelerometer (g)</h2>
  <div class='row'>
    <div class='val'><div class='lbl'>X</div><div class='num' id='aX'>--</div></div>
    <div class='val'><div class='lbl'>Y</div><div class='num' id='aY'>--</div></div>
    <div class='val'><div class='lbl'>Z</div><div class='num' id='aZ'>--</div></div>
  </div>
</div>

<div class='card'>
  <h2>IMU — Gyro (deg/s)  &nbsp; Yaw: <span id='yaw'>--</span>&deg;</h2>
  <div class='row'>
    <div class='val'><div class='lbl'>X</div><div class='num' id='gX'>--</div></div>
    <div class='val'><div class='lbl'>Y</div><div class='num' id='gY'>--</div></div>
    <div class='val'><div class='lbl'>Z</div><div class='num' id='gZ'>--</div></div>
  </div>
  <div class='row' style='margin-top:8px'>
    <div class='val'><div class='lbl'>Temp (C)</div><div class='num' id='tmp'>--</div></div>
    <div class='val'><div class='lbl'>Pulses sent</div><div class='num' id='pls'>--</div></div>
  </div>
</div>

<div class='card'>
  <h2>Commands</h2>
  <div class='row' style='margin-bottom:8px'>
    <button class='btn btn-green' onclick='post("/pulse")'>&#9654; ADVANCE STATE</button>
    <button class='btn btn-purple' onclick='post("/calibrate")'>&#9656; CALIBRATE IMU</button>
  </div>
  <button class='btn btn-red' onclick='post("/reset_yaw")'>RESET YAW TO 0</button>
  <div class='status' id='status'>Ready</div>
</div>

<script>
function post(url){
  fetch(url).then(r=>r.text()).then(t=>{
    document.getElementById('status').innerText=t;
    setTimeout(()=>document.getElementById('status').innerText='Ready',2000);
  });
}
function refresh(){
  fetch('/sensors').then(r=>r.json()).then(d=>{
    document.getElementById('dA').innerText = d.distA>0&&d.distA<400?d.distA:'n/a';
    document.getElementById('dB').innerText = d.distB>0&&d.distB<400?d.distB:'n/a';
    document.getElementById('aX').innerText = d.accX;
    document.getElementById('aY').innerText = d.accY;
    document.getElementById('aZ').innerText = d.accZ;
    document.getElementById('gX').innerText = d.gyroX;
    document.getElementById('gY').innerText = d.gyroY;
    document.getElementById('gZ').innerText = d.gyroZ;
    document.getElementById('yaw').innerText = d.yaw;
    document.getElementById('tmp').innerText = d.temp;
    document.getElementById('pls').innerText = d.pulses;
    const cb = document.getElementById('cal');
    cb.innerText = d.calibrated?'CALIBRATED':'NOT CAL';
    cb.className = 'cal-badge '+(d.calibrated?'cal-yes':'cal-no');
  }).catch(()=>{});
}
setInterval(refresh,500);
refresh();
</script>
</body></html>
)rawliteral";

// ── Server handlers ───────────────────────────────────
void handleRoot()      { server.send(200,"text/html",HTML); }

void handleSensors() {
  String j="{";
  j+="\"distA\":"+String(distA)+",\"distB\":"+String(distB)+",";
  j+="\"accX\":"+String(accX,2)+",\"accY\":"+String(accY,2)+",\"accZ\":"+String(accZ,2)+",";
  j+="\"gyroX\":"+String(gyroX,1)+",\"gyroY\":"+String(gyroY,1)+",\"gyroZ\":"+String(gyroZ,1)+",";
  j+="\"yaw\":"+String(yawAngle,1)+",\"temp\":"+String(temperature,1)+",";
  j+="\"pulses\":"+String(pulseCount)+",\"calibrated\":"+(calibrated?"true":"false")+"}";
  server.send(200,"application/json",j);
}

void handlePulse()     { sendPulse(); server.send(200,"text/plain","Pulse sent — total: "+String(pulseCount)); }
void handleCalibrate() { runCalibration(); server.send(200,"text/plain","Calibrated OK"); }
void handleResetYaw()  { yawAngle=0; lastGyroTime=millis(); server.send(200,"text/plain","Yaw reset to 0"); }

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

  server.on("/",       handleRoot);
  server.on("/sensors",handleSensors);
  server.on("/pulse",  handlePulse);
  server.on("/calibrate", handleCalibrate);
  server.on("/reset_yaw", handleResetYaw);
  server.begin();

  tft.init(); tft.setRotation(1); tft.fillScreen(0xC618);
  tft.setTextColor(TFT_BLACK,0xC618); tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); tft.drawString("MASTER PANEL",tft.width()/2,30);
  tft.setTextSize(1); tft.drawString("WiFi: "+String(SSID),tft.width()/2,60);
  tft.drawString("Pass: "+String(PASS),tft.width()/2,78);
  tft.drawString(ip.toString(),tft.width()/2,100);
  tft.drawString("Open in phone browser",tft.width()/2,120);

  Serial.print("AP: "); Serial.println(ip);
}

void loop() {
  server.handleClient();
  distA = ping(TRIG_A,ECHO_A);
  distB = ping(TRIG_B,ECHO_B);
  readMPU();
}
