// ===================================================
// MASTER CONTROL PANEL v2
// Full debug + tuning panel for master T-Display S3.
// WiFi AP "Robot-Master" / "robot1234" → 192.168.4.1
//
// Features:
//   • Live sensors: dual ultrasonic, MPU-6050, yaw, temp
//   • Calibrate IMU / reset yaw
//   • Slave motor + actuator control (pulse bursts → GPIO17)
//   • MOTION servo control (pulse bursts → GPIO1)
//   • Navigate-home: set a bearing with MPU, PID drives back
//   • Record log with timestamps → download as .txt
//
// SLAVE  pulse protocol (GPIO 17 → slave GPIO 18):
//   Long ≥60ms  = state advance  |  Short burst = direct cmd
//   2=FWD 3=BACK 4=TURN_L 5=TURN_R 6=STOP 7=ACT_UP 8=ACT_DN 9=ACT_STOP
//
// MOTION pulse protocol (GPIO 1 → MOTION GP26):
//   Long ≥60ms  = state advance  |  Short burst = servo cmd
//   2=EXT 3=EXT_STOP 4=RETRACT 5=FLIP_RAISE 6=FLIP_STOP 7=FLIP_DUMP 8=NEUTRAL
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>

// ── WiFi ──────────────────────────────────────────────
const char* SSID = "Robot-Master";
const char* PASS = "robot1234";
WebServer server(80);

// ── Pins ──────────────────────────────────────────────
#define TRIG_A        2
#define ECHO_A        3
#define TRIG_B       43
#define ECHO_B       44
#define MPU_SDA      16
#define MPU_SCL      21
#define MPU_ADDR   0x68
#define BUTTON_PIN   10
#define STATE_OUT     1   // → MOTION GP26
// ESP-NOW slave MAC — from Context/mac_addresses.h and platformio.ini
#ifndef SLAVE_MAC
  #define SLAVE_MAC {0x30, 0x30, 0xF9, 0x59, 0x31, 0x78}
#endif
uint8_t slaveMac[] = SLAVE_MAC;

#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618
TFT_eSPI tft = TFT_eSPI();

// ── Sensors ───────────────────────────────────────────
float accX,accY,accZ,gyroX,gyroY,gyroZ,temperature;
float accOffX=0,accOffY=0,accOffZ=0;
float gyroOffX=0,gyroOffY=0,gyroOffZ=0;
bool  calibrated = false;
float yawAngle = 0;
unsigned long lastGyroTime = 0;
long  distA = 0, distB = 0;
int   pulseCount = 0;

// ── Recording ─────────────────────────────────────────
bool          recording = false;
unsigned long recStart  = 0;
String        logBuf    = "";

void logEvent(const String& msg) {
  if (!recording) return;
  unsigned long t = millis() - recStart;
  char ts[14]; sprintf(ts, "%lu.%02lus", t/1000, (t%1000)/10);
  logBuf += String(ts) + "  " + msg + "\n";
}

// ── Replay ────────────────────────────────────────────
struct ReplayCmd { unsigned long timeMs; int type; int value; };
// type 0=slave UART  1=motion burst  2=state pulse
const int MAX_REPLAY = 200;
ReplayCmd replayQueue[MAX_REPLAY];
int  replayCount = 0, replayIdx = 0;
bool replaying   = false;
unsigned long replayStart = 0;

void parseLog() {
  replayCount = 0;
  int pos = 0;
  while (pos < (int)logBuf.length() && replayCount < MAX_REPLAY) {
    int nl = logBuf.indexOf('\n', pos);
    if (nl < 0) nl = logBuf.length();
    String line = logBuf.substring(pos, nl); line.trim(); pos = nl + 1;
    if (line.length() == 0 || line.startsWith("+") || line.startsWith("-")) continue;

    // Parse timestamp e.g. "1.25s  ..."
    int si = line.indexOf('s');
    if (si < 0 || si > 10) continue;
    unsigned long tms = (unsigned long)(line.substring(0, si).toFloat() * 1000.0f);
    String rest = line.substring(si + 1); rest.trim();

    int val = 0;
    int type = -1;

    if (rest.startsWith("SLAVE")) {
      int p = rest.indexOf("("), n = rest.indexOf("p)");
      if (p >= 0 && n > p) { val = rest.substring(p+1, n).toInt(); type = 0; }
    } else if (rest.startsWith("NAV")) {
      if      (rest.indexOf("TURN_L") >= 0) val = 4;
      else if (rest.indexOf("TURN_R") >= 0) val = 5;
      else if (rest.indexOf("FWD")    >= 0) val = 2;
      else if (rest.indexOf("BACK")   >= 0) val = 3;
      else if (rest.indexOf("STOP")   >= 0) val = 6;
      if (val > 0) type = 0;
    } else if (rest.startsWith("MOTION")) {
      int p = rest.indexOf("("), n = rest.indexOf("p)");
      if (p >= 0 && n > p) { val = rest.substring(p+1, n).toInt(); type = 1; }
    } else if (rest.startsWith("STATE_ADVANCE")) {
      type = 2; val = 0;
    }

    if (type >= 0) replayQueue[replayCount++] = {tms, type, val};
  }
}

// ── Navigate-home PID ─────────────────────────────────
float homeYaw     = 0;
bool  homeSet     = false;
bool  navigating  = false;
float pidKp = 2.5f, pidKi = 0.05f, pidKd = 0.8f;
float pidIntegral = 0, pidPrev = 0;
unsigned long lastNavCmd = 0;

float normalizeAngle(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
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
  Wire.requestFrom(MPU_ADDR, 14, 1);
  int16_t rAX=Wire.read()<<8|Wire.read(), rAY=Wire.read()<<8|Wire.read(), rAZ=Wire.read()<<8|Wire.read();
  int16_t rT =Wire.read()<<8|Wire.read();
  int16_t rGX=Wire.read()<<8|Wire.read(), rGY=Wire.read()<<8|Wire.read(), rGZ=Wire.read()<<8|Wire.read();
  accX=rAX/16384.0-accOffX; accY=rAY/16384.0-accOffY; accZ=rAZ/16384.0-accOffZ;
  gyroX=rGX/131.0-gyroOffX; gyroY=rGY/131.0-gyroOffY; gyroZ=rGZ/131.0-gyroOffZ;
  temperature=(rT/340.0)+36.53;
  if(calibrated&&lastGyroTime>0){ float dt=(millis()-lastGyroTime)/1000.0; yawAngle+=gyroZ*dt; }
  lastGyroTime=millis();
}

void runCalibration() {
  const int S=200; float sAX=0,sAY=0,sAZ=0,sGX=0,sGY=0,sGZ=0;
  for(int i=0;i<S;i++){
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, 1);
    int16_t rAX=Wire.read()<<8|Wire.read(),rAY=Wire.read()<<8|Wire.read(),rAZ=Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read();
    int16_t rGX=Wire.read()<<8|Wire.read(),rGY=Wire.read()<<8|Wire.read(),rGZ=Wire.read()<<8|Wire.read();
    sAX+=rAX/16384.0; sAY+=rAY/16384.0; sAZ+=rAZ/16384.0;
    sGX+=rGX/131.0; sGY+=rGY/131.0; sGZ+=rGZ/131.0; delay(10);
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

// ── ESP-NOW helpers ──────────────────────────────────
void sendToSlave(uint8_t cmd, int speed = -1) {
  if (speed >= 0) {
    uint8_t data[2] = {cmd, (uint8_t)(constrain(speed,0,1023)/4)};
    esp_now_send(slaveMac, data, 2);
  } else {
    esp_now_send(slaveMac, &cmd, 1);
  }
  Serial.print("ESP-NOW slave cmd:"); Serial.print(cmd);
  if(speed>=0){Serial.print(" spd:"); Serial.print(speed);} Serial.println();
}

void sendStatePulse(){
  // MOTION: GPIO pulse | Slave: ESP-NOW state advance (0x00)
  digitalWrite(STATE_OUT,HIGH); delay(100); digitalWrite(STATE_OUT,LOW);
  sendToSlave(0x00);
  pulseCount++;
  logEvent("STATE_ADVANCE pulse#"+String(pulseCount));
}

void sendSlaveCmd(const String& s, int speed = -1){
  uint8_t cmd = (uint8_t)s.toInt();
  if(cmd >= 2 && cmd <= 9) sendToSlave(cmd, speed);
}

// Track current flipper angle on master — keep in sync with MOTION board
#define FLIP_R_RAISE_DEG  45
#define FLIP_R_DUMP_DEG  135
int motionFlipDeg = 90;

// Short burst on STATE_OUT — MOTION servo commands
// 15ms pulses: fast enough for burst 57 (180° abs) in ~1.7s
void sendMotionBurst(int n){
  for(int i=0;i<n;i++){
    digitalWrite(STATE_OUT,HIGH); delay(15);
    digitalWrite(STATE_OUT,LOW);  delay(15);
  }
}

// Send absolute flipper angle as a single burst (no intermediate hops)
void sendFlipAbsolute(int angle){
  motionFlipDeg = ((constrain(angle, 0, 180) + 2) / 5) * 5;  // round to nearest 5°
  int burst = 21 + (motionFlipDeg / 5);  // 21–57
  sendMotionBurst(burst);
  Serial.print("FLIP ABS burst:"); Serial.print(burst);
  Serial.print(" -> "); Serial.println(motionFlipDeg);
}

// ── Navigate-home update (called every loop) ─────────
void updateNavigation() {
  if (!navigating || !homeSet) return;
  if (millis() - lastNavCmd < 250) return;
  lastNavCmd = millis();

  float error = normalizeAngle(homeYaw - yawAngle);
  pidIntegral += error * 0.25f;
  pidIntegral = constrain(pidIntegral, -50, 50);
  float derivative = (error - pidPrev) / 0.25f;
  float output = pidKp*error + pidKi*pidIntegral + pidKd*derivative;
  pidPrev = error;

  bool inFront = abs(error) < 90.0f;

  if (abs(error) > 8.0f) {
    // Heading correction needed
    if (output > 0) { sendSlaveCmd("4"); logEvent("NAV TURN_L err="+String(error,1)); }
    else            { sendSlaveCmd("5"); logEvent("NAV TURN_R err="+String(error,1)); }
  } else {
    // Heading OK — drive toward home
    if (inFront) { sendSlaveCmd("2"); logEvent("NAV FWD yaw="+String(yawAngle,1)); }
    else         { sendSlaveCmd("3"); logEvent("NAV BACK yaw="+String(yawAngle,1)); }
  }
}

// ── Web page ─────────────────────────────────────────
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Master v2</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:10px}
  h1{color:#00d4ff;text-align:center;margin-bottom:10px;font-size:1.3em}
  .card{background:#16213e;border-radius:12px;padding:12px;margin-bottom:8px}
  .card h2{color:#00d4ff;font-size:.85em;text-transform:uppercase;margin-bottom:8px;letter-spacing:1px}
  .g2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
  .g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
  .val{background:#0a0a1a;border-radius:8px;padding:8px;text-align:center}
  .val .lbl{font-size:.65em;color:#888;margin-bottom:2px}
  .val .num{font-size:1.1em;color:#fdcb6e;font-weight:bold}
  .row{display:flex;gap:6px;margin-bottom:6px}
  .btn{border:none;border-radius:8px;padding:13px 8px;cursor:pointer;font-size:.85em;font-weight:bold;flex:1}
  .g{background:#00b894;color:#fff} .b{background:#0984e3;color:#fff}
  .o{background:#e17055;color:#fff} .p{background:#6c5ce7;color:#fff}
  .gr{background:#636e72;color:#fff} .red{background:#d63031;color:#fff}
  .teal{background:#00cec9;color:#fff} .full{width:100%;padding:14px}
  .rec{background:#636e72;color:#fff} .rec.on{background:#d63031}
  .cal{display:inline-block;padding:2px 7px;border-radius:20px;font-size:.7em;font-weight:bold}
  .cy{background:#00b894;color:#fff} .cn{background:#d63031;color:#fff}
  .nav-status{text-align:center;font-size:.85em;color:#fdcb6e;margin-top:4px}
  .status{background:#0a0a1a;border-radius:8px;padding:7px;text-align:center;font-size:.8em;color:#00d4ff;margin-top:6px}
  #log{background:#050510;border-radius:8px;padding:8px;font-family:monospace;font-size:.72em;height:160px;overflow-y:auto;color:#00ff88;white-space:pre;margin-top:8px}
</style></head><body>
<h1>&#129302; Master Panel v2</h1>

<!-- SENSORS -->
<div class='card'>
  <h2>Sensors &nbsp;<span id='cal' class='cal cn'>NOT CAL</span></h2>
  <div class='g2' style='margin-bottom:6px'>
    <div class='val'><div class='lbl'>Ultrasonic A (cm)</div><div class='num' id='dA'>--</div></div>
    <div class='val'><div class='lbl'>Ultrasonic B (cm)</div><div class='num' id='dB'>--</div></div>
  </div>
  <div class='g3'>
    <div class='val'><div class='lbl'>Acc X</div><div class='num' id='aX'>--</div></div>
    <div class='val'><div class='lbl'>Acc Y</div><div class='num' id='aY'>--</div></div>
    <div class='val'><div class='lbl'>Acc Z</div><div class='num' id='aZ'>--</div></div>
  </div>
  <div class='g3' style='margin-top:6px'>
    <div class='val'><div class='lbl'>Gyro X</div><div class='num' id='gX'>--</div></div>
    <div class='val'><div class='lbl'>Gyro Y</div><div class='num' id='gY'>--</div></div>
    <div class='val'><div class='lbl'>Gyro Z</div><div class='num' id='gZ'>--</div></div>
  </div>
  <div class='g3' style='margin-top:6px'>
    <div class='val'><div class='lbl'>Yaw (&deg;)</div><div class='num' id='yaw'>--</div></div>
    <div class='val'><div class='lbl'>Temp (&deg;C)</div><div class='num' id='tmp'>--</div></div>
    <div class='val'><div class='lbl'>Pulses</div><div class='num' id='pls'>--</div></div>
  </div>
  <div class='row' style='margin-top:8px'>
    <button class='btn p' onclick='post("/calibrate")'>CALIBRATE</button>
    <button class='btn b' onclick='post("/reset_yaw")'>RESET YAW</button>
    <button class='btn gr' onclick='post("/state_pulse")'>ADV STATE</button>
  </div>
</div>

<!-- SLAVE DRIVE -->
<div class='card'>
  <h2>Slave — Drive</h2>
  <label style='font-size:.8em;color:#aaa;margin-bottom:3px;display:block'>Speed: <span id='spd_lbl'>700</span> / 1023</label>
  <input type='range' min='0' max='1023' value='700' id='spd_sl' style='width:100%;height:28px;accent-color:#00d4ff;margin-bottom:8px'
    oninput="document.getElementById('spd_lbl').innerText=this.value">
  <div class='row'>
    <button class='btn g'  onclick='sl(2)'>&#9650; FWD</button>
    <button class='btn o'  onclick='sl(3)'>&#9660; BACK</button>
    <button class='btn gr' onclick='sl(6)'>&#9632; STOP</button>
  </div>
  <div class='row'>
    <button class='btn b' onclick='sl(4)'>&#9668; TURN L</button>
    <button class='btn b' onclick='sl(5)'>TURN R &#9658;</button>
  </div>
  <div class='row'>
    <button class='btn g' onclick='driveTimed(2,1000)'>FWD 1s</button>
    <button class='btn g' onclick='driveTimed(2,2000)'>FWD 2s</button>
    <button class='btn g' onclick='driveTimed(2,3000)'>FWD 3s</button>
  </div>
  <div class='row'>
    <button class='btn o' onclick='driveTimed(3,1000)'>BCK 1s</button>
    <button class='btn o' onclick='driveTimed(3,2000)'>BCK 2s</button>
    <button class='btn o' onclick='driveTimed(3,3000)'>BCK 3s</button>
  </div>
  <div class='row'>
    <button class='btn b' onclick='driveTimed(4,1000)'>TL 1s</button>
    <button class='btn b' onclick='driveTimed(4,2000)'>TL 2s</button>
    <button class='btn b' onclick='driveTimed(4,3000)'>TL 3s</button>
  </div>
</div>

<!-- SLAVE ACTUATOR -->
<div class='card'>
  <h2>Slave — Actuator</h2>
  <div class='row'>
    <button class='btn p' onclick='sl(7)'>&#9650; UP</button>
    <button class='btn p' onclick='sl(8)'>&#9660; DOWN</button>
    <button class='btn gr' onclick='sl(9)'>&#9632; STOP</button>
  </div>
  <div class='row'>
    <button class='btn p' onclick='actTimed("up",1000)'>UP 1s</button>
    <button class='btn p' onclick='actTimed("up",3000)'>UP 3s</button>
    <button class='btn p' onclick='actTimed("up",5000)'>UP 5s</button>
  </div>
  <div class='row'>
    <button class='btn o' onclick='actTimed("dn",1000)'>DN 1s</button>
    <button class='btn o' onclick='actTimed("dn",3000)'>DN 3s</button>
    <button class='btn o' onclick='actTimed("dn",5000)'>DN 5s</button>
  </div>
</div>

<!-- MOTION SERVOS -->
<div class='card'>
  <h2>Motion — Servos</h2>
  <div class='row'>
    <button class='btn p'  onclick='mo(2)'>EXTEND</button>
    <button class='btn gr' onclick='mo(3)'>EXT STOP</button>
    <button class='btn o'  onclick='mo(4)'>RETRACT</button>
  </div>
  <div class='row'>
    <button class='btn b' onclick='extTimed(1000)'>EXT 1s</button>
    <button class='btn b' onclick='extTimed(3000)'>EXT 3s</button>
    <button class='btn b' onclick='extTimed(5000)'>EXT 5s</button>
  </div>
  <div class='row'>
    <button class='btn o' onclick='retractTimed(1000)'>RETR 1s</button>
    <button class='btn o' onclick='retractTimed(2000)'>RETR 2s</button>
  </div>
  <div class='row'>
    <button class='btn p'  onclick='mo(5)'>RAISE</button>
    <button class='btn gr' onclick='mo(6)'>FLIP MID</button>
    <button class='btn o'  onclick='mo(7)'>DUMP</button>
  </div>
  <label style='font-size:.8em;color:#aaa;margin:6px 0 3px;display:block'>Flip increment: <span id='flip_inc_lbl'>5</span>°</label>
  <input type='range' min='1' max='30' value='5' id='flip_inc' style='width:100%;height:28px;accent-color:#6c5ce7;margin-bottom:6px'
    oninput="document.getElementById('flip_inc_lbl').innerText=this.value">
  <div class='row'>
    <button class='btn p' onclick='mo(9)'>FLIP +°</button>
    <button class='btn o' onclick='mo(10)'>FLIP -°</button>
  </div>
  <div class='g2' style='margin-top:6px'>
    <div style='background:#0a0a1a;border-radius:8px;padding:8px;text-align:center'>
      <div style='font-size:.65em;color:#888'>GP1 (Right)</div>
      <div style='font-size:1.1em;color:#a29bfe;font-weight:bold' id='pos_gp1'>90°</div>
    </div>
    <div style='background:#0a0a1a;border-radius:8px;padding:8px;text-align:center'>
      <div style='font-size:.65em;color:#888'>GP3 (Left)</div>
      <div style='font-size:1.1em;color:#a29bfe;font-weight:bold' id='pos_gp3'>90°</div>
    </div>
  </div>
  <label style='font-size:.8em;color:#aaa;margin:8px 0 3px;display:block'>Go to position: <span id='flip_pos_lbl'>90</span>°</label>
  <input type='range' min='0' max='180' value='90' id='flip_pos_sl' style='width:100%;height:28px;accent-color:#00cec9;margin-bottom:6px'
    oninput="document.getElementById('flip_pos_lbl').innerText=this.value">
  <button class='btn teal full' onclick='gotoFlipPos()' style='margin-bottom:6px'>&#8594; GO TO POSITION</button>
  <button class='btn teal full' id='range_btn' onclick='startRangeCheck()' style='margin-top:0'>&#9654; RANGE CHECK GP5</button>
  <div style='margin-top:8px;background:#0a0a1a;border-radius:8px;padding:8px'>
    <div style='display:flex;justify-content:space-between;font-size:.7em;color:#888;margin-bottom:4px'>
      <span>GP5 Sweep</span><span id='range_deg' style='color:#00cec9'>-- °</span>
    </div>
    <canvas id='range_canvas' width='280' height='100' style='width:100%;background:#050510;border-radius:6px;display:block'></canvas>
    <div id='range_status' style='text-align:center;font-size:.75em;color:#636e72;margin-top:4px'>Press RANGE CHECK to sweep</div>
  </div>
  <button class='btn red full' onclick='mo(8)' style='margin-top:4px'>ALL NEUTRAL</button>
</div>

<!-- NAVIGATE HOME -->
<div class='card'>
  <h2>Navigate Home (PID heading)</h2>
  <div class='row'>
    <button class='btn teal' onclick='postHome()'>&#127968; SET HOME</button>
    <button class='btn g'    onclick='post("/nav_start")' id='nav_btn'>&#9654; GO HOME</button>
    <button class='btn red'  onclick='post("/nav_stop")'>&#9632; STOP</button>
  </div>
  <div class='nav-status' id='nav_status'>Home not set</div>
  <div class='g3' style='margin-top:8px'>
    <div class='val'><div class='lbl'>Home yaw</div><div class='num' id='home_yaw'>--</div></div>
    <div class='val'><div class='lbl'>Error (&deg;)</div><div class='num' id='nav_err'>--</div></div>
    <div class='val'><div class='lbl'>PID out</div><div class='num' id='pid_out'>--</div></div>
  </div>
</div>

<!-- ROUTE MAP -->
<div class='card'>
  <h2>Route Map <span style='font-size:.7em;color:#636e72'>(dead reckoning)</span></h2>
  <canvas id='map' width='300' height='220' style='width:100%;background:#050510;border-radius:8px;border:1px solid #1a1a3a;display:block'></canvas>
  <div class='row' style='margin-top:8px'>
    <button class='btn gr' onclick='clearMap()'>&#10005; CLEAR</button>
    <button class='btn b'  onclick='centerMap()'>&#8857; CENTER</button>
  </div>
</div>

<!-- REPLAY -->
<div class='card'>
  <h2>Replay Recorded Sequence</h2>
  <div class='row'>
    <button class='btn g'   onclick='startReplay()'>&#9654; REPLAY</button>
    <button class='btn red' onclick='post("/stopreplay")'>&#9632; STOP</button>
  </div>
  <div class='status' id='rep_status'>No replay loaded</div>
</div>

<!-- RECORD LOG -->
<div class='card'>
  <h2>Command Log</h2>
  <div class='row'>
    <button class='btn rec' id='rec_btn' onclick='toggleRec()'>&#9210; RECORD</button>
    <button class='btn b'   onclick='downloadLog()'>&#11015; DOWNLOAD</button>
    <button class='btn gr'  onclick='clearLog()'>&#10005; CLEAR</button>
  </div>
  <div id='log'>-- no log --</div>
</div>

<div class='status' id='status'>Ready</div>

<script>
// ── State ──────────────────────────────────────────
let recActive=false, logPoll=null;
let moveDir=0;        // 1=fwd -1=back 0=stopped
let currentYaw=0;

// ── Map ────────────────────────────────────────────
const CVW=300, CVH=220;
let trail=[], robotX=CVW/2, robotY=CVH/2;
let originX=CVW/2, originY=CVH/2;
let homeMapX=null, homeMapY=null;
const STEP=3;   // pixels per 500ms tick when moving

function updateMapPos(){
  if(moveDir!==0){
    const r=currentYaw*Math.PI/180;
    robotX+=Math.sin(r)*moveDir*STEP;
    robotY-=Math.cos(r)*moveDir*STEP;
  }
  trail.push({x:robotX,y:robotY});
  if(trail.length>600) trail.shift();
  drawMap();
}

function drawMap(){
  const canvas=document.getElementById('map');
  if(!canvas) return;
  const ctx=canvas.getContext('2d');
  // Scale canvas pixels to CSS display size
  const scaleX=canvas.clientWidth/CVW, scaleY=canvas.clientHeight/CVH||1;
  canvas.width=CVW; canvas.height=CVH;

  // Background
  ctx.fillStyle='#050510'; ctx.fillRect(0,0,CVW,CVH);

  // Grid
  ctx.strokeStyle='#111128'; ctx.lineWidth=1;
  for(let x=0;x<CVW;x+=20){ ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,CVH); ctx.stroke(); }
  for(let y=0;y<CVH;y+=20){ ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(CVW,y); ctx.stroke(); }

  // Origin cross
  ctx.strokeStyle='#2a2a5a'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(originX-8,originY); ctx.lineTo(originX+8,originY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(originX,originY-8); ctx.lineTo(originX,originY+8); ctx.stroke();

  // Trail
  if(trail.length>1){
    ctx.strokeStyle='#00b894'; ctx.lineWidth=2; ctx.beginPath();
    ctx.moveTo(trail[0].x,trail[0].y);
    for(let i=1;i<trail.length;i++) ctx.lineTo(trail[i].x,trail[i].y);
    ctx.stroke();
  }

  // Home marker (cyan triangle)
  if(homeMapX!==null){
    ctx.fillStyle='#00d4ff';
    ctx.beginPath(); ctx.moveTo(homeMapX,homeMapY-7); ctx.lineTo(homeMapX+5,homeMapY+4); ctx.lineTo(homeMapX-5,homeMapY+4); ctx.closePath(); ctx.fill();
    ctx.fillStyle='#00d4ff'; ctx.font='9px Arial'; ctx.textAlign='center';
    ctx.fillText('HOME',homeMapX,homeMapY+14);
  }

  // Robot arrow
  const yr=currentYaw*Math.PI/180;
  ctx.save(); ctx.translate(robotX,robotY); ctx.rotate(yr);
  ctx.fillStyle='#fdcb6e';
  ctx.beginPath(); ctx.moveTo(0,-10); ctx.lineTo(6,7); ctx.lineTo(0,3); ctx.lineTo(-6,7); ctx.closePath(); ctx.fill();
  // Direction dot
  ctx.fillStyle='#fff'; ctx.beginPath(); ctx.arc(0,-10,2,0,Math.PI*2); ctx.fill();
  ctx.restore();

  // Yaw label
  ctx.fillStyle='#636e72'; ctx.font='10px Arial'; ctx.textAlign='left';
  ctx.fillText('Yaw: '+currentYaw.toFixed(1)+'°', 5, CVH-5);
}

function clearMap(){
  trail=[]; robotX=originX=CVW/2; robotY=originY=CVH/2;
  homeMapX=null; homeMapY=null; moveDir=0; drawMap();
}
function centerMap(){
  const dx=CVW/2-robotX, dy=CVH/2-robotY;
  trail=trail.map(p=>({x:p.x+dx,y:p.y+dy}));
  if(homeMapX!==null){homeMapX+=dx;homeMapY+=dy;}
  robotX=CVW/2; robotY=CVH/2; originX+=dx; originY+=dy; drawMap();
}

// ── Commands ──────────────────────────────────────
function post(url){ fetch(url).then(r=>r.text()).then(t=>stat(t)).catch(()=>{}); }

function sl(n){
  if(n===2) moveDir=1;
  else if(n===3) moveDir=-1;
  else if(n===6||n===7||n===8||n===9) moveDir=0;
  const spd = document.getElementById('spd_sl') ? document.getElementById('spd_sl').value : 700;
  fetch('/slave?n='+n+'&s='+spd).then(r=>r.text()).then(t=>stat(t));
}
let flipPos = 90;  // track flipper angle locally for display
function mo(n){
  const inc = document.getElementById('flip_inc') ? parseInt(document.getElementById('flip_inc').value) : 5;
  fetch('/motion?n='+n+'&inc='+inc).then(r=>r.text()).then(t=>{
    stat(t);
    // Update local position display
    if(n===9){ flipPos=Math.min(180,flipPos+inc); }
    else if(n===10){ flipPos=Math.max(0,flipPos-inc); }
    else if(n===5){ flipPos=45; }
    else if(n===6||n===8){ flipPos=90; }
    else if(n===7){ flipPos=135; }
    document.getElementById('pos_gp1').innerText=flipPos+'°';
    document.getElementById('pos_gp3').innerText=(180-flipPos)+'°';
  });
}
function stat(t){ document.getElementById('status').innerText=t; }

// Override post for set_home to mark on map
const _origPost=post;
function postHome(){
  homeMapX=robotX; homeMapY=robotY;
  fetch('/set_home').then(r=>r.text()).then(t=>{stat(t);drawMap();});
}

// ── Sensor refresh ────────────────────────────────
function refresh(){
  fetch('/sensors').then(r=>r.json()).then(d=>{
    document.getElementById('dA').innerText=d.dA>0&&d.dA<400?d.dA+'cm':'n/a';
    document.getElementById('dB').innerText=d.dB>0&&d.dB<400?d.dB+'cm':'n/a';
    ['aX','aY','aZ','gX','gY','gZ','yaw','tmp','pls'].forEach(k=>document.getElementById(k).innerText=d[k]);
    const c=document.getElementById('cal');
    c.innerText=d.cal?'CALIBRATED':'NOT CAL'; c.className='cal '+(d.cal?'cy':'cn');
    document.getElementById('home_yaw').innerText=d.homeYaw;
    document.getElementById('nav_err').innerText=d.navErr;
    document.getElementById('pid_out').innerText=d.pidOut;
    document.getElementById('nav_status').innerText=d.navState;
    currentYaw=parseFloat(d.yaw)||0;
    updateMapPos();
  }).catch(()=>{});
}
setInterval(refresh,500); refresh();
drawServoArm(90);

// ── Record ────────────────────────────────────────
function toggleRec(){
  fetch('/record').then(r=>r.text()).then(t=>{
    recActive=(t==='started');
    const b=document.getElementById('rec_btn');
    b.className='btn rec'+(recActive?' on':'');
    b.innerHTML=recActive?'&#9209; STOP REC':'&#9210; RECORD';
    if(recActive) logPoll=setInterval(fetchLog,1000);
    else { clearInterval(logPoll); fetchLog(); }
  });
}
function fetchLog(){
  fetch('/log').then(r=>r.text()).then(t=>{
    const el=document.getElementById('log'); el.innerText=t||'-- empty --'; el.scrollTop=el.scrollHeight;
  });
}
function downloadLog(){
  const a=document.createElement('a'); a.href='/download'; a.download='robot_log.txt'; a.click();
}
function startReplay(){
  fetch('/replay').then(r=>r.text()).then(t=>{
    document.getElementById('rep_status').innerText=t;
  });
}
// ── Range check animation ─────────────────────────
function drawServoArm(deg){
  const canvas=document.getElementById('range_canvas');
  if(!canvas) return;
  const ctx=canvas.getContext('2d');
  const W=canvas.width, H=canvas.height;
  const cx=W/2, cy=H-10, r=70;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#050510'; ctx.fillRect(0,0,W,H);
  // Arc background (30° to 150°)
  ctx.strokeStyle='#1a1a3a'; ctx.lineWidth=12; ctx.lineCap='round';
  ctx.beginPath();
  ctx.arc(cx,cy,r, Math.PI*(1-150/180), Math.PI*(1-30/180));
  ctx.stroke();
  // Swept arc (mid to current)
  ctx.strokeStyle='#00cec9'; ctx.lineWidth=12;
  const startRad = Math.PI*(1-90/180);
  const endRad   = Math.PI*(1-deg/180);
  ctx.beginPath();
  if(deg>=90) ctx.arc(cx,cy,r, startRad, endRad);
  else        ctx.arc(cx,cy,r, endRad, startRad);
  ctx.stroke();
  // Min/max markers
  ctx.strokeStyle='#2a2a5a'; ctx.lineWidth=2;
  [30,90,150].forEach(a=>{
    const rad=Math.PI*(1-a/180);
    ctx.beginPath();
    ctx.moveTo(cx+(r-8)*Math.cos(rad), cy+(r-8)*Math.sin(-rad));
    ctx.lineTo(cx+(r+8)*Math.cos(rad), cy+(r+8)*Math.sin(-rad));
    ctx.stroke();
    ctx.fillStyle='#444'; ctx.font='9px Arial'; ctx.textAlign='center';
    ctx.fillText(a+'°', cx+(r+20)*Math.cos(rad), cy+(r+20)*Math.sin(-rad)+4);
  });
  // Arm
  const armRad=Math.PI*(1-deg/180);
  ctx.strokeStyle='#fdcb6e'; ctx.lineWidth=3; ctx.lineCap='round';
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx+r*Math.cos(armRad), cy+r*Math.sin(-armRad));
  ctx.stroke();
  ctx.fillStyle='#fdcb6e';
  ctx.beginPath(); ctx.arc(cx,cy,5,0,Math.PI*2); ctx.fill();
  document.getElementById('range_deg').innerText=Math.round(deg)+'°';
}

function driveTimed(dir,ms){
  const spd = document.getElementById('spd_sl') ? document.getElementById('spd_sl').value : 700;
  const names={2:'FWD',3:'BACK',4:'TURN_L'};
  stat((names[dir]||'DRIVE')+' '+ms/1000+'s...');
  fetch('/drive_timed?dir='+dir+'&ms='+ms+'&s='+spd).then(r=>r.text()).then(t=>stat(t));
}
function extTimed(ms){
  stat('Extending '+ms/1000+'s...');
  fetch('/extend_timed?ms='+ms).then(r=>r.text()).then(t=>stat(t));
}
function retractTimed(ms){
  stat('Retracting '+ms/1000+'s...');
  fetch('/retract_timed?ms='+ms).then(r=>r.text()).then(t=>stat(t));
}
function actTimed(dir,ms){
  stat((dir==='up'?'Actuator UP ':'Actuator DN ')+ms/1000+'s...');
  fetch('/actuator_timed?dir='+dir+'&ms='+ms).then(r=>r.text()).then(t=>stat(t));
}
function gotoFlipPos(){
  const angle = Math.round(parseInt(document.getElementById('flip_pos_sl').value)/5)*5;
  fetch('/motion_abs?angle='+angle).then(r=>r.text()).then(t=>{
    stat(t);
    flipPos=angle;
    document.getElementById('pos_gp1').innerText=angle+'°';
    document.getElementById('pos_gp3').innerText=(180-angle)+'°';
  });
}
function startRangeCheck(){
  const btn=document.getElementById('range_btn');
  btn.disabled=true; btn.style.opacity='0.5';
  document.getElementById('range_status').style.color='#00cec9';
  const inc = document.getElementById('flip_inc') ? parseInt(document.getElementById('flip_inc').value) : 5;
  document.getElementById('range_status').innerText='Sweeping ('+inc+'° steps)...';
  mo(11);  // send command to MOTION board
  // Animate using slider increment — time per step scales with increment size
  const msPerStep = Math.max(150, inc * 30);  // larger steps = more time visible
  const RMIN=30, RMAX=150, MID=90;
  const steps=[];
  steps.push({t:0, deg:MID});
  let t=500;
  for(let d=MID+inc;d<=RMAX;d+=inc){ steps.push({t,deg:Math.min(d,RMAX)}); t+=msPerStep; }
  for(let d=RMAX-inc;d>=MID;d-=inc){ steps.push({t,deg:Math.max(d,MID)});  t+=msPerStep; }
  t+=300;
  for(let d=MID-inc;d>=RMIN;d-=inc){ steps.push({t,deg:Math.max(d,RMIN)}); t+=msPerStep; }
  for(let d=RMIN+inc;d<=MID;d+=inc){ steps.push({t,deg:Math.min(d,MID)});  t+=msPerStep; }
  steps.push({t:t+300,deg:MID});
  const totalMs=t+600;
  const start=performance.now();
  let si=0;
  function animate(now){
    const elapsed=now-start;
    while(si<steps.length-1 && steps[si+1].t<=elapsed) si++;
    // interpolate
    if(si<steps.length-1){
      const t0=steps[si].t, t1=steps[si+1].t;
      const frac=(elapsed-t0)/Math.max(1,t1-t0);
      drawServoArm(steps[si].deg+(steps[si+1].deg-steps[si].deg)*frac);
    } else {
      drawServoArm(90);
    }
    if(elapsed<totalMs){
      requestAnimationFrame(animate);
    } else {
      document.getElementById('range_status').innerText='Complete — check Serial for limits';
      document.getElementById('range_status').style.color='#00b894';
      btn.disabled=false; btn.style.opacity='1';
      drawServoArm(90);
    }
  }
  drawServoArm(90);
  requestAnimationFrame(animate);
}

function clearLog(){
  fetch('/clearlog').then(()=>{
    recActive=false; clearInterval(logPoll);
    document.getElementById('log').innerText='-- cleared --';
    const b=document.getElementById('rec_btn'); b.className='btn rec'; b.innerHTML='&#9210; RECORD';
  });
}
</script></body></html>
)rawliteral";

// ── Nav state for JSON ────────────────────────────────
float lastNavError = 0, lastPidOut = 0;

// ── Server handlers ───────────────────────────────────
void handleRoot()    { server.send(200,"text/html",HTML); }

void handleSensors() {
  float navErr = homeSet ? normalizeAngle(homeYaw - yawAngle) : 0;
  lastNavError = navErr;
  float pidO = pidKp*navErr;
  lastPidOut = pidO;
  String navState = !homeSet ? "Home not set" :
                    navigating ? ("Navigating — err="+String(navErr,1)+"°") : "Stopped";
  String j="{";
  j+="\"dA\":"+String(distA)+",\"dB\":"+String(distB)+",";
  j+="\"aX\":"+String(accX,2)+",\"aY\":"+String(accY,2)+",\"aZ\":"+String(accZ,2)+",";
  j+="\"gX\":"+String(gyroX,1)+",\"gY\":"+String(gyroY,1)+",\"gZ\":"+String(gyroZ,1)+",";
  j+="\"yaw\":"+String(yawAngle,1)+",\"tmp\":"+String(temperature,1)+",\"pls\":"+String(pulseCount)+",";
  j+="\"cal\":"+String(calibrated?"true":"false")+",";
  j+="\"homeYaw\":"+String(homeYaw,1)+",\"navErr\":"+String(navErr,1)+",";
  j+="\"pidOut\":"+String(pidO,1)+",\"navState\":\""+navState+"\"";
  j+="}";
  server.send(200,"application/json",j);
}

void handleSlave() {
  int n = server.arg("n").toInt();
  int s = server.arg("s").toInt();  // speed from slider (0 if not provided)
  if(n<2||n>9){ server.send(400,"text/plain","Bad"); return; }
  int spd = s > 0 ? s : 700;
  sendSlaveCmd(String(n), s > 0 ? s : -1);
  const char* lbl[]={"","","FWD","BACK","TURN_L","TURN_R","STOP","ACT_UP","ACT_DN","ACT_STOP"};
  // (Np) pattern preserved for replay parser; extra info appended
  if(n>=2 && n<=5) logEvent("SLAVE "+String(lbl[n])+" ("+String(n)+"p) spd="+String(spd));
  else             logEvent("SLAVE "+String(lbl[n])+" ("+String(n)+"p)");
  server.send(200,"text/plain","Slave: "+String(lbl[n])+" @ "+String(spd));
}

void handleMotion() {
  int n   = server.arg("n").toInt();
  int inc = max(1, (int)server.arg("inc").toInt());
  if(n<2||n>11){ server.send(400,"text/plain","Bad"); return; }

  if(n==9 || n==10) {
    // Absolute positioning — one burst, no intermediate hops
    motionFlipDeg = constrain(motionFlipDeg + (n==9 ? inc : -inc), 0, 180);
    sendFlipAbsolute(motionFlipDeg);
    String dir = (n==9)?"+":"-";
    logEvent("MOTION FLIP"+dir+String(inc)+"° -> "+String(motionFlipDeg)+"°");
    server.send(200,"text/plain","Flip -> "+String(motionFlipDeg)+"°");
  } else {
    // Sync motionFlipDeg for preset commands
    if(n==5) motionFlipDeg=FLIP_R_RAISE_DEG;
    else if(n==6||n==8) motionFlipDeg=90;
    else if(n==7) motionFlipDeg=FLIP_R_DUMP_DEG;
    sendMotionBurst(n);
    const char* lbl[]={"","","EXTEND","EXT_STOP","RETRACT","FLIP_RAISE","FLIP_MID","FLIP_DUMP","NEUTRAL","FLIP+","FLIP-","RANGE_GP5"};
    // Include angle for flip presets
    // (Np) pattern preserved for replay parser
    if(n==5)      logEvent("MOTION FLIP_RAISE (5p) GP1="+String(FLIP_R_RAISE_DEG)+"° GP3="+String(180-FLIP_R_RAISE_DEG)+"°");
    else if(n==6) logEvent("MOTION FLIP_MID (6p) GP1=90° GP3=90°");
    else if(n==7) logEvent("MOTION FLIP_DUMP (7p) GP1="+String(FLIP_R_DUMP_DEG)+"° GP3="+String(180-FLIP_R_DUMP_DEG)+"°");
    else if(n==8) logEvent("MOTION ALL_NEUTRAL (8p) GP1=90° GP3=90°");
    else          logEvent("MOTION "+String(lbl[n])+" ("+String(n)+"p)");
    server.send(200,"text/plain","Motion: "+String(lbl[n]));
  }
}

void handleStatePulse() { sendStatePulse(); server.send(200,"text/plain","State pulse #"+String(pulseCount)); }

void handleCalibrate() { runCalibration(); logEvent("CALIBRATE"); server.send(200,"text/plain","Calibrated OK"); }
void handleResetYaw()  { yawAngle=0; lastGyroTime=millis(); logEvent("RESET_YAW"); server.send(200,"text/plain","Yaw=0"); }

void handleSetHome() {
  homeYaw = yawAngle; homeSet = true;
  pidIntegral=0; pidPrev=0;
  logEvent("SET_HOME yaw="+String(homeYaw,1));
  server.send(200,"text/plain","Home set at yaw="+String(homeYaw,1)+"°");
}

void handleNavStart() {
  if(!homeSet){ server.send(200,"text/plain","Set home first"); return; }
  navigating=true; pidIntegral=0; pidPrev=0; lastNavCmd=0;
  logEvent("NAV_START");
  server.send(200,"text/plain","Navigating to yaw="+String(homeYaw,1)+"°");
}

void handleNavStop() {
  navigating=false;
  sendSlaveCmd("6");  // STOP slave motors
  logEvent("NAV_STOP");
  server.send(200,"text/plain","Navigation stopped");
}

void handleRecord() {
  if(!recording){ recording=true; recStart=millis(); logBuf="+0.00s  [RECORD START]\n"; server.send(200,"text/plain","started"); }
  else          { recording=false; logBuf+="---  [RECORD STOP]\n"; server.send(200,"text/plain","stopped"); }
}

void handleReplay() {
  if (logBuf.length() == 0) { server.send(200,"text/plain","No log recorded"); return; }
  parseLog();
  if (replayCount == 0) { server.send(200,"text/plain","Nothing to replay — record first"); return; }
  replayIdx = 0; replaying = true; replayStart = millis();
  server.send(200,"text/plain","Replaying "+String(replayCount)+" commands...");
}

void handleStopReplay() {
  replaying = false; replayIdx = 0;
  sendSlaveCmd("6");  // stop slave motors
  server.send(200,"text/plain","Replay stopped");
}

void handleDriveTimed() {
  int dir = server.arg("dir").toInt();  // 2=FWD 3=BACK 4=TURN_L
  int ms  = server.arg("ms").toInt();
  int spd = server.arg("s").toInt(); if(spd<=0) spd=700;
  const char* names[]={"","","FWD","BACK","TURN_L"};
  const char* lbl = (dir>=2&&dir<=4) ? names[dir] : "DRIVE";
  logEvent("SLAVE "+String(lbl)+" ("+String(dir)+"p) spd="+String(spd)+" duration="+String(ms/1000)+"s");
  sendToSlave((uint8_t)dir, spd);
  delay(ms);
  sendToSlave(6);  // STOP
  logEvent("SLAVE STOP (6p)");
  server.send(200,"text/plain",String(lbl)+" "+String(ms/1000)+"s done");
}

void handleExtendTimed() {
  int ms = server.arg("ms").toInt();
  logEvent("MOTION EXTEND (2p) duration="+String(ms/1000)+"s");
  sendMotionBurst(2);
  delay(ms);
  sendMotionBurst(3);
  logEvent("MOTION EXT_STOP (3p)");
  server.send(200,"text/plain","Extended "+String(ms/1000)+"s");
}

void handleRetractTimed() {
  int ms = server.arg("ms").toInt();
  logEvent("MOTION RETRACT (4p) duration="+String(ms/1000)+"s");
  sendMotionBurst(4);
  delay(ms);
  sendMotionBurst(3);
  logEvent("MOTION EXT_STOP (3p)");
  server.send(200,"text/plain","Retracted "+String(ms/1000)+"s");
}

void handleActuatorTimed() {
  String dir = server.arg("dir");
  int ms     = server.arg("ms").toInt();
  bool up    = (dir == "up");
  uint8_t startCmd = up ? 7 : 8;
  const char* lbl  = up ? "ACT_UP" : "ACT_DN";
  logEvent("SLAVE "+String(lbl)+" ("+String(startCmd)+"p) duration="+String(ms/1000)+"s");
  sendToSlave(startCmd);
  delay(ms);
  sendToSlave(9);  // ACT_STOP
  logEvent("SLAVE ACT_STOP (9p)");
  server.send(200,"text/plain",String(lbl)+" "+String(ms/1000)+"s done");
}

void handleMotionAbs() {
  int angle = server.arg("angle").toInt();
  sendFlipAbsolute(angle);
  logEvent("MOTION FLIP_ABS "+String(motionFlipDeg)+"°");
  server.send(200,"text/plain","Flip -> "+String(motionFlipDeg)+"°");
}

void handleLog()      { server.send(200,"text/plain",logBuf); }
void handleClearLog() { logBuf=""; recording=false; server.send(200,"text/plain","ok"); }

void handleDownload() {
  server.sendHeader("Content-Disposition","attachment; filename=robot_log.txt");
  server.sendHeader("Content-Type","text/plain");
  server.send(200,"text/plain",logBuf);
}

// ── Setup ─────────────────────────────────────────────
void setup() {
  pinMode(15,OUTPUT); digitalWrite(15,HIGH);
  Serial.begin(115200);

  pinMode(TRIG_A,OUTPUT); pinMode(ECHO_A,INPUT);
  pinMode(TRIG_B,OUTPUT); pinMode(ECHO_B,INPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(STATE_OUT,OUTPUT); digitalWrite(STATE_OUT,LOW);

  initMPU();

  // WiFi AP_STA: AP for phone, STA for ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false);          // keep STA disconnected so AP broadcasts cleanly
  WiFi.softAP(SSID, PASS);
  delay(200);                      // give AP time to come up before ESP-NOW init

  // ESP-NOW — wireless comms to slave (no wire needed)
  esp_now_init();
  { esp_now_peer_info_t p = {}; memcpy(p.peer_addr, slaveMac, 6); p.channel=0; p.encrypt=false; esp_now_add_peer(&p); }
  Serial.print("Master MAC: "); Serial.println(WiFi.macAddress());

  IPAddress ip = WiFi.softAPIP();

  server.on("/",          handleRoot);
  server.on("/sensors",   handleSensors);
  server.on("/slave",     handleSlave);
  server.on("/motion",    handleMotion);
  server.on("/state_pulse", handleStatePulse);
  server.on("/calibrate", handleCalibrate);
  server.on("/reset_yaw", handleResetYaw);
  server.on("/set_home",  handleSetHome);
  server.on("/nav_start", handleNavStart);
  server.on("/nav_stop",  handleNavStop);
  server.on("/replay",     handleReplay);
  server.on("/stopreplay", handleStopReplay);
  server.on("/drive_timed",    handleDriveTimed);
  server.on("/extend_timed",   handleExtendTimed);
  server.on("/retract_timed",  handleRetractTimed);
  server.on("/actuator_timed", handleActuatorTimed);
  server.on("/motion_abs",     handleMotionAbs);
  server.on("/record",    handleRecord);
  server.on("/log",       handleLog);
  server.on("/clearlog",  handleClearLog);
  server.on("/download",  handleDownload);
  server.begin();

  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_LIGHTGREY);
  tft.setTextColor(TFT_BLACK,TFT_LIGHTGREY); tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); tft.drawString("MASTER v2",tft.width()/2,25);
  tft.setTextSize(1);
  tft.drawString("WiFi: "+String(SSID),tft.width()/2,55);
  tft.drawString(ip.toString(),tft.width()/2,75);
  tft.drawString("192.168.4.1",tft.width()/2,92);

  Serial.print("AP: "); Serial.println(ip);
}

void loop() {
  server.handleClient();
  distA = ping(TRIG_A,ECHO_A);
  distB = ping(TRIG_B,ECHO_B);
  readMPU();
  updateNavigation();

  // ── Replay execution ──────────────────────────────
  if (replaying && replayIdx < replayCount) {
    unsigned long elapsed = millis() - replayStart;
    while (replayIdx < replayCount && elapsed >= replayQueue[replayIdx].timeMs) {
      ReplayCmd& c = replayQueue[replayIdx];
      if      (c.type == 0) sendSlaveCmd(String(c.value));
      else if (c.type == 1) sendMotionBurst(c.value);
      else if (c.type == 2) sendStatePulse();
      replayIdx++;
    }
    if (replayIdx >= replayCount) {
      replaying = false;
      sendSlaveCmd("6");  // auto-stop motors when replay finishes
      Serial.println("REPLAY COMPLETE");
    }
  }
}
