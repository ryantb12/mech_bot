// ===================================================
// MASTER SENSOR TEST v1
// Master board (LilyGO T-Display S3 #1)
//
// Displays live HC-SR04 distance + MPU-6050 IMU data.
// Press button once → runs IMU calibration (keep robot still).
// After calibration: shows calibrated readings + yaw tracking.
// Press button again at any time to re-calibrate.
// ===================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

// ===================================================
// PIN DEFINITIONS
// ===================================================
#define TRIG_A      2    // Ultrasonic A trigger
#define ECHO_A      3    // Ultrasonic A echo
#define TRIG_B     43    // Ultrasonic B trigger
#define ECHO_B     44    // Ultrasonic B echo
#define MPU_SDA    16
#define MPU_SCL    21
#define MPU_ADDR 0x68
#define BUTTON_PIN 10   // INPUT_PULLUP — active LOW

#ifdef TFT_LIGHTGREY
  #undef TFT_LIGHTGREY
#endif
#define TFT_LIGHTGREY 0xC618

// ===================================================
// GLOBALS
// ===================================================
TFT_eSPI tft = TFT_eSPI();

// IMU data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;

// Calibration offsets
float accOffX = 0, accOffY = 0, accOffZ = 0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
bool  calibrated = false;

// Yaw integration
float yawAngle = 0;
unsigned long lastGyroTime = 0;

// Distance
long distA = 0;   // Ultrasonic A (front)
long distB = 0;   // Ultrasonic B (second)

// ===================================================
// BUTTON
// ===================================================
bool buttonPressed() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
      delay(50);
      return true;
    }
  }
  return false;
}

// ===================================================
// HC-SR04
// ===================================================
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  return duration * 0.034 / 2;
}

long readDistanceACM() { return readUltrasonic(TRIG_A, ECHO_A); }
long readDistanceBCM() { return readUltrasonic(TRIG_B, ECHO_B); }

// ===================================================
// MPU-6050
// ===================================================
void initMPU6050() {
  Wire.begin(MPU_SDA, MPU_SCL);
  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  // Accel range ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);
  // Gyro range ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();
  int16_t rawT  = Wire.read() << 8 | Wire.read();
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

  accX = rawAX / 16384.0 - accOffX;
  accY = rawAY / 16384.0 - accOffY;
  accZ = rawAZ / 16384.0 - accOffZ;
  gyroX = rawGX / 131.0  - gyroOffX;
  gyroY = rawGY / 131.0  - gyroOffY;
  gyroZ = rawGZ / 131.0  - gyroOffZ;
  temperature = (rawT / 340.0) + 36.53;

  if (calibrated && lastGyroTime > 0) {
    float dt = (millis() - lastGyroTime) / 1000.0;
    yawAngle += gyroZ * dt;
  }
  lastGyroTime = millis();
}

// ===================================================
// CALIBRATION
// ===================================================
void runCalibration() {
  const int SAMPLES = 200;

  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATING", tft.width() / 2, 25);
  tft.setTextSize(1);
  tft.drawString("KEEP ROBOT STILL", tft.width() / 2, 50);

  // Progress bar outline
  int bx = 20, by = 70, bw = tft.width() - 40, bh = 16;
  tft.drawRect(bx, by, bw, bh, TFT_WHITE);

  float sAX = 0, sAY = 0, sAZ = 0;
  float sGX = 0, sGY = 0, sGZ = 0;

  for (int i = 0; i < SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    sAX += (Wire.read() << 8 | Wire.read()) / 16384.0;
    sAY += (Wire.read() << 8 | Wire.read()) / 16384.0;
    sAZ += (Wire.read() << 8 | Wire.read()) / 16384.0;
    Wire.read(); Wire.read(); // discard temp
    sGX += (Wire.read() << 8 | Wire.read()) / 131.0;
    sGY += (Wire.read() << 8 | Wire.read()) / 131.0;
    sGZ += (Wire.read() << 8 | Wire.read()) / 131.0;

    // Update progress bar
    int fill = map(i, 0, SAMPLES - 1, 0, bw - 4);
    tft.fillRect(bx + 2, by + 2, fill, bh - 4, TFT_GREEN);

    delay(10);
  }

  accOffX  = sAX / SAMPLES;
  accOffY  = sAY / SAMPLES;
  accOffZ  = sAZ / SAMPLES - 1.0;  // remove 1g from Z
  gyroOffX = sGX / SAMPLES;
  gyroOffY = sGY / SAMPLES;
  gyroOffZ = sGZ / SAMPLES;

  calibrated = true;
  yawAngle = 0;
  lastGyroTime = millis();

  // Show offsets
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("CALIBRATED!", tft.width() / 2, 18);

  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  int y = 42;
  tft.drawString("Acc  X: " + String(accOffX,  4), 10, y); y += 14;
  tft.drawString("Acc  Y: " + String(accOffY,  4), 10, y); y += 14;
  tft.drawString("Acc  Z: " + String(accOffZ,  4), 10, y); y += 14;
  tft.drawString("Gyro X: " + String(gyroOffX, 4), 10, y); y += 14;
  tft.drawString("Gyro Y: " + String(gyroOffY, 4), 10, y); y += 14;
  tft.drawString("Gyro Z: " + String(gyroOffZ, 4), 10, y);

  tft.setTextDatum(MC_DATUM);
  tft.drawString("PRESS BUTTON TO CONTINUE", tft.width() / 2, tft.height() - 10);

  while (!buttonPressed()) delay(10);

  tft.fillScreen(TFT_BLACK);
}

// ===================================================
// DISPLAY — live readings screen
// ===================================================
void showReadings() {
  tft.setTextDatum(TL_DATUM);

  // Header
  tft.fillRect(0, 0, tft.width(), 22, calibrated ? TFT_DARKGREEN : TFT_DARKCYAN);
  tft.setTextColor(TFT_WHITE, calibrated ? TFT_DARKGREEN : TFT_DARKCYAN);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  String header = calibrated ? "CALIBRATED — press btn to redo" : "RAW — press button to calibrate";
  tft.drawString(header, tft.width() / 2, 11);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  int y = 28;

  // Distance A + B
  tft.fillRect(0, y, tft.width(), 16, TFT_BLACK);
  tft.drawString("A: " + ((distA > 0 && distA < 400) ? String(distA) + " cm" : String("NO ECHO")), 5, y);
  tft.drawString("B: " + ((distB > 0 && distB < 400) ? String(distB) + " cm" : String("NO ECHO")), tft.width() / 2, y);
  y += 18;

  tft.drawLine(0, y, tft.width(), y, TFT_DARKGREY);
  y += 4;

  // Accel
  tft.fillRect(0, y, tft.width(), 30, TFT_BLACK);
  tft.drawString("ACCEL (g)", 5, y); y += 14;
  tft.drawString("X:" + String(accX, 2) + "  Y:" + String(accY, 2) + "  Z:" + String(accZ, 2), 5, y);
  y += 18;

  tft.drawLine(0, y, tft.width(), y, TFT_DARKGREY);
  y += 4;

  // Gyro
  tft.fillRect(0, y, tft.width(), 30, TFT_BLACK);
  tft.drawString("GYRO (deg/s)", 5, y); y += 14;
  tft.drawString("X:" + String(gyroX, 1) + "  Y:" + String(gyroY, 1) + "  Z:" + String(gyroZ, 1), 5, y);
  y += 18;

  tft.drawLine(0, y, tft.width(), y, TFT_DARKGREY);
  y += 4;

  // Yaw + temp
  tft.fillRect(0, y, tft.width(), 30, TFT_BLACK);
  if (calibrated) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("YAW: " + String(yawAngle, 1) + " deg", 5, y);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("YAW: (calibrate first)", 5, y);
  }
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  y += 14;
  tft.drawString("TEMP: " + String(temperature, 1) + " C", 5, y);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);  // backlight on

  Serial.begin(115200);

  pinMode(TRIG_A,     OUTPUT);
  pinMode(ECHO_A,     INPUT);
  pinMode(TRIG_B,     OUTPUT);
  pinMode(ECHO_B,     INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  initMPU6050();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  Serial.println("master_sensor_test_v1 ready");
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  distA = readDistanceACM();
  distB = readDistanceBCM();
  readMPU6050();

  showReadings();

  if (buttonPressed()) {
    runCalibration();
  }

  delay(50);
}
