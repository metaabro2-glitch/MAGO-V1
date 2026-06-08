/*
 * ============================================================
 *  MAGO V1 - ROV Controller (Arduino Nano/Uno)
 *  Deep Sea Exploration System
 *  by Sohayb Shaaben
 * ============================================================
 *
 *  Hardware Mapping:
 *    MPU9250 Gyro   : I2C  (SDA=A4, SCL=A5)
 *    JSN-SR04T Sonar: TRIG=9, ECHO=10
 *    LM35 Temp      : A0
 *    Flow Meter     : D2 (interrupt)
 *    Pressure Sensor: A1
 *    Thruster ESCs  : D5, D6, D11, D12
 *    LED            : D4
 *
 *  Baud Rate: 115200
 *
 *  Communication Protocol (Serial -> PC1):
 *    [GYRO]  Gx: <float> deg/s  Gy: <float> deg/s  Gz: <float> deg/s
 *    [SONAR] Distance: <float> cm
 *    [LM35]  Temperature: <float> C
 *    [FLOW]  Flow: <float> L/min Vol: <float> L | Pres: <float> kPa (<float> psi) | Depth: <float> m
 *
 *  Incoming Commands (from PC2 via PC1):
 *    Movement: 'z','s','d','q','e','a',' ' (stop)
 *    LED: "led on\n", "led off\n"
 *    Thruster power: "pwm:<value>\n" (1100-1900)
 */

#include <Wire.h>
#include <Servo.h>

// ─────────────── Pin Definitions ───────────────
#define PIN_LED        4
#define PIN_SONAR_TRIG 9
#define PIN_SONAR_ECHO 10
#define PIN_LM35       A0
#define PIN_PRESSURE   A1
#define PIN_FLOW       2

// ─────────────── Thruster Pins ───────────────
#define THRUSTER_FL    5   // Front-Left
#define THRUSTER_FR    6   // Front-Right
#define THRUSTER_BL    11  // Back-Left
#define THRUSTER_BR    12  // Back-Right

// ─────────────── MPU9250 I2C Address ───────────────
#define MPU9250_ADDR   0x68

// ─────────────── Constants ───────────────
#define ESC_ARM_PWM    1500  // Neutral / arm value
#define ESC_MIN_PWM    1100
#define ESC_MAX_PWM    1900
#define SENSOR_INTERVAL_MS  200  // Send sensor data every 200ms
#define SONAR_TIMEOUT_US    30000

// ─────────────── Servo Objects ───────────────
Servo thrusterFL;
Servo thrusterFR;
Servo thrusterBL;
Servo thrusterBR;

// ─────────────── Global Variables ───────────────
volatile unsigned long flowPulseCount = 0;
unsigned long lastFlowTime = 0;
float totalFlowVolume = 0.0;

float currentGx = 0.0, currentGy = 0.0, currentGz = 0.0;
float currentYaw = 0.0, currentRoll = 0.0, currentPitch = 0.0;

unsigned long lastSensorTime = 0;
int thrusterPowerPWM = 1500;  // Current thruster power level

// Movement state
char currentMovement = ' ';  // ' ' = IDLE

// ─────────────── Flow Meter Interrupt ───────────────
void flowPulseISR() {
  flowPulseCount++;
}

// ─────────────── MPU9250 Functions ───────────────
void mpu9250_init() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up MPU9250
  Wire.endTransmission(true);

  // Configure Gyroscope range: ±250 deg/s
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s
  Wire.endTransmission(true);

  // Configure Accelerometer range: ±2g
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);

  // Set sample rate divider
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x19);  // SMPLRT_DIV register
  Wire.write(0x07);  // Sample rate = 1kHz / (1+7) = 125Hz
  Wire.endTransmission(true);

  // Configure DLPF (Digital Low Pass Filter)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // Accel DLPF ~44Hz, Gyro DLPF ~42Hz
  Wire.endTransmission(true);
}

int16_t readMPU9250Word(uint8_t reg) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)2, (uint8_t)true);
  int16_t val = (Wire.read() << 8) | Wire.read();
  return val;
}

void readGyroData(float &gx, float &gy, float &gz) {
  // Gyroscope registers: 0x43-0x48
  int16_t rawGx = readMPU9250Word(0x43);
  int16_t rawGy = readMPU9250Word(0x45);
  int16_t rawGz = readMPU9250Word(0x47);

  // Sensitivity for ±250 deg/s: 131 LSB/(deg/s)
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
}

void readAccelData(float &ax, float &ay, float &az) {
  // Accelerometer registers: 0x3B-0x40
  int16_t rawAx = readMPU9250Word(0x3B);
  int16_t rawAy = readMPU9250Word(0x3D);
  int16_t rawAz = readMPU9250Word(0x3F);

  // Sensitivity for ±2g: 16384 LSB/g
  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
}

void computeOrientation(float &roll, float &pitch, float &yaw) {
  float ax, ay, az;
  readAccelData(ax, ay, az);

  // Roll and Pitch from accelerometer
  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Yaw from gyroscope integration (approximate, drifts over time)
  // For a proper yaw, you'd need a magnetometer; this uses gyro Z-rate
  static float yawAngle = 0.0;
  static unsigned long lastYawTime = 0;
  unsigned long now = millis();
  float dt = (now - lastYawTime) / 1000.0;
  if (lastYawTime == 0) dt = 0;
  lastYawTime = now;

  float gx, gy, gz;
  readGyroData(gx, gy, gz);
  yawAngle += gz * dt;

  // Normalize yaw to 0-360
  if (yawAngle < 0) yawAngle += 360.0;
  if (yawAngle >= 360) yawAngle -= 360.0;

  yaw = yawAngle;
}

// ─────────────── Sonar Function ───────────────
float readSonarDistance() {
  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);

  long duration = pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1.0;  // No echo / timeout

  float distance = (duration * 0.0343) / 2.0;  // cm
  return distance;
}

// ─────────────── Temperature Function ───────────────
float readTemperature() {
  int rawADC = analogRead(PIN_LM35);
  // LM35: 10mV per degree C
  // Arduino ADC: 5V / 1024 steps = 4.882mV per step
  float voltage = rawADC * (5.0 / 1024.0);
  float tempC = voltage * 100.0;
  return tempC;
}

// ─────────────── Flow Meter Function ───────────────
float readFlowRate() {
  // YF-S201: ~2.25 mL per pulse (calibration factor ~7.5 for 1L/min per freq Hz)
  // Frequency = pulseCount / timeWindow
  unsigned long now = millis();
  unsigned long dt = now - lastFlowTime;
  if (dt < 1000) return 0.0;  // Need at least 1 second window

  noInterrupts();
  unsigned long pulses = flowPulseCount;
  flowPulseCount = 0;
  interrupts();

  float frequency = (pulses * 1000.0) / dt;  // Hz
  float flowRate = frequency / 7.5;           // L/min

  // Accumulate volume
  float volumeIncrement = flowRate * (dt / 60000.0);  // L
  totalFlowVolume += volumeIncrement;

  lastFlowTime = now;
  return flowRate;
}

// ─────────────── Pressure / Depth Function ───────────────
void readPressureAndDepth(float &pressure_kPa, float &pressure_psi, float &depth_m) {
  int rawADC = analogRead(PIN_PRESSURE);
  // Example: MPX5700AP or similar pressure sensor
  // 5V supply, 0.2V to 4.7V output, 15 to 115 kPa range
  float voltage = rawADC * (5.0 / 1024.0);

  // For MPX5700 series: Vout = Vs * (0.0018 * P + 0.04)
  // => P = (Vout/Vs - 0.04) / 0.0018  (in kPa)
  pressure_kPa = (voltage / 5.0 - 0.04) / 0.0018;
  if (pressure_kPa < 0) pressure_kPa = 0;

  // Convert to PSI
  pressure_psi = pressure_kPa * 0.145038;

  // Depth: 1 atm = 101.325 kPa, gauge pressure = absolute - 1 atm
  // Fresh water: ~9.806 kPa per meter
  float gaugePressure = pressure_kPa - 101.325;
  if (gaugePressure < 0) gaugePressure = 0;
  depth_m = gaugePressure / 9.806;
}

// ─────────────── Thruster Control ───────────────
void setAllThrusters(int pwm) {
  pwm = constrain(pwm, ESC_MIN_PWM, ESC_MAX_PWM);
  thrusterFL.writeMicroseconds(pwm);
  thrusterFR.writeMicroseconds(pwm);
  thrusterBL.writeMicroseconds(pwm);
  thrusterBR.writeMicroseconds(pwm);
}

void stopAllThrusters() {
  thrusterFL.writeMicroseconds(ESC_ARM_PWM);
  thrusterFR.writeMicroseconds(ESC_ARM_PWM);
  thrusterBL.writeMicroseconds(ESC_ARM_PWM);
  thrusterBR.writeMicroseconds(ESC_ARM_PWM);
}

void handleMovement(char cmd) {
  currentMovement = cmd;
  int fwd = thrusterPowerPWM;
  int rev = ESC_ARM_PWM - (thrusterPowerPWM - ESC_ARM_PWM); // mirror around neutral
  rev = constrain(rev, ESC_MIN_PWM, ESC_MAX_PWM);
  int neutral = ESC_ARM_PWM;

  switch (cmd) {
    case 'z':  // FORWARD
      thrusterFL.writeMicroseconds(fwd);
      thrusterFR.writeMicroseconds(fwd);
      thrusterBL.writeMicroseconds(fwd);
      thrusterBR.writeMicroseconds(fwd);
      break;
    case 's':  // BACKWARD
      thrusterFL.writeMicroseconds(rev);
      thrusterFR.writeMicroseconds(rev);
      thrusterBL.writeMicroseconds(rev);
      thrusterBR.writeMicroseconds(rev);
      break;
    case 'q':  // LEFT
      thrusterFL.writeMicroseconds(rev);
      thrusterFR.writeMicroseconds(fwd);
      thrusterBL.writeMicroseconds(rev);
      thrusterBR.writeMicroseconds(fwd);
      break;
    case 'd':  // RIGHT
      thrusterFL.writeMicroseconds(fwd);
      thrusterFR.writeMicroseconds(rev);
      thrusterBL.writeMicroseconds(fwd);
      thrusterBR.writeMicroseconds(rev);
      break;
    case 'e':  // ROTATE RIGHT (CW)
      thrusterFL.writeMicroseconds(fwd);
      thrusterFR.writeMicroseconds(rev);
      thrusterBL.writeMicroseconds(fwd);
      thrusterBR.writeMicroseconds(rev);
      break;
    case 'a':  // ROTATE LEFT (CCW)
      thrusterFL.writeMicroseconds(rev);
      thrusterFR.writeMicroseconds(fwd);
      thrusterBL.writeMicroseconds(rev);
      thrusterBR.writeMicroseconds(fwd);
      break;
    case ' ':  // STOP
    default:
      stopAllThrusters();
      currentMovement = ' ';
      break;
  }
}

// ─────────────── LED Control ───────────────
void setLED(bool state) {
  digitalWrite(PIN_LED, state ? HIGH : LOW);
}

// ─────────────── Process Incoming Command ───────────────
void processCommand(String &cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Single-character movement commands
  if (cmd.length() == 1) {
    char c = cmd.charAt(0);
    if (c == 'z' || c == 's' || c == 'd' || c == 'q' ||
        c == 'e' || c == 'a' || c == ' ') {
      handleMovement(c);
      return;
    }
  }

  // LED commands
  if (cmd == "led on") {
    setLED(true);
    return;
  }
  if (cmd == "led off") {
    setLED(false);
    return;
  }

  // Thruster power command: "pwm:<value>"
  if (cmd.startsWith("pwm:")) {
    int val = cmd.substring(4).toInt();
    val = constrain(val, ESC_MIN_PWM, ESC_MAX_PWM);
    thrusterPowerPWM = val;
    return;
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  // Serial
  Serial.begin(115200);
  while (!Serial) { ; }

  // I2C for MPU9250
  Wire.begin();
  mpu9250_init();

  // Pin modes
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
  pinMode(PIN_FLOW, INPUT_PULLUP);

  // Attach flow meter interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowPulseISR, FALLING);

  // Attach ESCs
  thrusterFL.attach(THRUSTER_FL, ESC_MIN_PWM, ESC_MAX_PWM);
  thrusterFR.attach(THRUSTER_FR, ESC_MIN_PWM, ESC_MAX_PWM);
  thrusterBL.attach(THRUSTER_BL, ESC_MIN_PWM, ESC_MAX_PWM);
  thrusterBR.attach(THRUSTER_BR, ESC_MIN_PWM, ESC_MAX_PWM);

  // Arm ESCs (send neutral signal)
  stopAllThrusters();

  // Wait for ESCs to arm
  delay(2000);

  // Initialize flow timer
  lastFlowTime = millis();
  lastSensorTime = millis();

  Serial.println("[SYS] MAGO V1 Arduino Initialized");
}

// ═══════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  // ─── Read Incoming Commands ───
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // ─── Send Sensor Data Periodically ───
  unsigned long now = millis();
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;

    // 1) Gyro Data
    float gx, gy, gz;
    readGyroData(gx, gy, gz);
    currentGx = gx;
    currentGy = gy;
    currentGz = gz;
    Serial.print("[GYRO] Gx: ");
    Serial.print(gx, 2);
    Serial.print(" deg/s  Gy: ");
    Serial.print(gy, 2);
    Serial.print(" deg/s  Gz: ");
    Serial.print(gz, 2);
    Serial.println(" deg/s");

    // Orientation (Roll, Pitch, Yaw)
    computeOrientation(currentRoll, currentPitch, currentYaw);
    Serial.print("[ORIENT] Roll: ");
    Serial.print(currentRoll, 2);
    Serial.print(" Pitch: ");
    Serial.print(currentPitch, 2);
    Serial.print(" Yaw: ");
    Serial.print(currentYaw, 2);
    Serial.println(" deg");

    // 2) Sonar Distance
    float dist = readSonarDistance();
    if (dist > 0) {
      Serial.print("[SONAR] Distance: ");
      Serial.print(dist, 2);
      Serial.println(" cm");
    }

    // 3) Temperature
    float temp = readTemperature();
    Serial.print("[LM35] Temperature: ");
    Serial.print(temp, 2);
    Serial.println(" C");

    // 4) Flow + Pressure + Depth
    float flowRate = readFlowRate();
    float presKPa, presPSI, depthM;
    readPressureAndDepth(presKPa, presPSI, depthM);
    Serial.print("[FLOW] Flow: ");
    Serial.print(flowRate, 2);
    Serial.print(" L/min Vol: ");
    Serial.print(totalFlowVolume, 2);
    Serial.print(" L | Pres: ");
    Serial.print(presKPa, 2);
    Serial.print(" kPa (");
    Serial.print(presPSI, 2);
    Serial.print(" psi) | Depth: ");
    Serial.print(depthM, 2);
    Serial.println(" m");
  }
}
