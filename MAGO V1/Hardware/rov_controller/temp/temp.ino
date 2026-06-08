#include <Wire.h>
#include <Servo.h>

// ================================================================
//  MPU9250 GYROSCOPE
// ================================================================
const uint8_t MPU_ADDR = 0x68;

const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const uint8_t REG_WHO_AM_I     = 0x75;

const uint8_t FS_SEL = 0;
const float GYRO_SENSITIVITY[] = {131.0, 65.5, 32.8, 16.4};
const float GYRO_SCALE = GYRO_SENSITIVITY[FS_SEL];

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

const int   CALIBRATION_SAMPLES  = 1000;
const uint8_t CALIBRATION_DELAY_MS = 2;

// ================================================================
//  JSN-SR04T ULTRASONIC SENSOR
// ================================================================
const int trigPin = 9;
const int echoPin = 10;
const float SPEED_OF_SOUND = 34300.0;

// ================================================================
//  SERVO THRUSTERS — forwardSpeed read via serial
// ================================================================
Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;

int forwardSpeed    = 1600;

// ================================================================
//  SERIAL COMMAND BUFFER
// ================================================================
const byte CMD_BUF_SIZE = 20;
char cmdBuffer[CMD_BUF_SIZE];
byte cmdIndex = 0;

// ================================================================
//  LM35 TEMPERATURE SENSOR
// ================================================================
const int sensorPin = A0;

// ================================================================
//  YF-S401 FLOW, PRESSURE & DEPTH ESTIMATOR
// ================================================================
#define FLOW_SENSOR_PIN    2       // Must be interrupt-capable (D2 or D3)
#define CALIBRATION_FACTOR 98.0
#define FLOW_THRESHOLD     0.1
#define PRESSURE_K         0.28

// Tank dimensions — CHANGE THESE TO MATCH YOUR TANK (metres)
#define TANK_LENGTH_M 0.50
#define TANK_WIDTH_M  0.30
// For circular tank comment the above and use:
// #define TANK_DIAMETER_M 0.50

float tankAreaM2;

volatile unsigned int pulseCount = 0;
float flowRate       = 0.0;    // L/min
float totalVolume    = 0.0;    // Litres
float estPressureKpa = 0.0;    // kPa
float estDepthM      = 0.0;    // Metres

// ================================================================
//  TIMING (millis-based for each subsystem)
// ================================================================
unsigned long gyroPrevMillis       = 0;
unsigned long ultrasonicPrevMillis = 0;
unsigned long tempPrevMillis       = 0;
unsigned long flowPrevMillis       = 0;

const long gyroInterval       = 100;   // 10 Hz
const long ultrasonicInterval = 500;   // 2 Hz
const long tempInterval       = 1000;  // 1 Hz
const long flowInterval       = 1000;  // 1 Hz

// ================================================================
//  HELPER – inverted speed mirrors forwardSpeed
// ================================================================
int invertedSpeed() {
  return 3000 - forwardSpeed;
}

// ================================================================
//  FLOW SENSOR ISR
// ================================================================
void pulseCounter() {
  pulseCount++;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // ---------- MPU9250 init ----------
  Wire.begin();
  Wire.setClock(400000);

  uint8_t whoAmI = readRegister(REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI != 0x71 && whoAmI != 0x73 && whoAmI != 0x68 && whoAmI != 0x70) {
    Serial.println("Warning: Sensor not found! Check wiring and address.");
    while (1);
  }

  writeRegister(REG_PWR_MGMT_1, 0x00);
  delay(100);
  writeRegister(REG_SMPLRT_DIV, 0x07);
  writeRegister(REG_CONFIG, 0x00);
  writeRegister(REG_GYRO_CONFIG, FS_SEL << 3);

  Serial.println(F("\n[MPU9250] Gyroscope Initialized"));
  Serial.print(F("  Full-Scale Range: +/-"));
  Serial.print(250 * (1 << FS_SEL));
  Serial.println(F(" deg/s"));
  Serial.print(F("  Sensitivity: "));
  Serial.println(GYRO_SCALE);

  calibrateGyro();

  // ---------- JSN-SR04T init ----------
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delay(100);
  Serial.println(F("\n[JSN-SR04T] Distance Sensor Ready"));

  // ---------- Servo Thrusters init ----------
  thruster1.attach(5);
  thruster2.attach(6);
  thruster3.attach(11);
  thruster4.attach(12);
  stopAll();
  delay(2000);
  Serial.println(F("[Thrusters] Ready"));
  Serial.print(F("  forwardSpeed = "));
  Serial.println(forwardSpeed);
  Serial.println(F("  Commands: z/s/d/q/e/a/space | f<value> to set speed (1100-1900)"));

  // ---------- LM35 init ----------
  pinMode(sensorPin, INPUT);
  Serial.println(F("[LM35] Temperature Sensor Ready"));

  // ---------- YF-S401 Flow Sensor init ----------
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // Calculate tank area (m²) — choose your shape:
  // 1. Rectangular:
  tankAreaM2 = TANK_LENGTH_M * TANK_WIDTH_M;
  // 2. Circular (comment above, uncomment below):
  // tankAreaM2 = 3.14159 * (TANK_DIAMETER_M / 2.0) * (TANK_DIAMETER_M / 2.0);

  flowPrevMillis = millis();
  Serial.println(F("[YF-S401] Flow / Pressure / Depth Sensor Ready"));

  Serial.println(F("\n=================================================================="));
  Serial.println(F(" All Systems Online"));
  Serial.println(F("==================================================================\n"));
  delay(500);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  unsigned long currentMillis = millis();

  // ---- Thruster serial command (every iteration) ----
  handleThrusterCommand();

  // ---- Gyroscope read ----
  if (currentMillis - gyroPrevMillis >= gyroInterval) {
    gyroPrevMillis = currentMillis;
    float gx, gy, gz;
    readGyro(gx, gy, gz);
    Serial.print("[GYRO] Gx: "); Serial.print(gx, 2);
    Serial.print(" deg/s  Gy: "); Serial.print(gy, 2);
    Serial.print(" deg/s  Gz: "); Serial.print(gz, 2);
    Serial.println(" deg/s");
  }

  // ---- Ultrasonic read ----
  if (currentMillis - ultrasonicPrevMillis >= ultrasonicInterval) {
    ultrasonicPrevMillis = currentMillis;
    float distance = measureDistance();
    if (distance >= 0) {
      Serial.print("[SONAR] Distance: "); Serial.print(distance, 1);
      Serial.println(" cm");
    } else {
      Serial.println("[SONAR] Error: No echo (out of range)");
    }
  }

  // ---- Temperature read ----
  if (currentMillis - tempPrevMillis >= tempInterval) {
    tempPrevMillis = currentMillis;
    int reading = analogRead(sensorPin);
    float voltage = reading * (5.0 / 1024.0);
    float temperatureC = voltage * 100;
    Serial.print("[LM35] Temperature: "); Serial.print(temperatureC);
    Serial.println(" C");
  }

  // ---- Flow / Pressure / Depth read ----
  if (currentMillis - flowPrevMillis >= flowInterval) {
    flowPrevMillis = currentMillis;
    readFlowSensor();
  }
}

// ================================================================
//  YF-S401 – Read Flow Sensor & Compute Pressure / Depth
// ================================================================
void readFlowSensor() {
  // Safely read and reset pulse count
  noInterrupts();
  unsigned int count = pulseCount;
  pulseCount = 0;
  interrupts();

  // ---- Flow Rate (L/min) ----
  flowRate = (float)count / CALIBRATION_FACTOR;
  if (flowRate < FLOW_THRESHOLD) flowRate = 0.0;

  // ---- Total Volume (Litres) ----
  totalVolume += (flowRate / 60.0);

  // ---- Estimated Pressure Drop (kPa) ----
  if (flowRate > 0.0) {
    estPressureKpa = PRESSURE_K * (flowRate * flowRate);
  } else {
    estPressureKpa = 0.0;
  }

  // ---- Estimated Depth (Metres) ----
  if (tankAreaM2 > 0.0) {
    float volumeM3 = totalVolume / 1000.0;
    estDepthM = volumeM3 / tankAreaM2;
  } else {
    estDepthM = 0.0;
  }

  // Convert pressure to PSI (optional)
  float estPressurePsi = estPressureKpa * 0.145038;

  // ---- Print ----
  Serial.print("[FLOW] Flow: ");
  Serial.print(flowRate, 2);
  Serial.print(" L/min | Vol: ");
  Serial.print(totalVolume, 3);
  Serial.print(" L | Pres: ");
  Serial.print(estPressureKpa, 2);
  Serial.print(" kPa (");
  Serial.print(estPressurePsi, 2);
  Serial.print(" psi) | Depth: ");
  Serial.print(estDepthM, 3);
  Serial.println(" m");
}

// ================================================================
//  THRUSTER COMMAND HANDLER (non-blocking, line-buffered)
// ================================================================
void handleThrusterCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else {
      if (cmdIndex < CMD_BUF_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}

// ================================================================
//  PROCESS ONE COMPLETED COMMAND LINE
// ================================================================
void processCommand(const char* cmd) {

  // ---- Set forward speed: "f1600", "F1500", "f 1600" ----
  if (cmd[0] == 'f' || cmd[0] == 'F') {
    int startIdx = 1;
    while (cmd[startIdx] == ' ') startIdx++;

    int newSpeed = atoi(&cmd[startIdx]);

    if (newSpeed >= 1100 && newSpeed <= 1900) {
      forwardSpeed = newSpeed;
      Serial.print("[THR] forwardSpeed set to ");
      Serial.print(forwardSpeed);
      Serial.print("  invertedSpeed = ");
      Serial.println(invertedSpeed());
    } else {
      Serial.println("[THR] ERROR: Speed must be 1100-1900 us");
      Serial.print("[THR] Current forwardSpeed remains ");
      Serial.println(forwardSpeed);
    }
    return;
  }

  // ---- Single-character movement commands ----
  if (strlen(cmd) == 1) {
    int inv = invertedSpeed();
    switch (cmd[0]) {
      case 'z':
        thruster1.writeMicroseconds(forwardSpeed);
        thruster3.writeMicroseconds(forwardSpeed);
        thruster4.writeMicroseconds(1500);
        thruster2.writeMicroseconds(1500);
        Serial.println("[THR] Forward");
        break;
      case 's':
        thruster1.writeMicroseconds(inv);
        thruster3.writeMicroseconds(inv);
        thruster4.writeMicroseconds(1500);
        thruster2.writeMicroseconds(1500);
        Serial.println("[THR] Backward");
        break;
      case 'd':
        thruster1.writeMicroseconds(inv);
        thruster3.writeMicroseconds(forwardSpeed);
        thruster4.writeMicroseconds(1500);
        thruster2.writeMicroseconds(1500);
        Serial.println("[THR] Turn Right");
        break;
      case 'q':
        thruster1.writeMicroseconds(forwardSpeed);
        thruster3.writeMicroseconds(inv);
        thruster4.writeMicroseconds(1500);
        thruster2.writeMicroseconds(1500);
        Serial.println("[THR] Turn Left");
        break;
      case 'e':
        thruster1.writeMicroseconds(1500);
        thruster3.writeMicroseconds(1500);
        thruster4.writeMicroseconds(forwardSpeed);
        thruster2.writeMicroseconds(forwardSpeed);
        Serial.println("[THR] Strafe Right");
        break;
      case 'a':
        thruster1.writeMicroseconds(1500);
        thruster3.writeMicroseconds(1500);
        thruster4.writeMicroseconds(inv);
        thruster2.writeMicroseconds(inv);
        Serial.println("[THR] Strafe Left");
        break;
      case ' ':
        stopAll();
        Serial.println("[THR] Stop");
        break;
      default:
        Serial.print("[THR] Unknown command: ");
        Serial.println(cmd[0]);
        break;
    }
    return;
  }

  Serial.print("[THR] Unknown command: ");
  Serial.println(cmd);
}

// ================================================================
//  STOP ALL THRUSTERS
// ================================================================
void stopAll() {
  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);
  thruster3.writeMicroseconds(1500);
  thruster4.writeMicroseconds(1500);
}

// ================================================================
//  MPU9250 – Read Gyroscope
// ================================================================
void readGyro(float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  gx = (rawX / GYRO_SCALE) - gyroOffsetX;
  gy = (rawY / GYRO_SCALE) - gyroOffsetY;
  gz = (rawZ / GYRO_SCALE) - gyroOffsetZ;
}

// ================================================================
//  MPU9250 – Calibrate Gyroscope
// ================================================================
void calibrateGyro() {
  Serial.println(F("\n[MPU9250] Calibrating gyroscope — KEEP SENSOR STILL..."));

  long sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6);

    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();

    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;

    delay(CALIBRATION_DELAY_MS);
  }

  gyroOffsetX = (sumX / (float)CALIBRATION_SAMPLES) / GYRO_SCALE;
  gyroOffsetY = (sumY / (float)CALIBRATION_SAMPLES) / GYRO_SCALE;
  gyroOffsetZ = (sumZ / (float)CALIBRATION_SAMPLES) / GYRO_SCALE;

  Serial.println(F("[MPU9250] Calibration complete!"));
  Serial.print(F("  Offset X: ")); Serial.print(gyroOffsetX, 4); Serial.println(" deg/s");
  Serial.print(F("  Offset Y: ")); Serial.print(gyroOffsetY, 4); Serial.println(" deg/s");
  Serial.print(F("  Offset Z: ")); Serial.print(gyroOffsetZ, 4); Serial.println(" deg/s");
}

// ================================================================
//  MPU9250 – I2C Helpers
// ================================================================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

// ================================================================
//  JSN-SR04T – Measure Distance
// ================================================================
float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }

  float distance = (duration * SPEED_OF_SOUND) / 2000000.0;
  return distance;
}