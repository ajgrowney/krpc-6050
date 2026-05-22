// TODO: reinitialize gyro after a reconnect after being offline
#include <Arduino.h>
#include <Wire.h> // Uses A4 for SDA and A5 for SCL
#include <I2Cdev.h>
#include <MPU6050.h>

unsigned long lastPrintMs = 0;
unsigned long lastUpdateUs = 0;
unsigned long lastImuProbeMs = 0;
bool imuConnected = false;
constexpr uint8_t kRotaryBit = 0x01;
constexpr uint8_t kImuBit = 0x02;
int status = 0;

constexpr uint8_t kEncoderClkPin = 2;
constexpr uint8_t kEncoderDtPin = 3;
constexpr uint8_t kEncoderSwPin = 4;
long encoderCount = 0;
int lastClkState = HIGH;
int lastSwState = HIGH;
unsigned long lastButtonChangeMs = 0;


MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;

float gyroXBias = 0.0f;
float gyroYBias = 0.0f;
float gyroZBias = 0.0f;

void calibrateGyroBias() {
  const int sampleCount = 500;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;

  Serial.println("Hold IMU still for gyro calibration...");

  for (int index = 0; index < sampleCount; ++index) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(4);
  }

  gyroXBias = static_cast<float>(sumX) / sampleCount;
  gyroYBias = static_cast<float>(sumY) / sampleCount;
  gyroZBias = static_cast<float>(sumZ) / sampleCount;
}

void setup() {
  pinMode(kEncoderClkPin, INPUT_PULLUP);
  pinMode(kEncoderDtPin, INPUT_PULLUP);
  pinMode(kEncoderSwPin, INPUT_PULLUP);
  lastClkState = digitalRead(kEncoderClkPin);
  lastSwState = digitalRead(kEncoderSwPin);
  status = 1;

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  delay(1000);
  Serial.println("Starting MPU6050...");

  // 
  mpu.initialize();
  if (mpu.testConnection()) {
    calibrateGyroBias();
    status += 2;
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 connection failed.");
  }

  lastUpdateUs = micros();
  Serial.println("status,roll,pitch,yaw");
}

void updateEncoder() {
  int clkState = digitalRead(kEncoderClkPin);
  int dtState = digitalRead(kEncoderDtPin);

  if (clkState != lastClkState) {
    // Use one edge only so each detent is counted once.
    if (clkState == LOW) {
      if (dtState != clkState) {
        encoderCount++;
      } else {
        encoderCount--;
      }
    }
    lastClkState = clkState;
  }
}

void updateEncoderButton() {
  int swState = digitalRead(kEncoderSwPin);
  unsigned long nowMs = millis();

  if (swState != lastSwState) {
    lastButtonChangeMs = nowMs;
    lastSwState = swState;
  }

  // Simple debounce: treat a stable LOW as a press
  if ((nowMs - lastButtonChangeMs) > 20 && swState == LOW) {
    encoderCount = 0;

    // Wait for release so one press only resets once
    while (digitalRead(kEncoderSwPin) == LOW) {
    }
    lastSwState = HIGH;
  }
}

void loop() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) / 1000000.0f;
  lastUpdateUs = nowUs;

  // ---- Handle Rotary Enc Data ----
  if (status != kRotaryBit) {
    updateEncoder();
    updateEncoderButton();
  }

  // ---- Handle IMU data ----
  if (millis() - lastImuProbeMs >= 250) {
    lastImuProbeMs = millis();
    imuConnected = mpu.testConnection();

    if (imuConnected) {
      status |= kImuBit;
    } else {
      status &= ~kImuBit;
      rollDeg = 0.0f;
      pitchDeg = 0.0f;
      yawDeg = 0.0f;
    }
  }
  if (imuConnected) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float gyroXDegPerSec = (gx - gyroXBias) / 131.0f;
    float gyroYDegPerSec = (gy - gyroYBias) / 131.0f;
    float gyroZDegPerSec = (gz - gyroZBias) / 131.0f;

    float accelRollDeg = atan2(static_cast<float>(ay), static_cast<float>(az)) * 180.0f / PI;
    float accelPitchDeg = atan2(
      -static_cast<float>(ax),
      sqrt(static_cast<float>(ay) * ay + static_cast<float>(az) * az)
    ) * 180.0f / PI;

    rollDeg = 0.98f * (rollDeg + gyroXDegPerSec * dt) + 0.02f * accelRollDeg;
    pitchDeg = 0.98f * (pitchDeg + gyroYDegPerSec * dt) + 0.02f * accelPitchDeg;
    yawDeg += gyroZDegPerSec * dt;
  }

  // Check if it's time to send a payload
  if (millis() - lastPrintMs >= 50) {
    lastPrintMs = millis();
    Serial.print(status);
    Serial.print(",");
    Serial.print(rollDeg);
    Serial.print(",");
    Serial.print(pitchDeg);
    Serial.print(",");
    Serial.print(yawDeg);
    Serial.print(",");
    Serial.println(encoderCount);
  }
}
