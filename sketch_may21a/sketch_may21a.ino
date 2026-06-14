// TODO: reinitialize gyro after a reconnect after being offline
#include <Arduino.h>
#include <Wire.h> // Uses A4 for SDA and A5 for SCL
#include <I2Cdev.h>
#include <MPU6050.h>

unsigned long lastPrintMs = 0;
unsigned long lastUpdateUs = 0;
unsigned long lastImuProbeMs = 0;
uint16_t packetSequence = 0;
bool imuConnected = false;
constexpr uint8_t kRotaryBit = 0x01;
constexpr uint8_t kImuBit = 0x02;
int status = 0;

// Checkout docs/packet_dd.md for info on telemetry
constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kPacketVersion = 0x01;
constexpr size_t kPayloadSize = 14;
constexpr size_t kPacketSize = 17;
constexpr bool kEnableSerialTextDebug = false;

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
float accelRollDeg = 0.0f;
float accelPitchDeg = 0.0f;
float gyroXDegPerSec = 0.0f;
float gyroYDegPerSec = 0.0f;
float gyroZDegPerSec = 0.0f;

float gyroXBias = 0.0f;
float gyroYBias = 0.0f;
float gyroZBias = 0.0f;

void calibrateGyroBias() {
  const int sampleCount = 500;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;

  if (kEnableSerialTextDebug) {
    Serial.println("Hold IMU still for gyro calibration...");
  }

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

  if (kEnableSerialTextDebug) {
    Serial.print("gyro_bias,");
    Serial.print(gyroXBias);
    Serial.print(",");
    Serial.print(gyroYBias);
    Serial.print(",");
    Serial.println(gyroZBias);
  }
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
  if (kEnableSerialTextDebug) {
    Serial.println("Starting MPU6050...");
  }

  // 
  mpu.initialize();
  if (mpu.testConnection()) {
    calibrateGyroBias();
    status += 2;
    if (kEnableSerialTextDebug) {
      Serial.println("MPU6050 connected.");
    }
  } else {
    if (kEnableSerialTextDebug) {
      Serial.println("MPU6050 connection failed.");
    }
  }

  lastUpdateUs = micros();
  if (kEnableSerialTextDebug) {
    Serial.println("status,roll,pitch,yaw,encoder,gx,gy,gz,gyro_x_dps,gyro_y_dps,gyro_z_dps,accel_roll,accel_pitch");
  }
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

int16_t degreesToCentidegrees(float degrees) {
  float scaled = roundf(degrees * 100.0f);

  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }

  return static_cast<int16_t>(scaled);
}

void writeInt16LE(uint8_t* dest, int16_t value) {
  dest[0] = static_cast<uint8_t>(value & 0xFF);
  dest[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeUint16LE(uint8_t* dest, uint16_t value) {
  dest[0] = static_cast<uint8_t>(value & 0xFF);
  dest[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeInt32LE(uint8_t* dest, int32_t value) {
  dest[0] = static_cast<uint8_t>(value & 0xFF);
  dest[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dest[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dest[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint8_t crc8Maxim(const uint8_t* data, size_t length) {
  uint8_t crc = 0x00;

  for (size_t index = 0; index < length; ++index) {
    uint8_t inByte = data[index];

    for (uint8_t bit = 0; bit < 8; ++bit) {
      uint8_t mix = (crc ^ inByte) & 0x01;
      crc >>= 1;
      if (mix != 0) {
        crc ^= 0x8C;
      }
      inByte >>= 1;
    }
  }

  return crc;
}

/*
Print out human readabale csv debugging
*/
void serial_csv(int statusMask, float rollDegrees, float pitchDegrees, float yawDegrees, long encoderTicks) {
  Serial.print(statusMask);
  Serial.print(",");
  Serial.print(rollDegrees);
  Serial.print(",");
  Serial.print(pitchDegrees);
  Serial.print(",");
  Serial.print(yawDegrees);
  Serial.print(",");
  Serial.println(encoderTicks);
}

void serial_imu_debug_csv(
  int statusMask,
  float rollDegrees,
  float pitchDegrees,
  float yawDegrees,
  long encoderTicks,
  int16_t gyroXRaw,
  int16_t gyroYRaw,
  int16_t gyroZRaw,
  float gyroXDegreesPerSecond,
  float gyroYDegreesPerSecond,
  float gyroZDegreesPerSecond,
  float accelRollDegrees,
  float accelPitchDegrees
) {
  Serial.print(statusMask);
  Serial.print(",");
  Serial.print(rollDegrees);
  Serial.print(",");
  Serial.print(pitchDegrees);
  Serial.print(",");
  Serial.print(yawDegrees);
  Serial.print(",");
  Serial.print(encoderTicks);
  Serial.print(",");
  Serial.print(gyroXRaw);
  Serial.print(",");
  Serial.print(gyroYRaw);
  Serial.print(",");
  Serial.print(gyroZRaw);
  Serial.print(",");
  Serial.print(gyroXDegreesPerSecond);
  Serial.print(",");
  Serial.print(gyroYDegreesPerSecond);
  Serial.print(",");
  Serial.print(gyroZDegreesPerSecond);
  Serial.print(",");
  Serial.print(accelRollDegrees);
  Serial.print(",");
  Serial.println(accelPitchDegrees);
}

/*
Serial Binary for KSP
*/
void serial_binary(uint8_t statusMask, float rollDegrees, float pitchDegrees, float yawDegrees, int32_t encoderTicks) {
  uint8_t packet[kPacketSize];

  packet[0] = kSync0;
  packet[1] = kSync1;
  packet[2] = kPacketVersion;
  writeUint16LE(&packet[3], packetSequence);
  packet[5] = statusMask;
  writeInt16LE(&packet[6], degreesToCentidegrees(yawDegrees));
  writeInt16LE(&packet[8], degreesToCentidegrees(pitchDegrees));
  writeInt16LE(&packet[10], degreesToCentidegrees(rollDegrees));
  writeInt32LE(&packet[12], encoderTicks);
  packet[16] = crc8Maxim(&packet[2], kPayloadSize);

  Serial.write(packet, kPacketSize);
  packetSequence++;
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
  if (status & kRotaryBit) {
    updateEncoder();
    updateEncoderButton();
  }

  // ---- Handle IMU data ----
  if (millis() - lastImuProbeMs >= 250) {
    lastImuProbeMs = millis();
    imuConnected = mpu.testConnection();

    if (imuConnected) {
      status = status | kImuBit; // Set the kImuBit
    } else {
      // Clear the bit. Some 2's complement-ish. Don't wanna think about it too deep right now
      status &= ~kImuBit;
      rollDeg = 0.0f;
      pitchDeg = 0.0f;
      yawDeg = 0.0f;
    }
  }
  if (imuConnected) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gyroXDegPerSec = (gx - gyroXBias) / 131.0f;
    gyroYDegPerSec = (gy - gyroYBias) / 131.0f;
    gyroZDegPerSec = (gz - gyroZBias) / 131.0f;

    accelRollDeg = atan2(static_cast<float>(ay), static_cast<float>(az)) * 180.0f / PI;
    accelPitchDeg = atan2(
      -static_cast<float>(ax),
      sqrt(static_cast<float>(ay) * ay + static_cast<float>(az) * az)
    ) * 180.0f / PI;

    rollDeg = 0.98f * (rollDeg + gyroXDegPerSec * dt) + 0.02f * accelRollDeg;
    pitchDeg = 0.98f * (pitchDeg + gyroYDegPerSec * dt) + 0.02f * accelPitchDeg;
    yawDeg += gyroZDegPerSec * dt;
  }

  // Check if it's time to send a payload
  // Look at docs/packet_dd_add.md for framed payload info.
  if (millis() - lastPrintMs >= 25) {
    lastPrintMs = millis();
    if (kEnableSerialTextDebug) {
      serial_imu_debug_csv(
        status,
        rollDeg,
        pitchDeg,
        yawDeg,
        encoderCount,
        gx,
        gy,
        gz,
        gyroXDegPerSec,
        gyroYDegPerSec,
        gyroZDegPerSec,
        accelRollDeg,
        accelPitchDeg
      );
    } else {
      serial_binary(static_cast<uint8_t>(status), rollDeg, pitchDeg, yawDeg, static_cast<int32_t>(encoderCount));
    }
  }
}
