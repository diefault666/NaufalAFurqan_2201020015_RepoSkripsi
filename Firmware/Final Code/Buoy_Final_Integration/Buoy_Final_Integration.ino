#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ================= PIN CONFIG =================
#define I2C_SDA 17
#define I2C_SCL 18

#define LORA_SCK   11
#define LORA_MOSI  12
#define LORA_MISO  13
#define LORA_SS    10
#define LORA_RST   8
#define LORA_DIO0  7

#define LED_BIRU_BUOY 1

// ================= MPU6050 CONFIG =================
#define MPU_ADDR 0x68

// Range MPU6050:
// Accel ±2g  => 16384 LSB/g
// Gyro ±250 => 131 LSB/(deg/s)
const float ACC_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;
const float G = 9.80665;

// Bias accelerometer sementara.
// Satuan = g.
// Jika hasil kalibrasi final sudah ingin dimasukkan, ubah nilai ini.
float accBiasX = 0.0;
float accBiasY = 0.0;
float accBiasZ = 0.0;

// Bias gyro dari hasil kalibrasi sebelumnya.
// Satuan = rad/s
float gyroBiasX = -0.0315;
float gyroBiasY = -0.0065;
float gyroBiasZ = -0.0332;

// ================= TIMING =================
const unsigned long SAMPLE_INTERVAL_MS = 20;   // 50 Hz
const unsigned long TX_INTERVAL_MS = 2000;     // kirim LoRa tiap 2 detik
unsigned long lastSampleMillis = 0;
unsigned long lastTxMillis = 0;

// ================= MADGWICK =================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.08f;

float invSqrt(float x) {
  return 1.0f / sqrtf(x);
}

void madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
  float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
         + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
         + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

float getRollDeg() {
  return atan2f(2.0f * (q0 * q1 + q2 * q3),
                1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
}

float getPitchDeg() {
  float s = 2.0f * (q0 * q2 - q3 * q1);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return asinf(s) * 180.0f / PI;
}

// ================= HEAVE + KALMAN =================
float heaveAccMs2 = 0.0;
float velocity = 0.0;
float heave = 0.0;
float heaveFiltered = 0.0;

float kalmanX = 0.0;
float kalmanP = 1.0;
float kalmanQ = 0.002;
float kalmanR = 0.04;

float kalmanUpdate(float measurement) {
  kalmanP += kalmanQ;
  float K = kalmanP / (kalmanP + kalmanR);
  kalmanX = kalmanX + K * (measurement - kalmanX);
  kalmanP = (1.0 - K) * kalmanP;
  return kalmanX;
}

// ================= HS BUFFER =================
#define HEAVE_BUFFER_SIZE 100
float heaveBuffer[HEAVE_BUFFER_SIZE];
int heaveIndex = 0;
bool bufferFilled = false;

float Hs = 0.0;
float Tp = 2.80;   // sementara konstan untuk payload final

void updateHeaveBuffer(float value) {
  heaveBuffer[heaveIndex] = value;
  heaveIndex++;

  if (heaveIndex >= HEAVE_BUFFER_SIZE) {
    heaveIndex = 0;
    bufferFilled = true;
  }
}

float calculateHs() {
  int n = bufferFilled ? HEAVE_BUFFER_SIZE : heaveIndex;
  if (n < 10) return 0.0;

  float mean = 0.0;
  for (int i = 0; i < n; i++) {
    mean += heaveBuffer[i];
  }
  mean /= n;

  float variance = 0.0;
  for (int i = 0; i < n; i++) {
    float d = heaveBuffer[i] - mean;
    variance += d * d;
  }
  variance /= n;

  float stdDev = sqrtf(variance);
  return 4.0f * stdDev;
}

// ================= STATUS BMKG =================
String getStatusFromHs(float hs) {
  if (hs < 0.5) return "TENANG";
  else if (hs < 1.25) return "RENDAH";
  else if (hs < 2.5) return "SEDANG";
  else if (hs < 4.0) return "TINGGI";
  else if (hs < 6.0) return "SANGAT_TINGGI";
  else return "EKSTREM";
}

// ================= CRC =================
uint16_t calculateCRC(String data) {
  uint16_t crc = 0;
  for (int i = 0; i < data.length(); i++) {
    crc += (uint8_t)data[i];
  }
  return crc;
}

// ================= MPU FUNCTIONS =================
void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool readMPUBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, count);

  if (Wire.available() < count) {
    return false;
  }

  for (int i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }

  return true;
}

bool initMPU6050() {
  writeMPU(0x6B, 0x00); // wake up
  delay(100);

  writeMPU(0x1A, 0x03); // DLPF 44 Hz
  writeMPU(0x1B, 0x00); // gyro ±250 dps
  writeMPU(0x1C, 0x00); // accel ±2g

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, 1);

  if (!Wire.available()) {
    return false;
  }

  uint8_t who = Wire.read();
  return (who == 0x68 || who == 0x70);
}

bool readIMU(float &ax_g, float &ay_g, float &az_g,
             float &gx_rad, float &gy_rad, float &gz_rad) {
  uint8_t rawData[14];

  if (!readMPUBytes(0x3B, 14, rawData)) {
    return false;
  }

  int16_t axRaw = ((int16_t)rawData[0] << 8) | rawData[1];
  int16_t ayRaw = ((int16_t)rawData[2] << 8) | rawData[3];
  int16_t azRaw = ((int16_t)rawData[4] << 8) | rawData[5];

  int16_t gxRaw = ((int16_t)rawData[8] << 8) | rawData[9];
  int16_t gyRaw = ((int16_t)rawData[10] << 8) | rawData[11];
  int16_t gzRaw = ((int16_t)rawData[12] << 8) | rawData[13];

  ax_g = (axRaw / ACC_SCALE) - accBiasX;
  ay_g = (ayRaw / ACC_SCALE) - accBiasY;
  az_g = (azRaw / ACC_SCALE) - accBiasZ;

  float gx_dps = gxRaw / GYRO_SCALE;
  float gy_dps = gyRaw / GYRO_SCALE;
  float gz_dps = gzRaw / GYRO_SCALE;

  gx_rad = gx_dps * PI / 180.0 - gyroBiasX;
  gy_rad = gy_dps * PI / 180.0 - gyroBiasY;
  gz_rad = gz_dps * PI / 180.0 - gyroBiasZ;

  return true;
}

// ================= GRAVITY CORRECTION =================
float getWorldZAccelG(float ax, float ay, float az) {
  // Rotation matrix body ke world, elemen baris Z
  float r31 = 2.0f * (q1 * q3 - q0 * q2);
  float r32 = 2.0f * (q2 * q3 + q0 * q1);
  float r33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

  float worldZ = r31 * ax + r32 * ay + r33 * az;
  return worldZ;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("SMART BUOY - E01 Final Integration");
  Serial.println("BUOY UNIT / TRANSMITTER");
  Serial.println("NO OLED/LCD ON BUOY");
  Serial.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  Serial.println("====================================");

  pinMode(LED_BIRU_BUOY, OUTPUT);
  digitalWrite(LED_BIRU_BUOY, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!initMPU6050()) {
    Serial.println("ERROR: MPU6050/GY-87 not detected!");
    while (true) {
      digitalWrite(LED_BIRU_BUOY, !digitalRead(LED_BIRU_BUOY));
      delay(200);
    }
  }

  Serial.println("MPU6050 initialized.");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("ERROR: LoRa initialization failed!");
    while (true) {
      digitalWrite(LED_BIRU_BUOY, !digitalRead(LED_BIRU_BUOY));
      delay(100);
    }
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setTxPower(17);

  Serial.println("LoRa initialized successfully.");
  Serial.println("Starting final integration loop...");
  Serial.println();
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  if (now - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    float dt = (now - lastSampleMillis) / 1000.0;

    // Hindari dt terlalu besar saat awal boot
    if (dt <= 0.0 || dt > 0.2) {
      dt = SAMPLE_INTERVAL_MS / 1000.0;
    }

    lastSampleMillis = now;

    float ax, ay, az, gx, gy, gz;

    if (readIMU(ax, ay, az, gx, gy, gz)) {
      madgwickUpdateIMU(gx, gy, gz, ax, ay, az, dt);

      float worldZ_g = getWorldZAccelG(ax, ay, az);
      float heaveAccG = worldZ_g - 1.0f;
      heaveAccMs2 = heaveAccG * G;

      // Deadband untuk mengurangi drift saat diam
      if (fabs(heaveAccMs2) < 0.08) {
        heaveAccMs2 = 0.0;
      }

      velocity += heaveAccMs2 * dt;
      heave += velocity * dt;

      // Damping agar integrasi tidak liar
      velocity *= 0.96;
      heave *= 0.995;

      // Pembatas keamanan
      if (fabs(heave) > 3.0) {
        heave = 0.0;
        velocity = 0.0;
      }

      heaveFiltered = kalmanUpdate(heave);
      updateHeaveBuffer(heaveFiltered);
    } else {
      Serial.println("WARNING: MPU6050 read error.");
    }
  }

  if (now - lastTxMillis >= TX_INTERVAL_MS) {
    lastTxMillis = now;

    Hs = calculateHs();
    String status = getStatusFromHs(Hs);
    float roll = getRollDeg();
    float pitch = getPitchDeg();

    String dataNoCRC = String(now) + "," +
                       String(heaveFiltered, 3) + "," +
                       String(Hs, 3) + "," +
                       String(Tp, 2) + "," +
                       String(roll, 2) + "," +
                       String(pitch, 2) + "," +
                       status;

    uint16_t crc = calculateCRC(dataNoCRC);
    String payload = dataNoCRC + "," + String(crc);

    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    // LED biru berkedip saat paket dikirim
    digitalWrite(LED_BIRU_BUOY, HIGH);
    delay(40);
    digitalWrite(LED_BIRU_BUOY, LOW);

    Serial.print("TX Payload: ");
    Serial.println(payload);
    Serial.print("Heave: ");
    Serial.print(heaveFiltered, 3);
    Serial.print(" m | Hs: ");
    Serial.print(Hs, 3);
    Serial.print(" m | Roll: ");
    Serial.print(roll, 2);
    Serial.print(" | Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" | Status: ");
    Serial.println(status);
    Serial.println();
  }
}