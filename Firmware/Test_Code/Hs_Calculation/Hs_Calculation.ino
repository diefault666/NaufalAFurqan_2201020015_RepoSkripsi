#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>

#define DEBUG_SERIAL Serial0

#define I2C_SDA 17
#define I2C_SCL 18
#define LED_BIRU_BUOY 1

#define MPU6050_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

// ====== NILAI KALIBRASI HASIL C02/C03 ======
float accel_bias_x = 458.48;
float accel_bias_y = -91.82;
float accel_bias_z = 1129.05;

float accel_lsb_per_g_x = 16265.48;
float accel_lsb_per_g_y = 16308.15;
float accel_lsb_per_g_z = 16531.99;

float gyro_bias_x = -233.87;
float gyro_bias_y = -60.78;
float gyro_bias_z = -256.41;

const float GYRO_LSB_PER_DPS = 131.0;
const float GRAVITY = 9.80665;

// Sampling
const float SAMPLE_RATE_HZ = 50.0;
const unsigned long SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE_HZ;
unsigned long lastSampleMicros = 0;

// Madgwick
Madgwick filter;

// LED
unsigned long lastLedMillis = 0;
bool ledState = false;

// Counter
unsigned long sampleCounter = 0;

// Integrasi heave
float heave_velocity_ms = 0.0;
float heave_m = 0.0;
float heave_acc_filtered = 0.0;

// Parameter tuning C-6B/C-7
const float ACC_DEADBAND_MS2 = 0.15;
const float ACC_LPF_ALPHA = 0.15;
const float VELOCITY_DAMPING = 0.985;
const float HEAVE_DAMPING = 0.995;
const float STILL_ACC_THRESHOLD = 0.18;
const float STILL_GYRO_THRESHOLD = 1.5;
int stillCounter = 0;

// ====== KALMAN SEDERHANA UNTUK HEAVE ======
float heave_kalman = 0.0;
float kalman_error_est = 1.0;
float kalman_error_mea = 0.05;
float kalman_q = 0.001;

float kalmanUpdate(float measurement) {
  kalman_error_est = kalman_error_est + kalman_q;
  float kalman_gain = kalman_error_est / (kalman_error_est + kalman_error_mea);
  heave_kalman = heave_kalman + kalman_gain * (measurement - heave_kalman);
  kalman_error_est = (1.0 - kalman_gain) * kalman_error_est;
  return heave_kalman;
}

// ====== WINDOW UNTUK HS ======
const int WINDOW_SIZE = 500;  // 500 sampel = 10 detik pada 50 Hz
float heaveWindow[WINDOW_SIZE];
int windowIndex = 0;
int windowCount = 0;

float currentHs = 0.0;

void addHeaveToWindow(float value) {
  heaveWindow[windowIndex] = value;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  if (windowCount < WINDOW_SIZE) {
    windowCount++;
  }
}

float calculateMean() {
  if (windowCount == 0) return 0.0;

  float sum = 0.0;

  for (int i = 0; i < windowCount; i++) {
    sum += heaveWindow[i];
  }

  return sum / windowCount;
}

float calculateStdDev(float meanValue) {
  if (windowCount < 2) return 0.0;

  float sumSq = 0.0;

  for (int i = 0; i < windowCount; i++) {
    float diff = heaveWindow[i] - meanValue;
    sumSq += diff * diff;
  }

  float variance = sumSq / (windowCount - 1);
  return sqrt(variance);
}

float calculateHs() {
  if (windowCount < 100) {
    return 0.0;
  }

  float meanValue = calculateMean();
  float stdDev = calculateStdDev(meanValue);

  // Pendekatan statistik awal: Hs ≈ 4 × standar deviasi heave
  float hs = 4.0 * stdDev;

  // Hindari nilai negatif atau sangat kecil akibat noise
  if (hs < 0.001) {
    hs = 0.0;
  }

  return hs;
}

String getWaveStatus(float hs) {
  if (hs < 0.5) {
    return "TENANG";
  } else if (hs < 1.25) {
    return "RENDAH";
  } else if (hs < 2.5) {
    return "SEDANG";
  } else if (hs < 4.0) {
    return "TINGGI";
  } else if (hs < 6.0) {
    return "SANGAT_TINGGI";
  } else {
    return "EKSTREM";
  }
}

bool readMPU6050Burst(
  int16_t &ax_raw,
  int16_t &ay_raw,
  int16_t &az_raw,
  int16_t &gx_raw,
  int16_t &gy_raw,
  int16_t &gz_raw
) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  uint8_t error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }

  uint8_t bytesReceived = Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

  if (bytesReceived < 14) {
    return false;
  }

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();

  // Lewati temperature register
  Wire.read();
  Wire.read();

  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();

  if (
    ax_raw == 0 && ay_raw == 0 && az_raw == 0 &&
    gx_raw == 0 && gy_raw == 0 && gz_raw == 0
  ) {
    return false;
  }

  return true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setupMPU6050() {
  writeRegister(REG_PWR_MGMT_1, 0x00);
  delay(100);

  writeRegister(REG_ACCEL_CONFIG, 0x00); // Accel ±2g
  writeRegister(REG_GYRO_CONFIG, 0x00);  // Gyro ±250 dps

  delay(100);
}

void blinkLedActivity() {
  unsigned long now = millis();

  if (now - lastLedMillis >= 500) {
    lastLedMillis = now;
    ledState = !ledState;
    digitalWrite(LED_BIRU_BUOY, ledState ? HIGH : LOW);
  }
}

void resetHeaveIfStill(float heave_acc_ms2, float gx_dps, float gy_dps, float gz_dps) {
  float gyro_abs_sum = abs(gx_dps) + abs(gy_dps) + abs(gz_dps);

  if (abs(heave_acc_ms2) < STILL_ACC_THRESHOLD && gyro_abs_sum < STILL_GYRO_THRESHOLD) {
    stillCounter++;
  } else {
    stillCounter = 0;
  }

  if (stillCounter > 50) {
    heave_velocity_ms = 0.0;
    heave_acc_filtered = 0.0;
    heave_m *= 0.95;
    heave_kalman *= 0.95;
  }
}

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - TEST C08");
  DEBUG_SERIAL.println("Hs Calculation from Heave Filtered");
  DEBUG_SERIAL.println("SDA: GPIO17 | SCL: GPIO18");
  DEBUG_SERIAL.println("LED biru buoy: GPIO1");
  DEBUG_SERIAL.println("Debug: Serial0");
  DEBUG_SERIAL.println("Sample rate: 50 Hz");
  DEBUG_SERIAL.println("Window: 500 samples / 10 seconds");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  setupMPU6050();
  filter.begin(SAMPLE_RATE_HZ);

  for (int i = 0; i < WINDOW_SIZE; i++) {
    heaveWindow[i] = 0.0;
  }

  DEBUG_SERIAL.println("MPU6050 initialized.");
  DEBUG_SERIAL.println("Madgwick initialized.");
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println("sample,dt,roll_deg,pitch_deg,heave_acc_ms2,heave_m,heave_filtered_m,Hs_m,Status,windowCount");
}

void loop() {
  blinkLedActivity();

  unsigned long nowMicros = micros();

  if (nowMicros - lastSampleMicros < SAMPLE_INTERVAL_US) {
    return;
  }

  float dt = (nowMicros - lastSampleMicros) / 1000000.0;
  lastSampleMicros = nowMicros;

  if (dt <= 0 || dt > 0.1) {
    dt = 1.0 / SAMPLE_RATE_HZ;
  }

  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  bool ok = readMPU6050Burst(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  if (!ok) {
    DEBUG_SERIAL.println("ERROR: MPU6050 read failed. Sampel dilewati.");
    return;
  }

  float ax_g = (ax_raw - accel_bias_x) / accel_lsb_per_g_x;
  float ay_g = (ay_raw - accel_bias_y) / accel_lsb_per_g_y;
  float az_g = (az_raw - accel_bias_z) / accel_lsb_per_g_z;

  float acc_norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  if (acc_norm < 0.5 || acc_norm > 2.5) {
    DEBUG_SERIAL.println("ERROR: Data akselerometer tidak valid. Sampel dilewati.");
    return;
  }

  float gx_dps = (gx_raw - gyro_bias_x) / GYRO_LSB_PER_DPS;
  float gy_dps = (gy_raw - gyro_bias_y) / GYRO_LSB_PER_DPS;
  float gz_dps = (gz_raw - gyro_bias_z) / GYRO_LSB_PER_DPS;

  filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);

  float roll_deg = filter.getRoll();
  float pitch_deg = filter.getPitch();

  float roll_rad = roll_deg * PI / 180.0;
  float pitch_rad = pitch_deg * PI / 180.0;

  float acc_z_world_g =
      (-ax_g * sin(pitch_rad)) +
      (ay_g * sin(roll_rad) * cos(pitch_rad)) +
      (az_g * cos(roll_rad) * cos(pitch_rad));

  float heave_acc_g = acc_z_world_g - 1.0;
  float heave_acc_ms2 = heave_acc_g * GRAVITY;

  if (abs(heave_acc_ms2) > 5.0) {
    DEBUG_SERIAL.println("ERROR: Outlier heave acceleration. Sampel dilewati.");
    return;
  }

  if (abs(heave_acc_ms2) < ACC_DEADBAND_MS2) {
    heave_acc_ms2 = 0.0;
  }

  heave_acc_filtered =
      (ACC_LPF_ALPHA * heave_acc_ms2) +
      ((1.0 - ACC_LPF_ALPHA) * heave_acc_filtered);

  heave_velocity_ms += heave_acc_filtered * dt;
  heave_velocity_ms *= VELOCITY_DAMPING;

  heave_m += heave_velocity_ms * dt;
  heave_m *= HEAVE_DAMPING;

  resetHeaveIfStill(heave_acc_ms2, gx_dps, gy_dps, gz_dps);

  float heave_filtered_m = kalmanUpdate(heave_m);

  addHeaveToWindow(heave_filtered_m);

  currentHs = calculateHs();

  String status = getWaveStatus(currentHs);

  DEBUG_SERIAL.print(sampleCounter); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(dt, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(roll_deg, 2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pitch_deg, 2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heave_acc_ms2, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heave_m, 5); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heave_filtered_m, 5); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(currentHs, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(status); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(windowCount);

  sampleCounter++;
}