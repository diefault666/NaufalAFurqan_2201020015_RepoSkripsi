#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>

#define DEBUG_SERIAL Serial0

#define I2C_SDA 17
#define I2C_SCL 18
#define LED_BIRU_BUOY 1

#define MPU6050_ADDR 0x68

// Register MPU6050
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

// ====== NILAI KALIBRASI ======
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

bool read16Safe(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }

  uint8_t bytesReceived = Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);

  if (bytesReceived < 2) {
    return false;
  }

  value = (Wire.read() << 8) | Wire.read();
  return true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setupMPU6050() {
  // Bangunkan MPU6050
  writeRegister(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Accel range ±2g
  writeRegister(REG_ACCEL_CONFIG, 0x00);

  // Gyro range ±250 deg/s
  writeRegister(REG_GYRO_CONFIG, 0x00);

  delay(100);
}

bool readMPU6050(
  int16_t &ax_raw,
  int16_t &ay_raw,
  int16_t &az_raw,
  int16_t &gx_raw,
  int16_t &gy_raw,
  int16_t &gz_raw
) {
  bool ok = true;

  ok &= read16Safe(REG_ACCEL_XOUT_H, ax_raw);
  ok &= read16Safe(REG_ACCEL_XOUT_H + 2, ay_raw);
  ok &= read16Safe(REG_ACCEL_XOUT_H + 4, az_raw);

  // Skip temperature register
  ok &= read16Safe(REG_ACCEL_XOUT_H + 8, gx_raw);
  ok &= read16Safe(REG_ACCEL_XOUT_H + 10, gy_raw);
  ok &= read16Safe(REG_ACCEL_XOUT_H + 12, gz_raw);

  return ok;
}

void blinkLedActivity() {
  unsigned long now = millis();

  if (now - lastLedMillis >= 500) {
    lastLedMillis = now;
    ledState = !ledState;
    digitalWrite(LED_BIRU_BUOY, ledState ? HIGH : LOW);
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

  // Tolak data palsu jika semua raw bernilai 0
  if (
    ax_raw == 0 && ay_raw == 0 && az_raw == 0 &&
    gx_raw == 0 && gy_raw == 0 && gz_raw == 0
  ) {
    return false;
  }

  return true;
}

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - TEST C05");
  DEBUG_SERIAL.println("Gravity Correction + Heave Acceleration");
  DEBUG_SERIAL.println("SDA: GPIO17 | SCL: GPIO18");
  DEBUG_SERIAL.println("LED biru buoy: GPIO1");
  DEBUG_SERIAL.println("Debug: Serial0");
  DEBUG_SERIAL.println("Sample rate: 50 Hz");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  setupMPU6050();
  filter.begin(SAMPLE_RATE_HZ);

  DEBUG_SERIAL.println("MPU6050 initialized.");
  DEBUG_SERIAL.println("Madgwick initialized.");
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println("sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,acc_z_world_g,heave_acc_g,heave_acc_ms2");
}

void loop() {
  blinkLedActivity();

  unsigned long nowMicros = micros();

  if (nowMicros - lastSampleMicros < SAMPLE_INTERVAL_US) {
    return;
  }

  lastSampleMicros = nowMicros;

  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  bool ok = readMPU6050Burst(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  if (!ok) {
    DEBUG_SERIAL.println("ERROR: MPU6050 read failed. Cek kabel SDA/SCL/VCC/GND.");
    return;
  }

  // Konversi accelerometer hasil kalibrasi
  float ax_g = (ax_raw - accel_bias_x) / accel_lsb_per_g_x;
  float ay_g = (ay_raw - accel_bias_y) / accel_lsb_per_g_y;
  float az_g = (az_raw - accel_bias_z) / accel_lsb_per_g_z;

  float acc_norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  if (acc_norm < 0.5 || acc_norm > 2.5) {
    DEBUG_SERIAL.println("ERROR: Data akselerometer tidak valid. Sampel dilewati.");
    return;
  }

  // Konversi gyro hasil kalibrasi
  float gx_dps = (gx_raw - gyro_bias_x) / GYRO_LSB_PER_DPS;
  float gy_dps = (gy_raw - gyro_bias_y) / GYRO_LSB_PER_DPS;
  float gz_dps = (gz_raw - gyro_bias_z) / GYRO_LSB_PER_DPS;

  // Update Madgwick
  filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);

  float roll_deg = filter.getRoll();
  float pitch_deg = filter.getPitch();

  // Konversi derajat ke radian
  float roll_rad = roll_deg * PI / 180.0;
  float pitch_rad = pitch_deg * PI / 180.0;

  /*
    Koreksi gravitasi sederhana:
    Estimasi akselerasi vertikal global dari akselerasi sensor.
    Rumus ini cocok untuk tahap awal pengujian roll-pitch.
    
    acc_z_world_g mendekati 1 saat sensor diam.
  */
  float acc_z_world_g =
      (-ax_g * sin(pitch_rad)) +
      (ay_g * sin(roll_rad) * cos(pitch_rad)) +
      (az_g * cos(roll_rad) * cos(pitch_rad));

  // Hilangkan gravitasi
  float heave_acc_g = acc_z_world_g - 1.0;
  float heave_acc_ms2 = heave_acc_g * GRAVITY;

  DEBUG_SERIAL.print(sampleCounter); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ax_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gx_dps, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gy_dps, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gz_dps, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(roll_deg, 2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pitch_deg, 2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(acc_z_world_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heave_acc_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(heave_acc_ms2, 4);

  sampleCounter++;
}