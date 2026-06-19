#include <Wire.h>

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

const float GRAVITY = 9.80665;

// ====== NILAI KALIBRASI HASIL C02 ======
float accel_bias_x = 458.48;
float accel_bias_y = -91.82;
float accel_bias_z = 1129.05;

float accel_lsb_per_g_x = 16265.48;
float accel_lsb_per_g_y = 16308.15;
float accel_lsb_per_g_z = 16531.99;

float gyro_bias_x = -233.87;
float gyro_bias_y = -60.78;
float gyro_bias_z = -256.41;

// Gyro range ±250 dps = 131 LSB/(deg/s)
const float GYRO_LSB_PER_DPS = 131.0;

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);

  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
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

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - TEST C03");
  DEBUG_SERIAL.println("Read Calibrated GY-87 / MPU6050");
  DEBUG_SERIAL.println("SDA: GPIO17 | SCL: GPIO18");
  DEBUG_SERIAL.println("LED biru buoy: GPIO1");
  DEBUG_SERIAL.println("Debug: Serial0");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  setupMPU6050();

  DEBUG_SERIAL.println("MPU6050 initialized.");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax_g_raw,ay_g_raw,az_g_raw,ax_g_cal,ay_g_cal,az_g_cal,ax_ms2_cal,ay_ms2_cal,az_ms2_cal,gx_dps_cal,gy_dps_cal,gz_dps_cal");
}

void loop() {
  int16_t ax_raw = read16(REG_ACCEL_XOUT_H);
  int16_t ay_raw = read16(REG_ACCEL_XOUT_H + 2);
  int16_t az_raw = read16(REG_ACCEL_XOUT_H + 4);

  int16_t gx_raw = read16(REG_ACCEL_XOUT_H + 8);
  int16_t gy_raw = read16(REG_ACCEL_XOUT_H + 10);
  int16_t gz_raw = read16(REG_ACCEL_XOUT_H + 12);

  // Raw conversion memakai skala ideal MPU6050 ±2g
  float ax_g_raw = ax_raw / 16384.0;
  float ay_g_raw = ay_raw / 16384.0;
  float az_g_raw = az_raw / 16384.0;

  // Calibrated accelerometer
  float ax_g_cal = (ax_raw - accel_bias_x) / accel_lsb_per_g_x;
  float ay_g_cal = (ay_raw - accel_bias_y) / accel_lsb_per_g_y;
  float az_g_cal = (az_raw - accel_bias_z) / accel_lsb_per_g_z;

  float ax_ms2_cal = ax_g_cal * GRAVITY;
  float ay_ms2_cal = ay_g_cal * GRAVITY;
  float az_ms2_cal = az_g_cal * GRAVITY;

  // Calibrated gyroscope
  float gx_dps_cal = (gx_raw - gyro_bias_x) / GYRO_LSB_PER_DPS;
  float gy_dps_cal = (gy_raw - gyro_bias_y) / GYRO_LSB_PER_DPS;
  float gz_dps_cal = (gz_raw - gyro_bias_z) / GYRO_LSB_PER_DPS;

  DEBUG_SERIAL.print(ax_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gx_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gy_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gz_raw); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(ax_g_raw, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_g_raw, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_g_raw, 4); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(ax_g_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_g_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_g_cal, 4); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(ax_ms2_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_ms2_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_ms2_cal, 4); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(gx_dps_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gy_dps_cal, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(gz_dps_cal, 4);

  // LED biru buoy berkedip sebagai indikator alat aktif
  digitalWrite(LED_BIRU_BUOY, HIGH);
  delay(50);
  digitalWrite(LED_BIRU_BUOY, LOW);

  delay(500);
}