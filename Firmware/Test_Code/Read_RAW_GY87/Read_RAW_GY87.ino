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

// Setting range awal
// Accel ±2g => 16384 LSB/g
// Gyro ±250 deg/s => 131 LSB/(deg/s)
const float ACCEL_SCALE_LSB_PER_G = 16384.0;
const float GYRO_SCALE_LSB_PER_DPS = 131.0;
const float GRAVITY = 9.80665;

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
  // Bangunkan MPU6050 dari sleep mode
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
  DEBUG_SERIAL.println("SMART BUOY - TEST C02");
  DEBUG_SERIAL.println("Read Raw GY-87 / MPU6050");
  DEBUG_SERIAL.println("SDA: GPIO17 | SCL: GPIO18");
  DEBUG_SERIAL.println("Accel range: +-2g");
  DEBUG_SERIAL.println("Gyro range : +-250 deg/s");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  setupMPU6050();

  DEBUG_SERIAL.println("MPU6050 initialized.");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps");
}

void loop() {
  int16_t ax_raw = read16(REG_ACCEL_XOUT_H);
  int16_t ay_raw = read16(REG_ACCEL_XOUT_H + 2);
  int16_t az_raw = read16(REG_ACCEL_XOUT_H + 4);

  int16_t gx_raw = read16(REG_ACCEL_XOUT_H + 8);
  int16_t gy_raw = read16(REG_ACCEL_XOUT_H + 10);
  int16_t gz_raw = read16(REG_ACCEL_XOUT_H + 12);

  float ax_g = ax_raw / ACCEL_SCALE_LSB_PER_G;
  float ay_g = ay_raw / ACCEL_SCALE_LSB_PER_G;
  float az_g = az_raw / ACCEL_SCALE_LSB_PER_G;

  float ax_ms2 = ax_g * GRAVITY;
  float ay_ms2 = ay_g * GRAVITY;
  float az_ms2 = az_g * GRAVITY;

  float gx_dps = gx_raw / GYRO_SCALE_LSB_PER_DPS;
  float gy_dps = gy_raw / GYRO_SCALE_LSB_PER_DPS;
  float gz_dps = gz_raw / GYRO_SCALE_LSB_PER_DPS;

  DEBUG_SERIAL.print(ax_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gx_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gy_raw); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gz_raw); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(ax_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_g, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_g, 4); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(ax_ms2, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(ay_ms2, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(az_ms2, 4); DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.print(gx_dps, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(gy_dps, 4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(gz_dps, 4);

  digitalWrite(LED_BIRU_BUOY, HIGH);
  delay(50);
  digitalWrite(LED_BIRU_BUOY, LOW);

  delay(500);
}