#include <Wire.h>

#define DEBUG_SERIAL Serial0

#define I2C_SDA 17
#define I2C_SCL 18

#define LED_BIRU_BUOY 1

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - TEST C01");
  DEBUG_SERIAL.println("I2C Scanner GY-87 dan LCD I2C");
  DEBUG_SERIAL.println("SDA: GPIO17 | SCL: GPIO18");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); 
}

void loop() {
  byte error, address;
  int deviceCount = 0;

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      DEBUG_SERIAL.print("I2C device ditemukan di alamat 0x");

      if (address < 16) {
        DEBUG_SERIAL.print("0");
      }

      DEBUG_SERIAL.print(address, HEX);

      if (address == 0x68) {
        DEBUG_SERIAL.print("  --> kemungkinan GY-87 / MPU6050");
      } else if (address == 0x27) {
        DEBUG_SERIAL.print("  --> kemungkinan LCD I2C");
      } else if (address == 0x3F) {
        DEBUG_SERIAL.print("  --> kemungkinan LCD I2C alternatif");
      }

      DEBUG_SERIAL.println();
      deviceCount++;
    } else if (error == 4) {
      DEBUG_SERIAL.print("Unknown error di alamat 0x");
      if (address < 16) {
        DEBUG_SERIAL.print("0");
      }
      DEBUG_SERIAL.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    DEBUG_SERIAL.println("Tidak ada perangkat I2C ditemukan.");
    DEBUG_SERIAL.println("Cek kabel SDA, SCL, VCC, dan GND.");
  } else {
    DEBUG_SERIAL.print("Total perangkat I2C ditemukan: ");
    DEBUG_SERIAL.println(deviceCount);
  }

  digitalWrite(LED_BIRU_BUOY, HIGH);
  delay(250);
  digitalWrite(LED_BIRU_BUOY, LOW);
  delay(2750);
}