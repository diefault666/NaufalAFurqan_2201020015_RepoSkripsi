#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Serial
#define DEBUG_SERIAL Serial0

// I2C OLED
#define I2C_SDA 17
#define I2C_SCL 18

#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LoRa SX1278
#define LORA_SCK  11
#define LORA_MISO 13
#define LORA_MOSI 12
#define LORA_SS   10
#define LORA_RST  8
#define LORA_DIO0 7

const long LORA_FREQ = 433E6;

// Output gateway
#define LED_BIRU_GATEWAY 1
#define LED_HIJAU 4
#define LED_KUNING 5
#define LED_MERAH 6
#define BUZZER_PIN 15

// Timeout
unsigned long lastReceiveMillis = 0;
const unsigned long TIMEOUT_MS = 5000;

// Blink status
unsigned long lastStatusBlinkMillis = 0;
bool statusBlinkState = false;

// OLED update
unsigned long lastOledMillis = 0;
const unsigned long OLED_INTERVAL_MS = 500;

// Data terakhir
unsigned long latestTimestamp = 0;
float latestHeave = 0.0;
float latestHs = 0.0;
float latestTp = 0.0;
float latestRoll = 0.0;
float latestPitch = 0.0;
String latestStatus = "WAITING";
int latestRSSI = 0;
float latestSNR = 0.0;
bool dataValid = false;

uint16_t simpleCRC(String data) {
  uint16_t crc = 0;

  for (int i = 0; i < data.length(); i++) {
    crc += data[i];
  }

  return crc;
}

String getField(String data, int index) {
  int fieldStart = 0;
  int fieldEnd = -1;
  int currentIndex = 0;

  for (int i = 0; i <= data.length(); i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      fieldStart = fieldEnd + 1;
      fieldEnd = i;

      if (currentIndex == index) {
        return data.substring(fieldStart, fieldEnd);
      }

      currentIndex++;
    }
  }

  return "";
}

int countFields(String data) {
  int count = 1;

  for (int i = 0; i < data.length(); i++) {
    if (data.charAt(i) == ',') {
      count++;
    }
  }

  return count;
}

void allStatusOff() {
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void updateStatusOutput(String status) {
  unsigned long now = millis();

  allStatusOff();

  if (status == "TENANG") {
    digitalWrite(LED_HIJAU, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }

  else if (status == "RENDAH") {
    if (now - lastStatusBlinkMillis >= 700) {
      lastStatusBlinkMillis = now;
      statusBlinkState = !statusBlinkState;
    }

    digitalWrite(LED_HIJAU, statusBlinkState ? HIGH : LOW);
    digitalWrite(LED_KUNING, statusBlinkState ? LOW : HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }

  else if (status == "SEDANG") {
    digitalWrite(LED_KUNING, HIGH);

    if (now - lastStatusBlinkMillis >= 700) {
      lastStatusBlinkMillis = now;
      statusBlinkState = !statusBlinkState;
    }

    digitalWrite(BUZZER_PIN, statusBlinkState ? HIGH : LOW);
  }

  else if (
    status == "TINGGI" ||
    status == "SANGAT_TINGGI" ||
    status == "EKSTREM"
  ) {
    digitalWrite(LED_MERAH, HIGH);

    if (now - lastStatusBlinkMillis >= 200) {
      lastStatusBlinkMillis = now;
      statusBlinkState = !statusBlinkState;
    }

    digitalWrite(BUZZER_PIN, statusBlinkState ? HIGH : LOW);
  }

  else {
    allStatusOff();
  }
}

void blinkBlueLedOnReceive() {
  digitalWrite(LED_BIRU_GATEWAY, LOW);
  delay(80);
  digitalWrite(LED_BIRU_GATEWAY, HIGH);
}

void updateBlueLedTimeout() {
  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    // Menunggu data / tidak ada data
    digitalWrite(LED_BIRU_GATEWAY, HIGH);
  }
}

bool parsePayload(String payload) {
  int fieldCount = countFields(payload);

  if (fieldCount != 8) {
    DEBUG_SERIAL.print("INVALID FIELD COUNT: ");
    DEBUG_SERIAL.println(fieldCount);
    return false;
  }

  String timestampStr = getField(payload, 0);
  String heaveStr = getField(payload, 1);
  String hsStr = getField(payload, 2);
  String tpStr = getField(payload, 3);
  String rollStr = getField(payload, 4);
  String pitchStr = getField(payload, 5);
  String statusStr = getField(payload, 6);
  String crcStr = getField(payload, 7);

  String dataNoCRC = "";
  dataNoCRC += timestampStr;
  dataNoCRC += ",";
  dataNoCRC += heaveStr;
  dataNoCRC += ",";
  dataNoCRC += hsStr;
  dataNoCRC += ",";
  dataNoCRC += tpStr;
  dataNoCRC += ",";
  dataNoCRC += rollStr;
  dataNoCRC += ",";
  dataNoCRC += pitchStr;
  dataNoCRC += ",";
  dataNoCRC += statusStr;

  uint16_t crcReceived = crcStr.toInt();
  uint16_t crcCalculated = simpleCRC(dataNoCRC);

  if (crcReceived != crcCalculated) {
    DEBUG_SERIAL.print("CRC ERROR | received=");
    DEBUG_SERIAL.print(crcReceived);
    DEBUG_SERIAL.print(" calculated=");
    DEBUG_SERIAL.println(crcCalculated);
    return false;
  }

  latestTimestamp = timestampStr.toInt();
  latestHeave = heaveStr.toFloat();
  latestHs = hsStr.toFloat();
  latestTp = tpStr.toFloat();
  latestRoll = rollStr.toFloat();
  latestPitch = pitchStr.toFloat();
  latestStatus = statusStr;
  dataValid = true;

  DEBUG_SERIAL.println("---- VALID SMART BUOY DATA ----");
  DEBUG_SERIAL.print("Timestamp : "); DEBUG_SERIAL.println(latestTimestamp);
  DEBUG_SERIAL.print("Heave     : "); DEBUG_SERIAL.print(latestHeave, 3); DEBUG_SERIAL.println(" m");
  DEBUG_SERIAL.print("Hs        : "); DEBUG_SERIAL.print(latestHs, 3); DEBUG_SERIAL.println(" m");
  DEBUG_SERIAL.print("Tp        : "); DEBUG_SERIAL.print(latestTp, 2); DEBUG_SERIAL.println(" s");
  DEBUG_SERIAL.print("Roll      : "); DEBUG_SERIAL.print(latestRoll, 2); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Pitch     : "); DEBUG_SERIAL.print(latestPitch, 2); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Status    : "); DEBUG_SERIAL.println(latestStatus);
  DEBUG_SERIAL.print("CRC       : "); DEBUG_SERIAL.println(crcReceived);

  return true;
}

void updateOLED() {
  unsigned long now = millis();

  if (now - lastOledMillis < OLED_INTERVAL_MS) {
    return;
  }

  lastOledMillis = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SMART BUOY GW");

  display.setCursor(88, 0);
  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    display.print("WAIT");
  } else {
    display.print("RX");
  }

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (!dataValid || millis() - lastReceiveMillis > TIMEOUT_MS) {
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.println("Menunggu data LoRa...");
    display.setCursor(0, 34);
    display.println("LED biru ON = WAIT");
    display.setCursor(0, 50);
    display.print("RSSI:");
    display.print(latestRSSI);
    display.print(" SNR:");
    display.print(latestSNR, 1);
  } else {
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print("Hs:");
    display.print(latestHs, 2);

    display.setTextSize(1);
    display.setCursor(105, 22);
    display.print("m");

    display.setCursor(0, 36);
    display.print("Status: ");
    display.println(latestStatus);

    display.setCursor(0, 48);
    display.print("H:");
    display.print(latestHeave, 2);
    display.print(" R:");
    display.print(latestRoll, 0);
    display.print(" P:");
    display.print(latestPitch, 0);

    display.setCursor(0, 57);
    display.print("RSSI:");
    display.print(latestRSSI);
    display.print(" SNR:");
    display.print(latestSNR, 1);
  }

  display.display();
}

void showOledBoot() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SMART BUOY GW");
  display.println("D04 LoRa OLED");
  display.println("Init...");
  display.display();
}

void setup() {
  pinMode(LED_BIRU_GATEWAY, OUTPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_BIRU_GATEWAY, HIGH);
  allStatusOff();

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - D04 Gateway OLED LED Buzzer");
  DEBUG_SERIAL.println("GATEWAY UNIT / RECEIVER");
  DEBUG_SERIAL.println("LoRa DIO0 tetap GPIO7");
  DEBUG_SERIAL.println("Buzzer gateway GPIO15");
  DEBUG_SERIAL.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  DEBUG_SERIAL.println("====================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    DEBUG_SERIAL.println("WARNING: OLED tidak terdeteksi. Cek alamat 0x3C atau wiring.");
  } else {
    DEBUG_SERIAL.println("OLED initialized.");
    showOledBoot();
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    DEBUG_SERIAL.println("ERROR: LoRa init failed. Cek wiring, power 3.3V, dan antena.");
    while (true) {
      digitalWrite(LED_BIRU_GATEWAY, HIGH);
      delay(100);
      digitalWrite(LED_BIRU_GATEWAY, LOW);
      delay(100);
    }
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.enableCrc();

  lastReceiveMillis = millis();

  DEBUG_SERIAL.println("LoRa initialized successfully.");
  DEBUG_SERIAL.println("Waiting for real smart buoy payload...");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String payload = "";

    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    latestRSSI = LoRa.packetRssi();
    latestSNR = LoRa.packetSnr();

    lastReceiveMillis = millis();

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Raw Payload: ");
    DEBUG_SERIAL.println(payload);
    DEBUG_SERIAL.print("RSSI: ");
    DEBUG_SERIAL.print(latestRSSI);
    DEBUG_SERIAL.print(" dBm | SNR: ");
    DEBUG_SERIAL.println(latestSNR);

    bool valid = parsePayload(payload);

    if (valid) {
      blinkBlueLedOnReceive();
      updateStatusOutput(latestStatus);
    }
  }

  updateBlueLedTimeout();

  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    latestStatus = "WAITING";
    allStatusOff();
  } else {
    updateStatusOutput(latestStatus);
  }

  updateOLED();
}