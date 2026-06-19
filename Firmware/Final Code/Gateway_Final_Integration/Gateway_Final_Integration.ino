#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_SDA 17
#define I2C_SCL 18

#define LORA_SCK   11
#define LORA_MOSI  12
#define LORA_MISO  13
#define LORA_SS    10
#define LORA_RST   8
#define LORA_DIO0  7

#define LED_BIRU_GATEWAY 1
#define LED_HIJAU        4
#define LED_KUNING       5
#define LED_MERAH        6
#define BUZZER_PIN       15

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const unsigned long TIMEOUT_MS = 5000;
unsigned long lastReceiveMillis = 0;
bool dataValid = false;
bool timeoutPrinted = false;

struct SmartBuoyData {
  String timestamp;
  float heave;
  float hs;
  float tp;
  float roll;
  float pitch;
  String status;
  uint16_t crc;
  int rssi;
  float snr;
};

SmartBuoyData latestData;

uint16_t calculateCRC(String data) {
  uint16_t crc = 0;
  for (int i = 0; i < data.length(); i++) {
    crc += (uint8_t)data[i];
  }
  return crc;
}

int splitCSV(String input, String parts[], int maxParts) {
  int count = 0;
  int start = 0;

  for (int i = 0; i < input.length(); i++) {
    if (input.charAt(i) == ',') {
      if (count < maxParts) {
        parts[count++] = input.substring(start, i);
      }
      start = i + 1;
    }
  }

  if (count < maxParts) {
    parts[count++] = input.substring(start);
  }

  return count;
}

bool parsePayload(String payload, SmartBuoyData &data) {
  payload.trim();

  String parts[8];
  int n = splitCSV(payload, parts, 8);

  if (n != 8) {
    Serial.print("Packet error: jumlah field = ");
    Serial.println(n);
    return false;
  }

  String dataNoCRC = parts[0] + "," + parts[1] + "," + parts[2] + "," + parts[3] + "," +
                     parts[4] + "," + parts[5] + "," + parts[6];

  uint16_t calculatedCRC = calculateCRC(dataNoCRC);
  uint16_t receivedCRC = (uint16_t)parts[7].toInt();

  if (calculatedCRC != receivedCRC) {
    Serial.println("CRC ERROR!");
    Serial.print("Calculated CRC: ");
    Serial.println(calculatedCRC);
    Serial.print("Received CRC  : ");
    Serial.println(receivedCRC);
    return false;
  }

  data.timestamp = parts[0];
  data.heave = parts[1].toFloat();
  data.hs = parts[2].toFloat();
  data.tp = parts[3].toFloat();
  data.roll = parts[4].toFloat();
  data.pitch = parts[5].toFloat();
  data.status = parts[6];
  data.crc = receivedCRC;

  return true;
}

// ================= LED + BUZZER =================
void allStatusOff() {
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void updateStatusOutput(String status) {
  allStatusOff();

  if (status == "TENANG") {
    digitalWrite(LED_HIJAU, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (status == "RENDAH") {
    digitalWrite(LED_KUNING, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (status == "SEDANG") {
    digitalWrite(LED_KUNING, HIGH);

    // Buzzer pelan
    if ((millis() / 500) % 2 == 0) digitalWrite(BUZZER_PIN, HIGH);
    else digitalWrite(BUZZER_PIN, LOW);
  }
  else if (status == "TINGGI" || status == "SANGAT_TINGGI" || status == "EKSTREM") {
    digitalWrite(LED_MERAH, HIGH);

    // Buzzer cepat
    if ((millis() / 150) % 2 == 0) digitalWrite(BUZZER_PIN, HIGH);
    else digitalWrite(BUZZER_PIN, LOW);
  }
  else {
    allStatusOff();
  }
}

void blinkBlueLedOnReceive() {
  digitalWrite(LED_BIRU_GATEWAY, HIGH);
  delay(50);
  digitalWrite(LED_BIRU_GATEWAY, LOW);
}

// ================= OLED =================
void showWaitingOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("SMART BUOY - RX");

  display.setCursor(0, 16);
  display.println("WAITING DATA...");

  display.setCursor(0, 32);
  display.println("LoRa Timeout");
  display.println("LED biru ON");

  display.display();
}

void updateOLED(SmartBuoyData data) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("SMART BUOY - RX");

  display.setCursor(0, 11);
  display.print("Hs: ");
  display.print(data.hs, 3);
  display.println(" m");

  display.setCursor(0, 22);
  display.print("Heave: ");
  display.print(data.heave, 3);
  display.println(" m");

  display.setCursor(0, 33);
  display.print("St: ");
  display.println(data.status);

  display.setCursor(0, 44);
  display.print("R:");
  display.print(data.rssi);
  display.print(" S:");
  display.print(data.snr, 1);

  display.setCursor(0, 55);
  display.print("Roll:");
  display.print(data.roll, 1);
  display.print(" P:");
  display.print(data.pitch, 1);

  display.display();
}

// ================= SERIAL PRINT =================
void printValidData(SmartBuoyData data) {
  Serial.println("---- VALID SMART BUOY DATA ----");
  Serial.print("Timestamp : ");
  Serial.println(data.timestamp);
  Serial.print("Heave     : ");
  Serial.print(data.heave, 3);
  Serial.println(" m");
  Serial.print("Hs        : ");
  Serial.print(data.hs, 3);
  Serial.println(" m");
  Serial.print("Tp        : ");
  Serial.print(data.tp, 2);
  Serial.println(" s");
  Serial.print("Roll      : ");
  Serial.print(data.roll, 2);
  Serial.println(" deg");
  Serial.print("Pitch     : ");
  Serial.print(data.pitch, 2);
  Serial.println(" deg");
  Serial.print("Status    : ");
  Serial.println(data.status);
  Serial.print("CRC       : ");
  Serial.println(data.crc);
  Serial.println();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("SMART BUOY - E01 Final Integration");
  Serial.println("GATEWAY UNIT / RECEIVER");
  Serial.println("LoRa DIO0 tetap GPIO7");
  Serial.println("Buzzer gateway GPIO15");
  Serial.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  Serial.println("====================================");

  pinMode(LED_BIRU_GATEWAY, OUTPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_BIRU_GATEWAY, HIGH);
  allStatusOff();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED initialization failed.");
  } else {
    Serial.println("OLED initialized.");
    showWaitingOLED();
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("ERROR: LoRa initialization failed!");
    while (true) {
      digitalWrite(LED_BIRU_GATEWAY, !digitalRead(LED_BIRU_GATEWAY));
      delay(150);
    }
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();

  Serial.println("LoRa initialized successfully.");
  Serial.println("Waiting for smart buoy payload...");

  lastReceiveMillis = millis();
}

// ================= LOOP =================
void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String payload = "";

    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    Serial.print("Raw Payload: ");
    Serial.println(payload);
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.println(snr);

    SmartBuoyData receivedData;
    bool valid = parsePayload(payload, receivedData);

    if (valid) {
      receivedData.rssi = rssi;
      receivedData.snr = snr;
      latestData = receivedData;

      dataValid = true;
      timeoutPrinted = false;
      lastReceiveMillis = millis();

      blinkBlueLedOnReceive();
      updateStatusOutput(latestData.status);
      updateOLED(latestData);
      printValidData(latestData);
    } else {
      Serial.println("Packet discarded.");
      Serial.println();
    }
  }

  // Timeout handling
  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    dataValid = false;
    allStatusOff();

    // LED biru menyala terus saat waiting / timeout
    digitalWrite(LED_BIRU_GATEWAY, HIGH);

    showWaitingOLED();

    if (!timeoutPrinted) {
      Serial.println("TIMEOUT: Data LoRa tidak diterima lebih dari 5 detik.");
      Serial.println("Gateway masuk mode WAITING.");
      Serial.println("LED biru menyala terus.");
      Serial.println();
      timeoutPrinted = true;
    }
  } else {
    if (dataValid) {
      updateStatusOutput(latestData.status);
    }
  }
}