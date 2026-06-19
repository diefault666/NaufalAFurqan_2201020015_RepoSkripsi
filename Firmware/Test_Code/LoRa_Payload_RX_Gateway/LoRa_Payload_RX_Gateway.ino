#include <SPI.h>
#include <LoRa.h>

#define DEBUG_SERIAL Serial0

#define LORA_SCK  11
#define LORA_MISO 13
#define LORA_MOSI 12
#define LORA_SS   10
#define LORA_RST  8
#define LORA_DIO0 7

#define LED_BIRU_GATEWAY 1

const long LORA_FREQ = 433E6;

unsigned long lastReceiveMillis = 0;
const unsigned long TIMEOUT_MS = 5000;

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

  unsigned long timestamp = timestampStr.toInt();
  float heave = heaveStr.toFloat();
  float hs = hsStr.toFloat();
  float tp = tpStr.toFloat();
  float roll = rollStr.toFloat();
  float pitch = pitchStr.toFloat();
  String status = statusStr;

  DEBUG_SERIAL.println("---- VALID SMART BUOY DATA ----");
  DEBUG_SERIAL.print("Timestamp : "); DEBUG_SERIAL.println(timestamp);
  DEBUG_SERIAL.print("Heave     : "); DEBUG_SERIAL.print(heave, 3); DEBUG_SERIAL.println(" m");
  DEBUG_SERIAL.print("Hs        : "); DEBUG_SERIAL.print(hs, 3); DEBUG_SERIAL.println(" m");
  DEBUG_SERIAL.print("Tp        : "); DEBUG_SERIAL.print(tp, 2); DEBUG_SERIAL.println(" s");
  DEBUG_SERIAL.print("Roll      : "); DEBUG_SERIAL.print(roll, 2); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Pitch     : "); DEBUG_SERIAL.print(pitch, 2); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Status    : "); DEBUG_SERIAL.println(status);
  DEBUG_SERIAL.print("CRC       : "); DEBUG_SERIAL.println(crcReceived);

  return true;
}

void setup() {
  pinMode(LED_BIRU_GATEWAY, OUTPUT);
  digitalWrite(LED_BIRU_GATEWAY, HIGH);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - D02 LoRa Payload RX");
  DEBUG_SERIAL.println("GATEWAY UNIT / RECEIVER");
  DEBUG_SERIAL.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  DEBUG_SERIAL.println("====================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    DEBUG_SERIAL.println("ERROR: LoRa init failed.");
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

  DEBUG_SERIAL.println("LoRa initialized successfully.");
  DEBUG_SERIAL.println("Waiting for smart buoy payload...");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String payload = "";

    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    lastReceiveMillis = millis();

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Raw Payload: ");
    DEBUG_SERIAL.println(payload);
    DEBUG_SERIAL.print("RSSI: ");
    DEBUG_SERIAL.print(rssi);
    DEBUG_SERIAL.print(" dBm | SNR: ");
    DEBUG_SERIAL.println(snr);

    bool valid = parsePayload(payload);

    if (valid) {
      digitalWrite(LED_BIRU_GATEWAY, LOW);
      delay(80);
      digitalWrite(LED_BIRU_GATEWAY, HIGH);
    }
  }

  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    digitalWrite(LED_BIRU_GATEWAY, HIGH);
  }
}