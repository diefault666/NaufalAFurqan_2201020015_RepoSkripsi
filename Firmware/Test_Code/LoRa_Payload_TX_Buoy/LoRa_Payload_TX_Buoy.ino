#include <SPI.h>
#include <LoRa.h>

#define DEBUG_SERIAL Serial0

#define LORA_SCK  11
#define LORA_MISO 13
#define LORA_MOSI 12
#define LORA_SS   10
#define LORA_RST  8
#define LORA_DIO0 7

#define LED_BIRU_BUOY 1

const long LORA_FREQ = 433E6;

unsigned long packetCounter = 0;
unsigned long lastSendMillis = 0;
const unsigned long SEND_INTERVAL_MS = 2000;

uint16_t simpleCRC(String data) {
  uint16_t crc = 0;

  for (int i = 0; i < data.length(); i++) {
    crc += data[i];
  }

  return crc;
}

String getStatusFromHs(float hs) {
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

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);
  digitalWrite(LED_BIRU_BUOY, LOW);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - D02 LoRa Payload TX");
  DEBUG_SERIAL.println("BUOY UNIT / TRANSMITTER");
  DEBUG_SERIAL.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  DEBUG_SERIAL.println("====================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    DEBUG_SERIAL.println("ERROR: LoRa init failed.");
    while (true) {
      digitalWrite(LED_BIRU_BUOY, HIGH);
      delay(100);
      digitalWrite(LED_BIRU_BUOY, LOW);
      delay(100);
    }
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.enableCrc();
  LoRa.setTxPower(17);

  DEBUG_SERIAL.println("LoRa initialized successfully.");
  DEBUG_SERIAL.println("Start sending smart buoy payload...");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSendMillis >= SEND_INTERVAL_MS) {
    lastSendMillis = now;

    unsigned long timestamp = millis();

    // Data simulasi agar status berubah saat diuji
    float heave = 0.10 * sin(packetCounter * 0.3);
    float hs;

    if (packetCounter < 5) {
      hs = 0.20;       // TENANG
    } else if (packetCounter < 10) {
      hs = 0.80;       // RENDAH
    } else if (packetCounter < 15) {
      hs = 1.60;       // SEDANG
    } else {
      hs = 0.30;       // kembali TENANG
    }

    float tp = 2.80;
    float roll = 1.20;
    float pitch = -3.50;
    String status = getStatusFromHs(hs);

    String dataNoCRC = "";
    dataNoCRC += timestamp;
    dataNoCRC += ",";
    dataNoCRC += String(heave, 3);
    dataNoCRC += ",";
    dataNoCRC += String(hs, 3);
    dataNoCRC += ",";
    dataNoCRC += String(tp, 2);
    dataNoCRC += ",";
    dataNoCRC += String(roll, 2);
    dataNoCRC += ",";
    dataNoCRC += String(pitch, 2);
    dataNoCRC += ",";
    dataNoCRC += status;

    uint16_t crc = simpleCRC(dataNoCRC);

    String payload = dataNoCRC + "," + String(crc);

    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    DEBUG_SERIAL.print("Sent payload: ");
    DEBUG_SERIAL.println(payload);

    digitalWrite(LED_BIRU_BUOY, HIGH);
    delay(80);
    digitalWrite(LED_BIRU_BUOY, LOW);

    packetCounter++;

    if (packetCounter > 19) {
      packetCounter = 0;
    }
  }
}