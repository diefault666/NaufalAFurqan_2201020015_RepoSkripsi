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

void setup() {
  pinMode(LED_BIRU_GATEWAY, OUTPUT);
  digitalWrite(LED_BIRU_GATEWAY, HIGH);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - D01 LoRa RX");
  DEBUG_SERIAL.println("GATEWAY UNIT / RECEIVER");
  DEBUG_SERIAL.println("Frequency: 433 MHz");
  DEBUG_SERIAL.println("====================================");

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

  DEBUG_SERIAL.println("LoRa initialized successfully.");
  DEBUG_SERIAL.println("Waiting for packets...");
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

    DEBUG_SERIAL.print("Received: ");
    DEBUG_SERIAL.print(payload);
    DEBUG_SERIAL.print(" | RSSI: ");
    DEBUG_SERIAL.print(rssi);
    DEBUG_SERIAL.print(" dBm | SNR: ");
    DEBUG_SERIAL.println(snr);

    digitalWrite(LED_BIRU_GATEWAY, LOW);
    delay(80);
    digitalWrite(LED_BIRU_GATEWAY, HIGH);
  }

  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    digitalWrite(LED_BIRU_GATEWAY, HIGH);
  }
}