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

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);
  digitalWrite(LED_BIRU_BUOY, LOW);

  DEBUG_SERIAL.begin(115200);
  delay(3000);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SMART BUOY - D01 LoRa TX");
  DEBUG_SERIAL.println("BUOY UNIT / TRANSMITTER");
  DEBUG_SERIAL.println("Frequency: 433 MHz");
  DEBUG_SERIAL.println("====================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    DEBUG_SERIAL.println("ERROR: LoRa init failed. Cek wiring, power 3.3V, dan antena.");
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
  DEBUG_SERIAL.println("Start sending packets...");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSendMillis >= SEND_INTERVAL_MS) {
    lastSendMillis = now;

    String payload = "HELLO_SMART_BUOY,";
    payload += packetCounter;
    payload += ",";
    payload += millis();

    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    DEBUG_SERIAL.print("Sent packet: ");
    DEBUG_SERIAL.println(payload);

    digitalWrite(LED_BIRU_BUOY, HIGH);
    delay(80);
    digitalWrite(LED_BIRU_BUOY, LOW);

    packetCounter++;
  }
}