#define LED_BIRU_BUOY 1

unsigned long counter = 0;

void setup() {
  pinMode(LED_BIRU_BUOY, OUTPUT);

  Serial0.begin(115200);
  delay(5000);

  Serial0.println();
  Serial0.println("====================================");
  Serial0.println("SMART BUOY - TEST ESP32-S3");
  Serial0.println("Uji Blink LED Biru Buoy");
  Serial0.println("====================================");
}

void loop() {
  digitalWrite(LED_BIRU_BUOY, HIGH);
  Serial0.print("Counter: ");
  Serial0.print(counter);
  Serial0.println(" | LED: ON");
  delay(1000);

  digitalWrite(LED_BIRU_BUOY, LOW);
  Serial0.print("Counter: ");
  Serial0.print(counter);
  Serial0.println(" | LED: OFF");
  delay(1000);

  counter++;
}