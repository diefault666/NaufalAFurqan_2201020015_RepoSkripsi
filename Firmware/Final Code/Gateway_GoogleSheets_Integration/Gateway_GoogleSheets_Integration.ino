#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>

// ================= PIN CONFIG =================
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

const char* WIFI_SSID = "A16 milik Gerda";
const char* WIFI_PASSWORD = "123456788";

String GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbxN6jkz4T63wFFiq8g1CiD62ySFLJyln7BCwu5WTyWQ7ZeXl2zzr6KHmpiWwh9Q7ju2UQ/exec";

const unsigned long SHEETS_SEND_INTERVAL_MS = 10000;
unsigned long lastSheetsSendMillis = 0;

const long GMT_OFFSET_SEC = 8 * 3600;
const int DAYLIGHT_OFFSET_SEC = 0;

const unsigned long TIMEOUT_MS = 5000;
unsigned long lastReceiveMillis = 0;
bool hasLatestData = false;
bool dataOnline = false;
bool timeoutPrinted = false;

// Refresh OLED setiap 1 detik supaya waktu dan umur data ikut berubah
const unsigned long OLED_REFRESH_INTERVAL_MS = 1000;
unsigned long lastOLEDRefreshMillis = 0;

// Status kirim Google Sheets terakhir
bool lastSheetsOK = false;
unsigned long lastSheetsOKMillis = 0;

// ================= DATA STRUCT =================
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

// ================= CRC =================
uint16_t calculateCRC(String data) {
  uint16_t crc = 0;

  for (int i = 0; i < data.length(); i++) {
    crc += (uint8_t)data[i];
  }

  return crc;
}

// ================= HELPER SPLIT =================
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

// ================= PARSE PAYLOAD =================
bool parsePayload(String payload, SmartBuoyData &data) {
  payload.trim();

  String parts[8];
  int n = splitCSV(payload, parts, 8);

  if (n != 8) {
    Serial.print("Packet error: jumlah field = ");
    Serial.println(n);
    return false;
  }

  String dataNoCRC = parts[0] + "," +
                     parts[1] + "," +
                     parts[2] + "," +
                     parts[3] + "," +
                     parts[4] + "," +
                     parts[5] + "," +
                     parts[6];

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

// ================= TIME HELPER =================
String getClockString() {
  struct tm timeinfo;

  if (getLocalTime(&timeinfo, 20)) {
    char buffer[9];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
    return String(buffer);
  }

  // Jika NTP belum tersedia, tampilkan uptime
  unsigned long seconds = millis() / 1000;
  unsigned int hh = seconds / 3600;
  unsigned int mm = (seconds % 3600) / 60;
  unsigned int ss = seconds % 60;

  char buffer[12];
  sprintf(buffer, "%02u:%02u:%02u", hh, mm, ss);
  return String(buffer);
}

String getShortClockString() {
  struct tm timeinfo;

  if (getLocalTime(&timeinfo, 20)) {
    char buffer[6];
    strftime(buffer, sizeof(buffer), "%H:%M", &timeinfo);
    return String(buffer);
  }

  unsigned long seconds = millis() / 1000;
  unsigned int mm = (seconds / 60) % 100;
  unsigned int ss = seconds % 60;

  char buffer[6];
  sprintf(buffer, "%02u:%02u", mm, ss);
  return String(buffer);
}

unsigned long getDataAgeSeconds() {
  if (!hasLatestData) return 0;
  return (millis() - lastReceiveMillis) / 1000;
}

// ================= WIFI FUNCTION =================
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;

  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com");
    Serial.println("NTP time configured.");
  } else {
    Serial.println("WiFi connection failed.");
    Serial.println("Gateway tetap berjalan untuk LoRa dan OLED.");
  }
}

// ================= GOOGLE SHEETS FUNCTION =================
bool sendToGoogleSheets(SmartBuoyData data, bool crcOK) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Google Sheets skipped: WiFi not connected.");
    return false;
  }

  HTTPClient http;

  http.begin(GOOGLE_SCRIPT_URL);

  // Jangan gunakan follow redirect untuk Apps Script ini.
  // http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"timestamp\":\"" + data.timestamp + "\",";
  json += "\"heave\":" + String(data.heave, 3) + ",";
  json += "\"hs\":" + String(data.hs, 3) + ",";
  json += "\"tp\":" + String(data.tp, 2) + ",";
  json += "\"roll\":" + String(data.roll, 2) + ",";
  json += "\"pitch\":" + String(data.pitch, 2) + ",";
  json += "\"status\":\"" + data.status + "\",";
  json += "\"rssi\":" + String(data.rssi) + ",";
  json += "\"snr\":" + String(data.snr, 2) + ",";
  json += "\"crc\":" + String(data.crc) + ",";
  json += "\"crc_ok\":\"" + String(crcOK ? "YA" : "TIDAK") + "\",";
  json += "\"keterangan\":\"LoRa Gateway\"";
  json += "}";

  Serial.print("Sending JSON to Google Sheets: ");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("Google Sheets HTTP Code: ");
  Serial.println(httpCode);

  bool success = false;

  // Pada Google Apps Script, HTTP 302 tetap dianggap berhasil
  // karena data terbukti masuk ke Google Sheets.
  if (httpCode == 200 || httpCode == 201 || httpCode == 302) {
    success = true;
    Serial.println("Google Sheets: request accepted.");
  } else {
    String response = http.getString();
    Serial.print("Google Sheets Response: ");
    Serial.println(response);
  }

  http.end();

  return success;
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
    if ((millis() / 500) % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  else if (status == "TINGGI" || status == "SANGAT_TINGGI" || status == "EKSTREM") {
    digitalWrite(LED_MERAH, HIGH);

    // Buzzer cepat
    if ((millis() / 150) % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
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

// ================= OLED LAYOUT =================
void drawHeader(String modeText) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("SMART BUOY");

  display.setCursor(78, 0);
  display.print(getShortClockString());

  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(103, 11);
  display.print(modeText);
}

String shortStatus(String status) {
  if (status == "TENANG") return "TENANG";
  if (status == "RENDAH") return "RENDAH";
  if (status == "SEDANG") return "SEDANG";
  if (status == "TINGGI") return "TINGGI";
  if (status == "SANGAT_TINGGI") return "S.TINGGI";
  if (status == "EKSTREM") return "EKSTRM";
  return status;
}

void showStartupOLED() {
  display.clearDisplay();
  drawHeader("INIT");

  display.setCursor(0, 14);
  display.println("Gateway mulai...");

  display.setCursor(0, 26);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("WiFi : OK");
  } else {
    display.println("WiFi : OFF");
  }

  display.setCursor(0, 38);
  display.println("LoRa : Init");

  display.setCursor(0, 50);
  display.println("Menunggu data buoy");

  display.display();
}

void showNoDataOLED() {
  display.clearDisplay();
  drawHeader("WAIT");

  display.setCursor(0, 15);
  display.println("Belum ada data");

  display.setCursor(0, 27);
  display.println("Menunggu payload");

  display.setCursor(0, 39);
  display.print("WiFi:");
  display.print(WiFi.status() == WL_CONNECTED ? "OK" : "OFF");

  display.setCursor(62, 39);
  display.print("Sheet:");
  display.print(lastSheetsOK ? "OK" : "-");

  display.setCursor(0, 52);
  display.println("LED biru ON");

  display.display();
}

void updateOLED(SmartBuoyData data, bool isLive) {
  display.clearDisplay();

  String modeText = isLive ? "LIVE" : "WAIT";
  drawHeader(modeText);

  // Baris 1: Hs besar dan status
  display.setCursor(0, 12);
  display.print("Hs:");
  display.print(data.hs, 3);
  display.print("m");

  display.setCursor(62, 12);
  display.print(shortStatus(data.status));

  // Baris 2: Heave dan Tp
  display.setCursor(0, 23);
  display.print("Hv:");
  display.print(data.heave, 3);
  display.print("m");

  display.setCursor(76, 23);
  display.print("Tp:");
  display.print(data.tp, 1);

  // Baris 3: Roll dan Pitch
  display.setCursor(0, 34);
  display.print("R:");
  display.print(data.roll, 1);

  display.setCursor(58, 34);
  display.print("P:");
  display.print(data.pitch, 1);

  // Baris 4: RSSI dan SNR
  display.setCursor(0, 45);
  display.print("RSSI:");
  display.print(data.rssi);

  display.setCursor(70, 45);
  display.print("SNR:");
  display.print(data.snr, 1);

  // Baris 5: umur data, WiFi, Sheets
  display.setCursor(0, 56);

  if (isLive) {
    display.print("Age:");
  } else {
    display.print("Last:");
  }

  display.print(getDataAgeSeconds());
  display.print("s ");

  display.print(WiFi.status() == WL_CONNECTED ? "W:OK" : "W:--");

  display.setCursor(94, 56);
  display.print(lastSheetsOK ? "G:OK" : "G:--");

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
  Serial.println("SMART BUOY");
  Serial.println("GATEWAY UNIT");
  Serial.println("OLED displays last data during waiting/timeout");
  Serial.println("OLED shows gateway time");
  Serial.println("LoRa DIO0 tetap GPIO7");
  Serial.println("Buzzer gateway GPIO15");
  Serial.println("Payload: Timestamp,Heave,Hs,Tp,Roll,Pitch,Status,CRC");
  Serial.println("Gateway -> WiFi -> Google Sheets");
  Serial.println("HTTP 302 from Apps Script treated as success");
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
    showStartupOLED();
  }

  connectWiFi();
  showStartupOLED();
  delay(1200);

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
  showNoDataOLED();
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  // Reconnect WiFi jika terputus
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconnectAttempt = 0;

    if (now - lastReconnectAttempt > 10000) {
      lastReconnectAttempt = now;
      Serial.println("WiFi disconnected. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

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

      hasLatestData = true;
      dataOnline = true;
      timeoutPrinted = false;
      lastReceiveMillis = millis();

      blinkBlueLedOnReceive();
      updateStatusOutput(latestData.status);
      updateOLED(latestData, true);
      printValidData(latestData);

      // Kirim ke Google Sheets setiap 10 detik
      if (millis() - lastSheetsSendMillis >= SHEETS_SEND_INTERVAL_MS) {
        lastSheetsSendMillis = millis();

        bool sheetsOK = sendToGoogleSheets(latestData, true);

        lastSheetsOK = sheetsOK;

        if (sheetsOK) {
          lastSheetsOKMillis = millis();
          Serial.println("Google Sheets: data sent successfully.");
        } else {
          Serial.println("Google Sheets: failed to send data.");
        }

        Serial.println();

        // Setelah proses HTTP, tampilkan ulang data terakhir supaya OLED tidak kosong.
        updateOLED(latestData, true);
      }
    } else {
      Serial.println("Packet discarded.");
      Serial.println();

      // Jika packet rusak, OLED tetap mempertahankan data valid terakhir.
      if (hasLatestData) {
        bool stillLive = (millis() - lastReceiveMillis <= TIMEOUT_MS);
        updateOLED(latestData, stillLive);
      }
    }
  }

  // Timeout handling
  if (millis() - lastReceiveMillis > TIMEOUT_MS) {
    dataOnline = false;
    allStatusOff();

    // LED biru menyala terus saat waiting / timeout
    digitalWrite(LED_BIRU_GATEWAY, HIGH);

    // Revisi utama:
    // Saat timeout, OLED tetap menampilkan data terakhir.
    if (hasLatestData) {
      updateOLED(latestData, false);
    } else {
      showNoDataOLED();
    }

    if (!timeoutPrinted) {
      Serial.println("TIMEOUT: Data LoRa tidak diterima lebih dari 5 detik.");
      Serial.println("Gateway masuk mode WAITING.");
      Serial.println("OLED tetap menampilkan data terakhir.");
      Serial.println("LED biru menyala terus.");
      Serial.println();
      timeoutPrinted = true;
    }
  } else {
    if (hasLatestData && dataOnline) {
      updateStatusOutput(latestData.status);
    }
  }

  // Refresh OLED setiap 1 detik agar waktu dan umur data tetap berubah
  if (millis() - lastOLEDRefreshMillis >= OLED_REFRESH_INTERVAL_MS) {
    lastOLEDRefreshMillis = millis();

    if (hasLatestData) {
      bool stillLive = (millis() - lastReceiveMillis <= TIMEOUT_MS);
      updateOLED(latestData, stillLive);
    } else {
      showNoDataOLED();
    }
  }
}