# NaufalAFurqan_2201020015_RepoSkripsi

## Implementasi Fusi Algoritma Madgwick dan Kalman Filter Pada Prototype Smart Buoy Untuk Optimasi Monitoring Gelombang Laut

Repository ini berisi kode program, data pengujian, dan dokumentasi pendukung untuk penelitian skripsi mengenai **prototype Smart Buoy** berbasis Internet of Things (IoT). Sistem ini dirancang untuk melakukan monitoring gelombang laut dengan memanfaatkan sensor IMU 6-DoF, komunikasi LoRa, serta integrasi penyimpanan data ke Google Sheets.

## Identitas Peneliti

| Keterangan | Detail |
|---|---|
| Nama | Naufal A. Furqan |
| NIM | 2201020015 |
| Program Studi | Teknologi Informasi |
| Konsentrasi | Internet of Things (IoT) |
| Judul | Implementasi Fusi Algoritma Madgwick dan Kalman Filter Pada Prototype Smart Buoy Untuk Optimasi Monitoring Gelombang Laut |

## Deskripsi Sistem

Prototype Smart Buoy ini terdiri dari dua unit utama, yaitu **unit buoy/stasiun pengirim** dan **unit gateway/stasiun penerima**. Unit buoy membaca data pergerakan menggunakan sensor IMU GY-87, mengolah orientasi menggunakan algoritma Madgwick, melakukan koreksi gravitasi, menghitung estimasi gerakan vertikal/heave, kemudian menstabilkan hasil estimasi menggunakan Kalman Filter. Data hasil pengolahan dikirim ke gateway menggunakan komunikasi LoRa.

Unit gateway menerima data dari buoy, melakukan validasi payload, menampilkan status pada OLED, mengaktifkan indikator LED dan buzzer sesuai kategori gelombang, serta mengirimkan data ke Google Sheets sebagai media penyimpanan berbasis cloud.

## Fitur Utama

- Pembacaan data sensor IMU 6-DoF menggunakan GY-87.
- Estimasi orientasi roll dan pitch menggunakan algoritma Madgwick.
- Koreksi gravitasi untuk memperoleh percepatan vertikal.
- Estimasi heave sebagai dasar perhitungan tinggi gelombang.
- Penyaringan nilai heave menggunakan Kalman Filter.
- Perhitungan nilai tinggi gelombang signifikan atau Hs.
- Klasifikasi status gelombang berdasarkan nilai Hs.
- Komunikasi data menggunakan LoRa SX1278/Ra-02 433 MHz.
- Tampilan data pada OLED gateway.
- Indikator LED dan buzzer sebagai peringatan kondisi gelombang.
- Integrasi data monitoring ke Google Sheets.

## Teknologi dan Komponen

### Perangkat Keras

- ESP32-S3 N16R8 sebagai unit buoy/stasiun pengirim.
- ESP32-S3 N16R8 sebagai unit gateway/stasiun penerima.
- Sensor GY-87 untuk pembacaan data IMU.
- Modul LoRa SX1278/Ra-02 433 MHz untuk komunikasi nirkabel.
- OLED I2C 128x64 pada gateway.
- LED indikator status gelombang.
- LED biru sebagai indikator aktivitas sistem/komunikasi.
- Buzzer sebagai alarm peringatan.
- Catu daya berbasis baterai dan panel surya.

### Perangkat Lunak

- Arduino IDE.
- Bahasa pemrograman C/C++ untuk ESP32.
- Google Apps Script untuk integrasi Google Sheets.
- Google Sheets sebagai media penyimpanan dan pemantauan data.

## Struktur Repository

```text
NaufalAFurqan_2201020015_RepoSkripsi/
├── Firmware/
│   ├── Final Code/
│   │   ├── Buoy_Final_Integration/
│   │   ├── Gateway_Final_Integration/
│   │   └── Gateway_GoogleSheets_Integration/
│   │
│   └── Test_Code/
│       ├── Blink_LED_Biru_Buoy/
│       ├── Buoy_RealData_LoRa_TX/
│       ├── Gateway_Oled_LED_Buzzer/
│       ├── Gateway_Timeout_Test/
│       ├── Gravity_Correction_Diam_Retake/
│       ├── Gravity_Correction_Heave_Acceleration/
│       ├── Heave_Integration/
│       ├── Hs_Calculation/
│       ├── Hs_LED_Buzzer_Status/
│       ├── I2C_Scanner_GY87_LCD/
│       ├── Kalman_Heave_Filter/
│       ├── LoRa_Basic_RX_Gateway/
│       ├── LoRa_Basic_TX_Buoy/
│       ├── LoRa_Payload_RX_Gateway/
│       ├── LoRa_Payload_TX_Buoy/
│       ├── Madgwick_Roll_Pitch/
│       ├── Oled_Hs_Status/
│       ├── READ_Calibrated_GY87/
│       └── Read_RAW_GY87/
│
├── Test_Data/
│   ├── Algorithm_Test/
│   ├── Calibration/
│   ├── Final_Integraion/
│   ├── Google_Sheets/
│   ├── LoRa_Test/
│   └── Raw_Sensor/
│
└── README.md
```

## Keterangan Folder

| Folder | Keterangan |
|---|---|
| `Firmware/Final Code` | Berisi program akhir untuk unit buoy, gateway, dan integrasi Google Sheets. |
| `Firmware/Test_Code` | Berisi kode pengujian bertahap untuk sensor, algoritma, LoRa, OLED, LED, buzzer, dan perhitungan Hs. |
| `Test_Data/Raw_Sensor` | Berisi data mentah hasil pembacaan sensor sebelum proses kalibrasi. |
| `Test_Data/Calibration` | Berisi data hasil kalibrasi sensor GY-87 pada beberapa kondisi orientasi. |
| `Test_Data/Algorithm_Test` | Berisi data pengujian algoritma Madgwick, koreksi gravitasi, heave, Kalman Filter, dan Hs. |
| `Test_Data/LoRa_Test` | Berisi data pengujian komunikasi LoRa, payload, gateway, timeout, OLED, LED, dan buzzer. |
| `Test_Data/Google_Sheets` | Berisi data log hasil integrasi sistem dengan Google Sheets dan juga hasil pengujian dengan alat manual. |
| `Test_Data/Final_Integraion` | Berisi log dan ringkasan pengujian sistem terintegrasi. |

## Alur Kerja Sistem

1. Sensor GY-87 membaca data akselerometer dan giroskop.
2. Data sensor dikalibrasi untuk mengurangi bias pembacaan.
3. Algoritma Madgwick digunakan untuk memperoleh estimasi orientasi roll dan pitch.
4. Sistem melakukan koreksi gravitasi untuk memperoleh percepatan vertikal.
5. Percepatan vertikal diintegrasikan untuk memperoleh estimasi heave.
6. Kalman Filter digunakan untuk menstabilkan estimasi heave.
7. Sistem menghitung nilai tinggi gelombang signifikan atau Hs.
8. Nilai Hs diklasifikasikan menjadi status kondisi gelombang.
9. Data dikirim dari buoy ke gateway menggunakan LoRa.
10. Gateway menampilkan data pada OLED, mengaktifkan indikator LED/buzzer, dan mengirim data ke Google Sheets.

## Status Gelombang

| Status | Rentang Hs | Indikator |
|---|---:|---|
| Tenang | Hs < 0,5 m | LED hijau aktif, buzzer mati |
| Rendah | 0,5 m ≤ Hs < 1,25 m | LED kuning aktif, buzzer mati |
| Sedang | 1,25 m ≤ Hs < 2,5 m | LED kuning aktif, buzzer pelan |
| Tinggi | 2,5 m ≤ Hs < 4 m | LED merah aktif, buzzer cepat |
| Sangat Tinggi | 4 m ≤ Hs < 6 m | LED merah aktif, buzzer cepat |
| Ekstrem | Hs ≥ 6 m | LED merah aktif, buzzer cepat |

## Cara Menggunakan Program

### 1. Clone Repository

```bash
git clone https://github.com/diefault666/NaufalAFurqan_2201020015_RepoSkripsi.git
```

### 2. Buka Folder Firmware

Masuk ke folder firmware sesuai kebutuhan:

```text
Firmware/Final Code/Buoy_Final_Integration
Firmware/Final Code/Gateway_Final_Integration
Firmware/Final Code/Gateway_GoogleSheets_Integration
```

### 3. Buka Program Menggunakan Arduino IDE atau PlatformIO

Pilih board ESP32 yang sesuai, kemudian sesuaikan konfigurasi berikut jika diperlukan:

- jenis board ESP32/ESP32-S3,
- port COM,
- pin sensor dan modul LoRa,
- alamat I2C sensor/OLED,
- URL Web App Google Apps Script,
- konfigurasi jaringan WiFi pada gateway jika menggunakan Google Sheets.

### 4. Upload Program ke Mikrokontroler

- Upload kode buoy ke ESP32-S3 N16R8.
- Upload kode gateway ke ESP32-S3 N16R8.
- Buka Serial Monitor untuk melihat proses pembacaan sensor, pengiriman LoRa, penerimaan payload, dan pengiriman data ke Google Sheets.

## Data Pengujian

Data pengujian pada repository ini digunakan sebagai pendukung analisis pada penelitian. Data yang tersedia meliputi:

- data mentah sensor GY-87,
- data kalibrasi sensor,
- data hasil pengujian Madgwick,
- data koreksi gravitasi,
- data integrasi heave,
- data Kalman Filter,
- data perhitungan Hs,
- data pengujian LoRa,
- data integrasi gateway,
- data log Google Sheets.

## Tujuan Repository

Repository ini dibuat sebagai dokumentasi teknis dan arsip kode program untuk mendukung penelitian skripsi. Isi repository dapat digunakan untuk melihat proses pengembangan sistem mulai dari pengujian sensor, pengujian algoritma, komunikasi LoRa, hingga integrasi sistem Smart Buoy secara keseluruhan.

## Penulis

**Naufal A. Furqan**  
Program Studi Teknologi Informasi  
Universitas Bumigora
