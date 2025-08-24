#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

// =================== Multiplexer ===================
#define MUX_SIG 34
#define S0 12
#define S1 14
#define S2 27
#define S3 26

// =================== IMU ===================
MPU6050 mpuRight;
Adafruit_ADXL345_Unified adxlRight = Adafruit_ADXL345_Unified(54321);

// =================== ESP-NOW ===================
volatile bool slaveConnected = false;

typedef struct struct_message {
  int ss49e[9];
  int mpu[3];
  float adxl[3];
} struct_message;

struct_message receivedData;

// Select channel on 16-ch MUX (CD74HC4067 style)
void muxSelect(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

// ✅ NEW signature for ESP-NOW receive (IDF v5.x)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(receivedData)) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    slaveConnected = true;
  } else {
    // ukuran tidak cocok, abaikan
  }
}

void setup() {
  Serial.begin(115200);

  // Multiplexer pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // I2C
  Wire.begin(21, 22);

  // MPU6050 init
  mpuRight.initialize();
  if (!mpuRight.testConnection()) {
    Serial.println("MPU6050 Right not connected!");
  }

  // ADXL345 init
  if (!adxlRight.begin()) {
    Serial.println("ADXL345 Right not detected!");
  } else {
    // Optional: set range biar stabil
    adxlRight.setRange(ADXL345_RANGE_16_G);
  }

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  // Optional: pastikan channel sama dengan slave jika perlu
  // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // ✅ Registrasi callback pakai signature baru
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // 1️⃣ Read Right sensors
  int ss49eRight[9];
  for (int i = 0; i < 9; i++) {
    muxSelect(i);
    delayMicroseconds(400);            // sedikit settle time
    ss49eRight[i] = analogRead(MUX_SIG);
  }

  // 2️⃣ Read Right MPU6050
  int16_t ax, ay, az;
  mpuRight.getAcceleration(&ax, &ay, &az);

  // 3️⃣ Read Right ADXL345
  sensors_event_t event;
  adxlRight.getEvent(&event);

  // 4️⃣ Print combined data
  if (slaveConnected) {
    // Serial.print("Sensor Left: ");
    for (int i = 0; i < 9; i++) { Serial.print(receivedData.ss49e[i]); Serial.print(i==8?' ':' '); }
    // Serial.print("| Sensor Right: ");
    for (int i = 0; i < 9; i++) { Serial.print(ss49eRight[i]); Serial.print(i==8?' ':' '); }
    // Serial.print("| Left MPU: ");
    Serial.print(receivedData.mpu[0]); Serial.print(" ");
    Serial.print(receivedData.mpu[1]); Serial.print(" ");
    Serial.print(receivedData.mpu[2]); Serial.print(" ");
    // Serial.print("| Left ADXL: ");
    Serial.print(receivedData.adxl[0], 2); Serial.print(" ");
    Serial.print(receivedData.adxl[1], 2); Serial.print(" ");
    Serial.print(receivedData.adxl[2], 2); Serial.print(" ");
    // Serial.print("| Right MPU: ");
    Serial.print(ax); Serial.print(" "); Serial.print(ay); Serial.print(" "); Serial.print(az); Serial.print(" ");
    // Serial.print("| Right ADXL: ");
    Serial.print(event.acceleration.x, 2); Serial.print(" ");
    Serial.print(event.acceleration.y, 2); Serial.print(" ");
    Serial.println(event.acceleration.z, 2);
    // Serial.println();
  } else {
    Serial.println("Waiting for Slave connection...");
  }

  delay(50);
}
