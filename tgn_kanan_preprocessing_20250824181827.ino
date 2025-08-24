#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

// ===== Edge Impulse (ganti bila nama project berbeda) =====
#include <ADXL345_MPU6050_inferencing.h>

// =================== Multiplexer (Right hand flex) ===================
#define MUX_SIG 34
#define S0 12
#define S1 14
#define S2 27
#define S3 26

// =================== IMU (Right) ===================
MPU6050 mpuRight;
Adafruit_ADXL345_Unified adxlRight = Adafruit_ADXL345_Unified(54321);

// =================== ESP-NOW ===================
volatile bool leftAvailable = false;

// Struktur data dari Slave (Left)
typedef struct struct_message {
  int   ss49e[9];  // 9 flex kiri
  int   mpu[3];    // ax, ay, az (MPU kiri)
  float adxl[3];   // x, y, z (ADXL kiri)
} struct_message;

struct_message leftData_latest;

// Buffer local Right
int ss49eRight[9];        // 9 flex kanan
int16_t axR, ayR, azR;    // MPU accel kanan
sensors_event_t adxlEvtR; // ADXL kanan

// Sampling time-series
// Total fitur EI = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE (contoh 300)
// Tiap frame = 30 fitur (9L+9R+3Lmpu+3Ladxl+3Rmpu+3Radxl)
const size_t FRAME_FEATURES = 30;
const int SAMPLE_DELAY_MS = 20;  // ~50 Hz per snapshot; sesuaikan dgn training kamu

// ======= Helper: pilih channel MUX 16-ch =======
static inline void muxSelect(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  delayMicroseconds(300); // settling time
}

// ======= ESP-NOW Rx callback (IDF v5 signature) =======
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(leftData_latest)) {
    memcpy((void*)&leftData_latest, incomingData, sizeof(leftData_latest));
    leftAvailable = true;
  }
}

// =================== Normalisasi (SAMAKAN dgn training) ===================
static inline float norm_flex(int raw) {
  // contoh: return (float)raw / 4095.0f;
  return (float)raw;
}
static inline float norm_mpu(int raw) {
  // contoh: return ((float)raw) / 16384.0f; // jika ingin skala g
  return (float)raw;
}
static inline float norm_adxl(float v) {
  // contoh: return v / 9.80665f; // jika ingin g
  return v;
}

// =================== Setup ===================
void setup() {
  Serial.begin(115200);

  // MUX pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // I2C
  Wire.begin(21, 22);

  // IMU Right init
  mpuRight.initialize();
  if (!mpuRight.testConnection()) {
    Serial.println("MPU6050 Right NOT connected!");
  }
  if (!adxlRight.begin()) {
    Serial.println("ADXL345 Right NOT detected!");
  } else {
    adxlRight.setRange(ADXL345_RANGE_16_G);
  }

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (1) { delay(10); }
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Edge Impulse init (versi baru biasanya void)
  run_classifier_init();

  Serial.print("EI expects features: ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  if (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE % FRAME_FEATURES != 0) {
    Serial.println("ERROR: EI input size not multiple of 30. Check your pipeline.");
  } else {
    Serial.print("Frames per inference: ");
    Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / FRAME_FEATURES);
  }

  Serial.println("Master ready (ESP-NOW + MUX + IMU + EI).");
}

// ======= Baca satu snapshot (isi 30 fitur) ke buffer mulai offset =======
void read_one_frame(float *features, size_t offset) {
  // 1) Baca 9 flex kanan
  for (int i = 0; i < 9; i++) {
    muxSelect(i);
    ss49eRight[i] = analogRead(MUX_SIG);
  }

  // 2) Baca IMU kanan
  mpuRight.getAcceleration(&axR, &ayR, &azR);
  adxlRight.getEvent(&adxlEvtR);

  // 3) Ambil snapshot kiri (pakai latest yang diterima)
  //     Kalau leftAvailable belum true, fitur kiri = 0 (atau tahan lama nilai terakhir)
  //     Di sini kalau belum available, isi 0 agar konsisten.
  int   mpuLx = leftAvailable ? leftData_latest.mpu[0] : 0;
  int   mpuLy = leftAvailable ? leftData_latest.mpu[1] : 0;
  int   mpuLz = leftAvailable ? leftData_latest.mpu[2] : 0;
  float adxlLx = leftAvailable ? leftData_latest.adxl[0] : 0.0f;
  float adxlLy = leftAvailable ? leftData_latest.adxl[1] : 0.0f;
  float adxlLz = leftAvailable ? leftData_latest.adxl[2] : 0.0f;

  // 4) Packing urutan 30 fitur:
  //     [0..8] flex L, [9..17] flex R, [18..20] MPU L, [21..23] ADXL L,
  //     [24..26] MPU R, [27..29] ADXL R
  size_t idx = offset;

  // 9 flex LEFT
  for (int i = 0; i < 9; i++) {
    float v = leftAvailable ? norm_flex(leftData_latest.ss49e[i]) : 0.0f;
    features[idx++] = v;
  }

  // 9 flex RIGHT
  for (int i = 0; i < 9; i++) {
    features[idx++] = norm_flex(ss49eRight[i]);
  }

  // 3 MPU LEFT
  features[idx++] = norm_mpu(mpuLx);
  features[idx++] = norm_mpu(mpuLy);
  features[idx++] = norm_mpu(mpuLz);

  // 3 ADXL LEFT
  features[idx++] = norm_adxl(adxlLx);
  features[idx++] = norm_adxl(adxlLy);
  features[idx++] = norm_adxl(adxlLz);

  // 3 MPU RIGHT
  features[idx++] = norm_mpu(axR);
  features[idx++] = norm_mpu(ayR);
  features[idx++] = norm_mpu(azR);

  // 3 ADXL RIGHT
  features[idx++] = norm_adxl(adxlEvtR.acceleration.x);
  features[idx++] = norm_adxl(adxlEvtR.acceleration.y);
  features[idx++] = norm_adxl(adxlEvtR.acceleration.z);
}

// =================== Variabel Global untuk Counter Gesture ===================
String lastGesture = "";           // Menyimpan gesture terakhir yang *ditampilkan*
String gestureBeingTracked = "";   // Gesture yang sedang dihitung counternya
int gestureCounter = 0;            // Counter untuk gesture yang sedang di-track

void loop() {
  // Pastikan ukuran input EI adalah kelipatan 30 (tiap snapshot)
  if (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE % FRAME_FEATURES != 0) {
    Serial.println("Cannot run: EI input size not multiple of 30.");
    delay(500);
    return;
  }

  const size_t N_FRAMES = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / FRAME_FEATURES;
  static float feature_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

  // Kumpulkan N_FRAMES snapshot time-series
  for (size_t f = 0; f < N_FRAMES; f++) {
    read_one_frame(feature_buffer, f * FRAME_FEATURES);
    delay(SAMPLE_DELAY_MS);
  }

  // Bungkus ke signal
  signal_t signal;
  int sig_ok = numpy::signal_from_buffer(feature_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (sig_ok != 0) {
    Serial.print("Failed create signal: ");
    Serial.println(sig_ok);
    return;
  }

  // Inferensi
  ei_impulse_result_t result;
  EI_IMPULSE_ERROR e = run_classifier(&signal, &result, false);
  if (e != EI_IMPULSE_OK) {
    Serial.print("Classifier error: ");
    Serial.println(e);
    return;
  }

  // Ambil hasil terbaik
  int best_idx = -1;
  float best_score = -1.0f;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > best_score) {
      best_score = result.classification[i].value;
      best_idx = (int)i;
    }
  }

  // =================== LOGIKA BARU DENGAN COUNTER & FILTER ===================
  const float CONFIDENCE_THRESHOLD = 0.80f; // Minimal confidence 80%
  const int REQUIRED_STREAK = 5;            // Harus terdeteksi 5x berturut-turut

  if (best_idx >= 0 && best_score >= CONFIDENCE_THRESHOLD) {
    String currentGesture = result.classification[best_idx].label;

    // 🔥 Periksa apakah gestur BUKAN "Transisi" sebelum melanjutkan
    if (currentGesture != "Transisi") {
      // Jika gesture saat ini sama dengan yang sedang di-track, tambahkan counter
      if (currentGesture == gestureBeingTracked) {
        gestureCounter++;
      } else {
        // Jika gesture baru terdeteksi, reset counter dan mulai track gesture baru ini
        gestureBeingTracked = currentGesture;
        gestureCounter = 1;
      }

      // Cek apakah sudah mencapai jumlah streak yang dibutuhkan
      if (gestureCounter >= REQUIRED_STREAK) {
        // Tampilkan HANYA jika gesture yang terkonfirmasi ini BERBEDA dari yang terakhir ditampilkan
        if (gestureBeingTracked != lastGesture) {
          Serial.print("Gesture CONFIRMED: ");
          Serial.print(gestureBeingTracked);
          Serial.print(" (");
          Serial.print(best_score * 100.0f, 1);
          Serial.println("%)");

          // Update gesture terakhir yang ditampilkan agar tidak spam output
          lastGesture = gestureBeingTracked;
        }
      }
    } else {
      // Jika gestur adalah "Transisi", perlakukan seperti di bawah threshold (reset counter)
      gestureCounter = 0;
      gestureBeingTracked = "";
      if (lastGesture != "Netral") {
        Serial.println("Gesture: Netral");
        lastGesture = "Netral";
      }
    }
  } else {
    // Jika confidence rendah atau tidak ada gesture terdeteksi, reset semuanya
    gestureCounter = 0;
    gestureBeingTracked = "";

    // Jika gesture terakhir yang ditampilkan bukan "Netral", maka tampilkan "Netral"
    if (lastGesture != "Netral") {
      Serial.println("Gesture: Netral");
      lastGesture = "Netral";
    }
  }
}
