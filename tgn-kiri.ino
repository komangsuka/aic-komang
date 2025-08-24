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
MPU6050 mpuLeft;
Adafruit_ADXL345_Unified adxlLeft = Adafruit_ADXL345_Unified(12345);

// =================== ESP-NOW ===================
uint8_t masterAddress[] = {0x78, 0x1C, 0x3C, 0xB8, 0x94, 0xB4}; // Master MAC
bool masterConnected = false;

// =================== Sensor Array ===================
int ss49eLeft[9];

typedef struct struct_message {
  int ss49e[9];
  int mpu[3];
  float adxl[3];
} struct_message;

struct_message dataToSend;

// Function to select MUX channel
void muxSelect(int channel){
  digitalWrite(S0, bitRead(channel,0));
  digitalWrite(S1, bitRead(channel,1));
  digitalWrite(S2, bitRead(channel,2));
  digitalWrite(S3, bitRead(channel,3));
}

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    masterConnected = true;
    Serial.println("Data sent successfully to Master");
  } else {
    masterConnected = false;
    Serial.println("Failed to send data. Master not detected.");
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
  mpuLeft.initialize();
  if(!mpuLeft.testConnection()){
    Serial.println("MPU6050 Left not connected!");
  }

  // ADXL345 init
  if(!adxlLeft.begin()){
    Serial.println("ADXL345 Left not detected!");
  }

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (Master)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Master peer");
    return;
  }
}

void loop() {
  // 1️⃣ Read SS49E sensors
  for(int i=0;i<9;i++){
    muxSelect(i);
    delay(2);
    ss49eLeft[i] = analogRead(MUX_SIG);
    dataToSend.ss49e[i] = ss49eLeft[i];
  }

  // 2️⃣ Read MPU6050
  int16_t ax, ay, az;
  mpuLeft.getAcceleration(&ax, &ay, &az);
  dataToSend.mpu[0] = ax;
  dataToSend.mpu[1] = ay;
  dataToSend.mpu[2] = az;

  // 3️⃣ Read ADXL345
  sensors_event_t event; 
  adxlLeft.getEvent(&event);
  dataToSend.adxl[0] = event.acceleration.x;
  dataToSend.adxl[1] = event.acceleration.y;
  dataToSend.adxl[2] = event.acceleration.z;

  // 4️⃣ Send data to Master
  esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));

  // Status
  if(masterConnected){
    Serial.println("Master detected, data sent");
  } else {
    Serial.println("Master not detected yet");
  }

  delay(50); // Kirim tiap 50ms
}
