#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TCA9548A_ADDR 0x70
#define ESPNOW_CHANNEL 6
#define SWITCH_PIN 4

// separate MPU objects for each sensor
Adafruit_MPU6050 mpu0;
Adafruit_MPU6050 mpu1;

// data structure for one sensor
typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
} SensorData;

// combined struct for both sensors
typedef struct {
  SensorData sensor1; // channel 0
  SensorData sensor2; // channel 1
  bool gripperClosed; //gripper
} MultiSensorData;

MultiSensorData allData;

// your receiver MAC
uint8_t receiverMac[] = { 0x20, 0x43, 0xA8, 0xE6, 0xE9, 0xC4 };

void selectMuxChannel(int channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(2);
}

bool readMPU(Adafruit_MPU6050 &mpu, SensorData &data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (isnan(a.acceleration.x) || isnan(g.gyro.x)) return false;

  data.ax = a.acceleration.x;
  data.ay = a.acceleration.y;
  data.az = a.acceleration.z;
  data.gx = g.gyro.x;
  data.gy = g.gyro.y;
  data.gz = g.gyro.z;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receiverMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // --- init MPU0 on channel 0 ---
  selectMuxChannel(0);
  if (!mpu0.begin()) {
    Serial.println("MPU0 not found!");
    delay(1000);
    ESP.restart();
  }
  mpu0.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu0.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu0.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- init MPU1 on channel 1 ---
  selectMuxChannel(1);
  if (!mpu1.begin()) {
    Serial.println("MPU1 not found!");
    delay(1000);
    ESP.restart();
  }
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Transmitter ready with 2 sensors");
}

void loop() {
  // --- read sensor0 ---
  selectMuxChannel(0);
  if (!readMPU(mpu0, allData.sensor1)) {
    Serial.println("Sensor1 read fail");
    return;
  }

  // --- read sensor1 ---
  selectMuxChannel(1);
  if (!readMPU(mpu1, allData.sensor2)) {
    Serial.println("Sensor2 read fail");
    return;
  }

  allData.gripperClosed = (digitalRead(SWITCH_PIN) == HIGH);

  // --- send both sensors ---
  esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&allData, sizeof(allData));

  if (result == ESP_OK) {
    Serial.printf("CH0: ax=%+.2f ay=%+.2f az=%+.2f | CH1: ax=%+.2f ay=%+.2f az=%+.2f\n",
                  allData.sensor1.ax, allData.sensor1.ay, allData.sensor1.az,
                  allData.sensor2.ax, allData.sensor2.ay, allData.sensor2.az);
  } else {
    Serial.println("Send Error");
  }

  Serial.print(" Grip: ");
  Serial.println(allData.gripperClosed ? "CLOSED" : "OPEN");

  delay(50); // ~20Hz
}
