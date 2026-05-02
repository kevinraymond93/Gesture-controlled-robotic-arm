#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Voltage detection pins =====
#define VOLTAGE_PIN 36
#define OE_PIN 25
#define VOLTAGE_THRESHOLD 500
#define GRIP_OPEN_PULSE  420
#define GRIP_CLOSE_PULSE 260
#define GRIPPER_CHANNEL 4  

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ============================ Motion tuning ==========================
const float deadzone = 2.0;
const int maxStep = 8;

// tiny tuning
const float SENSITIVITY = 180.0; // was 200 — lower = less sensitive
const float ELBOW_INVERT = 1.0;  // set -1.0 to invert elbow direction
const float NEW_DEADZONE = 1.0;  // smaller deadzone for gentle moves
const int NEW_maxStep = 30;      // allow bigger steps if you want faster response

// ===== simple accel-swap params (used only for sensor1 base/shoulder) =====
const float ACC_SWAP_FACTOR = 1.2; // if |ay| > ACC_SWAP_FACTOR * |az|, swap axes

// ========================= Sensor data struct ========================
typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
} SensorData;

typedef struct {
  SensorData sensor1;   // channel 0 (used for base/elbow)
  SensorData sensor2;   // channel 1 (used for wrists)
  bool gripperClosed;   // gripper
} CombinedData;

CombinedData receivedData;

bool swapped = false; // global flag set each loop based on sensor1 accel

// ============================ Motor struct ===========================
struct Motor {
  int channel;
  int minPulse;
  int maxPulse;
  int currentPulse;
  float prevAccel;
  float prevGyro;
  bool firstPacketReceived;
};

// existing motors (sensor1)
Motor baseMotor     = {0, 0,   470, (0+470)/2, 0.0, 0.0, false};
Motor shoulderMotor = {1, 120, 500, 120,      0.0, 0.0, false};

// new wrist motors (sensor2) — MG90S limits 80..500, center 290
Motor wristRotMotor   = {2, 80, 500, 500, 0.0, 0.0, false}; // rotation <- sensor2.gx
Motor wristPitchMotor = {3, 80, 500, (80+500)/2, 0.0, 0.0, false}; // pitch    <- sensor2.gz

// ============================= Callback ==============================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  if (!baseMotor.firstPacketReceived) {
    // align prev values with first packet → no jump
    baseMotor.prevAccel = receivedData.sensor1.ax;
    baseMotor.prevGyro = receivedData.sensor1.gz;
    baseMotor.firstPacketReceived = true;
    Serial.println("First packet received → base aligned");
  }

  if (!shoulderMotor.firstPacketReceived) {
    // shoulder uses sensor1 gy
    shoulderMotor.prevAccel = receivedData.sensor1.az;
    shoulderMotor.prevGyro = receivedData.sensor1.gy;
    shoulderMotor.firstPacketReceived = true;
    Serial.println("First packet received → shoulder aligned");
  }

  if (!wristRotMotor.firstPacketReceived) {
    // wrist rotation uses sensor2.gx
    wristRotMotor.prevAccel = receivedData.sensor2.ax;
    wristRotMotor.prevGyro = receivedData.sensor2.gx;
    wristRotMotor.firstPacketReceived = true;
    Serial.println("First packet received → wrist rotation aligned");
  }

  if (!wristPitchMotor.firstPacketReceived) {
    // wrist pitch uses sensor2.gz
    wristPitchMotor.prevAccel = receivedData.sensor2.ay;
    wristPitchMotor.prevGyro = receivedData.sensor2.gz;
    wristPitchMotor.firstPacketReceived = true;
    Serial.println("First packet received → wrist pitch aligned");
  }
}

// ========================== Base motor update ========================
void updateBaseMotor(Motor* m, SensorData& s) {
    // loop delay in seconds (~20ms)
    const float dt = 0.02;

    // choose source gyro for base motion
    // default: base uses gz; when swapped -> use gy
    float base_rate = swapped ? s.gy : s.gz;

    // compute step (same feel: angleIncrement * scale -> step)
    float angleIncrement = base_rate * dt; // deg/s * s ≈ deg (scale used previously anyway)
    int step = (int)(angleIncrement * SENSITIVITY);
    step = constrain(step, -NEW_maxStep, NEW_maxStep);

    // only move if step is above deadzone
    if (abs(step) < NEW_DEADZONE) return;

    // update motor pulse
    m->currentPulse += step;
    m->currentPulse = constrain(m->currentPulse, m->minPulse, m->maxPulse);
    pwm.setPWM(m->channel, 0, m->currentPulse);
}

// ========================== Shoulder motor update ====================
void updateShoulderMotor(Motor* m, SensorData& s) {
  const float dt = 0.02;

  // choose source gyro for shoulder motion
  // default: shoulder uses gy, when swapped -> use gz
  float sh_rate = swapped ? s.gz : s.gy;

  int step = (int)(sh_rate * dt * 200);
  step = (int)(ELBOW_INVERT * step); // set ELBOW_INVERT = -1.0 to invert
  step = constrain(step, -maxStep, maxStep);
  if (abs(step) < deadzone) return;

  m->currentPulse += step;
  m->currentPulse = constrain(m->currentPulse, m->minPulse, m->maxPulse);
  pwm.setPWM(m->channel, 0, m->currentPulse);
}

// =================== Wrist rotation update (sensor2.gx) ===================
void updateWristRotMotor(Motor* m, SensorData& s) {
  const float dt = 0.02;

  float rate = s.gx; // sensor2.gx controls wrist rotation
  int step = (int)(rate * dt * SENSITIVITY);
  step = constrain(step, -NEW_maxStep, NEW_maxStep);
  if (abs(step) < NEW_DEADZONE) return;

  m->currentPulse += step;
  m->currentPulse = constrain(m->currentPulse, m->minPulse, m->maxPulse);
  pwm.setPWM(m->channel, 0, m->currentPulse);
}

// =================== Wrist pitch update (sensor2.gy) ===================
void updateWristPitchMotor(Motor* m, SensorData& s) {
  const float dt = 0.02;

  float rate = s.gy; // <-- changed: use gyro Y from sensor2
  int step = (int)(rate * dt * SENSITIVITY);
  step = constrain(step, -NEW_maxStep, NEW_maxStep);
  if (abs(step) < NEW_DEADZONE) return;

  m->currentPulse += step;
  m->currentPulse = constrain(m->currentPulse, m->minPulse, m->maxPulse);
  pwm.setPWM(m->channel, 0, m->currentPulse);
}

// ============================== Setup ================================
void setup() {
  Serial.begin(115200);

  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH); // start disabled
  pinMode(VOLTAGE_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);

  pwm.begin();
  pwm.setPWMFreq(50);

  // hold servo at manual position until first packet
  pwm.setPWM(baseMotor.channel, 0, baseMotor.currentPulse);
  pwm.setPWM(shoulderMotor.channel, 0, shoulderMotor.currentPulse);
  pwm.setPWM(wristRotMotor.channel, 0, wristRotMotor.currentPulse);
  pwm.setPWM(wristPitchMotor.channel, 0, wristPitchMotor.currentPulse);

  Serial.println("Receiver ready. Waiting for voltage signal...");
}

// =============================== Loop =================================
void loop() {
  int voltageReading = analogRead(VOLTAGE_PIN);

  // update swap flag (simple, immediate)
  // use sensor1 accel values to decide swap
  // ensure we have fresh data first
  if (baseMotor.firstPacketReceived && shoulderMotor.firstPacketReceived) {
    float ay = receivedData.sensor1.ay;
    float az = receivedData.sensor1.az;
    if ( fabs(ay) > ACC_SWAP_FACTOR * fabs(az) ) swapped = true;
    else swapped = false;
  }

  if (voltageReading > VOLTAGE_THRESHOLD) {
    if (!baseMotor.firstPacketReceived || !shoulderMotor.firstPacketReceived ||
        !wristRotMotor.firstPacketReceived || !wristPitchMotor.firstPacketReceived) return;

    if (digitalRead(OE_PIN)) {
      digitalWrite(OE_PIN, LOW);
      Serial.println("Voltage detected → servos enabled");
      pwm.setPWM(baseMotor.channel, 0, baseMotor.currentPulse);
      pwm.setPWM(shoulderMotor.channel, 0, shoulderMotor.currentPulse);
      pwm.setPWM(wristRotMotor.channel, 0, wristRotMotor.currentPulse);
      pwm.setPWM(wristPitchMotor.channel, 0, wristPitchMotor.currentPulse);
      delay(30);
    }

    // base/elbow use sensor1, wrists use sensor2
    updateBaseMotor(&baseMotor, receivedData.sensor1);
    updateShoulderMotor(&shoulderMotor, receivedData.sensor1);
    updateWristRotMotor(&wristRotMotor, receivedData.sensor2);
    updateWristPitchMotor(&wristPitchMotor, receivedData.sensor2);

  { //gripper
    static bool lastGripState = false;
    static unsigned long gripMoveTime = 0;
    static bool moving = false;

    if (receivedData.gripperClosed != lastGripState) {
        pwm.setPWM(GRIPPER_CHANNEL, 0,
            receivedData.gripperClosed ? GRIP_CLOSE_PULSE : GRIP_OPEN_PULSE);

        gripMoveTime = millis();
        moving = true;
        lastGripState = receivedData.gripperClosed;
    }

    // after 250ms, stop updating (but keep torque)
    if (moving && millis() - gripMoveTime > 250) {
        moving = false;
    }
  }


  } else {
    if (digitalRead(OE_PIN) == LOW) {
      digitalWrite(OE_PIN, HIGH);
      Serial.println("Voltage lost → servos disabled");
    }
  }

  delay(20);
}
