#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}
#include <Wire.h>
#include <Servo.h>
#include <BH1750.h>

const bool USE_BH1750 = true;
const bool USE_ESPNOW_STATUS = true;

// CONFIG
uint8_t KEY_MAC[] = { 0x5C, 0xCF, 0x7F, 0x00, 0xD5, 0x0E };

const int PIN_BUZZER  = D8;
const int PIN_SERVO   = 2;
const int PIN_I2C_SDA = D2;
const int PIN_I2C_SCL = D1;

const int SERVO_UNLOCK_POS = 140;
const int SERVO_LOCK_POS   = 15;
const int SERVO_STEP_DEG   = 2;
const unsigned long SERVO_STEP_INTERVAL_MS  = 15;
const unsigned long SERVO_ATTACH_SETTLE_MS  = 50;
const float WEIGHT_THRESHOLD_V  = 0.060f;
const float LIGHT_THRESHOLD_LUX = 40.0f;

const unsigned long CONFIRM_DURATION_MS = 5000;

const unsigned long FIRST_LOCK_MS[3]  = {  60000, 1800000, 2700000 };
const unsigned long GRACE_MS[3]       = {  30000,  300000,  600000 };
const unsigned long SECOND_LOCK_MS[3] = {  60000, 1800000, 2700000 };

const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 500;
const unsigned long DEBUG_PRINT_INTERVAL_MS   = 2000;
const unsigned long ESPNOW_STATUS_INTERVAL_MS = 500;
const unsigned long KEY_TIMEOUT_MS            = 3000;

const unsigned long ESPNOW_IDLE_ALIVE_INTERVAL_MS = 2000;

const unsigned long ALARM_BEEP_ON_MS  = 500;
const unsigned long ALARM_BEEP_OFF_MS = 500;

// PROTOCOL
enum LockState : uint8_t {
  STATE_IDLE = 0,
  STATE_CONFIRMING,
  STATE_LOCKING,
  STATE_LOCKED_FIRST,
  STATE_UNLOCKING_GRACE,
  STATE_GRACE_WAIT,
  STATE_GRACE_ALARM,
  STATE_LOCKING_SECOND,
  STATE_LOCKED_SECOND,
  STATE_UNLOCKING_FINAL,
  STATE_COMPLETE
};

enum KeyMsgType : uint8_t {
  KEY_HEARTBEAT  = 1,
  KEY_SET_PRESET = 2
};

struct KeyPacket {
  uint8_t type;
  uint8_t preset;
};

struct LockboxStatusPacket {
  uint8_t  state;
  uint8_t  phoneDetected;
  uint8_t  keyConnected;
  uint8_t  sessionPart;
  uint32_t remainingMs;
  uint8_t  activePreset;
};

// GLOBALS
BH1750 lightMeter;
Servo servo;

bool sensorOK = false;

float luxReading = 0;
float weightVoltage = 0;
bool  weightDetected = false;
bool  phoneDetected  = false;

unsigned long lastSensorSampleMs  = 0;
unsigned long lastDebugPrintMs    = 0;
unsigned long lastStatusSendMs    = 0;
unsigned long lastKeyHeardMs      = 0;
unsigned long lastIdleAliveSendMs = 0;

bool keyConnected = false;

uint8_t activePreset = 0;

bool confirmForSecondSession = false;
unsigned long graceDeadlineMs = 0;

bool servoAttached = false;
int  currentServoPos = SERVO_UNLOCK_POS;
int  targetServoPos  = SERVO_UNLOCK_POS;
int  servoDir = 0;
unsigned long servoAttachMs = 0;
unsigned long lastServoStepMs = 0;

bool alarmActive = false;
bool alarmBeepOn = false;
unsigned long lastAlarmToggleMs = 0;

LockState state = STATE_IDLE;
unsigned long stateStartMs = 0;
LockboxStatusPacket lastSentPkt;
bool hasLastSentPkt = false;

bool bootServoInitActive = false;
unsigned long bootServoDetachAtMs = 0;

// SERVO
void beginServoMove(int from, int to) {
  
  if (!servoAttached) {
    servo.attach(PIN_SERVO);
    servoAttached = true;
    servoAttachMs = millis();
  }
  currentServoPos = from;
  targetServoPos  = to;
  servoDir = (to > from) ? 1 : -1;


  servo.write(currentServoPos);
  lastServoStepMs = millis();
}

bool updateServo(){
  if (!servoAttached) return true;

  unsigned long now = millis();
  if (now - servoAttachMs < SERVO_ATTACH_SETTLE_MS) return false;

  if (currentServoPos == targetServoPos) {
    servo.detach();
    servoAttached = false;
    return true;
  
  }

  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) return false;
  lastServoStepMs = now;
  currentServoPos += servoDir * SERVO_STEP_DEG;

  if ((servoDir > 0 && currentServoPos >= targetServoPos) ||(servoDir < 0 && currentServoPos <= targetServoPos)) {
    currentServoPos = targetServoPos;
  }

  servo.write(currentServoPos);

  if (currentServoPos == targetServoPos) {
    servo.detach();
    servoAttached = false;
    return true;
  }
  return false;
}

void updateBootServoInitIfNeeded() {
  if (!bootServoInitActive) return;
  unsigned long now = millis();
  if (now >= bootServoDetachAtMs) {
    servo.detach();
    bootServoInitActive = false;
  }
}

// ALARM
void startAlarm() {
  alarmActive = true;
  alarmBeepOn = true;
  tone(PIN_BUZZER, 800);
  lastAlarmToggleMs = millis();
  Serial.println("ALARM STARTED");
}

void stopAlarm() {
  alarmActive = false;
  alarmBeepOn = false;
  noTone(PIN_BUZZER);
  Serial.println("ALARM STOPPED");
}

void updateAlarm() {
  if (!alarmActive) return;

  unsigned long now = millis();
  unsigned long interval = alarmBeepOn ? ALARM_BEEP_ON_MS : ALARM_BEEP_OFF_MS;

  if (now - lastAlarmToggleMs >= interval) {
    lastAlarmToggleMs = now;
    alarmBeepOn = !alarmBeepOn;
    if (alarmBeepOn) tone(PIN_BUZZER, 800);
    else noTone(PIN_BUZZER);
  }
}

void playConfirmStartBeep() { tone(PIN_BUZZER, 1500, 120); }
void playConfirmEndBeep()   { tone(PIN_BUZZER, 2000, 180); }

// SENSORS
void sampleSensors() {
  unsigned long now = millis();
  if (now - lastSensorSampleMs < SENSOR_SAMPLE_INTERVAL_MS) return;
  lastSensorSampleMs = now;

  int raw = analogRead(A0);
  weightVoltage = raw * (3.3f / 1023.0f);
  weightDetected = (weightVoltage > WEIGHT_THRESHOLD_V);

  float lux = luxReading;

  if (USE_BH1750) {
    bool stateAllowsLight =(state == STATE_IDLE) ||(state == STATE_CONFIRMING) ||(state == STATE_GRACE_WAIT) ||(state == STATE_GRACE_ALARM);

    if (sensorOK && weightDetected && stateAllowsLight) {
      yield();
      lux = lightMeter.readLightLevel();
      yield();
      
      }
  } 
  else {
    lux = 0.0f;
  }

  luxReading = lux;
  bool isLight = (luxReading > LIGHT_THRESHOLD_LUX);
  phoneDetected = (weightDetected && isLight);
}

// DEBUG
const char* stateToString(LockState s) {
  switch(s) {
    case STATE_IDLE:            return "IDLE";
    case STATE_CONFIRMING:      return "CONFIRMING";
    case STATE_LOCKING:         return "LOCKING";
    case STATE_LOCKED_FIRST:    return "LOCKED_1ST";
    case STATE_UNLOCKING_GRACE: return "UNLOCKING";
    case STATE_GRACE_WAIT:      return "GRACE_WAIT";
    case STATE_GRACE_ALARM:     return "GRACE_ALARM";
    case STATE_LOCKING_SECOND:  return "LOCKING_2ND";
    case STATE_LOCKED_SECOND:   return "LOCKED_2ND";
    case STATE_UNLOCKING_FINAL: return "UNLOCKING";
    case STATE_COMPLETE:        return "COMPLETE";
    default:                    return "UNKNOWN";
  }

}

void printDebug() {

  unsigned long now = millis();
  if (now - lastDebugPrintMs < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMs = now;

  Serial.print("[");
  Serial.print(stateToString(state));
  Serial.print("] Lux=");
  Serial.print(luxReading);
  Serial.print(" V=");
  Serial.print(weightVoltage, 3);
  Serial.print(" Phone=");
  Serial.print(phoneDetected);
  Serial.print(" Key=");
  Serial.print(keyConnected ? "YES" : "NO");
  Serial.print(" Preset=");
  Serial.println(activePreset + 1);
}



// ESP-NOW
void onDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  (void)mac;
  lastKeyHeardMs = millis();
  if (len == 1) return;
  if (len != sizeof(KeyPacket)) return;

  KeyPacket *pkt = (KeyPacket*)data;

  if (pkt->type == KEY_SET_PRESET && state == STATE_IDLE && pkt->preset >= 1 && pkt->preset <= 3) {
    activePreset = (uint8_t)(pkt->preset - 1);
    Serial.print("Preset set to ");
    Serial.println(pkt->preset);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed!");
    return;
  }



  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);

  if (esp_now_add_peer(KEY_MAC, ESP_NOW_ROLE_COMBO, 1, nullptr, 0) == 0){
    Serial.println("ESP-NOW peer added");
  }
}

// STATUS
void sendStatusIfDue() {


  if (!USE_ESPNOW_STATUS) {
    return;
  }

  unsigned long now = millis();
  if (now - lastStatusSendMs < ESPNOW_STATUS_INTERVAL_MS) return;
  lastStatusSendMs = now;

  bool forceIdleAlive =(state == STATE_IDLE) && (now - lastIdleAliveSendMs >= ESPNOW_IDLE_ALIVE_INTERVAL_MS);

  if (!keyConnected && !forceIdleAlive){
    return;
  } 

  unsigned long elapsed = now - stateStartMs;
  unsigned long remaining = 0;
  uint8_t sessionPart = 0;

  

  switch(state){
    case STATE_CONFIRMING:
      remaining = (elapsed < CONFIRM_DURATION_MS) ? (CONFIRM_DURATION_MS - elapsed) : 0;
      sessionPart = confirmForSecondSession ? 3 : 1;
      break;

    case STATE_LOCKED_FIRST:
      remaining = (elapsed < FIRST_LOCK_MS[activePreset]) ? (FIRST_LOCK_MS[activePreset] - elapsed) : 0;
      sessionPart = 1;
      break;

    case STATE_GRACE_WAIT:
    case STATE_GRACE_ALARM:
      remaining = (graceDeadlineMs > now) ? (graceDeadlineMs - now) : 0;
      sessionPart = 2;
      break;

    case STATE_LOCKED_SECOND:
      remaining = (elapsed < SECOND_LOCK_MS[activePreset]) ? (SECOND_LOCK_MS[activePreset] - elapsed) : 0;
      sessionPart = 3;
      break;

    default:
      remaining = 0;
      sessionPart = 0;
      break;
  }

  LockboxStatusPacket pkt;
  pkt.state         = (uint8_t)state;
  pkt.phoneDetected = phoneDetected ? 1 : 0;
  pkt.keyConnected  = keyConnected ? 1 : 0;
  pkt.sessionPart   = sessionPart;
  pkt.remainingMs   = remaining;
  pkt.activePreset  = activePreset + 1;

  bool changed = !hasLastSentPkt ||memcmp(&pkt, &lastSentPkt, sizeof(pkt)) != 0;

  if (!changed && !forceIdleAlive){return;} 

  yield();
  uint8_t res = esp_now_send(KEY_MAC, (uint8_t*)&pkt, sizeof(pkt));
  yield();
  if (res == 0){
    lastSentPkt = pkt;
    hasLastSentPkt = true;
    if (forceIdleAlive) lastIdleAliveSendMs = now;
  }
}

// SETUP / LOOP
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== LOCKBOX STUDY SESSION ===");
  Serial.print("Reset reason: ");
  Serial.println(ESP.getResetReason());

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (USE_BH1750){
    sensorOK = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    Serial.println(sensorOK ? "BH1750 OK" : "BH1750 FAILED");
  } 
  else{
    Serial.println("BH1750 DISABLED");
  }


  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  servo.attach(PIN_SERVO);
  servo.write(SERVO_UNLOCK_POS);
  bootServoInitActive = true;
  bootServoDetachAtMs = millis() + 300;


  setupEspNow();

  state = STATE_IDLE;
  stateStartMs = millis();
  graceDeadlineMs = 0;
  activePreset = 0;
  
}

void loop() {
  yield();

  unsigned long now = millis();
  keyConnected = (now - lastKeyHeardMs) < KEY_TIMEOUT_MS;

  updateBootServoInitIfNeeded();
  sampleSensors();
  printDebug();
  sendStatusIfDue();
  updateAlarm();

  switch (state) {
    case STATE_IDLE:
      if (phoneDetected && keyConnected) {
        Serial.println("→ CONFIRMING (hold to start)");
        confirmForSecondSession = false;
        playConfirmStartBeep();
        state = STATE_CONFIRMING;
        stateStartMs = now;
      }
      break;

    case STATE_CONFIRMING:
      if (!phoneDetected || !keyConnected) {
        if (confirmForSecondSession) {
          Serial.println("→ GRACE_WAIT (2nd confirm cancelled)");
          state = STATE_GRACE_WAIT;
        } else {
          Serial.println("→ IDLE (cancelled)");
          state = STATE_IDLE;
        }
        break;
      }

      if (now - stateStartMs >= CONFIRM_DURATION_MS) {
        playConfirmEndBeep();

        if (!confirmForSecondSession) {
          Serial.println("LOCKING (First session starting)");
          beginServoMove(SERVO_UNLOCK_POS, SERVO_LOCK_POS);
          state = STATE_LOCKING;
        } else {
          Serial.println("LOCKING_SECOND (Second session starting)");
          beginServoMove(SERVO_UNLOCK_POS, SERVO_LOCK_POS);
          state = STATE_LOCKING_SECOND;
        }
      }
      break;


    case STATE_LOCKING:
      if (updateServo()) {
        Serial.println("LOCKED_FIRST");
        state = STATE_LOCKED_FIRST;
        stateStartMs = now;
      }
      break;

    case STATE_LOCKED_FIRST:
      if (now - stateStartMs >= FIRST_LOCK_MS[activePreset]) {
        Serial.println("UNLOCKING_GRACE (Break time!)");
        beginServoMove(SERVO_LOCK_POS, SERVO_UNLOCK_POS);
        state = STATE_UNLOCKING_GRACE;
      }
      break;


    case STATE_UNLOCKING_GRACE:
      if (updateServo()) {
        Serial.println("GRACE_WAIT (Break window)");
        state = STATE_GRACE_WAIT;
        stateStartMs = now;
        graceDeadlineMs = now + GRACE_MS[activePreset];
      
      }
      break;

    case STATE_GRACE_WAIT:

      if (phoneDetected && keyConnected) {
        Serial.println("CONFIRMING (hold for 2nd session)");
        confirmForSecondSession = true;
        playConfirmStartBeep();
        state = STATE_CONFIRMING;
        stateStartMs = now;
        break;
      }


      if (graceDeadlineMs != 0 && now >= graceDeadlineMs) {
        Serial.println("GRACE_ALARM (Put phone back!)");
        startAlarm();
        state = STATE_GRACE_ALARM;
      }
      break;

    case STATE_GRACE_ALARM:
      if (phoneDetected && keyConnected) {
        stopAlarm();
        Serial.println("CONFIRMING (hold for 2nd session)");
        confirmForSecondSession = true;
        playConfirmStartBeep();
        state = STATE_CONFIRMING;
        stateStartMs = now;
      }


      break;

    case STATE_LOCKING_SECOND:
      if (updateServo()){
        Serial.println("LOCKED_SECOND");
        state = STATE_LOCKED_SECOND;
        stateStartMs = now;
      }

      break;

    case STATE_LOCKED_SECOND:
      if (now - stateStartMs >= SECOND_LOCK_MS[activePreset]) {
        Serial.println("UNLOCKING_FINAL (Session complete!)");
        beginServoMove(SERVO_LOCK_POS, SERVO_UNLOCK_POS);
        state = STATE_UNLOCKING_FINAL;
      }
      break;

    case STATE_UNLOCKING_FINAL:
      if (updateServo()) {
        Serial.println("COMPLETE (Well done!)");
        state = STATE_COMPLETE;
        stateStartMs = now;
      }
      break;

    case STATE_COMPLETE:


      if (now - stateStartMs >= 2000){

        Serial.println("IDLE (Ready for next session)");
        state = STATE_IDLE;
        graceDeadlineMs = 0;
      }
      break;
  }





}