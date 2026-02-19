#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}
#include <U8g2lib.h>
#include <Wire.h>




// CONFIG 
uint8_t BOX_MAC[] = { 0x48, 0xE7, 0x29, 0x72, 0x80, 0xF9 };

const unsigned long HEARTBEAT_INTERVAL_MS = 500;
const unsigned long STATUS_TIMEOUT_MS = 3000;
const unsigned long DISPLAY_INTERVAL_MS = 100;
const unsigned long BUTTON_SCAN_MS = 10;
#define OLED_SCL D5
#define OLED_SDA D6
#define BUTTON_PIN D1
#define BUTTON_DEBOUNCE_MS 250

U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(
  U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE
);




//  PROTOCOL 
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
  KEY_HEARTBEAT = 1,
  KEY_SET_PRESET = 2
};

struct KeyPacket {
  uint8_t type;
  uint8_t preset;  // 1..3 for KEY_SET_PRESET
};

struct LockboxStatusPacket {
  uint8_t state;
  uint8_t phoneDetected;
  uint8_t keyConnected;
  uint8_t sessionPart;
  uint32_t remainingMs;
  uint8_t activePreset;  // 1..3
};




// STATE 
LockboxStatusPacket lastStatus;
bool hasStatus = false;
unsigned long lastStatusTimeMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastButtonScanMs = 0;
bool lastButtonState = HIGH;
unsigned long lastButtonPressMs = 0;
uint8_t localPreset = 1;





// ESP-NOW 
void onDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  (void)mac;
  if (len == sizeof(LockboxStatusPacket)) {
    memcpy(&lastStatus, data, sizeof(LockboxStatusPacket));
    hasStatus = true;


    lastStatusTimeMs = millis();
    if (lastStatus.activePreset >= 1 && lastStatus.activePreset <= 3) {
      localPreset = lastStatus.activePreset;
    }
  }
}

void onDataSent(uint8_t *mac, uint8_t status) {
  (void)mac;
  (void)status;
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
  esp_now_register_send_cb(onDataSent);
  esp_now_add_peer(BOX_MAC, ESP_NOW_ROLE_COMBO, 1, nullptr, 0);
}

void sendHeartbeatIfDue(){

  unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;
  lastHeartbeatMs = now;

  KeyPacket pkt;
  pkt.type = KEY_HEARTBEAT;
  pkt.preset = 0;
  esp_now_send(BOX_MAC, (uint8_t*)&pkt, sizeof(pkt));
}




//  BUTTON 
void checkButtonIfDue() {

  unsigned long now = millis();
  if (now - lastButtonScanMs < BUTTON_SCAN_MS) return;
  lastButtonScanMs = now;

  // Only allow preset changes when lockbox is idle
  if (!hasStatus){
    return;
  } 
  if (now - lastStatusTimeMs > STATUS_TIMEOUT_MS){
    return;
  }
  if (lastStatus.state != STATE_IDLE){
    return;
  }

  bool btn = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && btn == LOW && now - lastButtonPressMs > BUTTON_DEBOUNCE_MS) {
    lastButtonPressMs = now;
    uint8_t nextPreset = (localPreset % 3) + 1;


    KeyPacket pkt;
    pkt.type = KEY_SET_PRESET;
    pkt.preset = nextPreset;
    esp_now_send(BOX_MAC, (uint8_t*)&pkt, sizeof(pkt));

    Serial.print("Requested preset ");
    Serial.println(nextPreset);
  }

  lastButtonState = btn;
}




//  DISPLAY 
const char* stateToText(uint8_t s) {
  switch (s) {
    case STATE_IDLE: return "IDLE";
    case STATE_CONFIRMING: return "CLOSE LID";
    case STATE_LOCKING: return "LOCKING";
    case STATE_LOCKED_FIRST: return "SESSION 1/2";
    case STATE_UNLOCKING_GRACE: return "UNLOCKING";
    case STATE_GRACE_WAIT: return "BREAK TIME";
    case STATE_GRACE_ALARM: return "PUT IT BACK!";
    case STATE_LOCKING_SECOND: return "LOCKING";
    case STATE_LOCKED_SECOND: return "SESSION 2/2";
    case STATE_UNLOCKING_FINAL: return "UNLOCKING";
    case STATE_COMPLETE: return "COMPLETE!";
    default: return "UNKNOWN";
  }
}



void drawNoSignalScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso20_tr);
  u8g2.drawStr(10, 30, "NO SIGNAL");
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(10, 50, "Waiting for lockbox...");
  u8g2.sendBuffer();
}

void drawStatusScreen() {
  u8g2.clearBuffer();

  const char* stateTxt = stateToText(lastStatus.state);
  if (strlen(stateTxt) > 11){
     u8g2.setFont(u8g2_font_logisoso16_tr);
  }
   
  else{
    u8g2.setFont(u8g2_font_logisoso18_tr);
  }
    

  int16_t x_state = 64 - (u8g2.getStrWidth(stateTxt) / 2);
  u8g2.drawStr(x_state, 20, stateTxt);

  if (lastStatus.remainingMs > 0) {
    unsigned long totalSeconds = lastStatus.remainingMs / 1000;
    unsigned long minutes = totalSeconds / 60;
    unsigned long seconds = totalSeconds % 60;

    u8g2.setFont(u8g2_font_logisoso24_tn);
    char timeBuf[12];
    if (minutes > 0){
      snprintf(timeBuf, sizeof(timeBuf), "%lu:%02lu", minutes, seconds);
    }
    else{
      snprintf(timeBuf, sizeof(timeBuf), "%lu", seconds);
    }

    int16_t x_time = 64 - (u8g2.getStrWidth(timeBuf) / 2);
    u8g2.drawStr(x_time, 60, timeBuf);

  }

  if (lastStatus.state == STATE_IDLE){

    u8g2.setFont(u8g2_font_6x13_tf);
    char buf[12];
    snprintf(buf, sizeof(buf), "MODE %d", localPreset);

    int16_t x = 64 - (u8g2.getStrWidth(buf) / 2);
    u8g2.drawStr(x, 64, buf);
  }

  u8g2.sendBuffer();
}

void updateDisplayIfDue() {
  unsigned long now = millis();
  if (now - lastDisplayMs < DISPLAY_INTERVAL_MS){
    return;
  }
  lastDisplayMs = now;

  if (!hasStatus || (now - lastStatusTimeMs > STATUS_TIMEOUT_MS)) {
    drawNoSignalScreen();
  } else {
    drawStatusScreen();
  }
}

// SETUP / LOOP 
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(4, 20, "Key booting...");
  u8g2.sendBuffer();

  setupEspNow();
}

void loop() {
  sendHeartbeatIfDue();
  checkButtonIfDue();
  updateDisplayIfDue();
  yield();
}