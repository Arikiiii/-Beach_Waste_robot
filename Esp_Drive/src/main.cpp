#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "pinconfig.h"
// ESP32 ขับเคลื่อน  (รีโมท ESP-NOW  /  โหมดออโต้ + ต่อ Pi ผ่าน USB Serial)
//
//  ค่าตั้งค่า/ขา/ความเร็ว/เวลา ทั้งหมดอยู่ใน pinconfig.h
//
//  โหมดรีโมท : รีโมท -> ESP (ESP-NOW)   แพ็กเก็ต {x, y, pick, seq}   x,y = -255..255
//     ESP ผสมสัญญาณจอยเป็นมอเตอร์ซ้าย/ขวาเอง
//     pick เพิ่มขึ้นทุกครั้งที่กดปุ่มจอย -> ESP ส่ง "P" ให้ Pi เพื่อสั่งเก็บของ
//  โหมดออโต้ : ปิด ESP-NOW  รถวิ่งสุ่มเอง + หลบสิ่งกีดขวางด้วย E18-D80NK 2 ตัวหน้ารถ
//     เจอซ้าย -> เลี้ยวขวา / เจอขวา -> เลี้ยวซ้าย / เจอพร้อมกัน -> ถอยแล้วหมุนออก
//     Pi รัน AI ตลอดเวลาที่อยู่ในโหมดออโต้ เจอวัตถุ -> Pi ส่ง B,1 -> ESP ล็อกล้อ -> แขนเก็บ -> B,0 -> วิ่งต่อ
//
//  Pi -> ESP (USB Serial)
//     B,1   AI/แขนกำลังทำงาน (Pi ต้องส่งซ้ำทุก ~1 วินาที ถ้าขาดเกิน BUSY_TIMEOUT_MS จะปลดเอง)
//     B,0   AI/แขนทำงานเสร็จ
//  ESP -> Pi (USB Serial)
//     S,<mode>,<link>,<ch>,<auto>   ทุก 500 ms และทุกครั้งที่สถานะเปลี่ยน
//                        mode: 0=สแตนบาย  1=ทำงาน
//                        link: 1=รีโมทเชื่อมต่อ 0=ไม่มีสัญญาณ (ในโหมดออโต้เป็น 0 เสมอ)
//                        ch  : ช่อง ESP-NOW      auto: 1=เลือกโหมดออโต้ไว้ 0=โหมดรีโมท
//     P                  ผู้ใช้กดปุ่มเก็บของบนรีโมท (ส่งเฉพาะโหมดรีโมท+ทำงาน+ไม่ busy)
//     GAP,<ms>           คำสั่งจากรีโมทห่างผิดปกติระหว่างขับ
//     A,<state>,L=,R=    (ดีบัก DEBUG_AUTO) สถานะโหมดออโต้
//
//  ปุ่ม
//     ปุ่มหลัก (GPIO27)  : เปิดเครื่อง = สแตนบาย (ไม่ขยับ ไม่ทำอะไร ทั้งโหมดรีโมทและออโต้)
//                          กด 1 ครั้ง = เริ่มทำงาน   กดซ้ำ = สลับ ทำงาน/สแตนบาย
//     ปุ่มโหมด (PIN_BTN_MODE): สลับ รีโมท <-> ออโต้ (ใช้ได้ทั้งสแตนบายและทำงาน ยกเว้นตอนกำลังเก็บของ)
//
//  ไฟ
//     เหลือง = สถานะงาน : ติดค้าง=สแตนบาย  กะพริบเร็ว=กำลังเก็บวัตถุ  ดับ=ทำงานปกติ
//     เขียว  = รีโมท    : ติดค้าง=เชื่อมต่อ  กะพริบช้า=ไม่มีสัญญาณ  ดับ=อยู่โหมดออโต้ (ESP-NOW ปิด)
//     ไฟออโต้ (PIN_LED_AUTO) : ติด=เลือกโหมดออโต้  กะพริบเร็ว=นับถอยหลังก่อนเริ่มวิ่ง  ดับ=โหมดรีโมท
//
//  ถ้าไม่ได้รับแพ็กเก็ตจากรีโมทนานเกิน FAILSAFE_MS จะหยุดมอเตอร์เอง
//  ช่องสัญญาณ: ตอนเปิดเครื่องสแกน Wi-Fi แล้วเลือกช่องที่ถูกรบกวนน้อยที่สุด (ฝั่งรีโมทไล่หาช่องนี้เอง)

#define CH_L 0
#define CH_R 1

// ================== โครงสร้างข้อมูลที่รับ (ต้องเหมือนฝั่งรีโมททุกประการ) ==================
typedef struct __attribute__((packed)) {
  int16_t x;
  int16_t y;
  uint8_t pick;
  uint8_t seq;
} ControlPacket;

portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket rxPkt;
volatile bool rxNew = false;

unsigned long lastCmdMs = 0, lastBusyMs = 0;
unsigned long lastBeatMs = 0, lastBlinkMs = 0, lastRampMs = 0, lastDbgMs = 0;
int dbgX = 0, dbgY = 0;
int targetL = 0, targetR = 0, curL = 0, curR = 0;
bool lastMoving = false;
bool linked = false;     // รีโมทเชื่อมต่ออยู่ (ได้รับแพ็กเก็ตล่าสุดภายใน FAILSAFE_MS)
bool havePick = false;
uint8_t lastPick = 0;
bool active   = false;   // false = สแตนบาย (เปิดเครื่องมาเป็นสแตนบาย)
bool autoMode = false;   // false = โหมดรีโมท, true = โหมดออโต้
bool aiBusy   = false;   // AI/แขนกำลังทำงาน
bool blinkOn  = false;
bool espnowOn = false;
uint8_t channel = 1;
String rx;

// ================== ปุ่ม / เซนเซอร์ ==================
struct Button { uint8_t pin; bool stable; bool last; unsigned long changeMs; };
Button btnMain = { PIN_BUTTON,   false, false, 0 };
Button btnMode = { PIN_BTN_MODE, false, false, 0 };

// คืน true หนึ่งครั้งตอนกดลง (หลังผ่าน debounce)
bool buttonPressed(Button &b, unsigned long now) {
  bool raw = digitalRead(b.pin);
  if (raw != b.last) { b.changeMs = now; b.last = raw; }
  if (now - b.changeMs > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    return raw == (bool)BTN_PRESSED_LEVEL;
  }
  return false;
}

struct Sensor { uint8_t pin; bool raw; bool blocked; unsigned long changeMs; };
Sensor sensL = { PIN_SENSOR_L, false, false, 0 };
Sensor sensR = { PIN_SENSOR_R, false, false, 0 };

void updateSensor(Sensor &s, unsigned long now) {
  bool r = (digitalRead(s.pin) == SENSOR_DETECT_LEVEL);
  if (r != s.raw) { s.raw = r; s.changeMs = now; }
  if (now - s.changeMs >= SENSOR_FILTER_MS) s.blocked = r;   // ต้องคงที่นานพอถึงเปลี่ยนค่า
}

// ================== ESP-NOW รับข้อมูล (รองรับ core 2.x และ 3.x) ==================
void storePacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(ControlPacket)) return;
  portENTER_CRITICAL(&rxMux);
  memcpy(&rxPkt, data, sizeof(ControlPacket));
  rxNew = true;
  portEXIT_CRITICAL(&rxMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  storePacket(data, len);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  storePacket(data, len);
}
#endif

// ================== เลือกช่องที่ถูกรบกวนน้อยที่สุด ==================
void setChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  channel = ch;
}

uint8_t pickBestChannel() {
  // น้ำหนักการซ้อนทับของช่อง 2.4 GHz ตามระยะห่างช่อง (0..4)
  const float W[5] = {1.0f, 0.8f, 0.55f, 0.3f, 0.1f};
  float score[CH_MAX + 1];
  for (int c = 0; c <= CH_MAX; c++) score[c] = 0;

  for (int pass = 0; pass < SCAN_PASSES; pass++) {
    int n = WiFi.scanNetworks(false, true);   // บล็อกจนสแกนเสร็จ รวม SSID ซ่อน
    for (int i = 0; i < n; i++) {
      int apCh = WiFi.channel(i);
      if (apCh < CH_MIN || apCh > CH_MAX) continue;
      float strength = constrain(WiFi.RSSI(i) + 100, 5, 70);   // ยิ่งแรงยิ่งรบกวนมาก
      for (int c = CH_MIN; c <= CH_MAX; c++) {
        int d = abs(c - apCh);
        if (d <= 4) score[c] += strength * W[d];
      }
    }
    WiFi.scanDelete();
  }

  int best = CH_MIN;
  for (int c = CH_MIN; c <= CH_MAX; c++) {
    if (score[c] < score[best]) best = c;     // คะแนนเท่ากัน -> เอาช่องเลขน้อยกว่า
  }
  Serial.print("Channel scores:");
  for (int c = CH_MIN; c <= CH_MAX; c++) Serial.printf(" %d:%.0f", c, score[c]);
  Serial.println();
  return best;
}

// ================== เปิด/ปิด ESP-NOW (ปิดตอนโหมดออโต้) ==================
bool espnowEnable() {
  if (espnowOn) return true;
  setChannel(channel);                          // ใช้ช่องเดิมที่สแกนไว้ตอนเปิดเครื่อง
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  espnowOn = true;
  return true;
}

void espnowDisable() {
  if (!espnowOn) return;
  esp_now_unregister_recv_cb();
  esp_now_deinit();
  espnowOn = false;
  portENTER_CRITICAL(&rxMux);
  rxNew = false;                                // ทิ้งแพ็กเก็ตค้าง
  portEXIT_CRITICAL(&rxMux);
  linked = false;
  havePick = false;                             // กลับมาเชื่อมต่อใหม่ต้องซิงก์ pick ใหม่ (กันสั่งเก็บเอง)
  lastMoving = false;
}

// ================== PWM / มอเตอร์ ==================
void pwmSetup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setMotor(int pwmPin, int dirPin, int ch, int value, bool invert) {
  value = constrain(value, -255, 255);
  bool fwd = value >= 0;
  if (invert) fwd = !fwd;
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  pwmWrite(pwmPin, ch, abs(value));
}

void drive(int l, int r) {
  setMotor(PIN_L_PWM, PIN_L_DIR, CH_L, l, INVERT_L);
  setMotor(PIN_R_PWM, PIN_R_DIR, CH_R, r, INVERT_R);
}

// เดินเข้าหา target ทีละก้าว; ถ้าต้องกลับทิศ ให้ลดเป็น 0 ก่อนแล้วค่อยเร่งทิศใหม่
int stepToward(int cur, int target) {
  if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) target = 0;
  int step = (abs(target) < abs(cur)) ? RAMP_DOWN_STEP : RAMP_UP_STEP;
  int d = target - cur;
  if (d > step)  return cur + step;
  if (d < -step) return cur - step;
  return target;
}

void setTarget(int l, int r) {
  targetL = constrain(l, -255, 255);
  targetR = constrain(r, -255, 255);
}

// หยุดทันที ไม่ผ่าน ramp (ใช้ตอนแขนทำงาน / สแตนบาย / Pi หาย)
void hardStop() {
  targetL = targetR = curL = curR = 0;
  drive(0, 0);
}

void sendStatus() {
  Serial.printf("S,%d,%d,%u,%d\n", active ? 1 : 0, linked ? 1 : 0, channel, autoMode ? 1 : 0);
  lastBeatMs = millis();
}

// ================== โหมดออโต้: วิ่งสุ่ม + หลบสิ่งกีดขวาง ==================
enum AutoState : uint8_t { A_FORWARD, A_REVERSE, A_TURN_L, A_TURN_R };
const char *const AUTO_NAME[] = { "FWD", "REV", "TURN_L", "TURN_R" };

AutoState autoState = A_FORWARD;
unsigned long autoStateMs = 0, autoStateDur = 0;
int wanderDir = 0;                    // -1 โค้งซ้าย, 0 ตรง, +1 โค้งขวา
unsigned long nextWanderMs = 0, wanderEndMs = 0;
bool autoCountdown = false;           // นับถอยหลังก่อนเริ่มวิ่ง
unsigned long autoCountdownMs = 0;
bool autoWasBusy = false;
bool nextTurnRight = true;            // ถอยแล้วหมุนออก: สลับซ้าย/ขวาทีละครั้ง (ไม่สุ่ม)
// หยุดชั่วคราวจาก Pi (คำสั่ง H) กันรถวิ่งเลยตอนเพิ่งเห็นวัตถุ ก่อนที่ B,1 จะมา
const unsigned long HOLD_MS = 600;
unsigned long holdUntilMs = 0;
bool autoHeld = false;
// หยุดรถทันทีที่ส่ง P (โหมดรีโมท) รอ B,1 จาก Pi ถ้า Pi ไม่ตอบภายในเวลานี้ให้ปล่อย
const unsigned long PICK_PENDING_MS = 1500;
bool pickPending = false;
unsigned long pickPendMs = 0;

unsigned long rnd(unsigned long lo, unsigned long hi) {   // สุ่มในช่วง lo..hi
  if (hi <= lo) return lo;
  return (unsigned long)random((long)lo, (long)hi + 1);
}

int pctToPwm(int pct) { return MAX_PWM * constrain(pct, 0, 100) / 100; }

// ตั้งเป้าหมายมอเตอร์ตามสถานะออโต้ปัจจุบัน
// ทิศ: ขวา = ล้อซ้ายเร็วกว่า (ตรงกับการผสมสัญญาณจอยด้านบน)
void autoApply() {
  int sp = pctToPwm(AUTO_SPEED_PCT);
  int tp = pctToPwm(AUTO_TURN_PCT);
  switch (autoState) {
    case A_FORWARD:
      if (wanderDir < 0)      setTarget(sp * WANDER_INNER_PCT / 100, sp);   // โค้งซ้าย: ล้อซ้ายช้ากว่า
      else if (wanderDir > 0) setTarget(sp, sp * WANDER_INNER_PCT / 100);   // โค้งขวา: ล้อขวาช้ากว่า
      else                    setTarget(sp, sp);
      break;
    case A_REVERSE: setTarget(-sp, -sp); break;
    case A_TURN_R:  setTarget(tp, -tp);  break;   // หมุนอยู่กับที่ไปทางขวา
    case A_TURN_L:  setTarget(-tp, tp);  break;   // หมุนอยู่กับที่ไปทางซ้าย
  }
}

void autoSet(AutoState s, unsigned long now, unsigned long dur) {
  autoState = s;
  autoStateMs = now;
  autoStateDur = dur;
  if (s == A_FORWARD) {
    wanderDir = 0;
    nextWanderMs = now + rnd(WANDER_GAP_MIN_MS, WANDER_GAP_MAX_MS);
  }
  autoApply();
  if (DEBUG_AUTO) Serial.printf("A,%s,L=%d,R=%d\n", AUTO_NAME[s], sensL.blocked, sensR.blocked);
}

void autoStartTurn(bool right, unsigned long now) {
  autoSet(right ? A_TURN_R : A_TURN_L, now, AVOID_TURN_MS);
}

void autoBeginCountdown(unsigned long now) {
  autoCountdown = true;
  autoCountdownMs = now;
  setTarget(0, 0);
}

void autoUpdate(unsigned long now) {
  if (!autoMode || !active) return;

  // AI/แขนทำงาน: hardStop ไว้แล้วตอนรับ B,1 -> หยุดตัดสินใจ จนกว่า B,0
  if (aiBusy) { autoWasBusy = true; autoCountdown = false; return; }
  if (autoWasBusy) { autoWasBusy = false; autoBeginCountdown(now); return; }   // B,0 แล้วหน่วงก่อนวิ่ง ไม่พุ่งออกทันที

  if (autoCountdown) {
    if (now - autoCountdownMs < AUTO_START_DELAY_MS) return;
    autoCountdown = false;
    autoSet(A_FORWARD, now, 0);
    return;
  }

  // Pi สั่งหยุดชั่วคราว (เห็นวัตถุ กำลังยืนยัน) -> หยุดล้อ หมดเวลาแล้ววิ่งต่อเอง
  if ((long)(holdUntilMs - now) > 0) {
    if (!autoHeld) { autoHeld = true; hardStop(); }
    return;
  }
  if (autoHeld) { autoHeld = false; autoSet(A_FORWARD, now, 0); }

  bool L = sensL.blocked, R = sensR.blocked;
  unsigned long el = now - autoStateMs;

  switch (autoState) {
    case A_FORWARD:
      if (L && R)      autoSet(A_REVERSE, now, AVOID_BACK_MS);   // เจอพร้อมกัน -> ถอยก่อน
      else if (L)      autoStartTurn(true,  now);                // เจอซ้าย -> เลี้ยวขวา
      else if (R)      autoStartTurn(false, now);                // เจอขวา -> เลี้ยวซ้าย
      else if (wanderDir != 0) {                                 // กำลังโค้งสุ่มอยู่ -> ครบเวลาแล้วกลับตรง
        if (now >= wanderEndMs) {
          wanderDir = 0;
          nextWanderMs = now + rnd(WANDER_GAP_MIN_MS, WANDER_GAP_MAX_MS);
          autoApply();
        }
      } else if (WANDER_ENABLE && now >= nextWanderMs) {         // ถึงเวลาโค้งสุ่ม
        wanderDir = (random(2) == 0) ? -1 : 1;
        wanderEndMs = now + rnd(WANDER_LEN_MIN_MS, WANDER_LEN_MAX_MS);
        autoApply();
      }
      break;

    case A_REVERSE:
      if (el >= autoStateDur) { autoStartTurn(nextTurnRight, now); nextTurnRight = !nextTurnRight; }   // ถอยครบ -> หมุนออก สลับทิศทีละครั้ง
      break;

    case A_TURN_L:
    case A_TURN_R: {
      bool towardBlocked = (autoState == A_TURN_R) ? R : L;       // เซนเซอร์ด้านที่กำลังหมุนเข้าไปหา
      if (towardBlocked || (el >= AVOID_TURN_TIMEOUT_MS && (L || R))) {
        autoSet(A_REVERSE, now, AVOID_BACK_MS);                   // หมุนไปเจออีกฝั่ง/ติดมุม -> ถอยแล้วลองใหม่
      } else if (el >= autoStateDur && !L && !R) {
        autoSet(A_FORWARD, now, 0);                               // หมุนครบเวลาและโล่งแล้ว -> วิ่งต่อ
      }
      break;
    }
  }
}

// ================== สลับโหมด รีโมท <-> ออโต้ ==================
void setAutoMode(bool on, unsigned long now) {
  if (on == autoMode) return;
  autoMode = on;
  hardStop();
  autoCountdown = false;
  autoWasBusy = false;
  if (on) {
    espnowDisable();                                  // ออโต้: ปิด ESP-NOW
    if (active) autoBeginCountdown(now);
  } else {
    if (!espnowEnable()) Serial.println("ESP-NOW init failed");
  }
  sendStatus();
}

// ================== รับคำสั่งจาก Pi ==================
void handleLine(const String &s) {
  int b;
  if (s == "H") { holdUntilMs = millis() + HOLD_MS; return; }
  if (s.startsWith("B,") && sscanf(s.c_str(), "B,%d", &b) == 1) {
    if (b != 0) {
      lastBusyMs = millis();
      pickPending = false;                              // Pi รับงานแล้ว
      if (!aiBusy) { aiBusy = true; hardStop(); }     // ล็อกล้อทันที
    } else {
      aiBusy = false;
    }
  }
}

// ================== ประมวลผลแพ็กเก็ตจากรีโมท ==================
void handlePacket(const ControlPacket &p, unsigned long now) {
  unsigned long gap = now - lastCmdMs;
  if (linked && lastMoving && gap > GAP_WARN_MS) Serial.printf("GAP,%lu\n", gap);

  bool wasLinked = linked;
  linked = true;
  lastCmdMs = now;

  bool remoteOk = active && !aiBusy && !autoMode && !pickPending;     // รับคำสั่งรีโมทได้เฉพาะ: ทำงาน + ไม่ busy + โหมดรีโมท

  // กดเก็บของ: ตัวนับ pick เปลี่ยน = มีการกด 1 ครั้ง
  // แพ็กเก็ตแรก/เพิ่งกลับมาเชื่อมต่อ -> แค่ซิงก์ค่า ไม่ถือเป็นการกด (กันสั่งเก็บเองตอนสัญญาณกลับมา)
  if (!wasLinked || !havePick) {
    lastPick = p.pick;
    havePick = true;
    sendStatus();
  } else if (p.pick != lastPick) {
    lastPick = p.pick;
    if (remoteOk) {
      Serial.println("P");
      pickPending = true;                         // หยุดรถทันที รอ B,1 จาก Pi
      pickPendMs = now;
      hardStop();
      remoteOk = false;
    }
  }

  // ผสมสัญญาณจอยเป็นมอเตอร์ซ้าย/ขวา
  float fx = constrain(p.x / 255.0f, -1.0f, 1.0f);
  float fy = constrain(p.y / 255.0f, -1.0f, 1.0f);
  float turn = fx * (1.0f - TURN_REDUCTION * fabsf(fy));   // ยิ่ง |y| มาก เลี้ยวยิ่งน้อย (y=0 หมุนอยู่กับที่ได้เต็ม)
  float l = fy + turn, r = fy - turn;
  float m = max(1.0f, max(fabsf(l), fabsf(r)));
  l /= m;
  r /= m;

  dbgX = p.x;
  dbgY = p.y;
  lastMoving = (p.x != 0 || p.y != 0);
  if (remoteOk) setTarget((int)(l * DRIVE_SPEED * MAX_PWM), (int)(r * DRIVE_SPEED * MAX_PWM));
  else if (!autoMode) setTarget(0, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_DIR, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_AUTO, OUTPUT);
  digitalWrite(PIN_LED_Y, LED_OFF);
  digitalWrite(PIN_LED_G, LED_OFF);
  digitalWrite(PIN_LED_AUTO, LED_OFF);
  pinMode(PIN_BUTTON,   INPUT_PULLDOWN);   // ปุ่มต่อเข้า 3.3V จึงต้อง pull-down
  pinMode(PIN_BTN_MODE, INPUT_PULLDOWN);
  pinMode(PIN_SENSOR_L, INPUT_PULLUP);     // E18-D80NK = NPN open-collector (เจอ = LOW)
  pinMode(PIN_SENSOR_R, INPUT_PULLUP);
  pwmSetup(PIN_L_PWM, CH_L);
  pwmSetup(PIN_R_PWM, CH_R);
  drive(0, 0);
  rx.reserve(32);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);          // ปิด power-save เพื่อให้รับ ESP-NOW ต่อเนื่อง
  uint8_t best = pickBestChannel();        // ใช้เวลาไม่กี่วินาทีตอนเปิดเครื่อง
  setChannel(best);

  if (!espnowEnable()) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  Serial.print("Receiver MAC: ");           // เอาค่านี้ไปใส่ receiverMAC ในโค้ดรีโมท
  Serial.println(WiFi.macAddress());
  Serial.printf("ESP-NOW channel: %u\n", channel);
}

void loop() {
  unsigned long now = millis();

  // ---- รับคำสั่งจาก Pi ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rx.trim();
      if (rx.length()) handleLine(rx);
      rx = "";
    } else if (rx.length() < 30) {
      rx += c;
    }
  }

  // ---- รับแพ็กเก็ตจากรีโมท (โหมดออโต้ ESP-NOW ปิดอยู่ จึงไม่มีแพ็กเก็ตเข้า) ----
  if (rxNew) {
    ControlPacket p;
    portENTER_CRITICAL(&rxMux);
    memcpy(&p, &rxPkt, sizeof(p));
    rxNew = false;
    portEXIT_CRITICAL(&rxMux);
    handlePacket(p, now);
  }

  // ---- Pi หายระหว่าง busy -> ปลด busy เอง กันค้างตลอดไป ----
  if (aiBusy && now - lastBusyMs > BUSY_TIMEOUT_MS) {
    aiBusy = false;
    hardStop();
  }

  // ---- Pi ไม่ตอบหลังกดเก็บ -> ปล่อยรถ ----
  if (pickPending && now - pickPendMs > PICK_PENDING_MS) pickPending = false;

  // ---- failsafe: ขาดสัญญาณรีโมท = หยุด ----
  if (linked && now - lastCmdMs > FAILSAFE_MS) {
    linked = false;
    lastMoving = false;
    setTarget(0, 0);                   // ลดความเร็วลงนุ่ม ๆ ผ่าน ramp
    sendStatus();
  }

  // ---- เซนเซอร์ + ตัดสินใจโหมดออโต้ ----
  updateSensor(sensL, now);
  updateSensor(sensR, now);
  autoUpdate(now);

  // ---- ramp มอเตอร์ทุก 10 ms ----
  if (now - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    curL = stepToward(curL, targetL);
    curR = stepToward(curR, targetR);
    drive(curL, curR);
  }

  // ---- ปุ่มหลัก: สแตนบาย <-> ทำงาน (ไม่มีขั้นปลดล็อกแล้ว กดครั้งเดียวเริ่มได้) ----
  if (buttonPressed(btnMain, now) && !aiBusy) {
    active = !active;
    autoCountdown = false;
    if (!active) hardStop();
    else if (autoMode) autoBeginCountdown(now);   // เริ่มทำงานในโหมดออโต้: หน่วงก่อนวิ่ง
    sendStatus();
  }

  // ---- ปุ่มสลับโหมด รีโมท <-> ออโต้ ----
  if (buttonPressed(btnMode, now) && !aiBusy) {
    setAutoMode(!autoMode, now);
  }

  // ---- ดีบัก ----
  if (DEBUG_DRIVE && now - lastDbgMs >= 300) {
    lastDbgMs = now;
    if (curL != 0 || curR != 0 || targetL != 0 || targetR != 0) {
      Serial.printf("D,x=%d,y=%d,tgt=%d/%d,cur=%d/%d\n", dbgX, dbgY, targetL, targetR, curL, curR);
    }
  }

  // ---- ส่งสถานะให้ Pi ทุก 500 ms ----
  if (now - lastBeatMs >= BEAT_MS) sendStatus();

  // ---- ไฟ ----
  if (now - lastBlinkMs >= BLINK_MS) {
    lastBlinkMs = now;
    blinkOn = !blinkOn;
  }
  bool g, y, a;
  if (aiBusy)       y = ((now / 250) % 2);   // กำลังเก็บวัตถุ: เหลืองกะพริบเร็ว
  else if (!active) y = true;                // สแตนบาย: เหลืองติดค้าง
  else              y = false;               // ทำงานปกติ: เหลืองดับ

  if (autoMode) g = false;                   // โหมดออโต้: ESP-NOW ปิด ไม่มีสถานะรีโมท
  else          g = linked ? true : blinkOn; // รีโมท: เชื่อมต่อ=เขียวค้าง / ไม่มีสัญญาณ=เขียวกะพริบช้า

  if (!autoMode)          a = false;
  else if (autoCountdown) a = ((now / 150) % 2);   // นับถอยหลังก่อนวิ่ง: กะพริบเร็ว
  else                    a = true;

  digitalWrite(PIN_LED_G,    g ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_Y,    y ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_AUTO, a ? LED_ON : LED_OFF);
}
