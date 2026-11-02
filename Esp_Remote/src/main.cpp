#include <Arduino.h>
#include "config.h"
/*
  ESP-NOW SENDER (รีโมท)
  ----------------------------------------------------------
  อ่านจอยสติ๊กแบบกดได้ แล้วส่งคำสั่งไปยังบอร์ดรถ (Receiver) ผ่าน ESP-NOW

  การต่อสาย (ESP32 DevKit):
    จอยสติ๊ก VCC -> 3.3V   (ห้ามใช้ 5V เพราะ ADC ของ ESP32 รับได้ไม่เกิน 3.3V)
    จอยสติ๊ก GND -> GND
    จอยสติ๊ก VRx -> GPIO34
    จอยสติ๊ก VRy -> GPIO35
    จอยสติ๊ก SW  -> GPIO32  (ปุ่มกดในตัว ต่อลง GND เมื่อกด ใช้ INPUT_PULLUP ภายใน)

  ปุ่มกดจอยสติ๊ก: กด 1 ครั้ง = สั่งเก็บของ (ไม่มีปุ่มยกเลิก)
    - ส่งเป็น "ตัวนับการกด" (pick) ไม่ใช่ระดับปุ่ม -> แพ็กเก็ตหลุดไปบางอันก็ไม่พลาดการกด
  LED บนบอร์ด GPIO2:  ติดค้าง = เชื่อมต่อรถแล้ว,  กะพริบ = กำลังค้นหารถ

  ช่องสัญญาณ ESP-NOW:
    ฝั่งรถ (Receiver) สแกน Wi-Fi รอบตัวตอนเปิดเครื่อง แล้วเลือกช่องที่ถูกรบกวนน้อยที่สุดเอง
    ฝั่งรีโมทนี้ "ไล่หา" ช่องของรถเอง (ส่งทดลองทีละช่อง จนมี ACK ตอบกลับ) จึงไม่ต้องตั้งช่องมือ
    ถ้าสัญญาณหลุดต่อเนื่อง จะไล่หาใหม่อัตโนมัติ

  ใช้ได้ทั้ง Arduino-ESP32 core 2.x และ 3.x
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ================== ตั้งค่า ==================
// ใส่ MAC address ของบอร์ด Receiver (ดูได้จาก Serial Monitor ตอนเปิดบอร์ด Receiver)
uint8_t receiverMAC[6] = {0x00, 0x4B, 0x12, 0x23, 0xEF, 0x4C};

// ================== โครงสร้างข้อมูลที่ส่ง (ต้องเหมือนฝั่ง Receiver ทุกประการ) ==================
typedef struct __attribute__((packed)) {
  int16_t x;
  int16_t y;
  uint8_t pick;  // ตัวนับการกดปุ่มเก็บของ (เพิ่มขึ้น 1 ทุกครั้งที่กด วนกลับที่ 255->0)
  uint8_t seq;
} ControlPacket;

int centerX = 2048;
int centerY = 2048;
uint8_t pickCount = 0;
int lastX = 0, lastY = 0;           // ค่าจอยล่าสุดที่ส่ง (ไว้กันกดปุ่มพลาดตอนดันจอยสุด)
uint8_t seqNo = 0;
uint32_t lastSend = 0;
uint32_t lastPrint = 0;

volatile uint32_t sendFail = 0;     // นับรวม (ไว้ debug)
volatile uint32_t failStreak = 0;   // ส่งไม่สำเร็จติดกันกี่ครั้ง
volatile bool sendDone = false;     // callback ตอบกลับแล้ว (ใช้ตอนค้นหาช่อง)
volatile bool sendOk = false;

bool linked = false;
uint8_t curChannel = 1;

// ================== Callback เมื่อส่งเสร็จ (รองรับ core 2.x และ 3.x) ==================
void handleSendResult(esp_now_send_status_t status) {
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  if (sendOk) {
    failStreak = 0;
  } else {
    sendFail++;
    failStreak++;
  }
  sendDone = true;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  handleSendResult(status);
}

// ================== ตั้งช่องสัญญาณ Wi-Fi/ESP-NOW ==================
void setChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  curChannel = ch;
}

// ================== อ่านแกนจอยสติ๊ก -> ค่า -255..255 ==================
int readRaw(int pin) {
  // อ่าน 8 ครั้ง ตัดค่าต่ำสุด/สูงสุดทิ้ง แล้วเฉลี่ย 6 ค่าที่เหลือ
  // (กันค่าสะดุดเป็นพัก ๆ จาก ADC ตอนวิทยุ ESP-NOW กำลังส่ง)
  int mn = 4095, mx = 0;
  long sum = 0;
  analogRead(pin);                       // ทิ้งการอ่านแรก: ADC ของ ESP32 ยังมีประจุค้างจากช่องก่อนหน้า (ทำให้ x เพี้ยนตาม y)
  for (int i = 0; i < 8; i++) {
    int v = analogRead(pin);
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return (sum - mn - mx) / 6;
}

int readAxis(int pin, int center, bool invert) {
  int d = readRaw(pin) - center;
  int v = 0;
  if (d > DEADZONE) {
    // OUTER_ZONE = ช่วงปลายสุดของ ADC ที่ถือว่าเต็มสเกลแล้ว
    // (ADC ของ ESP32 ไม่เป็นเส้นตรงใกล้ 0V และ 3.3V และจอยบางตัวดันไม่ถึงปลายจริง)
    int range = (4095 - center) - DEADZONE - OUTER_ZONE;
    v = (range > 0) ? map(d - DEADZONE, 0, range, 0, 255) : 255;
  } else if (d < -DEADZONE) {
    int range = center - DEADZONE - OUTER_ZONE;
    v = (range > 0) ? map(d + DEADZONE, 0, -range, 0, -255) : -255;
  }
  v = constrain(v, -255, 255);
  return invert ? -v : v;
}

// ================== ปุ่มกดจอยสติ๊ก (สั่งเก็บของ) ==================
void handleButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static uint32_t lastChange = 0;

  bool reading = digitalRead(PIN_JOY_SW);
  if (reading != lastReading) {
    lastChange = millis();
    lastReading = reading;
  }
  if (millis() - lastChange > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {          // กดลง 1 ครั้ง = สั่งเก็บ 1 ครั้ง
      // จอยบางตัวพอดันแรง ๆ จะไปกดสวิตช์ในตัวโดยไม่ตั้งใจ -> ไม่นับเมื่อจอยไม่อยู่กลาง
      if (abs(lastX) <= PICK_BLOCK_AXIS && abs(lastY) <= PICK_BLOCK_AXIS) {
        pickCount++;
      }
    }
  }
}

// ================== ส่งแพ็กเก็ต 1 ชุด ==================
void fillPacket(ControlPacket &p, bool neutral) {
  p.x = neutral ? 0 : readAxis(PIN_JOY_X, centerX, INVERT_X);
  p.y = neutral ? 0 : readAxis(PIN_JOY_Y, centerY, INVERT_Y);
  lastX = p.x;
  lastY = p.y;
  p.pick = pickCount;
  p.seq = seqNo++;
}

// ================== ไล่หาช่องสัญญาณของรถ ==================
// ส่งแพ็กเก็ตนิ่ง ๆ ทีละช่อง เริ่มจากช่องล่าสุด ช่องไหนมี ACK ตอบ = เจอรถ
bool searchChannel() {
  const uint8_t n = ESPNOW_CH_MAX - ESPNOW_CH_MIN + 1;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t ch = ESPNOW_CH_MIN + ((curChannel - ESPNOW_CH_MIN + i) % n);
    setChannel(ch);

    ControlPacket p;
    fillPacket(p, true);
    sendDone = false;
    if (esp_now_send(receiverMAC, (uint8_t *)&p, sizeof(p)) != ESP_OK) continue;

    uint32_t t = millis();
    while (!sendDone && millis() - t < ESPNOW_SEARCH_WAIT_MS) delay(1);

    if (sendDone && sendOk) {
      failStreak = 0;
      linked = true;
      Serial.printf("Linked on channel %u\n", ch);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_JOY_X, ADC_11db);   // อ่านได้เต็ม 0-3.3V
  analogSetPinAttenuation(PIN_JOY_Y, ADC_11db);

  // หาจุดกึ่งกลางของจอยสติ๊ก (ตอนเปิดเครื่องอย่าแตะจอย)
  long sx = 0, sy = 0;
  for (int i = 0; i < 64; i++) {
    sx += readRaw(PIN_JOY_X);
    sy += readRaw(PIN_JOY_Y);
    delay(5);
  }
  centerX = sx / 64;
  centerY = sy / 64;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);          // ปิด power-save เพื่อให้ส่ง/รับ ESP-NOW ต่อเนื่อง
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;          // 0 = ใช้ช่องปัจจุบันของบอร์ด (เราสลับช่องเองด้วย setChannel)
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) delay(1000);
  }

  setChannel(ESPNOW_CH_MIN);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Joystick center  X=%d  Y=%d\n", centerX, centerY);
  Serial.println("Searching for receiver...");
}

void loop() {
  handleButton();

  // LED: เชื่อมต่อแล้วติดค้าง / ยังไม่เจอรถกะพริบ
  digitalWrite(PIN_LED, (linked || ((millis() / 200) % 2)) ? HIGH : LOW);

  // ยังไม่เชื่อมต่อ -> ไล่หาช่องของรถ
  if (!linked) {
    searchChannel();
    return;
  }

  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    ControlPacket p;
    fillPacket(p, false);
    esp_now_send(receiverMAC, (uint8_t *)&p, sizeof(p));

    // ส่งไม่สำเร็จติดกันนานเกินกำหนด -> ถือว่าหลุด ไปไล่หาช่องใหม่
    if (failStreak >= ESPNOW_LOST_FAILS) {
      linked = false;
      Serial.println("Link lost -> searching...");
    }

#if DEBUG_SERIAL
    if (now - lastPrint >= 300) {
      lastPrint = now;
      Serial.printf("ch=%u x=%4d  y=%4d  pick=%u  sendFail=%lu\n",
                    curChannel, p.x, p.y, p.pick, (unsigned long)sendFail);
    }
#endif
  }
}