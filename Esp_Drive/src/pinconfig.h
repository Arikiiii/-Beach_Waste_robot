#pragma once
// ============================================================
//  pinconfig.h  -  ค่าตั้งค่าทั้งหมดของ ESP ขับเคลื่อน (ขา / ความเร็ว / เวลา)
//  แก้ที่ไฟล์นี้ไฟล์เดียว ไม่ต้องไปแก้ใน main.cpp
// ============================================================

// ===================== ขา (PIN) =====================
// เลี่ยง: GPIO 0,2,5,12,15 (strapping บูตเพี้ยนได้), 1,3 (USB Serial ไป Pi), 6-11 (flash)
//         34-39 (อินพุตอย่างเดียว ไม่มี pull-up ในตัว)

// === + LEFT MOTOR PIN + ===
#define PIN_L_PWM   16
#define PIN_L_DIR   17

// === + RIGHT MOTOR PIN + ===
#define PIN_R_PWM   18
#define PIN_R_DIR   19

// === + LED PIN + ===
#define PIN_LED_Y   25     // เหลือง = สถานะงาน
#define PIN_LED_G   26     // เขียว  = รีโมทเชื่อมต่อ
#define PIN_LED_AUTO 22    // ไฟบอกโหมดออโต้ (ติด = เลือกโหมดออโต้ไว้ / กะพริบเร็ว = นับถอยหลังก่อนเริ่มวิ่ง)

// === + BUTTON PIN + ===
#define PIN_BUTTON   27    // ปุ่มหลัก: สแตนบาย <-> ทำงาน   (ต่อเข้า 3.3V, กด = HIGH)
#define PIN_BTN_MODE 13    // ปุ่มสลับโหมด: รีโมท <-> ออโต้ (ต่อเข้า 3.3V, กด = HIGH)

// === + SENSOR PIN (E18-D80NK ติดหน้ารถ) + ===
// ขาสัญญาณ (สายดำ) ต่อเข้า GPIO นี้ตรง ๆ  ใช้ INPUT_PULLUP ในตัว (E18-D80NK เป็น NPN open-collector)
// ห้ามต่อ pull-up ไป 5V เด็ดขาด เพราะ ESP32 ทนได้ 3.3V; ถ้าสายยาวมีสัญญาณรบกวนให้เพิ่ม 10k ไป 3.3V
// ถ้าซ้าย/ขวาสลับกันตอนติดตั้งจริง ให้สลับเลขขาสองบรรทัดนี้
#define PIN_SENSOR_L 32    // เซนเซอร์หน้าซ้าย
#define PIN_SENSOR_R 33    // เซนเซอร์หน้าขวา

// ===================== ระดับสัญญาณ =====================
#define LED_ON   LOW
#define LED_OFF  HIGH
#define BTN_PRESSED_LEVEL   HIGH   // ปุ่มกด = HIGH (ต่อเข้า 3.3V + pull-down ในตัว)
#define SENSOR_DETECT_LEVEL LOW    // E18-D80NK เจอวัตถุ = LOW

// ===================== มอเตอร์ / โหมดรีโมท =====================
const bool  INVERT_L = true;       // มอเตอร์ซ้ายติดตั้งกลับทิศ
const bool  INVERT_R = false;      // ถ้ามอเตอร์ข้างไหนหมุนกลับทิศ ให้สลับค่า
const float DRIVE_SPEED = 0.80f;   // สเกลความเร็วสูงสุดโหมดรีโมท
const int   MAX_PWM  = 255;
const float TURN_REDUCTION = 0.6f; // ยิ่งดันหน้า/ถอยแรง การเลี้ยวยิ่งลดลง (0 = ไม่ลด, 1 = เหลือ 0 ที่ความเร็วเต็ม)
const int   PWM_FREQ = 1000;
const int   PWM_BITS = 8;

// ---- ramp: เร่ง/ลดความเร็วแบบนุ่ม ----
const unsigned long RAMP_INTERVAL_MS = 10;
const int RAMP_UP_STEP   = 12;     // PWM ต่อ 10 ms
const int RAMP_DOWN_STEP = 15;

// ===================== เวลา / ระบบ =====================
const unsigned long FAILSAFE_MS     = 300;    // ขาดแพ็กเก็ตรีโมทเกินนี้ = หยุด
const unsigned long GAP_WARN_MS     = 250;    // แพ็กเก็ตห่างเกินนี้ขณะขับ -> พิมพ์ GAP,<ms>
const unsigned long DEBOUNCE_MS     = 50;
const unsigned long BLINK_MS        = 500;
const unsigned long BEAT_MS         = 500;    // ส่งสถานะให้ Pi ทุกกี่ ms
const unsigned long BUSY_TIMEOUT_MS = 3000;   // Pi หายไปนานเท่านี้ -> เลิก busy เอง
const bool DEBUG_DRIVE = true;                // พิมพ์ D,... ทุก 300 ms ตอนเคลื่อนที่

// ---- เลือกช่อง ESP-NOW ----
const int CH_MIN = 1;
const int CH_MAX = 13;
const int SCAN_PASSES = 2;

// ===================== โหมดออโต้ (วิ่งสุ่ม + หลบสิ่งกีดขวาง) =====================
const int AUTO_SPEED_PCT = 65;                // ความเร็ววิ่งตรง/ถอย (% ของ MAX_PWM)  แนะนำ 50-60
const int AUTO_TURN_PCT  = 65;                // ความเร็วหมุนตัวหลบ (% ของ MAX_PWM)
const unsigned long AUTO_START_DELAY_MS = 2000;  // หน่วงก่อนเริ่มวิ่งเมื่อเข้าโหมดออโต้ (ให้ถอยมือออก) ตั้ง 0 = ไม่หน่วง

// ---- เซนเซอร์ ----
const unsigned long SENSOR_FILTER_MS = 20;    // สัญญาณต้องคงที่นานเท่านี้ถึงนับว่าเจอ/ไม่เจอ (กันสัญญาณกระตุก)

// ---- หลบสิ่งกีดขวาง ----
// เจอซ้าย -> หมุนขวา / เจอขวา -> หมุนซ้าย / เจอทั้งสอง -> ถอยก่อน แล้วหมุนสลับซ้าย/ขวาทีละครั้ง
const unsigned long AVOID_BACK_MS         = 600;    // เวลาถอย (หน้ารถเจอพร้อมกัน) ** หลังรถไม่มีเซนเซอร์ **
const unsigned long AVOID_TURN_MS         = 800;    // เวลาหมุนหลบ (คงที่ ไม่สุ่ม)
const unsigned long AVOID_TURN_TIMEOUT_MS = 2500;   // หมุนนานเท่านี้แล้วยังเจออยู่ -> ถอยแล้วลองใหม่ (ต้องมากกว่า AVOID_TURN_MS)

// ---- วิ่งสุ่ม (เลี้ยวโค้งเบา ๆ เป็นช่วง ๆ ระหว่างวิ่งตรง) ----
const bool WANDER_ENABLE = true;
const unsigned long WANDER_GAP_MIN_MS = 3000;       // วิ่งตรงนานเท่านี้ก่อนจะโค้งครั้งถัดไป (สุ่มระหว่าง MIN-MAX)
const unsigned long WANDER_GAP_MAX_MS = 8000;
const unsigned long WANDER_LEN_MIN_MS = 500;        // โค้งนานเท่านี้ (สุ่ม)
const unsigned long WANDER_LEN_MAX_MS = 1500;
const int WANDER_INNER_PCT = 40;                    // ล้อด้านในโค้งหมุนกี่ % ของล้อนอก (ต่ำ = โค้งแรง)
const bool DEBUG_AUTO = true;                       // พิมพ์ A,<state>,L=,R= ทุกครั้งที่เปลี่ยนสถานะออโต้
