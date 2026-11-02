#pragma once

#define PIN_JOY_X   34
#define PIN_JOY_Y   35
#define PIN_JOY_SW  32      // ปุ่มกดจอย = สั่งเก็บของ
#define PIN_LED     2       // ติดค้าง = เชื่อมต่อรถแล้ว / กะพริบ = กำลังค้นหา

#define INVERT_X    false   // ถ้าเลี้ยวซ้าย-ขวากลับด้าน ให้เปลี่ยนเป็น true
#define INVERT_Y    true  // ถ้าดันไปหน้าแล้วถอยหลัง ให้เปลี่ยนเป็น true
#define DEADZONE    200     // ช่วงตรงกลางที่ถือว่า "ไม่ขยับ" (หน่วย ADC 0-4095)
#define OUTER_ZONE  250     // ปลายสุดของจอย (หน่วย ADC) ที่ถือว่า "สุดแล้ว" = ±255 ถ้าดันสุดแล้วยังไม่ถึง 255 ให้เพิ่มค่านี้
#define PICK_BLOCK_AXIS  120   // ถ้าจอยเอียงเกินนี้ (0-255) จะไม่นับการกดปุ่มเก็บของ (กันกดพลาดตอนดันจอย) ตั้ง 255 = ปิดการกัน
#define SEND_INTERVAL_MS  20     // ส่งทุก 20 ms (50 ครั้ง/วินาที)
#define DEBOUNCE_MS       50
#define DEBUG_SERIAL      true

// ---- ESP-NOW: ไล่หาช่องสัญญาณของรถ ----
#define ESPNOW_CH_MIN          1    // ช่วงช่องที่ไล่หา (ต้องครอบคลุมช่องที่รถเลือกได้)
#define ESPNOW_CH_MAX          13
#define ESPNOW_SEARCH_WAIT_MS  30   // รอ ACK ต่อช่องนานสุดกี่ ms ตอนไล่หา
#define ESPNOW_LOST_FAILS      15   // ส่งไม่สำเร็จติดกันกี่ครั้ง (15 x 20ms = 0.3 วินาที) ถึงถือว่าหลุด
