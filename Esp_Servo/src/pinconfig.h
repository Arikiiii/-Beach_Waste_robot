#pragma once

// ---------- Pins ----------
#define JOINT1_PIN  19
#define JOINT2_PIN  21
#define GRIPPER_PIN 22

// หน่วงต่อ 1 องศา (ms) ยิ่งมากยิ่งช้า/นุ่ม
#define STEP_DELAY 30

// ======== Angle ========


// DEFAULT
#define DEFAULT_JOINT1 90
#define DEFAULT_JOINT2 0
#define DEFAULT_GRIPPER 0

//DOWN AND GRAPPING
#define JOINT1_DOWN 0
#define JOINT2_DOWN 55


// HALF POSITION JOIN1
#define JOINT1_H 45

// Gripper: 0 = เปิด, 90 = ปิด
// ถ้าของจริงกลับกัน ให้สลับสองค่านี้ที่เดียวพอ
#define GRIPPER_OPEN_ANGLE   0
#define GRIPPER_CLOSED_ANGLE 55

// ======== SLOT ========
// SLOT ( 1 ) OUT
#define SLOT_1_JOINT1 45
#define SLOT_1_JOINT2 18

// SLOT ( 2 ) HALF
#define SLOT_2_JOINT1 53
#define SLOT_2_JOINT2 0

// SLOT ( 3 ) IN
#define SLOT_3_JOINT1 90
#define SLOT_3_JOINT2 0

