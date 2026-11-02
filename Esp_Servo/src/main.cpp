#include <Arduino.h>
#include "ServoEasing.hpp"
#include "pinconfig.h"

// ESP32 แขนกล  (ต่อ Pi ผ่าน USB Serial, บรรทัดละคำสั่ง ลงท้าย \n)
//
//  Pi -> ESP
//     1 / 2 / 3   เก็บของแล้ววางช่อง 1/2/3
//     X           ยกเลิก (หยุดแขน แล้วกลับท่าพัก)
//  ESP -> Pi
//     READY            บูตเสร็จ แขนอยู่ท่าพักแล้ว (ครั้งเดียว)
//     ST,IDLE / ST,BUSY  สถานะแขน ส่งทุก 500 ms  (IDLE = ว่าง พร้อมรับคำสั่ง)
//     ACK              รับคำสั่งแล้ว เริ่มทำงาน
//     DONE             ทำครบ และกลับท่าพัก (อยู่ IDLE แล้ว)
//     CANCELLED        ยกเลิก และกลับท่าพัก (อยู่ IDLE แล้ว)
//     BUSY_REJECT      ถูกสั่งขณะยังไม่ว่าง
//     ERR              คำสั่งไม่รู้จัก

ServoEasing Joint1;
ServoEasing Joint2;
ServoEasing gripper;

bool DefaultPosition();
bool Move_Down_Pick();
bool Move_Up_Place(int joint_1, int joint_2);
bool Start_Sorting();

/* ====== Trash ======
'1' = SLOT (1)   '2' = SLOT (2)   '3' = SLOT (3)
*/
char classTrash = 0;

int SubStep = 1; // ลำดับย่อยในแต่ละขั้น

enum RobotState
{
  STATE_DEFAULT_POS, // กลับท่าพัก (Homing)
  STATE_IDLE,        // ว่าง รอคำสั่ง
  STATE_PICK_OBJECT, // ก้มไปหยิบ
  STATE_PLACE_OBJECT // ยกไปวางตามช่อง
};
RobotState CurrentState = STATE_DEFAULT_POS;

bool jobActive = false;       // มีงานที่รับไว้ และยังไม่ได้รายงาน DONE
bool cancelRequested = false; // กำลังกลับท่าพักเพราะถูกยกเลิก
bool bootReported = false;
unsigned long lastBeatMs = 0;
String rx;

bool armMoving()
{
  return Joint1.isMoving() || Joint2.isMoving() || gripper.isMoving();
}

void setup()
{
  Serial.begin(115200);
  Joint1.attach(JOINT1_PIN, 0, 500, 2400);    // มุมเริ่มต้น 0°
  Joint2.attach(JOINT2_PIN, 55, 500, 2400);   // มุมเริ่มต้น 55°
  gripper.attach(GRIPPER_PIN, 55, 500, 2400); // มุมเริ่มต้น 55°
  setSpeedForAllServos(40);                   // 40 องศา/วินาที

  delay(1000);
  rx.reserve(16);
}

void handleLine(const String &s)
{
  if (s == "X")
  {
    if (CurrentState == STATE_PICK_OBJECT || CurrentState == STATE_PLACE_OBJECT)
    {
      Joint1.stop();
      Joint2.stop();
      gripper.stop();
      CurrentState = STATE_DEFAULT_POS; // กลับท่าพัก แล้วค่อยรายงาน CANCELLED
      SubStep = 1;
      cancelRequested = true;
      jobActive = false;
    }
    else if (CurrentState == STATE_IDLE)
    {
      Serial.println("CANCELLED"); // ว่างอยู่แล้ว
    }
    return; // กำลังกลับท่าพักอยู่ -> ไม่ต้องทำอะไร
  }

  if (s.length() == 1 && s[0] >= '1' && s[0] <= '3')
  {
    if (CurrentState == STATE_IDLE)
    {
      classTrash = s[0];
      jobActive = true;
      cancelRequested = false;
      CurrentState = STATE_PICK_OBJECT;
      SubStep = 1;
      Serial.println("ACK");
    }
    else
    {
      Serial.println("BUSY_REJECT");
    }
    return;
  }
  Serial.println("ERR");
}

void loop()
{
  // ---- รับคำสั่งทีละบรรทัด ----
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      rx.trim();
      if (rx.length())
        handleLine(rx);
      rx = "";
    }
    else if (rx.length() < 12)
    {
      rx += c;
    }
  }

  // ---- state machine ----
  switch (CurrentState)
  {
  case STATE_DEFAULT_POS:
    if (DefaultPosition())
    {
      CurrentState = STATE_IDLE;
      SubStep = 1;
      if (cancelRequested)
      {
        Serial.println("CANCELLED");
        cancelRequested = false;
      }
      else if (jobActive)
      {
        Serial.println("DONE");
        jobActive = false;
      }
      else if (!bootReported)
      {
        Serial.println("READY");
        bootReported = true;
      }
    }
    break;

  case STATE_IDLE:
    break;

  case STATE_PICK_OBJECT:
    if (Move_Down_Pick())
    {
      CurrentState = STATE_PLACE_OBJECT;
      SubStep = 1;
    }
    break;

  case STATE_PLACE_OBJECT:
    if (Start_Sorting())
    {
      CurrentState = STATE_DEFAULT_POS; // วางเสร็จ -> กลับท่าพัก -> DONE
      SubStep = 1;
    }
    break;
  }

  // ---- heartbeat ----
  if (millis() - lastBeatMs >= 500)
  {
    lastBeatMs = millis();
    Serial.println(CurrentState == STATE_IDLE ? "ST,IDLE" : "ST,BUSY");
  }
}

// ###############################
//======== /> FUNCTION  </ ========

bool DefaultPosition()
{
  switch (SubStep)
  {
  case 1: // ยก joint 1 ก่อน กัน joint 2 ชนตัวถัง
    if (Joint1.getCurrentAngle() > JOINT1_H)
    {
      // สูงกว่าท่ากึ่งกลางอยู่แล้ว ไม่ต้องยกอีก
      SubStep = 2;
    }
    else
    {
      Joint1.setEaseTo(JOINT1_H);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 2;
    }
    break;
  case 2: // รอยกเสร็จ แล้วค่อยกลับท่าพักทั้งหมด
    if (!armMoving())
    {
      Joint1.setEaseTo(DEFAULT_JOINT1);
      Joint2.setEaseTo(DEFAULT_JOINT2);
      gripper.setEaseTo(GRIPPER_OPEN_ANGLE);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 3;
    }
    break;
  case 3:
    if (!armMoving())
    {
      SubStep = 1;
      return true;
    }
    break;
  }
  return false;
}

bool Move_Down_Pick()
{
  switch (SubStep)
  {
  case 1: // joint1 ลงครึ่งทาง และ joint2 กางออก
    Joint1.setEaseTo(JOINT1_H);
    Joint2.setEaseTo(JOINT2_DOWN);
    synchronizeAllServosAndStartInterrupt();
    SubStep = 2;
    break;
  case 2: // รอ joint2 กางสุดก่อน แล้วค่อยลงต่อ (กันชนตัวถัง)
    if (!armMoving())
    {
      Joint1.setEaseTo(JOINT1_DOWN);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 3;
    }
    break;
  case 3: // ลงถึงแล้ว -> หนีบ
    if (!armMoving())
    {
      gripper.setEaseTo(GRIPPER_CLOSED_ANGLE);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 4;
    }
    break;
  case 4: // หนีบเสร็จ
    if (!armMoving())
    {
      SubStep = 1;
      return true;
    }
    break;
  }
  return false;
}

bool Move_Up_Place(int joint_1, int joint_2)
{
  switch (SubStep)
  {
  case 1:

    Joint1.setEaseTo(JOINT1_H);
    synchronizeAllServosAndStartInterrupt();
    SubStep = 2;
    break;
  case 2:
    if (!armMoving())
    {
      Joint1.setEaseTo(joint_1);
      Joint2.setEaseTo(joint_2);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 3;
    }
    break;
  case 3: // ถึงช่องแล้ว -> ปล่อย
    if (!armMoving())
    {
      gripper.setEaseTo(GRIPPER_OPEN_ANGLE);
      synchronizeAllServosAndStartInterrupt();
      SubStep = 4;
    }
    break;
  case 4:
    if (!armMoving())
    {
      SubStep = 1;
      return true;
    }
    break;
  }
  return false;
}

bool Start_Sorting()
{
  switch (classTrash)
  {
  case '1':
    return Move_Up_Place(SLOT_1_JOINT1, SLOT_1_JOINT2);
  case '2':
    return Move_Up_Place(SLOT_2_JOINT1, SLOT_2_JOINT2);
  case '3':
    return Move_Up_Place(SLOT_3_JOINT1, SLOT_3_JOINT2);
  default:
    break;
  }
  return true; // ไม่ควรเกิด: ข้ามไปกลับท่าพัก
}
// If it's work , Don't tuch it.
// By AE-21 Dev.
