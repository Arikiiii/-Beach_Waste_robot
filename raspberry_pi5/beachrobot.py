#!/usr/bin/env python3
"""
pi_web10.py  -  Raspberry Pi 5 + Camera Module 3

หน้าเว็บบน Pi (อ่านอย่างเดียว ไม่มีปุ่มควบคุม) สำหรับ
  - ดูภาพสดจากกล้องแบบเรียลไทม์ + สถานะรถ/แขน/ผลตรวจจับ
  - รัน AI ตรวจจับแล้วส่งคำสั่งไป servo เมื่อผู้ใช้กดปุ่มบนจอยสติ๊กรีโมท

การควบคุมรถ:  รีโมท (ESP32 + จอยสติ๊ก) --ESP-NOW--> ESP ขับเคลื่อน  (Pi ไม่เกี่ยวกับการขับ)
เชื่อมต่อด้วย USB Serial:  Pi <-> ESP ขับเคลื่อน,  Pi <-> ESP servo

ESP ขับเคลื่อนส่งมาที่ Pi
  S,<mode>,<link>,<ch>,<auto>  ทุก 0.5 วินาที  mode=0 สแตนบาย / 1 ทำงาน  (เปลี่ยนด้วยปุ่มบนรถเท่านั้น)
                         link=1 รีโมทเชื่อมต่อ / 0 ไม่มีสัญญาณ,  ch=ช่อง ESP-NOW,  auto=1 เลือกโหมดออโต้
  P                      ผู้ใช้กดปุ่มเก็บของบนรีโมท (ESP ส่งเฉพาะตอนทำงาน) -> Pi เริ่มตรวจจับ + สั่งแขน
Pi ส่งไป ESP ขับเคลื่อน
  B,1 (ซ้ำทุก 1 วินาทีระหว่างงาน) / B,0   -> รถขยับไม่ได้ตั้งแต่สั่งเก็บ จนกว่าแขนกลับท่าพักและเป็น IDLE
ESP servo ส่ง  ST,IDLE / ST,BUSY  ทุก 0.5 วินาที และ DONE / CANCELLED / ERR หลังจบงาน

ติดตั้ง:  sudo apt install python3-picamera2 python3-opencv python3-serial python3-flask
รัน:       python3 pi_web10.py    แล้วเปิด  http://<IP ของ Pi>:5000
"""
import queue
import socket
import threading
import time
from collections import Counter

import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, request
from werkzeug.serving import WSGIRequestHandler
from picamera2 import Picamera2
from libcamera import Transform

# ================== ตั้งค่าโมเดล ==================
model_path = "best.onnx"
input_size = 320
conf_threshold = 0.45
nms_threshold = 0.45
class_names = ['bottle', 'bottle cap', 'box', 'can']
CLASS_TH = {0: "ขวด", 1: "ฝาขวด", 2: "กล่อง", 3: "กระป๋อง"}   # ชื่อไทยที่แสดงบนเว็บ
target_class_id = None

# ================== Serial ==================
DRIVE_PORT = "/dev/esp_drive"   # ESP ขับเคลื่อน (udev symlink)
SERVO_PORT = "/dev/esp_servo"   # ESP servo (udev symlink)
BAUD = 115200
ESP_TIMEOUT_S = 3.0           # ถ้า ESP ขับเคลื่อนเงียบเกินเวลานี้ ถือว่าขาดการเชื่อมต่อ

# ================== servo ==================
# คลาส -> ช่องของแขน   (0=bottle, 1=bottle cap, 2=box, 3=can)
#   ช่อง 1 (OUT)  : can
#   ช่อง 2 (HALF) : bottle, bottle cap
#   ช่อง 3 (IN)   : box
CLASS_TO_CMD = {0: 2, 1: 2, 2: 3, 3: 1}


SEND_WHEN_NOTHING = False
NOTHING_CMD = 0
SERVO_TIMEOUT_S = 60          # รอแขนทำงานเสร็จ (DONE) นานสุดกี่วินาที
SERVO_CANCEL_WAIT_S = 10      # หลังส่ง X รอ ESP servo ตอบกลับนานสุดกี่วินาที
SERVO_FRESH_S = 3.0           # ST,IDLE ต้องใหม่กว่านี้ ถึงนับว่าแขนว่าง
SERVO_IDLE_WAIT_S = 20        # จบงานแล้วรอแขนเป็น IDLE นานสุดกี่วินาที ก่อนปล่อยให้รถขยับ
SERVO_CMD_RETRIES = 2         # ถ้าแขนตอบ ERR (ไม่รู้จักคำสั่ง) ให้ส่งซ้ำอัตโนมัติอีกกี่ครั้ง
SERVO_FLUSH_S = 0.15          # หน่วงหลังส่งบรรทัดว่างเพื่อเคลียร์ขยะในบัฟเฟอร์ ESP ก่อนส่งคำสั่งจริง

# ================== ถ่ายภาพ ==================
N_FRAMES = 5
SETTLE_S = 0.5
CAM_SIZE = (640, 480)         # ขนาดที่ AI ใช้ตรวจจับ (ไม่กระทบสตรีม)

# ================== สตรีมภาพบนเว็บ (ลดความคมชัดเพื่อลดภาระ/หน่วง) ==================
STREAM_FPS = 15               # จำกัดเฟรมเรตของสตรีม
# โปรไฟล์ภาพ: (ขนาด, คุณภาพ JPEG)   sd = ใช้ปกติ (เบา ลื่น)   hd = ใช้ตอนกดเต็มจอ (คมขึ้น)
STREAM_PROFILES = {"sd": ((320, 240), 35),
                   "hd": ((640, 480), 60)}

# ================== กล้องกลับหัว ==================
ROTATE_180 = True             # กล้องติดตั้งกลับหัว -> หมุนภาพ 180 องศา
ROTATE_APPLIES_TO_AI = True   # True  = หมุนที่ตัวกล้อง ทั้งเว็บและ AI เห็นภาพหัวขึ้น (ตั้งค่าอยู่)
                              # False = หมุนเฉพาะภาพบนเว็บ AI เห็นภาพกลับหัวเหมือนเดิม
                              #         (ใช้ถ้าโมเดลเทรนด้วยภาพกลับหัว)

# ================== เว็บ ==================
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

# ================== โหมดออโต้ (ESP วิ่งสุ่ม + หลบสิ่งกีดขวางเอง, Pi รัน AI ตลอด) ==================
AUTO_DETECT_ENABLED = True    # False = ไม่รัน AI อัตโนมัติ (ยังสั่งเก็บด้วยจอยในโหมดรีโมทได้)
AUTO_CONFIRM_FRAMES = 2       # ต้องเจอวัตถุติดกันกี่เฟรมถึงสั่งหยุดรถและเก็บ (กันภาพหลอกเฟรมเดียว)
AUTO_COOLDOWN_S = 3.0         # หลังเก็บเสร็จ พักตรวจจับกี่วินาที (กันเจอวัตถุเดิมซ้ำทันที)
AUTO_DETECT_INTERVAL_S = 0.05 # พักระหว่างตรวจแต่ละเฟรม ลดภาระ CPU (0 = ตรวจเต็มสปีด)

# ---------------------------------------------------------------
net = cv2.dnn.readNetFromONNX(model_path)

drive = None
servo = None
drive_lock = threading.Lock()
servo_lock = threading.Lock()
state_lock = threading.Lock()
state = {"busy": False, "last": "ยังไม่เคยตรวจ", "t": 0, "phase": "", "detected": "", "job": 0,
         "mode": "standby", "auto": False, "auto_cooldown": 0.0, "link": False, "channel": 0, "esp_ts": 0.0,   # mode: locked/standby/active จาก S,<mode>,<link>,<ch>
         "servo_idle": False, "servo_ts": 0.0}         # จาก ST,IDLE / ST,BUSY ของ ESP servo
servo_events = queue.Queue()                          # DONE / CANCELLED / ERR / BUSY_REJECT
SERVO_RESULT = {"DONE": "แขนทำงานเสร็จ", "CANCELLED": "ยกเลิกกลางคัน",
                "ERR": "แขนไม่รู้จักคำสั่ง", "BUSY_REJECT": "แขนยังไม่ว่าง",
                "TIMEOUT": "แขนไม่ตอบ (timeout)"}


# ======================= โมเดล =======================
def letterbox(img, size):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh))
    top, left = (size - nh) // 2, (size - nw) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top:top + nh, left:left + nw] = resized
    return canvas, r, left, top


def detect(frame_bgr):
    img, r, padx, pady = letterbox(frame_bgr, input_size)
    blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (input_size, input_size),
                                 swapRB=True, crop=False)
    net.setInput(blob)
    out = net.forward()[0]
    if out.shape[0] < out.shape[1]:
        out = out.T

    nc = len(class_names)
    if out.shape[1] == 5 + nc:
        scores = out[:, 5:] * out[:, 4:5]
    else:
        scores = out[:, 4:4 + nc]

    cls_ids = scores.argmax(axis=1)
    confs = scores.max(axis=1)
    keep = confs >= conf_threshold
    if target_class_id is not None:
        keep &= (cls_ids == target_class_id)
    if not keep.any():
        return []

    boxes_raw, confs, cls_ids = out[keep, :4], confs[keep], cls_ids[keep]
    boxes = []
    for cx, cy, w, h in boxes_raw:
        x = (cx - w / 2 - padx) / r
        y = (cy - h / 2 - pady) / r
        boxes.append([int(x), int(y), int(w / r), int(h / r)])

    idxs = cv2.dnn.NMSBoxes(boxes, confs.tolist(), conf_threshold, nms_threshold)
    return [(int(cls_ids[i]), float(confs[i]), tuple(boxes[i]))
            for i in np.array(idxs).flatten()]


# ======================= กล้อง =======================
class Grabber(threading.Thread):
    """อ่านเฟรมจากกล้องตลอดเวลา เก็บเฟรมล่าสุดไว้ให้ทั้ง stream และ AI ใช้ร่วมกัน"""

    def __init__(self, cam):
        super().__init__(daemon=True)
        self.cam = cam
        self.frame = None
        self.seq = 0
        self.cond = threading.Condition()

    def run(self):
        while True:
            try:
                f = self.cam.capture_array()
            except Exception as e:             # เดิมถ้าพลาดครั้งเดียว เธรดตาย -> ภาพค้างถาวร
                print("camera error:", e)
                time.sleep(0.5)
                continue
            with self.cond:
                self.frame = f
                self.seq += 1
                self.cond.notify_all()

    def wait_new(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.frame, self.seq


grabber = None


def decide():
    time.sleep(SETTLE_S)
    votes = Counter()
    confs = {}
    seq = -1
    for _ in range(N_FRAMES):
        frame, seq = grabber.wait_new(seq)
        if frame is None:
            continue
        dets = detect(frame)
        if dets:
            cid, conf, _ = max(dets, key=lambda d: d[1])
            votes[cid] += 1
            confs.setdefault(cid, []).append(conf)
    if not votes:
        return None
    cid = votes.most_common(1)[0][0]
    return cid, sum(confs[cid]) / len(confs[cid])      # (คลาส, ความมั่นใจเฉลี่ย)


# ======================= Serial helper =======================
def open_serial(port, baud, timeout=0.1):
    """เปิด-ปิด-เปิดใหม่ แก้บั๊ก CP2102 ตั้ง baud ไม่ติดตอนเปิดพอร์ตครั้งแรกหลังบูต
    (รอบแรกได้ข้อมูลเสีย รอบสองได้ข้อมูลสะอาด)"""
    s = serial.Serial(port, baud, timeout=timeout)
    s.close()
    time.sleep(0.3)
    s = serial.Serial(port, baud, timeout=timeout)
    time.sleep(0.5)               # เผื่อ ESP รีเซ็ตจาก DTR/RTS ตอนเปิดพอร์ต
    s.reset_input_buffer()
    return s


def send_drive_line(text):
    try:
        with drive_lock:
            drive.write((text + "\n").encode())
    except Exception as e:
        print("drive write error:", e)


def servo_write(text):
    try:
        with servo_lock:
            servo.write((text + "\n").encode())
    except Exception as e:
        print("servo write error:", e)


# ======================= งาน AI =======================
def _active_locked():
    return state["mode"] == "active" and (time.time() - state["esp_ts"]) < ESP_TIMEOUT_S


def _servo_ready_locked():
    return state["servo_idle"] and (time.time() - state["servo_ts"]) < SERVO_FRESH_S


def set_phase(text):
    with state_lock:
        state["phase"] = text


def set_detected(text):
    with state_lock:
        state["detected"] = text


def run_detect(source):
    with state_lock:
        if state["busy"] or not _active_locked():
            return False
        if servo is not None and not _servo_ready_locked():
            state["last"] = "แขนยังไม่พร้อม (ไม่อยู่ในโหมด idle)"
            state["t"] = time.time()
            return False
        state["busy"] = True
        state["job"] += 1
        jid = state["job"]
        state["phase"] = "กำลังตรวจจับ..."
        state["detected"] = ""
    threading.Thread(target=busy_keepalive, args=(jid,), daemon=True).start()
    threading.Thread(target=_detect_job, args=(source,), daemon=True).start()
    return True


def busy_keepalive(jid):
    """ส่ง B,1 ซ้ำทุก 1 วินาทีตลอดงาน (ESP จะปลด busy เองถ้า Pi หายไป)"""
    while True:
        with state_lock:
            if not state["busy"] or state["job"] != jid:
                return
            send_drive_line("B,1")           # ไฟเหลืองกะพริบ ไฟเขียวดับ รถขยับไม่ได้
        time.sleep(1.0)


def wait_servo_done():
    """รอ DONE / CANCELLED / ERR จาก ESP servo (หมดเวลาจะส่ง X ให้แขนหยุด)"""
    deadline = time.time() + SERVO_TIMEOUT_S
    while time.time() < deadline:
        try:
            return servo_events.get(timeout=0.2)
        except queue.Empty:
            continue
    servo_write("X")                         # หมดเวลา -> สั่งหยุดเพื่อความปลอดภัย
    return "TIMEOUT"


def wait_servo_idle(timeout):
    """รอจนแขนรายงาน ST,IDLE สด ๆ (แขนกลับท่าพักแล้ว) คืน True ถ้าสำเร็จ"""
    end = time.time() + timeout
    while time.time() < end:
        with state_lock:
            if _servo_ready_locked():
                return True
        time.sleep(0.1)
    return False


def _drain_servo_events():
    while True:
        try:
            servo_events.get_nowait()
        except queue.Empty:
            break


def servo_prepare():
    """ส่งบรรทัดว่างไปล้างขยะค้างในบัฟเฟอร์รับของ ESP servo (สาเหตุที่คำสั่งแรกกลายเป็น ERR)
    แล้วทิ้งเหตุการณ์ ERR ที่อาจเกิดจากขยะนั้น ก่อนส่งคำสั่งจริง"""
    servo_write("")                          # = "\n" เปล่า ๆ
    time.sleep(SERVO_FLUSH_S)
    _drain_servo_events()


def send_to_servo(cmd):
    if servo is None:
        return "ยังไม่ได้ต่อ servo (ข้ามขั้นแขน)"
    result = "?"
    for attempt in range(SERVO_CMD_RETRIES + 1):
        servo_prepare()
        with state_lock:
            state["servo_idle"] = False      # ถือว่าไม่ว่างจนกว่าแขนจะบอก IDLE
        servo_write(str(cmd))
        result = wait_servo_done()
        if result != "ERR":
            break
        print(f"servo ตอบ ERR (ครั้งที่ {attempt + 1}) -> ส่งคำสั่งใหม่")
    return SERVO_RESULT.get(result, "?")


def _detect_job(source):
    msg = "-"
    try:
        print(f"[{source}] เริ่มตรวจจับ")
        res = decide()                        # รถหยุดอยู่แล้ว: busy_keepalive ส่ง B,1 ทันที (ESP หยุดมอเตอร์)
        if res is None:
            msg = "ไม่พบวัตถุ"
            set_detected("ไม่พบวัตถุ")
            if SEND_WHEN_NOTHING:
                set_phase("แขนกำลังทำงาน...")
                msg += f" | {send_to_servo(NOTHING_CMD)}"
        else:
            cid, conf = res
            name = f"{CLASS_TH.get(cid, class_names[cid])} ({class_names[cid]})"
            cmd = CLASS_TO_CMD.get(cid)
            if cmd is None:
                set_detected(f"{name} {conf*100:.0f}% - ไม่มีช่องรองรับ")
                msg = f"พบ {name} (ไม่ส่งคำสั่ง)"
            else:
                set_detected(f"{name} {conf*100:.0f}% -> ช่อง {cmd}")   # แจ้งทันทีที่ตรวจพบ
                set_phase(f"พบ {name} แขนกำลังเก็บ...")
                msg = f"พบ {name} -> ช่อง {cmd} | {send_to_servo(cmd)}"
        print(msg)
    except Exception as e:
        msg = f"ผิดพลาด: {e}"
        print(msg)
    finally:
        # รถยังขยับไม่ได้ จนกว่าแขนกลับท่าพักและเป็น IDLE
        if servo is not None:
            set_phase("รอแขนกลับท่าพัก...")
            if not wait_servo_idle(SERVO_IDLE_WAIT_S):
                msg += " | ⚠ แขนไม่รายงาน idle"
        with state_lock:
            state["busy"] = False
            state["phase"] = ""
            state["last"] = msg
            state["t"] = time.time()
            state["auto_cooldown"] = time.time() + AUTO_COOLDOWN_S
            send_drive_line("B,0")            # ปลดล็อกรถ (ไฟเหลืองดับ เขียวติดค้าง)


def auto_loop():
    """โหมดออโต้: รัน AI ตลอดเวลาที่รถอยู่โหมดออโต้+ทำงาน (ไม่รันตอนสแตนบาย/โหมดรีโมท/กำลังเก็บ)
    เจอวัตถุ -> run_detect() -> ส่ง B,1 -> ESP ล็อกล้อ -> แขนเก็บ -> B,0 -> รถวิ่งต่อ"""
    seq = -1
    hits = 0
    while True:
        with state_lock:
            on = (state["auto"] and _active_locked() and not state["busy"]
                  and time.time() >= state["auto_cooldown"]
                  and (servo is None or _servo_ready_locked()))
        if not on:
            hits = 0
            time.sleep(0.1)
            continue
        frame, seq = grabber.wait_new(seq)
        if frame is None:
            continue
        try:
            dets = detect(frame)
        except Exception as e:
            print("auto detect error:", e)
            time.sleep(0.5)
            continue
        hits = hits + 1 if dets else 0
        if hits >= AUTO_CONFIRM_FRAMES:
            hits = 0
            print("[AUTO] พบวัตถุ -> ล็อกล้อและเก็บ")
            run_detect("AUTO")
        elif AUTO_DETECT_INTERVAL_S > 0:
            time.sleep(AUTO_DETECT_INTERVAL_S)


def serial_listener():
    """อ่าน S,<mode>,<link>,<ch> และ P (กดปุ่มเก็บของบนรีโมท) จาก ESP ขับเคลื่อน"""
    modes = {"0": "standby", "1": "active"}
    while True:
        try:
            line = drive.readline().decode(errors="ignore").strip()
        except Exception:
            time.sleep(0.2)
            continue
        if line.startswith("S,"):
            parts = line.split(",")
            try:
                mode = modes[parts[1]]
                link = parts[2] == "1"
                ch = int(parts[3]) if len(parts) > 3 else 0
                auto = len(parts) > 4 and parts[4] == "1"
            except (IndexError, ValueError, KeyError):
                continue
            with state_lock:
                if mode != state["mode"]:
                    print("สถานะ ->", mode)
                if link != state["link"]:
                    print("รีโมท ->", "เชื่อมต่อ" if link else "ไม่มีสัญญาณ")
                state["mode"] = mode
                state["link"] = link
                state["channel"] = ch
                state["auto"] = auto
                state["esp_ts"] = time.time()
        elif line == "P":
            print("[จอย] กดปุ่มเก็บของ")
            if not run_detect("จอย"):
                print("  (ไม่เริ่ม: ไม่ได้อยู่ในโหมดทำงาน / กำลังทำงานอยู่ / แขนยังไม่พร้อม)")
        elif line:
            print("ESP>", line)               # ข้อความอื่นจาก ESP (ไว้ debug)


def servo_listener():
    """อ่าน ST,IDLE / ST,BUSY / READY / DONE / CANCELLED / ERR จาก ESP servo"""
    while True:
        try:
            line = servo.readline().decode(errors="ignore").strip()
        except Exception:
            time.sleep(0.2)
            continue
        if not line:
            continue
        if line in ("ST,IDLE", "ST,BUSY", "READY", "DONE", "CANCELLED"):
            with state_lock:
                state["servo_idle"] = line not in ("ST,BUSY",)
                state["servo_ts"] = time.time()
            if line in ("DONE", "CANCELLED"):
                servo_events.put(line)
        elif line in ("ERR", "BUSY_REJECT"):
            servo_events.put(line)
        else:
            print("SERVO>", line)


# ======================= เว็บ =======================
app = Flask(__name__)

PAGE = r"""<!doctype html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Robot Live</title>
<style>
  :root{--bg:#111418;--card:#1b2027;--fg:#e8eaed;--acc:#3ddc97;--warn:#ffb020;--red:#ff5c5c}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,sans-serif;
       display:flex;flex-direction:column;align-items:center;gap:12px;padding:12px}
  .card{background:var(--card);border-radius:14px;padding:12px;width:100%;max-width:640px}
  .vid{position:relative}
  #cam{width:100%;border-radius:10px;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
  #live{position:absolute;top:22px;left:22px;background:rgba(0,0,0,.6);border-radius:999px;
        padding:3px 10px;font-size:13px;font-weight:700;color:var(--red);z-index:2}
  #live.on{color:var(--acc)}
  #fs{position:absolute;top:18px;right:18px;width:44px;height:44px;border:0;border-radius:999px;
      background:rgba(0,0,0,.6);color:#fff;font-size:22px;line-height:1;cursor:pointer;z-index:2}
  #ov{display:none;position:absolute;left:0;right:0;bottom:0;z-index:2;pointer-events:none;
      padding:28px 14px calc(10px + env(safe-area-inset-bottom));text-align:center;
      background:linear-gradient(transparent,rgba(0,0,0,.8))}
  #ovm{font-size:15px;font-weight:700}
  #ovd{font-size:22px;font-weight:700;color:var(--acc);min-height:1.2em}
  /* โหมดเต็มจอ (ใช้ได้ทั้ง Fullscreen API และแบบ CSS สำหรับ iPhone) */
  .vid.full{position:fixed;inset:0;z-index:100;max-width:none;width:100%;height:100%;
            padding:0;border-radius:0;background:#000}
  .vid.full #cam{width:100%;height:100%;aspect-ratio:auto;border-radius:0}
  .vid.full #live{top:calc(12px + env(safe-area-inset-top))}
  .vid.full #fs{top:calc(8px + env(safe-area-inset-top));right:calc(8px + env(safe-area-inset-right))}
  .vid.full #ov{display:block}
  #mode{text-align:center;font-weight:700;font-size:17px}
  #mode.on{color:var(--acc)} #mode.off{color:var(--warn)} #mode.bad{color:var(--red)}
  #status{font-size:15px;opacity:.9;text-align:center;margin:8px 0 0}
  #detected{font-size:20px;font-weight:700;text-align:center;color:var(--acc);margin:0;min-height:1.2em}
</style></head><body>
<div class="card"><div id="mode">-</div></div>
<div class="card vid" id="vid">
  <img id="cam" alt="">
  <span id="live">● กำลังเชื่อมต่อ...</span>
  <button id="fs" type="button" title="เต็มจอ (ดับเบิลคลิกที่ภาพก็ได้)">⛶</button>
  <div id="ov"><div id="ovm"></div><div id="ovd"></div></div>
</div>
<div class="card">
  <p id="detected"></p>
  <p id="status">-</p>
</div>
<script>
const cam=document.getElementById('cam'), live=document.getElementById('live');
const vid=document.getElementById('vid'), fsBtn=document.getElementById('fs');
const modeEl=document.getElementById('mode'), det=document.getElementById('detected'), st=document.getElementById('status');
const ovm=document.getElementById('ovm'), ovd=document.getElementById('ovd');
const sleep=ms=>new Promise(r=>setTimeout(r,ms));

// ---------------- เต็มจอ ----------------
let full=false;
function applyFull(on){
  full=on; vid.classList.toggle('full',on);
  document.body.style.overflow=on?'hidden':'';
  fsBtn.textContent=on?'✕':'⛶';
}
async function enterFull(){
  applyFull(true);                                   // โหมด CSS ใช้ได้ทุกเครื่อง (รวม iPhone)
  try{
    if(vid.requestFullscreen) await vid.requestFullscreen();
    else if(vid.webkitRequestFullscreen) await vid.webkitRequestFullscreen();
  }catch(e){}
  try{ await screen.orientation.lock('landscape'); }catch(e){}   // มือถือ: หมุนแนวนอนให้ (ถ้าเครื่องรองรับ)
}
function exitFull(){
  applyFull(false);
  try{ screen.orientation.unlock(); }catch(e){}
  if(document.fullscreenElement) document.exitFullscreen();
  else if(document.webkitFullscreenElement) document.webkitExitFullscreen();
}
function toggleFull(){ full?exitFull():enterFull(); }
fsBtn.onclick=toggleFull;
cam.ondblclick=toggleFull;
function onFsChange(){                               // ผู้ใช้กด Esc / ปัดออกเอง
  if(full && !document.fullscreenElement && !document.webkitFullscreenElement){
    applyFull(false); try{ screen.orientation.unlock(); }catch(e){}
  }
}
document.addEventListener('fullscreenchange',onFsChange);
document.addEventListener('webkitfullscreenchange',onFsChange);
document.addEventListener('keydown',e=>{ if(e.key==='Escape'&&full) exitFull(); });

// ---------------- ภาพสด ----------------
// ดึงภาพทีละเฟรม (รอเฟรมใหม่จากเซิร์ฟเวอร์) แทน MJPEG ที่ค้างเงียบ ๆ ได้
// ถ้าเฟรมไหนช้า/หลุด จะหมดเวลาแล้วลองใหม่เองอัตโนมัติ
const MIN_DT=1000/15;
function setLive(ok,txt){ live.className=ok?'on':''; live.textContent=txt; }
async function frameLoop(){
  let lastSeq=-1, lastChange=Date.now(), prevUrl=null;
  while(true){
    const t0=Date.now();
    const ctrl=new AbortController();
    const to=setTimeout(()=>ctrl.abort(),4000);
    try{
      const r=await fetch('/snapshot?s='+(full?'hd':'sd')+'&after='+lastSeq+'&t='+t0,
                          {cache:'no-store',signal:ctrl.signal});
      if(!r.ok) throw new Error('HTTP '+r.status);
      const seq=parseInt(r.headers.get('X-Seq')||'-1',10);
      const blob=await r.blob();
      clearTimeout(to);
      if(seq!==lastSeq){
        lastSeq=seq; lastChange=Date.now();
        const url=URL.createObjectURL(blob);
        await new Promise(res=>{cam.onload=res; cam.onerror=res; cam.src=url;});
        if(prevUrl) URL.revokeObjectURL(prevUrl);
        prevUrl=url;
      }
      if(Date.now()-lastChange>3000) setLive(false,'● กล้องไม่ส่งภาพ');
      else setLive(true,'● LIVE');
    }catch(e){
      clearTimeout(to);
      setLive(false,'● OFFLINE');
      await sleep(1000);
    }
    const dt=Date.now()-t0;
    if(dt<MIN_DT) await sleep(MIN_DT-dt);
  }
}
frameLoop();

// ---------------- สถานะ ----------------
function poll(){
  fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{
    let txt,cls='off';
    if(!s.esp_ok){txt='⚠ ไม่ได้รับสถานะจาก ESP ขับเคลื่อน';cls='bad'}
    else if(s.busy) txt='● กำลังเก็บของ (รถหยุดนิ่ง)';
    else if(!s.active) txt='● สแตนบาย ('+(s.auto?'ออโต้':'รีโมท')+') — กดปุ่มที่ตัวรถเพื่อเริ่มทำงาน';
    else if(s.auto){txt='🤖 โหมดอัตโนมัติ — วิ่งสุ่ม/หลบสิ่งกีดขวาง และตรวจจับตลอด';cls='on'}
    else if(!s.link) txt='📡 ทำงาน แต่รีโมทไม่เชื่อมต่อ (ESP-NOW ช่อง '+s.channel+')';
    else{txt='● ทำงาน — กดปุ่มบนจอยเพื่อสั่งเก็บของ (ESP-NOW ช่อง '+s.channel+')';cls='on'}
    const dtxt=s.detected?('ตรวจพบ: '+s.detected):'';
    const stxt=s.busy?(s.phase||'กำลังทำงาน...'):'ผลล่าสุด: '+s.last;
    modeEl.textContent=txt; modeEl.className=cls;
    det.textContent=dtxt; st.textContent=stxt;
    ovm.textContent=txt+(stxt?' | '+stxt:''); ovd.textContent=dtxt;     // แสดงทับภาพตอนเต็มจอ
  }).catch(()=>{
    modeEl.textContent='⚠ เชื่อมต่อ Pi ไม่ได้'; modeEl.className='bad';
    det.textContent=''; st.textContent='เชื่อมต่อ Pi ไม่ได้';
    ovm.textContent='⚠ เชื่อมต่อ Pi ไม่ได้'; ovd.textContent='';
  });
}
setInterval(poll,500);poll();
</script></body></html>"""


@app.route("/")
def index():
    return Response(PAGE, mimetype="text/html")


_jpg_cache = {}                 # โปรไฟล์ -> (seq, bytes)  เข้ารหัส JPEG ครั้งเดียวต่อเฟรม ทุกผู้ชมใช้ร่วมกัน
_jpg_lock = threading.Lock()


def encode_frame(frame, seq, prof):
    with _jpg_lock:
        c = _jpg_cache.get(prof)
        if c and c[0] == seq:
            return c[1]
    size, quality = STREAM_PROFILES[prof]
    img = cv2.resize(frame, size, interpolation=cv2.INTER_AREA) \
        if (frame.shape[1], frame.shape[0]) != size else frame
    if ROTATE_180 and not ROTATE_APPLIES_TO_AI:
        img = cv2.rotate(img, cv2.ROTATE_180)             # หมุนเฉพาะภาพที่โชว์บนเว็บ
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return None
    data = jpg.tobytes()
    with _jpg_lock:
        _jpg_cache[prof] = (seq, data)
    return data


@app.route("/snapshot")
def snapshot():
    """ภาพ 1 เฟรม  ?s=sd|hd  ?after=<seq ล่าสุดที่หน้าเว็บมี> (รอเฟรมใหม่ได้นานสุด 1 วินาที)"""
    prof = request.args.get("s", "sd")
    if prof not in STREAM_PROFILES:
        prof = "sd"
    try:
        after = int(request.args.get("after", "-1"))
    except ValueError:
        after = -1
    frame, seq = grabber.wait_new(after, timeout=1.0)
    if frame is None:
        return Response("no frame", status=503, headers={"Cache-Control": "no-store"})
    data = encode_frame(frame, seq, prof)
    if data is None:
        return Response("encode error", status=500, headers={"Cache-Control": "no-store"})
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store", "X-Seq": str(seq)})


@app.route("/video")
def video():
    """MJPEG แบบเดิม (เก็บไว้ให้เปิดด้วยโปรแกรมอื่นเช่น VLC) หน้าเว็บหลักไม่ใช้แล้ว"""
    def gen():
        seq = -1
        last = 0.0
        min_dt = 1.0 / STREAM_FPS
        while True:
            frame, seq = grabber.wait_new(seq)
            if frame is None:
                continue
            now = time.time()
            if now - last < min_dt:            # จำกัด fps ของสตรีม
                continue
            last = now
            data = encode_frame(frame, seq, "sd")
            if data:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + data + b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store"})


@app.route("/status")
def status_route():
    with state_lock:
        return jsonify(busy=state["busy"], phase=state["phase"],
                       last=state["last"], t=state["t"],
                       detected=state["detected"], mode=state["mode"], active=_active_locked(), auto=state["auto"],
                       link=state["link"],
                       channel=state["channel"],
                       esp_ok=(time.time() - state["esp_ts"]) < ESP_TIMEOUT_S)


# ======================= เซิร์ฟเวอร์ (ลดหน่วง) =======================
class FastHandler(WSGIRequestHandler):
    """HTTP/1.1 keep-alive (ไม่ต้องจับมือ TCP ใหม่ทุกคำสั่งจอย) + ปิด Nagle (TCP_NODELAY)"""
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass


# ======================= main =======================
def main():
    global drive, servo, grabber
    drive = open_serial(DRIVE_PORT, BAUD)
    send_drive_line("B,0")                    # เคลียร์ busy ค้าง (ถ้ามี)
    try:
        servo = open_serial(SERVO_PORT, BAUD)
    except serial.SerialException as e:
        servo = None                          # ยังไม่ต่อ servo ก็ทดสอบส่วนอื่นได้
        print(f"คำเตือน: เปิด {SERVO_PORT} ไม่ได้ ({e}) -> ข้ามขั้นสั่งแขน")

    cam = Picamera2()
    flip = Transform(hflip=1, vflip=1) if (ROTATE_180 and ROTATE_APPLIES_TO_AI) else Transform()
    cam.configure(cam.create_preview_configuration(
        main={"size": CAM_SIZE, "format": "RGB888"},    # array เป็นลำดับ BGR ตรงกับ OpenCV
        transform=flip))
    cam.start()
    try:
        cam.set_controls({"AfMode": 2})
    except Exception:
        pass

    grabber = Grabber(cam)
    grabber.start()

    detect(np.zeros((CAM_SIZE[1], CAM_SIZE[0], 3), np.uint8))   # warm-up
    threading.Thread(target=serial_listener, daemon=True).start()
    if AUTO_DETECT_ENABLED:
        threading.Thread(target=auto_loop, daemon=True).start()
    if servo is not None:
        threading.Thread(target=servo_listener, daemon=True).start()

    print(f"พร้อม  เปิดเว็บที่ http://<IP ของ Pi>:{WEB_PORT}")
    try:
        app.run(host=WEB_HOST, port=WEB_PORT, threaded=True, request_handler=FastHandler)
    finally:
        send_drive_line("B,0")
        cam.stop()
        drive.close()
        if servo is not None:
            servo.close()


if __name__ == "__main__":
    main()
