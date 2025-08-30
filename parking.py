#   check_zigbee_state-> STATE_IDLE (부저 on)                     --ㅣ ---> prev_state
        #   (현재 상태)         -> STATE_ACTIVE (10초 전진 후 흰선 검출)          ㅣ     
        #                     -> STATE_DISCONNECT (15초 후진 후 STATE_IDLE)—ㅣ
        #
import time
import serial
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2

#상태 정의
STATE_IDLE = 0         # 신호 없음
STATE_ACTIVE = 1       # 신호 있음
STATE_DISCONNECT = 2   # 신호 끊김

# 설정 
BUZZER_PIN = 18
ZIGBEE_PORT = "/dev/ttyAMA0"
ZIGBEE_BAUDRATE = 9600
MOTION_THRESHOLD = 10000
MOTION_FRAME_LIMIT = 3

# Motor
IN1 = 5
ENA1 = 12
IN2 = 6
ENA2 = 13

# GPIO 핀 상태 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(ENA1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA2, GPIO.OUT)
GPIO.output(ENA1, GPIO.LOW)
GPIO.output(ENA2, GPIO.LOW)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.output(BUZZER_PIN, GPIO.LOW)

# 카메라 설정
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
mark_y = 120
markmin_x = 0
markmax_x = 320

# 카메라 인식 범위 설정
Broi_y_start = int(FRAME_HEIGHT * 2 / 3)
Broi_height = FRAME_HEIGHT - Broi_y_start
Croi_y_start = int(FRAME_HEIGHT * 1 / 5)
Croi_height = FRAME_HEIGHT - Croi_y_start

# 카메라 초기화
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
    )
)
picam2.start()

# 지그비 시리얼 초기화
try:
    zigbee_serial = serial.Serial(ZIGBEE_PORT, ZIGBEE_BAUDRATE, timeout=1.2)
except:
    # Zigbee 포트 열기 실패
    zigbee_serial = None

# 지그비 현재/이전 상태 판단
def check_zigbee_state(prev_state):
    current_state = STATE_IDLE
    if zigbee_serial and zigbee_serial.in_waiting > 0:
        data = zigbee_serial.readline().decode(errors="ignore").strip()
        if "OK" in data:
            current_state = STATE_ACTIVE

    if prev_state == STATE_ACTIVE and current_state == STATE_IDLE:
        return STATE_DISCONNECT

    return current_state

# 모터 전진 함수
def motor_forward(duration):
    GPIO.output(ENA1, GPIO.HIGH)
    GPIO.output(ENA2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(ENA1, GPIO.LOW)
    GPIO.output(ENA2, GPIO.LOW)

# 모터 후진 함수
def motor_backward(duration):
    GPIO.output(ENA1, GPIO.HIGH)
    GPIO.output(ENA2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(ENA1, GPIO.LOW)
    GPIO.output(ENA2, GPIO.LOW)

# 부저 on->off 함수
def buzzer_beep(duration=1):
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

# 부저  on->off 조건 함수
def handle_idle():
    global frame1, frame2, motion_count

    gray1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)

    roi1 = gray1[Broi_y_start:Broi_y_start + Broi_height, :]
    roi2 = gray2[Broi_y_start:Broi_y_start + Broi_height, :]

    diff = cv2.absdiff(roi1, roi2)
    _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
    motion_level = np.count_nonzero(thresh)

    if motion_level > MOTION_THRESHOLD:
        motion_count += 1
    else:
        motion_count = 0

    if motion_count >= MOTION_FRAME_LIMIT:
        buzzer_beep()
        motion_count = 0

    frame1 = frame2
    frame2 = picam2.capture_array()

# 10초 전진 후 흰선 검출
def handle_active():
    # 상태 1: 전진 및 흰선 감지
    motor_forward(10)  # 10초 전진

    # 흰선 탐색 루프
    while True:
        frame = picam2.capture_array()
        roi = frame[Croi_y_start:Croi_y_start + Croi_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((20, 20), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        min_area = 2000
        clean_mask = np.zeros_like(mask)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                clean_mask[labels == i] = 255

        m_y, m_x = np.where(clean_mask == 255)
        avg_y = int(np.mean(m_y)) if m_y.size > 0 else 0

        if abs(mark_y - avg_y) < 10:
            GPIO.output(ENA1, GPIO.LOW)
            GPIO.output(ENA2, GPIO.LOW)
            break
        else:
            GPIO.output(ENA1, GPIO.HIGH)
            GPIO.output(ENA2, GPIO.HIGH)
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)

    # Zigbee 끊김 대기 (연속 데이터 없음 기준)
    # (지그비 끊김 대기 중인 상태)
    no_data_count = 0
    while True:
        if zigbee_serial and zigbee_serial.in_waiting > 0:
            data = zigbee_serial.readline().decode(errors="ignore").strip()
            if "OK" in data:
                no_data_count = 0
        else:
            no_data_count += 1
            if no_data_count > 2:  # 2회 연속 데이터 없어야 끊겼다고 인식함
                motor_backward(15)
                return STATE_DISCONNECT
        time.sleep(0.5)


#  초기 상태 및 카메라 프레임
current_state = STATE_IDLE
prev_state = STATE_IDLE
frame1 = picam2.capture_array()
time.sleep(0.1)
frame2 = picam2.capture_array()
motion_count = 0

# 메인 루프(상태 변화)
try:
    while True:
        new_state = check_zigbee_state(prev_state)

        if new_state != current_state:
            current_state = new_state

        if current_state == STATE_IDLE:
            handle_idle()

        elif current_state == STATE_ACTIVE:
            result = handle_active()
            if result == STATE_DISCONNECT:
                current_state = STATE_DISCONNECT

        elif current_state == STATE_DISCONNECT:
            current_state = STATE_IDLE

        prev_state = current_state
#----------------------------------------------------------------------------------
        #   check_zigbee_state-> STATE_IDLE (부저 on)   --ㅣ ---> prev_state
        #   (현재 상태)         -> STATE_ACTIVE (10초 전진 후 흰선 검출)          ㅣ     
        #                     -> STATE_DISCONNECT (15초 후진 후 STATE_IDLE)—ㅣ
        #
        #————————————————————————————————————————
except KeyboardInterrupt:
    print("DONE")

finally:
    # 디버깅할때아니면작동안하는코드 / 주석처리하지말것
    GPIO.cleanup()
    cv2.destroyAllWindows()
    picam2.stop()
