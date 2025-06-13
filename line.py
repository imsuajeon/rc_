#!/usr/bin/env python3
# Pi-Cam Line Follower  (Picamera2 + OpenCV + Arduino Serial)
import cv2, numpy as np, time, serial, serial.tools.list_ports
from picamera2 import Picamera2, Preview

# ────────────────────────────────────────────────────────────
# 1. Arduino 자동 포트
port = next(p.device for p in serial.tools.list_ports.comports()
            if 'ttyACM' in p.device or 'Arduino' in p.description)
ser  = serial.Serial(port, 9600, timeout=1)
print("✓ Arduino:", port)

# 2. Picamera2 설정  (160×120 YUV420)
W, H = 640, 480
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},  # BGR과 호환
        controls={"FrameDurationLimits": (33333, 33333)}))  # 30 fps
picam2.start()
time.sleep(0.2)                                     # 센서 워밍업

# 3. 라인 검출 파라미터
gap, right_g, left_g = 0.10, 0.60, 0.40
kernel = np.ones((3,3), np.uint8)
miss = 0

print("▶ 시작!  Ctrl-C 로 종료")
while True:
    frame = picam2.capture_array()                  # (120,160,3) RGB
    crop  = frame[H//4+45:H//2-50, W//4:W//4*3]                      # 하단 절반
    gray  = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
    blur  = cv2.GaussianBlur(gray, (5,5), 0)
    _, th = cv2.threshold(blur, 0, 255,
                          cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    #cv2.imshow("Threshold", th)  # 디버그용
    # ── (3) 틈 메우기 & 잡음 제거 ──────────────
    mask = cv2.morphologyEx(th, cv2.MORPH_CLOSE,
                            kernel, iterations=2)
    #cv2.imshow("Mask", mask)  # 디버그용
    # ── (4) 컨투어 찾기 ────────────────────────
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_NONE)
    if cnts:
        miss = 0
        c  = max(cnts, key=cv2.contourArea)
        M  = cv2.moments(c);  cx = int(M['m10']/M['m00'])
        cx = int(cx * (180 / (W//2 )))  # crop 크기에 맞춤
        cmd = 'S'
        if cx >= 180 * right_g:  cmd = 'R'
        elif cx <= 180 * left_g: cmd = 'L'
        print(f"Sending: {cmd} {cx}")
        ser.write(bytes([ord(cmd), cx & 0xFF]))
        ser.flush()
        print(cmd, cx)
    else:
        miss += 1
        if miss > 20:
            ser.write(b'D\n'); miss = 0
            print("D (line lost)")

    if cv2.waitKey(1) == ord('q'):
        print("종료")
        break
    # 디버그 프레임 파일 갱신
    cv2.imwrite("last_crop.jpg", crop)
    cv2.imwrite("last_th.jpg", th)
    cv2.imwrite("last_mask.jpg", mask)
