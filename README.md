# 자율 주행 RC카 프로젝트

## 팀 구성원
- **송제예**: 전수아, 이채은

---

## 프로젝트 개요
본 프로젝트는 Raspberry Pi와 Arduino Uno를 활용하여 자율 주행 및 수동 조향을 모두 지원하는 RC 카를 구현합니다.  
- **Arduino**: RC 리시버(PWM) 신호 읽기, 수동/자율 모드 전환, 서보·ESC 제어  
- **Raspberry Pi**: 카메라(PiCamera2)로 라인 검출 후 명령 전송, OpenCV 기반 이미지 처리  

---

### 1. Arduino (`arduino.ino`)
1. **RC PWM 수신**  
   - CH1(pin A0), CH2(pin A1), CH7(pin A2) 채널을 PinChangeInterrupt로 캡처  
   - 상승 에지→시간 측정, 하강 에지→펄스폭(`nRCxPulseWidth`) 계산  
2. **모드 전환**  
   - CH7 펄스폭 ≥ 1500µs → **자율 모드** (`autoMode = true`)  
   - CH7 펄스폭 < 1500µs → **수동 모드**  
3. **자율 모드 동작**  
   - Serial로 Pi에서 보낸 명령(R/L/S/D)과 추가 데이터(cx)를 읽음  
   - 각 명령에 대응해 서보(`steering.writeMicroseconds`)·ESC(`esc.writeMicroseconds`) 조작  
   - 일정 시간 입력 없을 시(300ms) Fail-safe로 정지  
4. **수동 모드 동작**  
   - 수신된 PWM 값을 서보·ESC에 직접 쓰기  
5. **상태 출력**  
   - 매 루프마다 CH7 펄스폭과 모드(MANUAL/AUTO)를 Serial 모니터에 출력

| 명령 | 동작           | LED 핀 상태         | 설명                           |
|------|----------------|---------------------|--------------------------------|
| R    | 우회전         | 7 HIGH, 8 LOW, 9 LOW  | 핀 7(우회전 지시등) 켜짐        |
| L    | 좌회전         | 7 LOW, 8 HIGH, 9 LOW  | 핀 8(좌회전 지시등) 켜짐        |
| S    | 직진           | 7 LOW, 8 LOW, 9 HIGH  | 핀 9(전진등) 켜짐              |
| D    | 후진            | 7 HIGH, 8 HIGH, 9 HIGH | 핀 7(우회전)+핀 8(좌회전)+핀 9(전진) 동시 켜짐 |

![image](https://github.com/user-attachments/assets/7643b38f-2658-41f9-b974-e0689f9f8751)


---

### 2. Raspberry Pi (`line.py`)
1. **Arduino 자동 포트 연결**  
   - `serial.tools.list_ports`로 `ttyACM*` 또는 `Arduino` 디바이스 자동 탐색  
2. **PiCamera2 설정**  
   - 해상도 640×480, RGB888 포맷, 30FPS  
   - 센서 워밍업(200ms) 후 촬영 시작  
3. **라인 검출 파이프라인**  
   1. **ROI 설정**: 하단 중앙 크롭  
   2. **그레이스케일 변환** → **가우시안 블러** → **Otsu 이진화(THRESH_BINARY_INV)**  
   3. **모폴로지 클로징**(3×3 커널, 2회)로 틈 메우기 및 노이즈 제거  
   4. **컨투어 검출** → 면적 최대 컨투어 선택 → 모멘트로 무게중심(cx) 계산  
   5. **명령 결정**  
      - `cx ≥ 180×0.60` → `'R'` (우회전)  
      - `cx ≤ 180×0.40` → `'L'` (좌회전)  
      - 그 외 → `'S'` (직진)  
   6. **라인 미검출 시** miss 카운트 증가 → 20회 이상 누적 시 `'D'` (정지) 전송  
4. **명령 전송**  
   - `ser.write(bytes([ord(cmd), cx & 0xFF]))` 형식으로 2바이트 전송  
   - Serial flush 후 디버그 출력  
5. **디버그 이미지 저장**  
   - `last_crop.jpg`, `last_th.jpg`, `last_mask.jpg` 파일로 매 프레임 갱신

---

## 이미지 처리 및 라인트레이싱 제어 방법
- **크롭 영역**: 전체 프레임 중 차량 바로 앞 노면만 분석하여 속도 저하 최소화  
- **이진화 기법**: Otsu 알고리즘을 이용한 자동 임계치  
- **모폴로지**: 작은 틈 제거 및 연속성 보장  
- **무게중심(cx) 매핑**: 픽셀 좌표→0–180 범위로 스케일링 후 임계 구간(40%, 60%)에 따라 방향 결정  
- **Fail-safe**: 라인 상실 시 일정 루프 후 비상 정지

![image](https://github.com/user-attachments/assets/b7192fcf-0756-48f1-9e34-a74a9d876dc7)

원본 프레임에서 차량 바로 앞 노면만 잘라낸 부분

![image](https://github.com/user-attachments/assets/255e9dc7-3665-4754-a3f7-89cdc227d9b5)

자른 사진을 그레이스케일로 변환한 뒤 가우시안 블러를 적용하여 Otsu 기법으로 이진화

![image](https://github.com/user-attachments/assets/013a882c-a7b2-41a3-b997-e287c59fcde9)

모폴로지 클로징 연산 결과
틈새를 메꾸고 노이즈를 제거



---

## 라즈베리파이 ↔ 아두이노 통신 프로토콜
- **전송 포맷**: 2바이트 시퀀스  
  1. **명령 코드**: ASCII `'R'`, `'L'`, `'S'`, `'D'`  
  2. **추가 데이터**: `cx & 0xFF` (무게중심 하위 8비트)  
- **Baudrate**: 9600, **타임아웃**: 1초  
- **Fail-safe 관리**: 아두이노 측에서 `lastCmdTime`을 기준으로 300ms 이상 명령 수신 없을 시 정지

---

## 기여 분담

| 기능/모듈                 | 담당 팀원   | 주요 업무 및 설명                                                |
|---------------------------|------------|-----------------------------------------------------------------|
| Arduino 제어 로직 & 통신  | 이채은    | • RC PWM 캡처 및 수동/자율 모드 전환<br>• 서보·ESC 제어 및 Fail-safe 구현<br>• Raspberry Pi와의 Serial 프로토콜 설계 |
| Raspberry Pi 이미지 처리 & 문서화 | 전수아     | • 프레임 크롭 → 그레이스케일/블러 → Otsu 이진화 → 모폴로지 클로징<br>• 컨투어 기반 방향 결정 및 명령 전송<br>• README 작성, 디버그 이미지 저장 및 테스트 시나리오 정리 |

