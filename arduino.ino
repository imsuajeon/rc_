#include <Servo.h>
#include "PinChangeInterrupt.h"

#define NEUTRAL_THROTTLE 1500

#define pinRC1 A0  // 수동 조향 (CH1)
#define pinRC2 A1  // 수동 스로틀 (CH2)
#define pinRC7 A2  // 모드 전환 (CH7)

volatile int nRC1PulseWidth = 1500;
volatile int nRC2PulseWidth = 1500;
volatile int nRC7PulseWidth = 1500;

volatile unsigned long rise7 = 0;
volatile unsigned int  pw7   = 1500;
volatile bool          up7   = false;

/* 하드웨어 핀 */
const int STEER_PIN = 2;   // 조향 서보
const int ESC_PIN   = 3;   // ESC

Servo steering;
Servo esc;

/* PWM 기준값 */
const int SERVO_CENTER   = 1500;
const int SERVO_LEFT     = 1000;
const int SERVO_RIGHT    = 2000;
const int THROTTLE_FWD   = 1550;
const int THROTTLE_STOP  = 1500;

const unsigned long AUTO_TIMEOUT = 300;
unsigned long lastCmdTime = 0;

// 자율 모드 변수 및 임계값
bool autoMode = false;
const unsigned int AUTO_THRESHOLD = 1500;

/* 펄스 캡처 함수 */
volatile unsigned long ulRC1StartHigh = 0;
volatile boolean bNewRC1Pulse = false;
void pwmRC1(void) {
  if (digitalRead(pinRC1) == HIGH)
    ulRC1StartHigh = micros();
  else if (ulRC1StartHigh && !bNewRC1Pulse) {
    nRC1PulseWidth = micros() - ulRC1StartHigh;
    ulRC1StartHigh = 0;
    bNewRC1Pulse = true;
  }
}

volatile unsigned long ulRC2StartHigh = 0;
volatile boolean bNewRC2Pulse = false;
void pwmRC2(void) {
  if (digitalRead(pinRC2) == HIGH)
    ulRC2StartHigh = micros();
  else if (ulRC2StartHigh && !bNewRC2Pulse) {
    nRC2PulseWidth = micros() - ulRC2StartHigh;
    ulRC2StartHigh = 0;
    bNewRC2Pulse = true;
  }
}

volatile unsigned long ulRC7StartHigh = 0;
volatile boolean bNewRC7Pulse = false;
void pwmRC7(void) {
  if (digitalRead(pinRC7) == HIGH)
    ulRC7StartHigh = micros();
  else if (ulRC7StartHigh && !bNewRC7Pulse) {
    nRC7PulseWidth = micros() - ulRC7StartHigh;
    ulRC7StartHigh = 0;
    bNewRC7Pulse = true;
  }
}

void setup() {
  Serial.begin(9600);

  steering.attach(STEER_PIN);
  esc.attach(ESC_PIN);

  pinMode(pinRC1, INPUT_PULLUP);
  pinMode(pinRC2, INPUT_PULLUP);
  pinMode(pinRC7, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(pinRC1), pwmRC1, CHANGE);
  attachPCINT(digitalPinToPCINT(pinRC2), pwmRC2, CHANGE);
  attachPCINT(digitalPinToPCINT(pinRC7), pwmRC7, CHANGE);

  // ESC 아밍
  steering.writeMicroseconds(SERVO_CENTER);
  esc.writeMicroseconds(THROTTLE_STOP);
  delay(3000);

  Serial.println(F("READY"));
}

void loop() {
  // 1. 채널 7을 기준으로 모드 설정
  // nRC7PulseWidth 값이 AUTO_THRESHOLD 이상일 경우 자율 모드로 전환
  if (nRC7PulseWidth > AUTO_THRESHOLD) {
    autoMode = true;
  } else {
    autoMode = false;
  }

  // 2. 자율 모드: Pi로부터 명령 수신
  if (autoMode) {
    while (Serial.available()) {
      char c = Serial.read();
      uint8_t cx = Serial.read();
      lastCmdTime = millis();

      switch (c) {
        case 'R':
          steering.writeMicroseconds(SERVO_LEFT);
          esc.writeMicroseconds(THROTTLE_FWD);
          digitalWrite(7, HIGH); digitalWrite(8, LOW); digitalWrite(9, LOW);
          break;
        case 'L':
          steering.writeMicroseconds(SERVO_RIGHT);
          esc.writeMicroseconds(THROTTLE_FWD);
          digitalWrite(7, LOW); digitalWrite(8, HIGH); digitalWrite(9, LOW);
          break;
        case 'S':
          steering.writeMicroseconds(SERVO_CENTER);
          esc.writeMicroseconds(THROTTLE_FWD);
          digitalWrite(7, LOW); digitalWrite(8, LOW); digitalWrite(9, HIGH);
          break;
        case 'D':
          steering.writeMicroseconds(SERVO_CENTER);
          esc.writeMicroseconds(THROTTLE_STOP);
          digitalWrite(7, LOW); digitalWrite(8, HIGH); digitalWrite(9, HIGH);
          break;
      }
    }

    // 3. Fail-safe: 일정 시간 입력 없으면 정지
    if (millis() - lastCmdTime > AUTO_TIMEOUT) {
      steering.writeMicroseconds(SERVO_CENTER);
      esc.writeMicroseconds(THROTTLE_STOP);
    }

  } else {
    // 4. 수동 모드: RC PWM 값 직접 반영
    steering.writeMicroseconds(nRC1PulseWidth);  // 수동 조향
    esc.writeMicroseconds(nRC2PulseWidth);       // 수동 스로틀
  }

  // 5. 상태 출력
  Serial.print("RC7 Pulse: ");
  Serial.print(nRC7PulseWidth);
  Serial.print(" → ");
  Serial.println(autoMode ? "AUTO" : "MANUAL");

  delay(10);  // 100 Hz 루프
}
