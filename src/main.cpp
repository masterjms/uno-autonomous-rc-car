#include <Arduino.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

/*
  2WD Autonomous RC Car  v4
  Board  : Arduino Uno SMD R3
  Shield : HW-130 V0.0.4 / L293D Motor Shield V1
  Left   : M1  /  Right : M4
  Sonar  : HC-SR04, TRIG=A0, ECHO=A1

  주행 정책:
  1. 직진하다 벽 감지 → 정지
  2. 오른쪽 90도 회전 → 확인
       길이다 → 직진
       아니다 → 정면 복귀 (왼쪽 90도)
  3. 정면 확인
       길이다 → 직진 (이론상 없는 케이스)
       아니다 → 왼쪽 90도 회전 → 확인
  4. 왼쪽 확인
       길이다 → 직진
       아니다 → 유턴 (왼쪽 90도 한번 더) → 직진 (왔던 길)

  회전 방식: 교번 피벗 (한쪽 전진 + 반대쪽 후진 반복)
  후진 정책: 없음
  스윕 정책: 없음
*/

// =======================================================
// 1. 디버그
// =======================================================

#define DEBUG_SERIAL 1

// =======================================================
// 2. 핀
// =======================================================

const uint8_t TRIG_PIN = A0;
const uint8_t ECHO_PIN = A1;

// =======================================================
// 3. 블루투스
// =======================================================

const uint8_t BT_RX_PIN = A4;
const uint8_t BT_TX_PIN = A5;

SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// =======================================================
// 4. 모터
// =======================================================

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(4);

const uint8_t LEFT_MOTOR_INVERT  = 0;
const uint8_t RIGHT_MOTOR_INVERT = 0;

const int LEFT_TRIM  = -10;
const int RIGHT_TRIM = 35;

const int MOTOR_MIN = -255;
const int MOTOR_MAX = 255;

// =======================================================
// 5. 속도
// =======================================================

const int DRIVE_SPEED = 155;
const int BACK_SPEED  = 130;

/*
  피벗 속도: 트림 없이 직접 모터 제어하므로
  정지 마찰을 이길 수 있도록 충분히 높게 설정
*/
const int PIVOT_SPEED = 220;

// =======================================================
// 6. 거리 기준값
// =======================================================

const int DANGER_DISTANCE     = 12;
const int CLEAR_DISTANCE      = 22;
const int MIN_VALID_DISTANCE  = 2;
const int MAX_VALID_DISTANCE  = 200;
const int DANGER_CONFIRM_COUNT = 1;

// =======================================================
// 7. 초음파
// =======================================================

const unsigned long SONAR_INTERVAL_MS = 60;
const unsigned long SONAR_TIMEOUT_US  = 12000UL;

int distanceBuffer[3] = {80, 80, 80};
uint8_t distanceIndex  = 0;
int filteredDistance   = 80;
int lastValidDistance  = 80;

unsigned long lastSonarMs = 0;

// =======================================================
// 8. 타이밍
// =======================================================

const unsigned long STOP_SETTLE_MS   = 200;
const unsigned long ESCAPE_FWD_MS    = 600;

/*
  PIVOT_90_MS: 교번 피벗으로 90도 회전하는 데 걸리는 시간.
  실제 하드웨어에서 측정해서 이 값만 조정한다.
  - 너무 짧으면 90도 미만 회전
  - 너무 길면 90도 초과 회전
  기본값 700ms는 시작점, 실측 후 수정할 것.
*/
const unsigned long PIVOT_90_MS      = 350;
const unsigned long PIVOT_180_MS     = 700;  // 90도 * 2
const unsigned long CHECK_SETTLE_MS  = 350;  // 150 → 350

// =======================================================
// 9. 상태 정의
// =======================================================

/*
  ST_CRUISE       : 직진
  ST_STOP         : 정지 + 관성 제거
  ST_TURN_RIGHT   : 오른쪽 90도 회전
  ST_CHECK_RIGHT  : 오른쪽 방향 거리 확인
  ST_TURN_CENTER  : 정면 복귀 (왼쪽 90도)
  ST_CHECK_CENTER : 정면 거리 확인
  ST_TURN_LEFT    : 왼쪽 90도 회전
  ST_CHECK_LEFT   : 왼쪽 방향 거리 확인
  ST_UTURN        : 유턴 (왼쪽 90도 한번 더)
  ST_ESCAPE_FWD   : 탈출 전진
*/
enum AutoState {
  ST_CRUISE = 0,
  ST_STOP,
  ST_TURN_RIGHT,
  ST_CHECK_RIGHT,
  ST_TURN_LEFT_180,   // 정면 복귀 + 왼쪽 탐색을 한번에
  ST_CHECK_LEFT,
  ST_UTURN,
  ST_ESCAPE_FWD
};

AutoState state        = ST_CRUISE;
unsigned long stateStartMs = 0;
int dangerCount        = 0;

// =======================================================
// 10. 함수 선언
// =======================================================

void enterState(AutoState s);

void updateSonarIfNeeded();
int  readSonarCm();
bool isValidDistance(int cm);
void pushDistance(int cm);
int  median3(int a, int b, int c);

void runAutonomous();
void handleCruise();
void handleStop();
void handleTurnRight();
void handleCheckRight();
void handleTurnLeft180();
void handleCheckLeft();
void handleUturn();
void handleEscapeFwd();

void pivotRight();
void pivotLeft();
void setMotor(int left, int right);
void setOneMotor(AF_DCMotor &motor, int signedSpeed, uint8_t invert);
int  applyTrim(int speed, int trim);
void stopMotor();

const __FlashStringHelper *stateName(AutoState s);
void logln(const __FlashStringHelper *msg);

// =======================================================
// 11. setup
// =======================================================

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  stopMotor();

  logln(F("RC autonomous v4 - open loop 90deg"));

  stateStartMs = millis();
}

// =======================================================
// 12. loop
// =======================================================

void loop() {
  updateSonarIfNeeded();
  runAutonomous();
}

// =======================================================
// 13. 상태 진입
// =======================================================

void enterState(AutoState s) {
  state        = s;
  stateStartMs = millis();
  dangerCount  = 0;

#if DEBUG_SERIAL
  Serial.print(F("["));
  Serial.print(stateName(s));
  Serial.print(F("] D="));
  Serial.println(filteredDistance);

  BT.print(F("["));
  BT.print(stateName(s));
  BT.print(F("] D="));
  BT.println(filteredDistance);
#endif
}

// =======================================================
// 14. 초음파
// =======================================================

void updateSonarIfNeeded() {
  unsigned long now = millis();
  if (now - lastSonarMs < SONAR_INTERVAL_MS) return;
  lastSonarMs = now;

  int raw = readSonarCm();

  if (isValidDistance(raw)) {
    lastValidDistance = raw;
    pushDistance(raw);
  } else {
    pushDistance(lastValidDistance);
  }

  filteredDistance = median3(
    distanceBuffer[0],
    distanceBuffer[1],
    distanceBuffer[2]
  );
}

int readSonarCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1;
  return (int)(duration / 58UL);
}

bool isValidDistance(int cm) {
  return (cm >= MIN_VALID_DISTANCE && cm <= MAX_VALID_DISTANCE);
}

void pushDistance(int cm) {
  distanceBuffer[distanceIndex] = cm;
  if (++distanceIndex >= 3) distanceIndex = 0;
}

int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// =======================================================
// 15. 상태 머신
// =======================================================

void runAutonomous() {
  switch (state) {
    case ST_CRUISE:        handleCruise();      break;
    case ST_STOP:          handleStop();        break;
    case ST_TURN_RIGHT:    handleTurnRight();   break;
    case ST_CHECK_RIGHT:   handleCheckRight();  break;
    case ST_TURN_LEFT_180: handleTurnLeft180(); break;
    case ST_CHECK_LEFT:    handleCheckLeft();   break;
    case ST_UTURN:         handleUturn();       break;
    case ST_ESCAPE_FWD:    handleEscapeFwd();   break;
    default:
      stopMotor();
      enterState(ST_CRUISE);
      break;
  }
}
// =======================================================
// 16. 상태 처리
// =======================================================

void handleCruise() {
  if (filteredDistance <= DANGER_DISTANCE) {
    dangerCount++;
  } else {
    dangerCount = 0;
  }

  if (dangerCount >= DANGER_CONFIRM_COUNT) {
    stopMotor();
    enterState(ST_STOP);
    return;
  }

  setMotor(DRIVE_SPEED, DRIVE_SPEED);
}

void handleStop() {
  stopMotor();
  if (millis() - stateStartMs >= STOP_SETTLE_MS) {
    enterState(ST_TURN_RIGHT);
  }
}

/*
  오른쪽 90도 회전.
  pivotRight()를 PIVOT_90_MS 동안 실행.
*/
void handleTurnRight() {
  pivotRight();
  if (millis() - stateStartMs >= PIVOT_90_MS) {
    stopMotor();
    enterState(ST_CHECK_RIGHT);
  }
}

void handleCheckRight() {
  stopMotor();
  if (millis() - stateStartMs < CHECK_SETTLE_MS) return;
  if (filteredDistance >= CLEAR_DISTANCE) {
    logln(F("오른쪽 길 발견 → 직진"));
    enterState(ST_ESCAPE_FWD);
  } else {
    logln(F("오른쪽 막힘 → 왼쪽 180도"));
    enterState(ST_TURN_LEFT_180);
  }
}

/*
  왼쪽 180도:
  오른쪽으로 90도 돌아있는 상태에서 왼쪽으로 180도 돌면
  정면 복귀(90도) + 왼쪽 탐색(90도)을 한번에 처리.
  회전 횟수 줄여서 누적 오차 감소.
*/
void handleTurnLeft180() {
  pivotLeft();
  if (millis() - stateStartMs >= PIVOT_180_MS) {
    stopMotor();
    enterState(ST_CHECK_LEFT);
  }
}

void handleCheckLeft() {
  stopMotor();
  if (millis() - stateStartMs < CHECK_SETTLE_MS) return;
  if (filteredDistance >= CLEAR_DISTANCE) {
    logln(F("왼쪽 길 발견 → 직진"));
    enterState(ST_ESCAPE_FWD);
  } else {
    logln(F("왼쪽도 막힘 → 유턴"));
    enterState(ST_UTURN);
  }
}

/*
  유턴: 현재 왼쪽을 보고 있으므로 왼쪽으로 90도 한번 더.
  총 270도 회전한 셈 = 원래 방향에서 오른쪽으로 90도
  = 왔던 길 방향
*/
void handleUturn() {
  pivotLeft();

  if (millis() - stateStartMs >= PIVOT_90_MS) {
    stopMotor();
    logln(F("유턴 완료 → 직진 (왔던 길)"));
    enterState(ST_ESCAPE_FWD);
  }
}

/*
  탈출 전진.
  ESCAPE_FWD_MS 동안 전진 후 일반 주행 복귀.
  전진 중에도 DANGER 감지하면 즉시 정지 후 재탐색.
*/
void handleEscapeFwd() {
  if (filteredDistance <= DANGER_DISTANCE) {
    stopMotor();
    logln(F("탈출 중 장애물 감지 → 정지"));
    enterState(ST_STOP);
    return;
  }

  setMotor(DRIVE_SPEED, DRIVE_SPEED);

  if (millis() - stateStartMs >= ESCAPE_FWD_MS) {
    enterState(ST_CRUISE);
  }
}

// =======================================================
// 17. 피벗 제어
// =======================================================

/*
  pivotRight: 오른쪽 방향으로 차체를 돌린다.
    왼쪽 바퀴 전진 + 오른쪽 바퀴 후진

  pivotLeft: 왼쪽 방향으로 차체를 돌린다.
    오른쪽 바퀴 전진 + 왼쪽 바퀴 후진

  트림을 우회하여 setOneMotor()를 직접 호출한다.
  트림이 붙으면 전진/후진 간 힘 불균형이 생겨서
  실제 회전각이 틀어진다.
*/
void pivotRight() {
  setOneMotor(motorLeft,   PIVOT_SPEED, LEFT_MOTOR_INVERT);
  setOneMotor(motorRight, -PIVOT_SPEED, RIGHT_MOTOR_INVERT);
}

void pivotLeft() {
  setOneMotor(motorLeft,  -PIVOT_SPEED, LEFT_MOTOR_INVERT);
  setOneMotor(motorRight,  PIVOT_SPEED, RIGHT_MOTOR_INVERT);
}

// =======================================================
// 18. 모터 제어
// =======================================================

void setMotor(int left, int right) {
  left  = constrain(left,  MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  left  = applyTrim(left,  LEFT_TRIM);
  right = applyTrim(right, RIGHT_TRIM);

  left  = constrain(left,  MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  setOneMotor(motorLeft,  left,  LEFT_MOTOR_INVERT);
  setOneMotor(motorRight, right, RIGHT_MOTOR_INVERT);
}

int applyTrim(int speed, int trim) {
  if (speed > 0) return speed + trim;
  if (speed < 0) return speed - trim;
  return 0;
}

void setOneMotor(AF_DCMotor &motor, int signedSpeed, uint8_t invert) {
  int pwm = constrain(abs(signedSpeed), 0, 255);

  if (signedSpeed == 0) {
    motor.setSpeed(0);
    motor.run(RELEASE);
    return;
  }

  bool forward = (signedSpeed > 0);
  if (invert) forward = !forward;

  motor.setSpeed(pwm);
  motor.run(forward ? FORWARD : BACKWARD);
}

void stopMotor() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}

// =======================================================
// 19. 디버그
// =======================================================

const __FlashStringHelper *stateName(AutoState s) {
  switch (s) {
    case ST_CRUISE:       return F("직진");
    case ST_STOP:         return F("정지");
    case ST_TURN_RIGHT:   return F("우회전 90");
    case ST_CHECK_RIGHT:  return F("우측 확인");
    case ST_TURN_LEFT_180: return F("좌회전 180");
    case ST_CHECK_LEFT:    return F("좌측 확인");
    case ST_UTURN:        return F("유턴 90");
    case ST_ESCAPE_FWD:   return F("탈출 전진");
    default:              return F("알 수 없음");
  }
}

void logln(const __FlashStringHelper *msg) {
#if DEBUG_SERIAL
  Serial.println(msg);
  BT.println(msg);
#else
  (void)msg;
#endif
}
