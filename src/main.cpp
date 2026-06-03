#include <Arduino.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
/*
  2WD Autonomous RC Car  v3
  Board  : Arduino Uno SMD R3
  Shield : HW-130 V0.0.4 / L293D Motor Shield V1
  Left   : M1  /  Right : M4
  Sonar  : HC-SR04, TRIG=A0, ECHO=A1

  주행 전략 (v3):
  ┌─────────────────────────────────────────────────────┐
  │ 1. 교번 피벗 턴                                      │
  │    홀수 스텝 → 오른쪽 바퀴 전진                       │
  │    짝수 스텝 → 왼쪽 바퀴 후진                         │
  │    → 회전 중심이 차체 중앙에 가까워져 회전 반경 최소화  │
  │                                                     │
  │ 2. 좁은 스윕 + 후진 반복                             │
  │    한 라운드 = 한쪽 방향 MAX_SWEEP_STEPS(4) 스텝       │
  │    왼쪽 실패 → 오른쪽 / 오른쪽 실패 → 후진 후 재시도   │
  │    매 라운드마다 시작 방향 교체 (편향 방지)             │
  │                                                     │
  │ 3. 피크 탐색                                         │
  │    처음 CLEAR_DISTANCE 도달 → 즉시 직진 안 함          │
  │    거리가 정체/하강으로 바뀌는 피크 지점을 찾아서 직진   │
  │    → 코너 대각선에서 조기 직진하는 문제 방지            │
  │    피크 탐색도 PEAK_MAX_STEPS로 횟수 제한              │
  └─────────────────────────────────────────────────────┘
*/
// =======================================================
// 1. 디버그 설정
// =======================================================

#define DEBUG_SERIAL 1

const unsigned long DEBUG_PRINT_INTERVAL_MS = 300;

// =======================================================
// 2. 핀 설정
// =======================================================

const uint8_t TRIG_PIN = A0;
const uint8_t ECHO_PIN = A1;

// =======================================================
// 2-1. Bluetooth 로그 설정
// =======================================================

const uint8_t BT_RX_PIN = A4;  // Arduino RX <- HC-06 TXD
const uint8_t BT_TX_PIN = A5;  // Arduino TX -> HC-06 RXD

SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// =======================================================
// 3. 모터 설정
// =======================================================

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(4);

const uint8_t LEFT_MOTOR_INVERT  = 0;
const uint8_t RIGHT_MOTOR_INVERT = 0;

const int LEFT_TRIM  = -10;
const int RIGHT_TRIM = 35;

// =======================================================
// 4. 속도
// =======================================================

const int DRIVE_SPEED = 155;
const int TURN_SPEED  = 155;
const int BACK_SPEED  = 130;

const int MOTOR_MIN = -255;
const int MOTOR_MAX = 255;

// =======================================================
// 5. 거리 기준값
// =======================================================

const int DANGER_DISTANCE      = 13;
const int CLEAR_DISTANCE       = 20;
const int MIN_VALID_DISTANCE   = 2;
const int MAX_VALID_DISTANCE   = 100;
const int DANGER_CONFIRM_COUNT = 1;

// =======================================================
// 6. 초음파
// =======================================================

const unsigned long SONAR_INTERVAL_MS = 60;
const unsigned long SONAR_TIMEOUT_US  = 12000UL;

int distanceBuffer[3] = {80, 80, 80};
uint8_t distanceIndex = 0;
int filteredDistance  = 80;
int lastValidDistance = 80;

unsigned long lastSonarMs = 0;

// =======================================================
// 7. 타이밍
// =======================================================

const unsigned long STOP_SETTLE_MS    = 200;
const unsigned long BACK_MS           = 400;
const unsigned long ESCAPE_FORWARD_MS = 500;

const unsigned long SWEEP_STEP_MS    = 200;
const unsigned long SWEEP_SETTLE_MS  = 300;
const int           MAX_SWEEP_STEPS  = 5;
const int           PEAK_MAX_STEPS   = 3;

// =======================================================
// 8. 상태 정의
// =======================================================

enum AutoState {
  ST_CRUISE = 0,
  ST_STOP_BEFORE_BACK,
  ST_BACK,
  ST_SWEEP,
  ST_SWEEP_SETTLE,
  ST_ESCAPE_FORWARD
};

AutoState state = ST_CRUISE;
unsigned long stateStartMs = 0;
int dangerCount = 0;

// =======================================================
// 9. 스윕 전용 변수
// =======================================================

int sweepDir       = -1;
int sweepStepCount = 0;
int sweepRound     = 0;

bool peakMode      = false;
int peakStepCount  = 0;
int prevDistance   = 0;
int bestDistance   = 0;

bool triedBothDirections = false;
// 탈출 성공 방향 기억 (-1=왼쪽으로 탈출, +1=오른쪽으로 탈출)
int escapeDir = -1;

// 직진 타임아웃
const unsigned long CRUISE_TIMEOUT_MS = 10000UL;

// 끼임 감지: 이 시간 동안 거리 변화가 STUCK_DELTA 이하면 끼임
const unsigned long STUCK_CHECK_MS = 5000UL;
const int           STUCK_DELTA    = 3;

unsigned long cruiseStartMs   = 0;
int           cruiseStartDist = 0;
unsigned long stuckCheckMs    = 0;
int           stuckCheckDist  = 0;

// =======================================================
// 10. 함수 선언
// =======================================================

void enterState(AutoState nextState);
void beginSweep();
void beginNextHalf();

void updateSonarIfNeeded();
int  readSonarCm();
bool isValidDistance(int cm);
void pushDistance(int cm);
int  median3(int a, int b, int c);

void runAutonomous();
void handleCruise();
void handleStopBeforeBack();
void handleBack();
void handleSweep();
void handleSweepSettle();
void handleEscapeForward();

void pivotStep(int direction, int stepIndex);
void setMotor(int left, int right);
void setOneMotor(AF_DCMotor &motor, int signedSpeed, uint8_t invert);
int  applyTrim(int speed, int trim);
void stopMotor();

const __FlashStringHelper *stateName(AutoState s);
void debugEvent(const __FlashStringHelper *msg);
void debugPrint();

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

#if DEBUG_SERIAL
  Serial.println(F("RC autonomous v3"));
  Serial.println(F("교번 피벗 + 대칭 스윕 + 피크 탐색"));
  Serial.println(F("BT log: HC-06 RX=A2 TX=A3"));

  BT.println(F("RC autonomous v3"));
  BT.println(F("BT log ready"));
#endif

  stateStartMs = millis();
}

// =======================================================
// 12. loop
// =======================================================

void loop() {
  updateSonarIfNeeded();
  runAutonomous();
  debugPrint();
}

// =======================================================
// 13. 상태 진입
// =======================================================

void enterState(AutoState nextState) {
  state        = nextState;
  stateStartMs = millis();
  dangerCount  = 0;

#if DEBUG_SERIAL
  Serial.print(F("["));
  Serial.print(stateName(state));
  Serial.print(F("] D="));
  Serial.print(filteredDistance);
  Serial.print(F(" DIR="));
  Serial.print(sweepDir == -1 ? F("L") : F("R"));
  Serial.print(F(" STP="));
  Serial.print(sweepStepCount);
  Serial.print(F(" RND="));
  Serial.println(sweepRound);

  BT.print(F("상태 -> "));
  BT.println(stateName(state));
#endif
}

// =======================================================
// 14. 스윕 시작
// =======================================================

void beginSweep() {
  // 탈출 전진 중 충돌했다면 escapeDir의 반대 방향부터 탐색
  // 탈출 방향 = 왔던 방향이므로 반대쪽이 새 길일 확률이 높음
  if (sweepRound > 0) {
    sweepDir = -escapeDir;
  } else {
    sweepDir = (sweepRound % 2 == 0) ? -1 : 1;
  }

  sweepStepCount = 0;
  sweepRound++;

  peakMode      = false;
  peakStepCount = 0;
  prevDistance  = filteredDistance;
  bestDistance  = filteredDistance;

  enterState(ST_SWEEP);

#if DEBUG_SERIAL
  Serial.print(F("스윕 라운드="));
  Serial.print(sweepRound);
  Serial.print(F(" 방향="));
  Serial.println(sweepDir == -1 ? F("왼쪽") : F("오른쪽"));
  BT.print(F("스윕 라운드="));
  BT.print(sweepRound);
  BT.print(F(" 방향="));
  BT.println(sweepDir == -1 ? F("왼쪽") : F("오른쪽"));
#endif
}

// =======================================================
// 15. 현재 방향 실패 후 반대쪽 전환
// =======================================================

void beginNextHalf() {
  sweepDir       = -sweepDir;
  sweepStepCount = 0;

  peakMode      = false;
  peakStepCount = 0;
  prevDistance  = filteredDistance;
  bestDistance  = filteredDistance;

  enterState(ST_SWEEP);

#if DEBUG_SERIAL
  Serial.print(F("반대 방향 전환 -> "));
  Serial.println(sweepDir == -1 ? F("왼쪽") : F("오른쪽"));

  BT.print(F("반대 방향 전환 -> "));
  BT.println(sweepDir == -1 ? F("왼쪽") : F("오른쪽"));
#endif
}

// =======================================================
// 16. 초음파
// =======================================================

void updateSonarIfNeeded() {
  unsigned long now = millis();

  if (now - lastSonarMs < SONAR_INTERVAL_MS) {
    return;
  }

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

  if (duration == 0) {
    return -1;
  }

  return (int)(duration / 58UL);
}

bool isValidDistance(int cm) {
  return (cm >= MIN_VALID_DISTANCE && cm <= MAX_VALID_DISTANCE);
}

void pushDistance(int cm) {
  distanceBuffer[distanceIndex] = cm;

  if (++distanceIndex >= 3) {
    distanceIndex = 0;
  }
}

int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) {
    return b;
  }

  if ((b <= a && a <= c) || (c <= a && a <= b)) {
    return a;
  }

  return c;
}

// =======================================================
// 17. 자율주행 상태 머신
// =======================================================

void runAutonomous() {
  switch (state) {
    case ST_CRUISE:
      handleCruise();
      break;

    case ST_STOP_BEFORE_BACK:
      handleStopBeforeBack();
      break;

    case ST_BACK:
      handleBack();
      break;

    case ST_SWEEP:
      handleSweep();
      break;

    case ST_SWEEP_SETTLE:
      handleSweepSettle();
      break;

    case ST_ESCAPE_FORWARD:
      handleEscapeForward();
      break;

    default:
      stopMotor();
      enterState(ST_CRUISE);
      break;
  }
}

// =======================================================
// 18. 상태 처리
// =======================================================

void handleCruise() {
  // 처음 CRUISE 진입 시 기준값 초기화
  if (dangerCount == 0 && cruiseStartMs == 0) {
    cruiseStartMs   = millis();
    cruiseStartDist = filteredDistance;
    stuckCheckMs    = millis();
    stuckCheckDist  = filteredDistance;
  }

  if (filteredDistance <= DANGER_DISTANCE) {
    dangerCount++;
  } else {
    dangerCount = 0;
  }

  if (dangerCount >= DANGER_CONFIRM_COUNT) {
    cruiseStartMs = 0;
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
    return;
  }

  // 10초 직진 타임아웃
  if (millis() - cruiseStartMs >= CRUISE_TIMEOUT_MS) {
    cruiseStartMs = 0;
    debugEvent(F("직진 타임아웃 → 후진"));
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
    return;
  }

  // 끼임 감지: 5초마다 거리 변화 확인
  if (millis() - stuckCheckMs >= STUCK_CHECK_MS) {
    int delta = abs(filteredDistance - stuckCheckDist);
    stuckCheckMs   = millis();
    stuckCheckDist = filteredDistance;

    if (delta <= STUCK_DELTA) {
      cruiseStartMs = 0;
      debugEvent(F("끼임 감지 → 후진"));
      stopMotor();
      enterState(ST_STOP_BEFORE_BACK);
      return;
    }
  }

  setMotor(DRIVE_SPEED, DRIVE_SPEED);
}

void handleStopBeforeBack() {
  stopMotor();

  if (millis() - stateStartMs >= STOP_SETTLE_MS) {
    enterState(ST_BACK);
  }
}

void handleBack() {
  setMotor(-BACK_SPEED, -BACK_SPEED);

  if (millis() - stateStartMs >= BACK_MS) {
    stopMotor();
    beginSweep();
  }
}

void handleSweep() {
  pivotStep(sweepDir, sweepStepCount);

  if (millis() - stateStartMs >= SWEEP_STEP_MS) {
    stopMotor();
    enterState(ST_SWEEP_SETTLE);
  }
}

void pivotStep(int direction, int stepIndex) {
  bool useForward = (stepIndex % 2 == 0);

  // 피벗은 트림 없이 직접 모터 제어
  const int PV = 240;

  if (direction == -1) {
    // 왼쪽 회전
    if (useForward) {
      setOneMotor(motorRight,  PV, RIGHT_MOTOR_INVERT);  // 오른쪽 전진
      setOneMotor(motorLeft,    0, LEFT_MOTOR_INVERT);
    } else {
      setOneMotor(motorLeft,  -PV, LEFT_MOTOR_INVERT);   // 왼쪽 후진
      setOneMotor(motorRight,   0, RIGHT_MOTOR_INVERT);
    }
  } else {
    // 오른쪽 회전
    if (useForward) {
      setOneMotor(motorLeft,   PV, LEFT_MOTOR_INVERT);   // 왼쪽 전진
      setOneMotor(motorRight,   0, RIGHT_MOTOR_INVERT);
    } else {
      setOneMotor(motorRight, -PV, RIGHT_MOTOR_INVERT);  // 오른쪽 후진
      setOneMotor(motorLeft,    0, LEFT_MOTOR_INVERT);
    }
  }
}

void handleSweepSettle() {
  if (millis() - stateStartMs < SWEEP_SETTLE_MS) {
    return;
  }

  int d = filteredDistance;

  if (peakMode) {
    peakStepCount++;

    bool distanceFell = (d < prevDistance);
    bool peakLimitHit = (peakStepCount >= PEAK_MAX_STEPS);

#if DEBUG_SERIAL
    Serial.print(F("[피크탐색] 현재="));
    Serial.print(d);
    Serial.print(F(" 이전="));
    Serial.print(prevDistance);
    Serial.print(F(" 피크스텝="));
    Serial.println(peakStepCount);

    BT.print(F("[피크탐색] 현재="));
    BT.print(d);
    BT.print(F(" 이전="));
    BT.print(prevDistance);
    BT.print(F(" 피크스텝="));
    BT.println(peakStepCount);
#endif

    if (distanceFell || peakLimitHit) {
      if (distanceFell) {
        debugEvent(F("피크 확인 -> 탈출"));
      } else {
        debugEvent(F("피크 탐색 한계 -> 탈출"));
      }

      triedBothDirections = false;
      escapeDir = sweepDir;  // 탈출 성공 방향 기억
      enterState(ST_ESCAPE_FORWARD);
      return;
    
    }

    prevDistance = d;
    enterState(ST_SWEEP);
    return;
  }

  sweepStepCount++;

  if (d >= CLEAR_DISTANCE) {
    peakMode      = true;
    peakStepCount = 0;
    prevDistance  = d;
    bestDistance  = d;

    debugEvent(F("CLEAR 첫 감지 -> 피크 탐색 시작"));
    enterState(ST_SWEEP);
    return;
  }

  if (sweepStepCount < MAX_SWEEP_STEPS) {
    enterState(ST_SWEEP);
    return;
  }

  if (!triedBothDirections) {
    triedBothDirections = true;
    debugEvent(F("현재 방향 소진 -> 반대 방향"));
    beginNextHalf();
    return;
  }

  triedBothDirections = false;
  debugEvent(F("양쪽 소진 -> 후진 재시도"));
  enterState(ST_STOP_BEFORE_BACK);
}

void handleEscapeForward() {
  if (filteredDistance <= DANGER_DISTANCE) {
    stopMotor();
    debugEvent(F("탈출 중 장애물 재감지"));
    enterState(ST_STOP_BEFORE_BACK);
    return;
  }

  setMotor(DRIVE_SPEED, DRIVE_SPEED);

  if (millis() - stateStartMs >= ESCAPE_FORWARD_MS) {
    cruiseStartMs = 0;  // CRUISE 진입 전 타이머 초기화
    enterState(ST_CRUISE);
  }
}

// =======================================================
// 19. 모터 제어
// =======================================================

void setMotor(int left, int right) {
  left  = constrain(left, MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  left  = applyTrim(left, LEFT_TRIM);
  right = applyTrim(right, RIGHT_TRIM);

  left  = constrain(left, MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  setOneMotor(motorLeft,  left,  LEFT_MOTOR_INVERT);
  setOneMotor(motorRight, right, RIGHT_MOTOR_INVERT);
}

int applyTrim(int speed, int trim) {
  if (speed > 0) {
    return speed + trim;
  }

  if (speed < 0) {
    return speed - trim;
  }

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

  if (invert) {
    forward = !forward;
  }

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
// 20. 디버그
// =======================================================

const __FlashStringHelper *stateName(AutoState s) {
  switch (s) {
    case ST_CRUISE:
      return F("일반 주행");

    case ST_STOP_BEFORE_BACK:
      return F("후진 전 정지");

    case ST_BACK:
      return F("후진");

    case ST_SWEEP:
      return F("스윕 회전");

    case ST_SWEEP_SETTLE:
      return F("스윕 안정화");

    case ST_ESCAPE_FORWARD:
      return F("탈출 전진");

    default:
      return F("알 수 없음");
  }
}

void debugEvent(const __FlashStringHelper *msg) {
#if DEBUG_SERIAL
  Serial.print(F("이벤트: "));
  Serial.print(msg);
  Serial.print(F(" 거리="));
  Serial.print(filteredDistance);
  Serial.print(F(" 방향="));
  Serial.print(sweepDir == -1 ? F("L") : F("R"));
  Serial.print(F(" 스텝="));
  Serial.println(sweepStepCount);

  BT.print(F("이벤트: "));
  BT.print(msg);
  BT.print(F(" 거리="));
  BT.print(filteredDistance);
  BT.print(F(" 방향="));
  BT.print(sweepDir == -1 ? F("L") : F("R"));
  BT.print(F(" 스텝="));
  BT.println(sweepStepCount);
#else
  (void)msg;
#endif
}

void debugPrint() {
#if DEBUG_SERIAL
  static unsigned long lastDebugMs = 0;

  if (millis() - lastDebugMs < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastDebugMs = millis();

  BT.print(F("ST="));
  BT.print((int)state);
  BT.print(F(" D="));
  BT.print(filteredDistance);
  BT.print(F(" DIR="));
  BT.print(sweepDir == -1 ? F("L") : F("R"));
  BT.print(F(" STP="));
  BT.print(sweepStepCount);
  BT.print(F(" PEAK="));
  BT.print(peakMode ? F("Y") : F("N"));
  BT.print(F(" RND="));
  BT.println(sweepRound);
#endif
}