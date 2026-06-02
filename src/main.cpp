#include <Arduino.h>
#include <AFMotor.h>

/*
  2WD Autonomous RC Car
  Board  : Arduino Uno SMD R3
  Shield : HW-130 V0.0.4 / L293D Motor Shield V1 계열
  Left   : M1
  Right  : M4
  Sonar  : HC-SR04, TRIG=A0, ECHO=A1

  주행 전략:
  - 전방 초음파 센서 1개만 사용
  - 엔코더 없음
  - 범퍼 스위치 없음
  - 정확한 각도 회전이 아니라, 열린 공간 탐색 기반 회피
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
// 3. 모터 설정
// =======================================================

// 실제 사용 기준:
// 왼쪽 전방 구동 모터 = M1
// 오른쪽 전방 구동 모터 = M4
AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(4);

/*
  모터 방향 보정
*/
const uint8_t LEFT_MOTOR_INVERT  = 0;
const uint8_t RIGHT_MOTOR_INVERT = 0;

/*
  좌우 모터 출력 보정
*/
const int LEFT_TRIM  = -10;
const int RIGHT_TRIM = 35;

// =======================================================
// 4. 속도 튜닝값
// =======================================================

/*
  주행 속도는 거리별로 나누지 않고 하나로 통일한다.
*/
const int DRIVE_SPEED = 155;

/*
  회전과 후진도 토크 부족을 막기 위해 이전보다 올림.
*/
const int TURN_SPEED = 155;
const int BACK_SPEED = 130;

// 모터 PWM 범위
const int MOTOR_MIN = -255;
const int MOTOR_MAX = 255;

// =======================================================
// 5. 거리 기준값
// =======================================================

/*
  시험 도로가 매우 좁고 짧기 때문에 거리 기준값은 작게 유지한다.
  이 값들은 현재 시험 환경에 맞춰 사용자가 직접 조정한 값이므로 변경하지 않는다.
*/
const int DANGER_DISTANCE    = 10;   // 회피 시작
const int CLEAR_DISTANCE     = 12;  // 열린 공간 판단 기준

const int MIN_VALID_DISTANCE = 2;
const int MAX_VALID_DISTANCE = 200;

// 열린 공간 연속 확인 횟수
const int CLEAR_CONFIRM_COUNT = 2;

// 위험 거리 연속 확인 횟수
const int DANGER_CONFIRM_COUNT = 2;

// =======================================================
// 6. 초음파 측정값
// =======================================================

const unsigned long SONAR_INTERVAL_MS = 60;

/*
  12000us는 약 200cm 정도까지 측정.
  강의실 주행에서는 200cm 내외 timeout이면 충분하다.
*/
const unsigned long SONAR_TIMEOUT_US = 12000UL;

int distanceBuffer[3] = {80, 80, 80};
uint8_t distanceIndex = 0;
int filteredDistance = 80;
int lastValidDistance = 80;
bool sonarUpdated = false;

unsigned long lastSonarMs = 0;

// =======================================================
// 7. 상태 머신 시간값
// =======================================================

const unsigned long STOP_SETTLE_MS = 200;

/*
  벽에 붙었을 때 짧게 후진해서 회전 공간을 만든다.
*/
const unsigned long BACK_MS = 500;

/*
  한쪽 바퀴만 움직여 약 90도 회전하는 시간.
  실제 90도에 맞도록 주행 테스트하면서 이 값만 조정한다.
*/
const unsigned long PIVOT_TURN_MS = 700;

/*
  회피 성공 후 전진 보장 시간.
*/
const unsigned long ESCAPE_FORWARD_MS = 800;

// =======================================================
// 8. 상태 정의
// =======================================================

enum AutoState {
  ST_CRUISE = 0,

  ST_STOP_BEFORE_BACK,
  ST_BACK,
  ST_STOP_BEFORE_LEFT,

  ST_TURN_LEFT_SEARCH,
  ST_CHECK_LEFT,
  ST_RETURN_CENTER_FROM_LEFT,

  ST_ESCAPE_FORWARD,

  ST_TURN_RIGHT_SEARCH,
  ST_CHECK_RIGHT
};

AutoState state = ST_CRUISE;
unsigned long stateStartMs = 0;

int clearCount = 0;
int dangerCount = 0;

// 회피 방향 기록
// -1: 왼쪽 회피 성공
//  1: 오른쪽 회피 성공
int escapeDirection = -1;

// =======================================================
// 9. 함수 선언
// =======================================================

void enterState(AutoState nextState);

void updateSonarIfNeeded();
int readSonarCm();
bool isValidDistance(int cm);
void pushDistance(int cm);
int median3(int a, int b, int c);

void runAutonomous();
void handleCruise();
void handleStopBeforeBack();
void handleBack();
void handleStopBeforeLeft();
void handleTurnLeftSearch();
void handleCheckLeft();
void handleReturnCenterFromLeft();
void handleEscapeForward();
void handleTurnRightSearch();
void handleCheckRight();

void setMotor(int left, int right);
void setOneMotor(AF_DCMotor &motor, int signedSpeed, uint8_t invert);
int applyTrim(int speed, int trim);
void stopMotor();

const __FlashStringHelper *stateName(AutoState currentState);
void debugEvent(const __FlashStringHelper *message);
void debugPrint();

// =======================================================
// 10. setup
// =======================================================

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  stopMotor();

#if DEBUG_SERIAL
  Serial.println(F("RC autonomous start"));
  Serial.println(F("Motor: Left=M1, Right=M4"));
  Serial.println(F("Sonar: TRIG=A0, ECHO=A1"));
#endif

  stateStartMs = millis();
}

// =======================================================
// 11. loop
// =======================================================

void loop() {
  updateSonarIfNeeded();
  runAutonomous();
  debugPrint();
}

// =======================================================
// 12. 상태 진입 함수
// =======================================================

void enterState(AutoState nextState) {
  state = nextState;
  stateStartMs = millis();

  clearCount = 0;
  dangerCount = 0;

#if DEBUG_SERIAL
  Serial.print(F("상태 -> "));
  Serial.print((int)state);
  Serial.print(F(" "));
  Serial.println(stateName(state));

  switch (state) {
    case ST_STOP_BEFORE_BACK:
      debugEvent(F("장애물 감지!"));
      break;

    case ST_BACK:
      debugEvent(F("후진"));
      break;

    case ST_TURN_LEFT_SEARCH:
      debugEvent(F("왼쪽 회전..."));
      break;

    case ST_CHECK_LEFT:
      debugEvent(F("왼쪽 거리 확인"));
      break;

    case ST_RETURN_CENTER_FROM_LEFT:
      debugEvent(F("제자리로..."));
      break;

    case ST_TURN_RIGHT_SEARCH:
      debugEvent(F("오른쪽 회전..."));
      break;

    case ST_CHECK_RIGHT:
      debugEvent(F("오른쪽 거리 확인"));
      break;

    case ST_ESCAPE_FORWARD:
      debugEvent(F("길 발견! 전진"));
      break;

    case ST_CRUISE:
      debugEvent(F("일반 주행"));
      break;

    default:
      break;
  }
#endif
}

// =======================================================
// 13. 초음파 측정
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
    filteredDistance = median3(
      distanceBuffer[0],
      distanceBuffer[1],
      distanceBuffer[2]
    );
  } else {
    /*
      timeout 또는 비정상값이면 바로 0으로 처리하지 않고
      마지막 정상값을 사용해서 순간 튐을 줄인다.
    */
    pushDistance(lastValidDistance);
    filteredDistance = median3(
      distanceBuffer[0],
      distanceBuffer[1],
      distanceBuffer[2]
    );
  }
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

  int cm = (int)(duration / 58UL);
  return cm;
}

bool isValidDistance(int cm) {
  if (cm < MIN_VALID_DISTANCE) {
    return false;
  }

  if (cm > MAX_VALID_DISTANCE) {
    return false;
  }

  return true;
}

void pushDistance(int cm) {
  distanceBuffer[distanceIndex] = cm;
  distanceIndex++;

  if (distanceIndex >= 3) {
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
// 14. 자율주행 상태 머신
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

    case ST_STOP_BEFORE_LEFT:
      handleStopBeforeLeft();
      break;

    case ST_TURN_LEFT_SEARCH:
      handleTurnLeftSearch();
      break;

    case ST_CHECK_LEFT:
      handleCheckLeft();
      break;

    case ST_RETURN_CENTER_FROM_LEFT:
      handleReturnCenterFromLeft();
      break;

    case ST_ESCAPE_FORWARD:
      handleEscapeForward();
      break;

    case ST_TURN_RIGHT_SEARCH:
      handleTurnRightSearch();
      break;

    case ST_CHECK_RIGHT:
      handleCheckRight();
      break;

    default:
      stopMotor();
      enterState(ST_CRUISE);
      break;
  }
}

// =======================================================
// 15. 각 상태 처리
// =======================================================

void handleCruise() {
  int d = filteredDistance;

  if (d <= DANGER_DISTANCE) {
    dangerCount++;
  } else {
    dangerCount = 0;
  }

  if (dangerCount >= DANGER_CONFIRM_COUNT) {
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
    return;
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
    enterState(ST_STOP_BEFORE_LEFT);
  }
}

void handleStopBeforeLeft() {
  stopMotor();

  if (millis() - stateStartMs >= STOP_SETTLE_MS) {
    enterState(ST_TURN_LEFT_SEARCH);
  }
}

void handleTurnLeftSearch() {
  /*
    왼쪽 탐색:
    왼쪽 바퀴는 멈추고 오른쪽 바퀴만 전진시켜 좌회전한다.
  */
  setMotor(0, TURN_SPEED);

  unsigned long elapsed = millis() - stateStartMs;

  if (elapsed < PIVOT_TURN_MS) {
    return;
  }

  stopMotor();
  enterState(ST_CHECK_LEFT);
}

void handleCheckLeft() {
  stopMotor();

  if (millis() - stateStartMs < STOP_SETTLE_MS) {
    return;
  }

  if (filteredDistance >= CLEAR_DISTANCE) {
    clearCount = 1;
    escapeDirection = -1;
    debugEvent(F("왼쪽 길 발견"));
    enterState(ST_ESCAPE_FORWARD);
    return;
  }

  clearCount = 0;
  debugEvent(F("왼쪽 길이 없네요..."));
  enterState(ST_RETURN_CENTER_FROM_LEFT);
}

void handleReturnCenterFromLeft() {
  /*
    왼쪽 탐색 실패 후 오른쪽 바퀴만 후진시켜 원래 방향으로 돌아온다.
  */
  setMotor(0, -TURN_SPEED);

  if (millis() - stateStartMs >= PIVOT_TURN_MS) {
    stopMotor();
    enterState(ST_TURN_RIGHT_SEARCH);
  }
}

void handleEscapeForward() {
  /*
    회피 성공 후 일정 시간 전진을 보장한다.
  */
  if (filteredDistance <= DANGER_DISTANCE) {
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
    return;
  }

  setMotor(DRIVE_SPEED, DRIVE_SPEED);

  if (millis() - stateStartMs >= ESCAPE_FORWARD_MS) {
    enterState(ST_CRUISE);
  }
}

void handleTurnRightSearch() {
  /*
    오른쪽 탐색:
    오른쪽 바퀴는 멈추고 왼쪽 바퀴만 전진시켜 우회전한다.
  */
  setMotor(TURN_SPEED, 0);

  unsigned long elapsed = millis() - stateStartMs;

  if (elapsed < PIVOT_TURN_MS) {
    return;
  }

  stopMotor();
  enterState(ST_CHECK_RIGHT);
}

void handleCheckRight() {
  stopMotor();

  if (millis() - stateStartMs < STOP_SETTLE_MS) {
    return;
  }

  if (filteredDistance >= CLEAR_DISTANCE) {
    clearCount = 1;
    escapeDirection = 1;
    debugEvent(F("오른쪽 길 발견"));
    enterState(ST_ESCAPE_FORWARD);
    return;
  }

  clearCount = 0;
  debugEvent(F("오른쪽도 길이 없네요..."));
  enterState(ST_STOP_BEFORE_BACK);
}

// =======================================================
// 16. 모터 제어
// =======================================================

void setMotor(int left, int right) {
  left = constrain(left, MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  left = applyTrim(left, LEFT_TRIM);
  right = applyTrim(right, RIGHT_TRIM);

  left = constrain(left, MOTOR_MIN, MOTOR_MAX);
  right = constrain(right, MOTOR_MIN, MOTOR_MAX);

  setOneMotor(motorLeft, left, LEFT_MOTOR_INVERT);
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
  int pwm = abs(signedSpeed);
  pwm = constrain(pwm, 0, 255);

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

  if (forward) {
    motor.run(FORWARD);
  } else {
    motor.run(BACKWARD);
  }
}

void stopMotor() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);

  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}

// =======================================================
// 17. 디버그 출력
// =======================================================

const __FlashStringHelper *stateName(AutoState currentState) {
  switch (currentState) {
    case ST_CRUISE:
      return F("일반 주행");

    case ST_STOP_BEFORE_BACK:
      return F("후진 전 정지");

    case ST_BACK:
      return F("후진");

    case ST_STOP_BEFORE_LEFT:
      return F("왼쪽 회전 전 정지");

    case ST_TURN_LEFT_SEARCH:
      return F("왼쪽 회전");

    case ST_CHECK_LEFT:
      return F("왼쪽 거리 확인");

    case ST_RETURN_CENTER_FROM_LEFT:
      return F("제자리 복귀");

    case ST_ESCAPE_FORWARD:
      return F("회피 전진");

    case ST_TURN_RIGHT_SEARCH:
      return F("오른쪽 회전");

    case ST_CHECK_RIGHT:
      return F("오른쪽 거리 확인");

    default:
      return F("알 수 없음");
  }
}

void debugEvent(const __FlashStringHelper *message) {
#if DEBUG_SERIAL
  Serial.print(F("이벤트: "));
  Serial.print(message);
  Serial.print(F(" 거리="));
  Serial.println(filteredDistance);
#else
  (void)message;
#endif
}

void debugPrint() {
#if DEBUG_SERIAL
  static unsigned long lastDebugMs = 0;

  if (millis() - lastDebugMs < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastDebugMs = millis();

  Serial.print(F("상태="));
  Serial.print((int)state);

  Serial.print(F(" 거리="));
  Serial.print(filteredDistance);

  Serial.print(F(" 열린공간횟수="));
  Serial.print(clearCount);

  Serial.print(F(" 위험횟수="));
  Serial.println(dangerCount);
#endif
}
