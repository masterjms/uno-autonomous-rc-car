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

  setMotor(150, 150)을 실행했을 때:
  - 둘 다 전진하면 그대로 0
  - 왼쪽만 후진하면 LEFT_MOTOR_INVERT = 1
  - 오른쪽만 후진하면 RIGHT_MOTOR_INVERT = 1
*/
const uint8_t LEFT_MOTOR_INVERT  = 0;
const uint8_t RIGHT_MOTOR_INVERT = 0;

/*
  좌우 모터 출력 보정

  현재 테스트 결과:
  - 오른쪽 바퀴 힘이 약함
  - 직진 시 왼쪽으로 편향됨

  따라서 오른쪽 모터에 +18 보정값을 적용한다.
*/
const int LEFT_TRIM  = -10;
const int RIGHT_TRIM = 35;

// =======================================================
// 4. 속도 튜닝값
// =======================================================

/*
  이전 버전보다 전체 속도를 올림.

  이유:
  - 속도를 너무 낮추면 L293D + DC 모터 조합에서 기동 토크가 부족함
  - 오른쪽 바퀴가 제대로 돌지 않음
  - 직진 시 왼쪽으로 편향됨

  단, 초기에 너무 빠르게 벽에 박는 문제가 있었으므로
  원래 고속값까지는 올리지 않고 중간 수준으로 조정한다.
*/
const int HIGH_SPEED = 155;
const int MID_SPEED  = 135;
const int LOW_SPEED  = 115;

/*
  회전과 후진도 토크 부족을 막기 위해 이전보다 올림.
*/
const int TURN_SPEED = 130;
const int BACK_SPEED = 115;

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
const int EMERGENCY_DISTANCE = 2;   // 너무 가까움
const int DANGER_DISTANCE    = 5;   // 회피 시작
const int SLOW_DISTANCE      = 8;   // 감속 시작
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

const unsigned long STOP_SETTLE_MS = 100;

/*
  벽에 붙었을 때 짧게 후진해서 회전 공간을 만든다.
*/
const unsigned long BACK_MS = 500;

/*
  회전 시간이 너무 짧으면 실제로는 30도 정도만 회전할 수 있으므로
  회전 관련 시간을 길게 잡는다.
*/
const unsigned long TURN_MIN_MS = 350;
const unsigned long TURN_LEFT_MAX_MS = 950;
const unsigned long TURN_RIGHT_MIN_MS = 500;
const unsigned long TURN_RIGHT_MAX_MS = 1400;

/*
  열린 공간을 찾은 뒤 차체 폭 보정을 위해 추가 회전.
*/
const unsigned long TURN_MARGIN_MS = 150;

/*
  회피 성공 후 전진 보장 시간.
*/
const unsigned long ESCAPE_FORWARD_MS = 800;

/*
  긴급 탈출 루틴.
*/
const unsigned long FAIL_BACK_MS        = 850;
const unsigned long FAIL_TURN_MS        = 2600;
const unsigned long RECOVERY_FORWARD_MS = 600;

// =======================================================
// 8. 상태 정의
// =======================================================

enum AutoState {
  ST_CRUISE = 0,

  ST_STOP_BEFORE_BACK,
  ST_BACK,
  ST_STOP_BEFORE_LEFT,

  ST_TURN_LEFT_SEARCH,
  ST_LEFT_MARGIN,

  ST_ESCAPE_FORWARD,

  ST_TURN_RIGHT_SEARCH,
  ST_RIGHT_MARGIN,

  ST_FAIL_BACK,
  ST_FAIL_TURN,
  ST_RECOVERY_FORWARD
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
void handleLeftMargin();
void handleEscapeForward();
void handleTurnRightSearch();
void handleRightMargin();
void handleFailBack();
void handleFailTurn();
void handleRecoveryForward();

void setMotor(int left, int right);
void setOneMotor(AF_DCMotor &motor, int signedSpeed, uint8_t invert);
int applyTrim(int speed, int trim);
void stopMotor();

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
  Serial.print(F("STATE -> "));
  Serial.println((int)state);
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

    case ST_LEFT_MARGIN:
      handleLeftMargin();
      break;

    case ST_ESCAPE_FORWARD:
      handleEscapeForward();
      break;

    case ST_TURN_RIGHT_SEARCH:
      handleTurnRightSearch();
      break;

    case ST_RIGHT_MARGIN:
      handleRightMargin();
      break;

    case ST_FAIL_BACK:
      handleFailBack();
      break;

    case ST_FAIL_TURN:
      handleFailTurn();
      break;

    case ST_RECOVERY_FORWARD:
      handleRecoveryForward();
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

  /*
    긴급 거리 이하에서는 짧은 회피 루틴으로 가지 않고
    바로 긴급 탈출 루틴으로 보낸다.
  */
  if (d <= DANGER_DISTANCE) {
    stopMotor();
    enterState(ST_FAIL_BACK);
    return;
  }

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

  if (d <= SLOW_DISTANCE) {
    setMotor(LOW_SPEED, LOW_SPEED);
  } else if (d <= CLEAR_DISTANCE) {
    setMotor(MID_SPEED, MID_SPEED);
  } else {
    setMotor(HIGH_SPEED, HIGH_SPEED);
  }
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
    제자리 좌회전:
    왼쪽 바퀴 후진, 오른쪽 바퀴 전진
  */
  setMotor(-TURN_SPEED, TURN_SPEED);

  unsigned long elapsed = millis() - stateStartMs;

  /*
    최소 회전 시간 전까지는 거리값이 열려 보여도 무시한다.
  */
  if (elapsed < TURN_MIN_MS) {
    return;
  }

  if (filteredDistance >= CLEAR_DISTANCE) {
    clearCount++;
  } else {
    clearCount = 0;
  }

  if (clearCount >= CLEAR_CONFIRM_COUNT) {
    escapeDirection = -1;
    enterState(ST_LEFT_MARGIN);
    return;
  }

  if (elapsed >= TURN_LEFT_MAX_MS) {
    enterState(ST_TURN_RIGHT_SEARCH);
    return;
  }
}

void handleLeftMargin() {
  /*
    열린 공간을 찾은 뒤 조금 더 좌회전.
  */
  setMotor(-TURN_SPEED, TURN_SPEED);

  if (millis() - stateStartMs >= TURN_MARGIN_MS) {
    stopMotor();
    enterState(ST_ESCAPE_FORWARD);
  }
}

void handleEscapeForward() {
  /*
    회피 성공 후 일정 시간 전진을 보장한다.
  */
  if (filteredDistance <= DANGER_DISTANCE) {
    stopMotor();
    enterState(ST_FAIL_BACK);
    return;
  }

  setMotor(MID_SPEED, MID_SPEED);

  if (millis() - stateStartMs >= ESCAPE_FORWARD_MS) {
    enterState(ST_CRUISE);
  }
}

void handleTurnRightSearch() {
  /*
    오른쪽 탐색:
    왼쪽 탐색 실패 후 반대 방향으로 크게 돌아 열린 공간을 찾는다.
  */
  setMotor(TURN_SPEED, -TURN_SPEED);

  unsigned long elapsed = millis() - stateStartMs;

  if (elapsed < TURN_RIGHT_MIN_MS) {
    return;
  }

  if (filteredDistance >= CLEAR_DISTANCE) {
    clearCount++;
  } else {
    clearCount = 0;
  }

  if (clearCount >= CLEAR_CONFIRM_COUNT) {
    escapeDirection = 1;
    enterState(ST_RIGHT_MARGIN);
    return;
  }

  if (elapsed >= TURN_RIGHT_MAX_MS) {
    enterState(ST_FAIL_BACK);
    return;
  }
}

void handleRightMargin() {
  /*
    열린 공간을 찾은 뒤 조금 더 우회전.
  */
  setMotor(TURN_SPEED, -TURN_SPEED);

  if (millis() - stateStartMs >= TURN_MARGIN_MS) {
    stopMotor();
    enterState(ST_ESCAPE_FORWARD);
  }
}

void handleFailBack() {
  /*
    긴급 탈출 후진.
  */
  setMotor(-BACK_SPEED, -BACK_SPEED);

  if (millis() - stateStartMs >= FAIL_BACK_MS) {
    stopMotor();
    enterState(ST_FAIL_TURN);
  }
}

void handleFailTurn() {
  /*
    긴급 탈출용 큰 우회전.
  */
  setMotor(TURN_SPEED, -TURN_SPEED);

  if (millis() - stateStartMs >= FAIL_TURN_MS) {
    stopMotor();
    enterState(ST_RECOVERY_FORWARD);
  }
}

void handleRecoveryForward() {
  /*
    긴급 탈출 후 저속 전진.
  */
  if (filteredDistance <= DANGER_DISTANCE) {
    stopMotor();
    enterState(ST_FAIL_BACK);
    return;
  }

  setMotor(LOW_SPEED, LOW_SPEED);

  if (millis() - stateStartMs >= RECOVERY_FORWARD_MS) {
    enterState(ST_CRUISE);
  }
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

void debugPrint() {
#if DEBUG_SERIAL
  static unsigned long lastDebugMs = 0;

  if (millis() - lastDebugMs < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastDebugMs = millis();

  Serial.print(F("state="));
  Serial.print((int)state);

  Serial.print(F(" dist="));
  Serial.print(filteredDistance);

  Serial.print(F(" clear="));
  Serial.print(clearCount);

  Serial.print(F(" danger="));
  Serial.println(dangerCount);
#endif
}