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
  - 전방 초음파 1개만 사용
  - 엔코더 없음
  - 범퍼 스위치 없음
  - 정확한 각도 회전 대신, 열린 공간 탐색 기반 회피
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

// 사용자 정보 기준:
// 왼쪽 전방 구동 모터 = M1
// 오른쪽 전방 구동 모터 = M4
AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(4);

/*
  모터 방향 보정값

  setMotor(150, 150)을 했는데:
  - 둘 다 전진하면 그대로 0
  - 왼쪽만 후진하면 LEFT_MOTOR_INVERT = 1
  - 오른쪽만 후진하면 RIGHT_MOTOR_INVERT = 1
*/
const uint8_t LEFT_MOTOR_INVERT  = 0;
const uint8_t RIGHT_MOTOR_INVERT = 0;

/*
  좌우 모터 출력 보정

  전진할 때 차가 오른쪽으로 휘면:
  - 오른쪽 모터가 강하거나 왼쪽 모터가 약한 것
  - RIGHT_TRIM을 음수로 하거나 LEFT_TRIM을 양수로 조정

  전진할 때 차가 왼쪽으로 휘면:
  - LEFT_TRIM을 음수로 하거나 RIGHT_TRIM을 양수로 조정
*/
const int LEFT_TRIM  = 0;
const int RIGHT_TRIM = 0;

// =======================================================
// 4. 속도 튜닝값
// =======================================================

const int HIGH_SPEED = 170;
const int MID_SPEED  = 140;
const int LOW_SPEED  = 105;

const int TURN_SPEED = 140;
const int BACK_SPEED = 125;

// 모터 PWM 범위
const int MOTOR_MIN = -255;
const int MOTOR_MAX = 255;

// =======================================================
// 5. 거리 기준값
// =======================================================

/*
  차체:
  - 길이 약 22cm
  - 너비 약 18cm

  초음파는 차체 중앙 전방만 보기 때문에,
  차체 폭 여유를 고려해 CLEAR_DISTANCE를 넉넉히 잡는다.
*/
const int EMERGENCY_DISTANCE = 13;  // 너무 가까움
const int DANGER_DISTANCE    = 24;  // 회피 시작
const int SLOW_DISTANCE      = 40;  // 감속 시작
const int CLEAR_DISTANCE     = 55;  // 열린 공간 인정 기준

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
  400cm까지 기다리면 코드가 너무 오래 막힐 수 있으므로
  강의실 주행용으로는 200cm 내외 timeout이 더 적합.
*/
const unsigned long SONAR_TIMEOUT_US = 12000UL;

int distanceBuffer[3] = {80, 80, 80};
uint8_t distanceIndex = 0;
int filteredDistance = 80;
int lastValidDistance = 80;

unsigned long lastSonarMs = 0;

// =======================================================
// 7. 상태 머신 시간값
// =======================================================

const unsigned long STOP_SETTLE_MS      = 100;

/*
  차체 길이가 22cm이고 전방 센서만 있으므로,
  장애물 앞에서 바로 회전하지 말고 살짝 후진한다.
*/
const unsigned long BACK_MS             = 380;

/*
  회전 시작 후 이 시간 전까지는 열린 공간이 보여도 무시.
  너무 빨리 열린 공간으로 판단하면 코너 모서리에 걸릴 수 있다.
*/
const unsigned long TURN_MIN_MS         = 380;

/*
  왼쪽으로 이 시간 이상 돌았는데도 공간이 안 열리면
  왼쪽은 실패로 보고 오른쪽 탐색으로 넘어간다.
*/
const unsigned long TURN_LEFT_MAX_MS    = 1100;

/*
  왼쪽 탐색 실패 후 오른쪽을 보려면
  원래 정면을 지나 오른쪽까지 가야 하므로 더 길게 잡는다.
*/
const unsigned long TURN_RIGHT_MIN_MS   = 650;
const unsigned long TURN_RIGHT_MAX_MS   = 1800;

/*
  차체 폭 18cm 보정용.
  열린 공간을 찾은 뒤 바로 전진하지 말고
  같은 방향으로 조금 더 회전한다.
*/
const unsigned long TURN_MARGIN_MS      = 180;

/*
  회피 후 찔끔찔끔 재회피를 방지하기 위한 전진 보장 시간.
  단, EMERGENCY_DISTANCE 이하면 즉시 다시 회피.
*/
const unsigned long ESCAPE_FORWARD_MS   = 650;

/*
  양쪽 실패 시 탈출용.
*/
const unsigned long FAIL_BACK_MS        = 600;
const unsigned long FAIL_TURN_MS        = 1000;
const unsigned long RECOVERY_FORWARD_MS = 450;

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
      timeout 또는 비정상값이면 바로 0으로 처리하지 않는다.
      마지막 정상값을 유지해서 오동작을 줄인다.
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

  if (d <= EMERGENCY_DISTANCE) {
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
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
    최소 회전 시간 전까지는 거리값을 믿지 않는다.
    이유:
    - 회전 초기에 초음파가 장애물 모서리 틈을 보고
      열린 공간으로 착각할 수 있음
    - 차체 폭 18cm 때문에 바퀴가 아직 장애물에 걸릴 수 있음
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
    /*
      왼쪽이 계속 막혀 있으면 오른쪽 탐색.
      현재 왼쪽으로 돌아간 상태이므로,
      오른쪽 탐색은 더 오래 필요하다.
    */
    enterState(ST_TURN_RIGHT_SEARCH);
    return;
  }
}

void handleLeftMargin() {
  /*
    열린 공간을 찾은 뒤에도 조금 더 좌회전.
    차체 폭 18cm 보정용.
  */
  setMotor(-TURN_SPEED, TURN_SPEED);

  if (millis() - stateStartMs >= TURN_MARGIN_MS) {
    stopMotor();
    enterState(ST_ESCAPE_FORWARD);
  }
}

void handleEscapeForward() {
  /*
    회피 성공 후에는 바로 다시 회피하지 않고
    일정 시간 전진을 보장한다.

    단, 너무 가까우면 즉시 회피.
  */
  if (filteredDistance <= EMERGENCY_DISTANCE) {
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
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
    왼쪽 탐색 실패 후 시작하므로,
    현재 차체는 왼쪽으로 어느 정도 돌아간 상태다.

    오른쪽을 보려면
    왼쪽 방향 → 원래 정면 → 오른쪽 방향
    으로 더 오래 돌아야 한다.
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
    열린 공간을 찾은 뒤에도 조금 더 우회전.
    차체 폭 18cm 보정용.
  */
  setMotor(TURN_SPEED, -TURN_SPEED);

  if (millis() - stateStartMs >= TURN_MARGIN_MS) {
    stopMotor();
    enterState(ST_ESCAPE_FORWARD);
  }
}

void handleFailBack() {
  /*
    양쪽 탐색이 모두 실패한 경우:
    더 길게 후진해서 공간을 확보한다.
  */
  setMotor(-BACK_SPEED, -BACK_SPEED);

  if (millis() - stateStartMs >= FAIL_BACK_MS) {
    stopMotor();
    enterState(ST_FAIL_TURN);
  }
}

void handleFailTurn() {
  /*
    막다른 형태에서 빠져나오기 위한 큰 회전.
    기본은 오른쪽 큰 회전.
  */
  setMotor(TURN_SPEED, -TURN_SPEED);

  if (millis() - stateStartMs >= FAIL_TURN_MS) {
    stopMotor();
    enterState(ST_RECOVERY_FORWARD);
  }
}

void handleRecoveryForward() {
  /*
    탈출 후 바로 고속 주행하지 않고
    잠깐 중속 전진해서 안정화.
  */
  if (filteredDistance <= EMERGENCY_DISTANCE) {
    stopMotor();
    enterState(ST_STOP_BEFORE_BACK);
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