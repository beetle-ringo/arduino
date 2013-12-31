// Подключаем специальную библиотеку, предоставляющую функции
// приёма и передачи ИК-команд. Сайт проекта:
// https://github.com/shirriff/Arduino-IRremote
#include "IRremote.h"

#include <Servo.h>

const long COMMAND_FORWARD = 527175;
const long COMMAND_BACKWARD = 920391;
const long COMMAND_TURN_LEFT = 789319;
const long COMMAND_TURN_RIGHT = 658247;
const long COMMAND_FORWARD_LEFT = 2887;
const long COMMAND_FORWARD_RIGHT = 265031;
const long COMMAND_BACKWARD_LEFT = 396103;
const long COMMAND_BACKWARD_RIGHT = 68423;
const long COMMAND_STOP = 133959;
const long COMMAND_VERY_SLOW = 420679;
const long COMMAND_SLOW = 617287;
const long COMMAND_FAST = 92999;
const long COMMAND_VERY_FAST = 944967;

// Аналоговый вход контроллера, к которму подключен ИК-приёмник:
const int IR_PIN = A0;
const int LEFT_SERVO_PIN = 2;
const int CENTRAL_SERVO_PIN = 4;
const int RIGHT_SERVO_PIN = 7;

const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

const long SIDE_SERVOS_FULL_AMPLITUDE = 30;
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;
const long CENTRAL_SERVO_AMPLITUDE = 15;

const long STEP_PERIOD_VERY_SLOW = 2000;
const long STEP_PERIOD_SLOW = 1500;
const long STEP_PERIOD_FAST = 1000;
const long STEP_PERIOD_VERY_FAST = 500;

long lastMillis;
long globalPhase;
float angleShiftLeftServo;
float angleShiftRightServo;
float angleShiftCentralServo;
long stepPeriod;
long amplitudeLeftServo;
long amplitudeRightServo;
boolean isAttached;
boolean isStopped;

// Создаём объект ИК-приёмник:
IRrecv irrecv(IR_PIN);

Servo LeftServo;
Servo RightServo;
Servo CentralServo;

void attachServos() {
  if (!isAttached) {
    LeftServo.attach(LEFT_SERVO_PIN);
    RightServo.attach(RIGHT_SERVO_PIN);
    CentralServo.attach(CENTRAL_SERVO_PIN);
    isAttached = true;
  }
}

void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // Начинаем прослушивание ИК-сигналов:
  irrecv.enableIRIn();
 
  attachServos();
  isStopped = true;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    // Ждём полсекунды, чтобы серводвигатели вышли в нулевое положение и отключаем их:
    if (millisPassed >= 500) {
      lastMillis = 0;
      detachServos();
    }

    globalPhase = 0;
  } else {
    lastMillis = millisNow;
    
    globalPhase += millisPassed;
    globalPhase = globalPhase % stepPeriod;
  }
  
  // Описываем структуру results, в которую будут помещаться
  // принятые и декодированные ИК-команды:
  decode_results results;
  
  // Если ИК-команда принята и успешно декодирована, то выводим
  // полученный код в последовательный порт контроллера:
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case COMMAND_FORWARD:
      case COMMAND_FORWARD_LEFT:
      case COMMAND_FORWARD_RIGHT:
        attachServos();
        isStopped = false;
        angleShiftLeftServo = 0;
        angleShiftRightServo = 0;
        angleShiftCentralServo = PI/2;
        
        amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
        amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
        if (results.value == COMMAND_FORWARD_LEFT) amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
        else if (results.value == COMMAND_FORWARD_RIGHT) amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
        
        break;
      case COMMAND_BACKWARD:
      case COMMAND_BACKWARD_LEFT:
      case COMMAND_BACKWARD_RIGHT:
        attachServos();
        isStopped = false;
        angleShiftLeftServo = 0;
        angleShiftRightServo = 0;
        angleShiftCentralServo = -PI/2;

        amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
        amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
        if (results.value == COMMAND_BACKWARD_LEFT) amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
        else if (results.value == COMMAND_BACKWARD_RIGHT) amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
        break;
      case COMMAND_TURN_LEFT:
        attachServos();
        isStopped = false;
        angleShiftLeftServo = 0;
        angleShiftRightServo = PI;
        angleShiftCentralServo = -PI/2;
        amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
        amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
        break;
      case COMMAND_TURN_RIGHT:
        attachServos();
        isStopped = false;
        angleShiftLeftServo = 0;
        angleShiftRightServo = PI;
        angleShiftCentralServo = PI/2;
        amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
        amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
        break;
      case COMMAND_STOP:
        attachServos();
        isStopped = true;
        angleShiftLeftServo = 0;
        angleShiftRightServo = 0;
        angleShiftCentralServo = 0;
        amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
        amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
        break;
      case COMMAND_VERY_SLOW:
        stepPeriod = STEP_PERIOD_VERY_SLOW;
        break;
      case COMMAND_SLOW:
        stepPeriod = STEP_PERIOD_SLOW;
        break;
      case COMMAND_FAST:
        stepPeriod = STEP_PERIOD_FAST;
        break;
      case COMMAND_VERY_FAST:
        stepPeriod = STEP_PERIOD_VERY_FAST;
        break;
    }
    irrecv.resume();
  }
  
  if (isAttached) {
    LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
    RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
    CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
  }
}

