// EN: We use special library to receive and decode commands from IR remote.
// RU: Подключаем специальную библиотеку, предоставляющую функции
//     приёма и передачи ИК-команд. Сайт проекта:
//     https://github.com/shirriff/Arduino-IRremote
#include "IRremote.h"

#include "ir_command_codes.h"

#include <Servo.h>

// EN: Analog pin where IR detector is pluged in.
// RU: Аналоговый вход контроллера, к которму подключен ИК-приёмник.
const int IR_PIN = A0;

// EN: Servo pins.
// RU: Цифровые выводы контролера, к которым подключены серводвигатели.
const int LEFT_SERVO_PIN = 2;
const int CENTRAL_SERVO_PIN = 4;
const int RIGHT_SERVO_PIN = 7;

// EN: Servo "zero" angle positions.
// RU: Центральное ("нулевое") положение серводвигателей в градусах.
const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

// EN: Amplitude of left and right servos.
// RU: Амплитула левого и правого серводвигателей.
const long SIDE_SERVOS_FULL_AMPLITUDE = 30;
// EN: Half amplitude of left and right servos. Is used when robot is turning
//     left or right while moving forward or backward.
// RU: Уменьшенная амплитула левого и правого серводвигателей. Используется
//     при поворотах совмещённых с движением вперёд или назад.
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;
// EN: Amplitude of central servo.
// RU: Амплитула центрального серводвигателя.
const long CENTRAL_SERVO_AMPLITUDE = 20;

// EN: Periods for different speeds.
// RU: Периоды колебаний для различных скоростей.
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

// EN: IRrecv class performs the decoding.
// RU: Создаём объект ИК-приёмник. Этот объект принимает и декодирует ИК-сигналы от пульта.
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

// EN: In some positions servos can make noise and vibrate.
//     To avoid this noise and vibration detach servos when robot is stopped.
// RU: В некоторых положениях серводвигатели могут вибрировать и шуметь.
//     Чтобы это избежать во время остановок робота, сервы надо отключать.
void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // EN: Start the IR receiver.
  // RU: Начинаем прослушивание ИК-сигналов.
  irrecv.enableIRIn();
 
  attachServos();
  isStopped = true;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

// EN: Gets angle for servo.
//     Param amplitude - amplitude of oscillating process,
//     param phaseMillis - current duration of oscillating,
//     param shiftAndle - phase of oscillating process.
// RU: Получение угла для серводвигателя.
//     Параметр amplitude - амплитуда колебаний,
//     Параметр phaseMillis - текущая продолжительность колебаний,
//     Параметр shiftAndle - фаза колебаний.
int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

template<typename T,size_t N>
boolean hasCode(T (&commandCodes)[N], long code) {
  for (int i = 0; i < N; i++) {
    if (commandCodes[i] == code) {
      return true;
    }
  }
  return false;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    // EN: We should wait for half a second. After that we think that servos are in zero
    //     position and we can detach them.
    // RU: Ждём полсекунды, чтобы серводвигатели вышли в нулевое положение и отключаем их.
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
  
  // EN: Declaration of the structure that is used for received and decoded IR commands.
  // RU: Описываем структуру results, в которую будут помещаться
  //     принятые и декодированные ИК-команды:
  decode_results results;
  
  // EN: We can handle IR command if it is received and decoded successfully.
  // RU: Если команда успешно принята и декодирована, то мы можем её обрабатывать.
  if (irrecv.decode(&results)) {
    
    if (hasCode(IR_COMMAND_FORWARD_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
    
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = PI/2;
        
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if(hasCode(IR_COMMAND_BACKWARD_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {

      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = -PI/2;

      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if (hasCode(IR_COMMAND_TURN_LEFT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = -PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_TURN_RIGHT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_STOP_CODES, results.value)) {
      attachServos();
      isStopped = true;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = 0;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_VERY_SLOW_CODES, results.value)) {
      // EN: globalPhase correction to save servo positions when changing period.
      // RU: Корректировка globalPhase чтобы сохранить положение серв при смене периода колебаний ног.
      globalPhase = globalPhase * STEP_PERIOD_VERY_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_SLOW;
    } else if (hasCode(IR_COMMAND_SLOW_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_SLOW;
    } else if (hasCode(IR_COMMAND_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_FAST;
    } else if (hasCode(IR_COMMAND_VERY_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_VERY_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_FAST;
    }
    // EN: Once a code has been decoded, the resume() method must be called to resume receiving codes.
    // RU: После декодирования кода для продолжения приёма должен вызаваться метод resume().
    irrecv.resume();
  }
  
  if (isAttached) {
    LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
    RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
    CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
  }
}

