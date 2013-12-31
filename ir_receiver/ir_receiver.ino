// Подключаем специальную библиотеку, предоставляющую функции
// приёма и передачи ИК-команд:
#include "IRremote.h"
// https://github.com/shirriff/Arduino-IRremote

// Аналоговый вход контроллера, к которму подключен ИК-приёмник:
const int IR_PIN = A0;

// Создаём объект ИК-приёмник:
IRrecv irrecv(IR_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("ready");
  
  // Начинаем прослушивание ИК-сигналов:
  irrecv.enableIRIn();
}

void loop() {
  // Описываем структуру results, в которую будут помещаться
  // принятые и декодированные ИК-команды:
  decode_results results;
  
  // Если ИК-команда принята и успешно декодирована, то выводим
  // полученный код в последовательный порт контроллера:
  if (irrecv.decode(&results)) {
    Serial.println(results.value);
    irrecv.resume();
  }
}
