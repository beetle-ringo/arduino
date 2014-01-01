// EN: IRremote library lets you send and receive IR remote codes
//     in multiple protocols.
// RU: Подключаем специальную библиотеку, предоставляющую функции
//     приёма и передачи ИК-команд:
#include "IRremote.h"
// https://github.com/shirriff/Arduino-IRremote

// EN: Pin to connect IR detector.
// RU: Аналоговый вход контроллера, к которму подключен ИК-приёмник.
const int IR_PIN = A0;

// EN: Create an instance of receiver.
// RU: Создаём объект ИК-приёмник.
IRrecv irrecv(IR_PIN);

void setup() {
  // EN: Initialize serial port for 9600 baud.
  Serial.begin(9600);
  Serial.println("ready");
  
  // EN: Start the IR receiver.
  // RU: Начинаем прослушивание ИК-сигналов.
  irrecv.enableIRIn();
}

void loop() {
  // EN: Declare structure for decoded commands.
  // RU: Описываем структуру results, в которую будут помещаться
  //     принятые и декодированные ИК-команды.
  decode_results results;
  
  // EN: If IR command is receicved and decoded then send the result to serial port.
  // RU: Если ИК-команда принята и успешно декодирована, то выводим
  //     полученный код в последовательный порт контроллера.
  if (irrecv.decode(&results)) {
    Serial.println(results.value);
    
    // EN: Must be called to resume decoding.
    // RU: Должно вызываться чтобы продолжить декодирование.
    irrecv.resume();
  }
}
