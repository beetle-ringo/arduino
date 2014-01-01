// EN: Most of Arduino controllers have an integrated LED that is connected to
//     digital pin 13. Let's make a constant LED to name this pin.
// RU: На большинстве Arduino-контроллеров есть встроенный светодиод, подключенный к
//     цифровому выводу 13 (пин 13). Дадим этому пину имя LED:
const int LED = 13;

// EN: Funtion setup() is called automatically each time the controller is turned on or reseted.
// RU: Функция setup() вызывается автоматически после включения или перезапуска контроллера.
void setup() {
  // EN: Initializing pin 13 for output.
  // RU: Инициализация цифрового пина для вывода.
  pinMode(LED, OUTPUT);     
}

// EN: Function loop() is called on and on in an endless loop.
// RU: Функция loop() вызывается автоматически снова и снова в бесконечном цикле.
void loop() {
  // EN: Set high level on pin 13 (turn the led on).
  // RU: Подать уровень логической единицы на пин 13 (зажечь светодиод).
  digitalWrite(LED, HIGH);
  // EN: Pauses the program for 1 second.
  // RU: Приостановить выполнение скетча на секунду.
  delay(1000);
  // EN: Set low level on pin 13 (turn the led off).
  // RU: Подать уровень логического нуля на пин 13 (потушить светодиод):
  digitalWrite(LED, LOW);
  // EN: Pauses the program for 1 second again.
  // RU: Снова приостановить выполнение скетча на секунду:
  delay(1000);
}
