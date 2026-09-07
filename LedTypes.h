#pragma once

// Вынесено в отдельный header специально, чтобы избежать бага
// автогенерации прототипов функций в Arduino IDE: она сканирует
// только .ino-файл и вставляет forward-declaration всех функций в
// самое начало файла — если сигнатура функции использует
// пользовательский enum, а вставка происходит раньше его объявления
// по тексту файла, компиляция падает с ошибкой вида
// "variable or field 'X' declared void" / "'LedState' was not
// declared in this scope". Типы, подключённые через #include, в эту
// автогенерацию не попадают — проблема просто не возникает.

enum LedState {
  LED_STATE_OFF,
  LED_STATE_WIFI_CONNECTING,  // зелёный/красный, чередуются
  LED_STATE_AP_CONFIG,        // синий, мигает
  LED_STATE_WAITING_PRINTER,  // красный, горит — принтер выключен/не подключен
  LED_STATE_IDLE_READY,       // зелёный, горит
  LED_STATE_PRINTING,         // зелёный, мигает
  LED_STATE_RESET_HELD,       // красный, мигает
  LED_STATE_PRINTER_ERROR     // жёлтый, мигает — ошибка принтера ИЛИ статус не определён
};
