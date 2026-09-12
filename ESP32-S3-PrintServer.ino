/*
  ESP32-S3-PrintServer.ino
  ----------------------------------------------------------------
  WiFi -> USB Host мост для сетевого принтера (протестировано на HP-подобных
  USB-принтерах класса 0x07, обобщённое устройство "ESP32 Print Server") на плате ESP32-S3 N16R8 с двумя USB-C.

  Поддерживает:
    - RAW / JetDirect / AppSocket  -> TCP порт 9100
    - LPR / LPD (RFC 1179)         -> TCP порт 515
    - Автономная настройка WiFi через точку доступа, если не
      удалось подключиться к сохранённой сети (WiFiManager-style)
    - Сброс сохранённых WiFi-настроек удержанием кнопки (см. RESET_HOLD_MS)
    - Индикация состояния через адресный RGB-светодиод (WS2812)

  Библиотеки:
    EspUsbHost (tanakamasayuki)
      Arduino IDE -> Library Manager -> "EspUsbHost"
      https://github.com/tanakamasayuki/EspUsbHost
    Adafruit NeoPixel
      Arduino IDE -> Library Manager -> "Adafruit NeoPixel"

  ----------------------------------------------------------------
  ЛОГИКА ЗАПУСКА:
  1. Пытаемся подключиться к WiFi с сохранёнными в NVS (Preferences)
     данными. Таймаут WIFI_CONNECT_TIMEOUT_MS.
  2. Если данных нет или подключиться не удалось за это время —
     поднимаем точку доступа AP_SSID / AP_PASSWORD и HTTP-страницу
     конфигурации на 192.168.4.1. Вводите SSID/пароль вашей сети,
     жмём "Сохранить" — плата сохранит их в NVS и перезагрузится.
  3. Кнопка на пине RESET_BUTTON_PIN (замкнута на GND) — если
     удерживать 30 секунд, сохранённые WiFi-данные стираются и
     плата уходит в режим точки доступа заново.

  ----------------------------------------------------------------
  ИНДИКАЦИЯ RGB-СВЕТОДИОДОМ:
    Жёлтый, мигает   — идёт подключение к сохранённой WiFi-сети
    Синий, мигает    — режим точки доступа, ждём ввода WiFi-настроек
    Жёлтый, горит    — WiFi подключен, ждём подключения принтера
    Зелёный, горит   — всё готово, принтер на связи, простой
    Зелёный, мигает  — идёт приём/печать задания
    Красный, мигает  — удерживается кнопка сброса конфигурации

  ВАЖНО: RGB_LED_PIN ниже — это ПРЕДПОЛОЖЕНИЕ (GPIO48, распространённый
  пин встроенного WS2812 на платах ESP32-S3-DevKitC-1). На вашей
  конкретной плате (Lonely Binary N16R8) он может отличаться —
  сверьтесь со схемой/маркировкой платы и поправьте константу.
  ----------------------------------------------------------------

  Настройки платы в Arduino IDE (Tools):
    Board            : ESP32S3 Dev Module
    USB CDC On Boot  : Disabled
    USB Mode         : USB-OTG (TinyUSB)   <-- обязательно!
    Upload Mode      : UART0 / Hardware CDC
    PSRAM            : OPI PSRAM
    Flash Size       : 16MB
    Partition Scheme : 16M Flash (3MB APP/9.9MB FATFS) или аналог

  ПЕРВЫЙ ЗАПУСК — РЕЖИМ USB-ДИАГНОСТИКИ:
  Если не уверены в номере интерфейса принтера, включите
  SCAN_ONLY_MODE = true, подключите принтер, смотрите Serial
  Monitor — printAllDeviceInfo() покажет все интерфейсы и их
  классы. Найдите bInterfaceClass = 0x07, впишите номер в
  PRINTER_INTERFACE_NUMBER, верните SCAN_ONLY_MODE = false.
  ----------------------------------------------------------------
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_eap_client.h" // WPA2-Enterprise (PEAP/TTLS+MSCHAPv2) — актуальный заголовок для IDF 5.1+;
                            // старый esp_wpa2.h убран из SDK и даёт ошибку компиляции "Use esp_eap_client.h instead"

// ==================== OLED-ДИСПЛЕЙ (опционально) ====================
// Поставьте 0, если дисплей физически не подключён — тогда весь связанный
// код и обе библиотеки ниже не понадобятся вообще (можно даже не
// устанавливать Adafruit_SSD1306/Adafruit_GFX через Library Manager).
#define ENABLE_OLED_DISPLAY 1

#if ENABLE_OLED_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Выбор начертания шрифта на OLED: 1 — жирный, 0 — обычный. Оба файла
// шрифтов из одного комплекта Adafruit_GFX, просто разной насыщенности.
#define OLED_FONT_BOLD 0

#if OLED_FONT_BOLD
#include <Fonts/FreeSansBold9pt7b.h>
#define OLED_FONT FreeSansBold9pt7b
#else
#include <Fonts/FreeSans9pt7b.h>
#define OLED_FONT FreeSans9pt7b
#endif
#endif

#include "EspUsbHost.h"
#include "LedTypes.h"

// ==================== НАСТРОЙКИ ====================

const char *MDNS_HOSTNAME = "ESP32-PrintServer"; // доступ по ESP32-PrintServer.local

const uint16_t RAW_PORT   = 9100;         // JetDirect/AppSocket
const uint16_t LPR_PORT   = 515;          // LPD (RFC 1179)

// Номер USB-интерфейса принтера (класс 0x07). Обычно 0.
// Если не уверены — включите SCAN_ONLY_MODE и проверьте по Serial.
const uint8_t PRINTER_INTERFACE_NUMBER = 0;
const bool SCAN_ONLY_MODE = false;

const size_t CHUNK_SIZE = 4096;
const uint32_t IO_TIMEOUT_MS = 10000;

// ---- WiFi fallback / автонастройка ----
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000; // 30с на подключение
const char *AP_SSID     = "ESP32-PrintServer";
const char *AP_PASSWORD = "12345678";
const IPAddress AP_IP(192, 168, 4, 1);

// ---- Кнопка сброса конфигурации ----
// Пин замыкается на GND при нажатии (используется внутренний pull-up).
// GPIO0 — часто это штатная кнопка BOOT на платах ESP32-S3, но
// проверьте распиновку своей конкретной платы перед использованием.
const int RESET_BUTTON_PIN = 0;
const uint32_t RESET_HOLD_MS = 10000; // держать 10с для сброса

// ---- SSDP / UPnP — обнаружение в сети + постоянный веб-портал ----
const uint16_t SSDP_PORT = 1900;
const IPAddress SSDP_MULTICAST_IP(239, 255, 255, 250);
const uint16_t HTTP_PORT = 80;
const char *UPNP_DESCRIPTION_PATH = "/upnp/description.xml";
const uint32_t SSDP_REANNOUNCE_INTERVAL_MS = 5UL * 60UL * 1000UL; // повторный анонс раз в 5 минут

const int RGB_LED_PIN   = 48;
const int RGB_LED_COUNT = 1;
// RGB_BRIGHTNESS не объявляем — на платах ESP32-S3 с готовой поддержкой
// встроенного адресного светодиода эта константа уже определена в
// pins_arduino.h конкретного board-варианта (обычно 64). Объявление
// своей же константы с тем же именем даёт ошибку переопределения —
// используем готовую из ядра.

#if ENABLE_OLED_DISPLAY
// 0.91" I2C OLED на SSD1306, разрешение 128x32 — самый распространённый
// вариант именно у дисплеев этого размера (0.96" обычно уже 128x64).
// Пины SDA/SCL — под конкретную плату может понадобиться поменять,
// GPIO8/9 просто свободны и не пересекаются с RESET_BUTTON_PIN (0) и
// RGB_LED_PIN (48) из этого проекта.
const int OLED_SDA_PIN = 8;
const int OLED_SCL_PIN = 9;
const int OLED_SCREEN_WIDTH = 128;
const int OLED_SCREEN_HEIGHT = 32;
const int OLED_RESET_PIN = -1;      // большинство модулей 0.91" не имеют отдельного пина сброса
const uint8_t OLED_I2C_ADDRESS = 0x3C; // самый частый адрес; на некоторых модулях — 0x3D
#endif

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================

EspUsbHost usb;
WiFiServer rawServer(RAW_PORT);
WiFiServer lprServer(LPR_PORT);
WebServer configServer(80); // теперь работает ПОСТОЯННО — и AP-настройка, и статус-портал
Preferences prefs;
Adafruit_NeoPixel pixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

#if ENABLE_OLED_DISPLAY
Adafruit_SSD1306 oled(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
bool oledReady = false;

// Защита от выгорания: статичный текст (IP-адрес) на одном и том же
// месте экрана долгие часы/дни способен "выжечь" пиксели OLED-матрицы.
// Через OLED_BURN_IN_PROTECT_MS после включения экран гасится и больше
// не перерисовывается до следующей перезагрузки платы. Гасить именно
// через clearDisplay()/display() достаточно — в OLED, в отличие от LCD
// с подсветкой, чёрный пиксель физически не потребляет ток и не
// "горит", так что отдельная команда аппаратного отключения матрицы
// не нужна.
const unsigned long OLED_BURN_IN_PROTECT_MS = 30UL * 60UL * 1000UL; // 30 минут
unsigned long oledOnSinceMs = 0;
bool oledBurnInProtected = false;
#endif

// ---- SSDP / UPnP ----
WiFiUDP ssdpUdp;
String deviceUuid;              // стабильный uuid, хранится в NVS — используется как UPnP UDN
uint32_t lastSsdpAnnounceMs = 0;
char ssdpRecvBuf[600];
bool portalStarted = false;     // чтобы не регистрировать маршруты повторно при реконнектах

volatile bool printerReady = false;
volatile uint8_t printerAddress = 0;
bool printerHasIssue = false;       // выставляется по результату PJL-опроса статуса (включая "статус не определён")
bool printerIsBusyPrinting = false; // принтер сейчас физически печатает (по PJL DISPLAY)
uint32_t lastKnownWriteErrors = 0;  // базовая линия для usb.vendorWriteStats().errors — см. sendToPrinter()
String lastPrinterStatusRaw = "";   // сырой текст последнего ответа @PJL INFO STATUS
long lastLoggedPjlCode = -999;      // сентинел "ещё не логировали" — гарантирует лог первого опроса после подключения

// Настоящие строковые дескрипторы USB устройства (не PJL, не Device ID —
// это то же самое, что читает демо-скетч EspUsbHostDeviceInfo.ino:
// стандартные iManufacturer/iProduct/iSerialNumber, которые библиотека
// и так получает при обычной перечислении устройства). Заполняются
// сразу при подключении, без сетевых round-trip'ов.
String printerManufacturer = "";
String printerProduct = "";
String printerSerial = "";

uint8_t chunkBuffer[CHUNK_SIZE];

String savedSsid;
String savedPassword;
bool savedIsEnterprise = false; // true — WPA2-Enterprise (PEAP/TTLS + MSCHAPv2), иначе обычный WPA2-Personal
String savedEapIdentity;        // внешняя identity (часто совпадает с username, но не всегда)
String savedEapUsername;        // внутренний username для MSCHAPv2

uint32_t resetButtonPressStart = 0;
uint32_t resetButtonLastReport = 0;
bool resetButtonWasDown = false;
bool resetHeldOverrideLed = false; // true, пока кнопка сброса зажата — красный мигает поверх всего

// ==================== RGB-ИНДИКАЦИЯ ====================

volatile LedState currentLedState = LED_STATE_OFF;

// Параметр здесь намеренно int, а не LedState: Arduino IDE автоматически
// генерирует forward-declaration для каждой функции и вставляет его в
// самое начало файла — ДО того как компилятор дойдёт до объявления
// пользовательских типов (даже до #include, если они идут после
// начального блока комментариев). Если в СИГНАТУРЕ функции стоит
// пользовательский enum, это ломает компиляцию с ошибкой вида
// "'LedState' was not declared in this scope". Внутри тела функции и
// как тип обычных переменных LedState использовать можно свободно —
// проблема именно с параметрами/возвращаемым типом функций.
void setLedState(int s) {
  currentLedState = (LedState)s;
}

// Вызывать часто (каждую итерацию любого цикла, включая блокирующие
// while-циклы ожидания WiFi/AP-портала/печати), чтобы анимация мигания
// была плавной независимо от того, где сейчас выполнение.
void updateLed() {
  LedState effective = resetHeldOverrideLed ? LED_STATE_RESET_HELD : currentLedState;

  uint32_t color = 0;
  uint32_t altColor = 0; // если не 0 — мигание идёт между color и altColor, а не color/выкл
  bool blink = false;
  uint32_t periodMs = 300;

  switch (effective) {
    case LED_STATE_WIFI_CONNECTING:
      color = pixel.Color(0, 255, 0);     // зелёный
      altColor = pixel.Color(255, 0, 0);  // ...чередуется с красным
      blink = true; periodMs = 300;
      break;
    case LED_STATE_AP_CONFIG:
      color = pixel.Color(0, 80, 255);    // синий
      blink = true; periodMs = 400;
      break;
    case LED_STATE_WAITING_PRINTER:
      color = pixel.Color(255, 0, 0);     // красный, горит — принтер выключен/не подключен
      blink = false;
      break;
    case LED_STATE_IDLE_READY:
      color = pixel.Color(0, 255, 0);     // зелёный
      blink = false;
      break;
    case LED_STATE_PRINTING:
      color = pixel.Color(0, 255, 0);     // зелёный
      blink = true; periodMs = 150;       // мигает быстрее — видно активность
      break;
    case LED_STATE_RESET_HELD:
      color = pixel.Color(255, 0, 0);     // красный
      blink = true; periodMs = 200;
      break;
    case LED_STATE_PRINTER_ERROR:
      color = pixel.Color(255, 200, 0);   // жёлтый — ошибка ИЛИ статус не определён
      blink = true; periodMs = 350;
      break;
    case LED_STATE_OFF:
    default:
      color = 0;
      blink = false;
      break;
  }

  bool on = true;
  if (blink) {
    on = (millis() % (periodMs * 2)) < periodMs;
  }

  uint32_t finalColor;
  if (blink && altColor != 0) {
    finalColor = on ? color : altColor; // чередование двух цветов, никогда не гаснет
  } else {
    finalColor = on ? color : 0;
  }

  pixel.setPixelColor(0, finalColor);
  pixel.show();
}

#if ENABLE_OLED_DISPLAY
// Инициализация дисплея — вызывается один раз из setup(). Если дисплей
// физически не подключён (или не тот I2C-адрес), begin() вернёт false —
// в этом случае просто оставляем oledReady=false и дальше по коду
// молча пропускаем все попытки что-то на него вывести, без зависаний.
void initOled() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  if (!oledReady) {
    Serial.println("[OLED] Дисплей не найден на шине I2C — пропускаем (проверьте подключение/адрес).");
    return;
  }
  // ВАЖНО: без явного setTextColor() ничего не будет видно на экране,
  // даже если begin() отработал успешно и весь остальной код рисования
  // верен — Adafruit_GFX по умолчанию не гарантирует, что цвет текста
  // отличается от цвета фона (в некоторых версиях библиотеки оба
  // изначально совпадают), и текст рисуется буквально тем же цветом,
  // что и фон, то есть невидимым. Для монохромного SSD1306 есть только
  // один "цвет" — SSD1306_WHITE (включенный пиксель).
  oled.setTextColor(SSD1306_WHITE);
  // На всякий случай явно снимаем "приглушённый" режим — публичный API
  // библиотеки не даёт выставить контраст произвольным числом (нужная
  // команда 0x81 спрятана как защищённый метод), но dim(false)
  // принудительно возвращает контраст к тому "полному" значению,
  // которое сама же begin() подобрала для текущего режима питания.
  oled.dim(false);
  // Переключаемся на пропорциональный шрифт из комплекта библиотеки —
  // встроенный "классический" шрифт слишком грубый (только целые
  // множители x1/x2/x3... одной и той же сетки 6px/символ), из-за чего
  // для длинных IP-адресов x2 просто не влезает по ширине, а x1
  // выглядит очень мелко. У этого шрифта цифры уже, и высота не
  // привязана к шагам по 8px — getTextBounds() ниже сам всё измерит
  // уже для него.
  oled.setFont(&OLED_FONT);
  oled.clearDisplay();
  oled.display();
  Serial.println("[OLED] Дисплей найден и готов.");
  oledOnSinceMs = millis();
  showOledSplash();
}

// Заставка на старте — показывается сразу после успешной инициализации,
// до того как появится реальный IP-адрес (WiFi ещё только подключается,
// либо плата поднимает собственную точку доступа). Использует ту же
// логику единой строки, что и updateOledIp().
void showOledSplash() {
  if (!oledReady) return;
  oled.clearDisplay();
  drawOledIpLineAutoSize("PrintServer");
  oled.display();
}

// Отображает текущий IP-адрес, подобрав максимально крупный размер
// шрифта, который ещё помещается по ширине экрана — короткие адреса
// (например "10.0.0.5") показываются одной строкой покрупнее, длинные
// разбиваются на две строки, чтобы использовать всю высоту 32 пикселя.
// Стандартный встроенный шрифт Adafruit_GFX — 6x8 пикселей на символ
// при размере 1 (это фиксированная метрика самой библиотеки).
void drawOledIpLine(const String &text, int size, int y) {
  oled.setTextSize(size);
  oled.setTextWrap(false); // иначе getTextBounds()/print() сами перенесут строку, если она не влезает
  int16_t x1, y1;
  uint16_t w, h;
  oled.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
  int x = (OLED_SCREEN_WIDTH - (int)w) / 2 - x1;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

// Общая логика для сплэша и для показа IP: строго одна строка,
// подбираем наибольший размер шрифта, который одновременно проходит
// и по ширине экрана (строка не вылезет за края), и по высоте (символы
// не обрежутся сверху/снизу) — итоговый размер это минимум из двух
// ограничений. Реальные ширину/высоту строки при каждом размере узнаём
// через getTextBounds() самой библиотеки, а не считаем вручную —
// точнее и не завязано на то, какой именно шрифт сейчас активен. Сама
// отрисовка на экран (clearDisplay/display) — на совести вызывающего
// кода, эта функция только считает размер и рисует текст в буфер.
void drawOledIpLineAutoSize(const String &text) {
  const int marginPx = 0; // отступ по краям — 0, у видимой области дисплея и так есть свой запас
  const int maxWidthPx = OLED_SCREEN_WIDTH - marginPx;
  const int maxHeightPx = OLED_SCREEN_HEIGHT - marginPx;

  oled.setTextWrap(false); // иначе getTextBounds() для не влезающей по ширине
                           // строки вернёт уже "перенесённые" (заниженные) размеры
  int16_t x1, y1;
  uint16_t w, h;

  int sizeByWidth = 1;
  for (int s = 2; s <= 10; s++) {
    oled.setTextSize(s);
    oled.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
    if ((int)w > maxWidthPx) break;
    sizeByWidth = s;
  }
  int sizeByHeight = 1;
  for (int s = 2; s <= 10; s++) {
    oled.setTextSize(s);
    oled.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
    if ((int)h > maxHeightPx) break;
    sizeByHeight = s;
  }
  int size = min(sizeByWidth, sizeByHeight);
  if (size < 1) size = 1;

  oled.setTextSize(size);
  oled.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
  int y = (OLED_SCREEN_HEIGHT - (int)h) / 2 - y1;
  drawOledIpLine(text, size, y);
}

void updateOledIp(const String &ip) {
  if (!oledReady) return;
  if (oledBurnInProtected) return; // экран уже погашен насовсем до перезагрузки

  if (millis() - oledOnSinceMs >= OLED_BURN_IN_PROTECT_MS) {
    oledBurnInProtected = true;
    oled.clearDisplay();
    oled.display();
    Serial.println("[OLED] Прошло 30 минут — гасим экран (защита от выгорания).");
    return;
  }

  // Перерисовываем, только когда IP реально изменился — не дёргаем
  // шину I2C впустую на каждой итерации loop().
  static String lastShown = "\x01"; // заведомо не равно ни одному реальному IP
  if (ip == lastShown) return;
  lastShown = ip;

  oled.clearDisplay();
  drawOledIpLineAutoSize(ip);
  oled.display();
}
#endif

// Пересчитывает "фоновое" состояние индикации по факту WiFi/принтера.
// Используется после подключения WiFi, после (от)ключения принтера,
// после завершения печати — то есть везде, где нет более приоритетного
// временного состояния (AP-портал, подключение к WiFi, печать, ресет).
void recomputeIdleLedState() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!printerReady) {
      setLedState(LED_STATE_WAITING_PRINTER);
    } else if (printerHasIssue) {
      setLedState(LED_STATE_PRINTER_ERROR);
    } else {
      setLedState(LED_STATE_IDLE_READY);
    }
  } else {
    setLedState(LED_STATE_WIFI_CONNECTING);
  }
}

// ==================== NVS: ХРАНЕНИЕ WiFi-НАСТРОЕК ====================

bool loadWifiCredentials() {
  prefs.begin("wifi", true); // read-only
  savedSsid = prefs.getString("ssid", "");
  savedPassword = prefs.getString("pass", "");
  savedIsEnterprise = prefs.getBool("ent", false);
  savedEapIdentity = prefs.getString("eapId", "");
  savedEapUsername = prefs.getString("eapUser", "");
  prefs.end();
  return savedSsid.length() > 0;
}

// enterprise=true — WPA2-Enterprise (PEAP/TTLS+MSCHAPv2): identity — внешняя
// identity (если оставить пустой, используется значение username),
// username — логин для MSCHAPv2, pass — соответствующий пароль.
// Для обычной домашней сети (enterprise=false) identity/username не нужны.
void saveWifiCredentials(const String &ssid, const String &pass, bool enterprise,
                          const String &identity, const String &username) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putBool("ent", enterprise);
  prefs.putString("eapId", identity);
  prefs.putString("eapUser", username);
  prefs.end();
  savedIsEnterprise = enterprise;
  savedEapIdentity = identity;
  savedEapUsername = username;
  Serial.printf("[NVS] Сохранены новые WiFi-настройки: SSID=\"%s\"%s\n",
                ssid.c_str(), enterprise ? " (WPA2-Enterprise)" : "");
}

void clearWifiCredentials() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  Serial.println("[NVS] WiFi-настройки очищены.");

  // Сброс кнопкой (или через веб-портал) — это, по сути, "аварийный
  // выход" на случай, если что-то настроено неправильно и нормальным
  // путём (зайдя на портал) уже не попасть. Пароль портала в этой
  // ситуации тоже должен слетать, иначе человек физически сбросит
  // WiFi, но всё равно не сможет зайти на портал заново для повторной
  // настройки, если забыл пароль портала.
  savePortalPassword("");
}

// ==================== NVS: ХРАНЕНИЕ ЯЗЫКА ИНТЕРФЕЙСА ====================

String uiLanguage = "ru"; // по умолчанию русский, пока не прочитали NVS / не сохранили другой

void loadUiLanguage() {
  prefs.begin("ui", true); // read-only
  uiLanguage = prefs.getString("lang", "ru");
  prefs.end();
  if (uiLanguage != "ru" && uiLanguage != "en") uiLanguage = "ru"; // защита от мусора в NVS
}

void saveUiLanguage(const String &lang) {
  uiLanguage = (lang == "en") ? "en" : "ru";
  prefs.begin("ui", false);
  prefs.putString("lang", uiLanguage);
  prefs.end();
  Serial.printf("[NVS] Язык интерфейса сохранён: %s\n", uiLanguage.c_str());
}

// ==================== NVS: ПАРОЛЬ ПОРТАЛА ====================
// Логин зафиксирован как "admin" (HTTP Basic Auth всё равно требует пару
// логин+пароль, а отдельное поле для логина только усложнило бы форму
// без реальной пользы для одного устройства). Пустой пароль = защита
// портала выключена — это сохраняет обратную совместимость с уже
// работающими прошивками, где портал открыт для всех в сети.

const char *PORTAL_AUTH_USER = "admin";
String portalPassword = ""; // пусто = аутентификация не требуется

void loadPortalPassword() {
  prefs.begin("portal", true); // read-only
  portalPassword = prefs.getString("pass", "");
  prefs.end();
}

void savePortalPassword(const String &pass) {
  portalPassword = pass;
  prefs.begin("portal", false);
  prefs.putString("pass", pass);
  prefs.end();
  Serial.println(pass.length() > 0 ? "[NVS] Пароль портала установлен."
                                    : "[NVS] Пароль портала снят.");
}

// Вызывать первой строкой в каждом обработчике, который должен быть
// защищён. Возвращает false и сама отправляет HTTP 401 с заголовком
// WWW-Authenticate, если аутентификация не пройдена — в этом случае
// обработчик обязан немедленно завершиться (return), ничего больше не
// отправляя. Если пароль портала не задан — пропускает всех без проверки.
bool requirePortalAuth() {
  if (portalPassword.length() == 0) return true;
  if (configServer.authenticate(PORTAL_AUTH_USER, portalPassword.c_str())) return true;
  configServer.requestAuthentication();
  return false;
}

// ==================== КНОПКА СБРОСА ====================

void setupResetButton() {
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
}

// Вызывать регулярно из любого loop() / блокирующего цикла.
// При удержании RESET_HOLD_MS -> стирает конфиг и перезагружает плату.
// Также обновляет RGB-индикацию (красный мигает, пока кнопка зажата).
void checkResetButton() {
  bool down = (digitalRead(RESET_BUTTON_PIN) == LOW);

  if (down) {
    if (!resetButtonWasDown) {
      resetButtonWasDown = true;
      resetButtonPressStart = millis();
      resetButtonLastReport = resetButtonPressStart; // отсчёт от момента нажатия
      resetHeldOverrideLed = true;
      Serial.printf("[RESET] Кнопка нажата, удерживайте %lu с для сброса WiFi-настроек...\n",
                    (unsigned long)RESET_HOLD_MS / 1000);
    } else {
      uint32_t held = millis() - resetButtonPressStart;
      if (millis() - resetButtonLastReport > 5000) {
        resetButtonLastReport = millis();
        Serial.printf("[RESET] Удержание: %lu / %lu с\n",
                      (unsigned long)held / 1000, (unsigned long)RESET_HOLD_MS / 1000);
      }
      if (held >= RESET_HOLD_MS) {
        Serial.printf("[RESET] %lu с удержания — сброс WiFi-настроек и перезагрузка!\n",
                      (unsigned long)RESET_HOLD_MS / 1000);
        clearWifiCredentials();
        delay(200);
        ESP.restart();
      }
    }
  } else {
    if (resetButtonWasDown) {
      Serial.println("[RESET] Кнопка отпущена, сброс отменён.");
    }
    resetButtonWasDown = false;
    resetHeldOverrideLed = false;
  }

  updateLed();
}

// ==================== HTTP-ПОРТАЛ НАСТРОЙКИ WiFi (режим AP) ====================

// Страница первоначальной настройки WiFi (режим AP) — в том же тёмном
// карточном стиле, что и основной дашборд, с той же кнопкой
// переключения языка (сохраняется в NVS через /set-lang так же, как
// и везде — язык общий для всего портала, не только для дашборда).
String buildWifiSetupHtml() {
  bool en = (uiLanguage == "en");
  String title      = en ? "WiFi Setup"                              : "Настройка WiFi";
  String heading    = en ? "WiFi setup for the print server"         : "Настройка WiFi для принт-сервера";
  String scanBtn    = en ? "Scan networks"                           : "Сканировать сети";
  String scanning   = en ? "Scanning..."                             : "Сканирование...";
  String chooseNet  = en ? "Choose a network"                        : "Выберите сеть";
  String chooseHint = en ? "Press \u201cScan networks\u201d to see what's available."
                          : "Нажмите «Сканировать сети», чтобы увидеть доступные сети.";
  String manualLabel = en ? "Or enter manually (SSID)"                : "Или введите вручную (SSID)";
  String manualHint  = en ? "Fill in if the network is hidden or wasn't found by the scan."
                           : "Заполните, если сеть скрыта или не найдена при сканировании.";
  String netTypeLabel = en ? "Network type"                          : "Тип сети";
  String personalOpt  = en ? "Regular (password or open)"            : "Обычная (пароль или открытая)";
  String enterpriseOpt = en ? "WPA2-Enterprise (802.1X)"              : "WPA2-Enterprise (802.1X)";
  String passLabel  = en ? "Network password"                        : "Пароль сети";
  String passHint   = en ? "Leave blank to keep the currently saved password. For an open network with no "
                           "password, check the box below."
                         : "Оставьте пустым, чтобы сохранить текущий сохранённый пароль. Для открытой сети "
                           "без пароля отметьте галочку ниже.";
  String openLabel  = en ? "This is an open network (no password)"   : "Это открытая сеть (без пароля)";
  String saveBtn    = en ? "Save and connect"                        : "Сохранить и подключиться";
  String macLabel   = en ? "MAC address"                             : "MAC-адрес";
  String nextLang      = en ? "ru" : "en";
  String nextLangLabel = en ? "\u0420\u0443\u0441\u0441\u043a\u0438\u0439" : "English";
  String entLabel   = en ? "This is a WPA2-Enterprise network (802.1X, e.g. eduroam / corporate WiFi)"
                          : "Это сеть WPA2-Enterprise (802.1X, например eduroam / корпоративный WiFi)";
  String identityLabel = en ? "Identity" : "Identity";
  String identityHint  = en ? "Usually matches your username. Format depends on your organization (e.g. "
                              "user@domain or DOMAIN\\user). Blank = keep the current value."
                            : "Обычно совпадает с именем пользователя. Формат зависит от организации "
                              "(например, user@domain или DOMAIN\\user). Пустое поле = оставить текущее значение.";
  String usernameLabel = en ? "Username" : "Имя пользователя";
  String entPassLabel  = en ? "Password" : "Пароль";
  String entPassHint   = en ? "Blank = keep the currently saved password. PEAP/MSCHAPv2 without a CA "
                              "certificate is supported; if the network requires EAP-TLS or server "
                              "certificate validation, this won't work."
                            : "Пустое поле = оставить текущий сохранённый пароль. Поддерживаются "
                              "PEAP/MSCHAPv2 без CA-сертификата; если сеть требует EAP-TLS или проверку "
                              "сертификата сервера — эта схема не подойдёт.";
  String portalPassLabel = en ? "Portal password (optional)" : "Пароль портала (необязательно)";
  String portalPassHint2 = en ? "Protects this web interface with a login prompt (login: admin). Leave "
                                "blank to leave the portal unprotected — you can always set this later "
                                "from the dashboard."
                              : "Защищает этот веб-интерфейс запросом логина/пароля (логин: admin). "
                                "Оставьте пустым, чтобы не защищать портал — это всегда можно будет "
                                "настроить позже на дашборде.";

  String html;
  html += "<!DOCTYPE html><html lang=\"" + uiLanguage + "\"><head><meta charset=\"utf-8\">"
          "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
          "<title>" + title + "</title><style>"
          "*{margin:0;padding:0;box-sizing:border-box}"
          "body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
          "background:#0f172a;color:#e2e8f0;min-height:100vh}"
          ".wrap{max-width:460px;margin:0 auto;padding:20px}"
          ".topbar{display:flex;justify-content:space-between;align-items:flex-start;gap:10px;margin-bottom:18px}"
          "h1{font-size:1.15rem;font-weight:600;line-height:1.4}"
          ".card{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:20px;margin-bottom:14px}"
          "label{display:block;margin-top:14px;font-size:.85rem;color:#94a3b8}"
          "input[type=text],input[type=password]{width:100%;padding:10px;font-size:1rem;box-sizing:border-box;"
          "margin-top:6px;background:#0f172a;border:1px solid #475569;border-radius:8px;color:#e2e8f0}"
          "input:focus{outline:none;border-color:#3b82f6}"
          ".hint{font-size:.78rem;color:#64748b;margin-top:6px}"
          "button{margin-top:20px;width:100%;padding:12px;font-size:1rem;font-weight:600;"
          "background:#3b82f6;color:#fff;border:none;border-radius:8px;cursor:pointer}"
          "button:hover{background:#2563eb}"
          "button:disabled{opacity:.6;cursor:default}"
          ".btn-lang{background:#991b1b;color:#fff;padding:8px 16px;border:none;border-radius:8px;"
          "font-size:.85rem;font-weight:600;cursor:pointer;white-space:nowrap;flex-shrink:0}"
          ".btn-lang:hover{background:#7f1d1d}"
          ".mac-row{display:flex;justify-content:space-between;font-size:.82rem;color:#94a3b8;margin-bottom:10px}"
          ".mac-row b{color:#e2e8f0}"
          ".chk-row{display:flex;align-items:center;gap:8px;margin-top:16px}"
          ".chk-row input[type=checkbox]{width:auto;margin:0}"
          ".chk-row label{margin:0;font-size:.85rem;color:#94a3b8}"
          ".radio-row{margin-top:8px;display:flex;align-items:center;gap:8px}"
          ".radio-row input[type=radio]{width:auto;margin:0}"
          ".radio-row label{margin:0;font-size:.9rem;color:#e2e8f0}"
          "#netList{margin-top:10px}"
          ".net-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;"
          "border-bottom:1px solid #293548;cursor:pointer;font-size:.9rem}"
          ".net-row:last-child{border-bottom:none}"
          ".net-rssi{color:#64748b;font-size:.78rem;white-space:nowrap;margin-left:10px}"
          "#entFields{display:none}"
          "</style></head><body><div class=\"wrap\">"
          "<div class=\"topbar\"><h1>&#x1F5A8; " + heading + "</h1>"
          "<form method=\"POST\" action=\"/set-lang\">"
          "<input type=\"hidden\" name=\"lang\" value=\"" + nextLang + "\">"
          "<button type=\"submit\" class=\"btn-lang\">" + nextLangLabel + "</button>"
          "</form></div>"

          "<div class=\"mac-row\"><span>" + macLabel + "</span><b>" + WiFi.softAPmacAddress() + "</b></div>"

          "<form action=\"/save\" method=\"POST\">"

          "<div class=\"card\">"
          "<button type=\"button\" onclick=\"scanNets(this)\">" + scanBtn + "</button>"
          "<div style=\"margin-top:16px\">"
          "<label style=\"margin-top:0\">" + chooseNet + "</label>"
          "<div class=\"hint\" id=\"scanHint\">" + chooseHint + "</div>"
          "<div id=\"netList\"></div>"
          "</div>"
          "<label>" + manualLabel + "</label>"
          "<input type=\"text\" name=\"ssid\" id=\"ssidField\">"
          "<div class=\"hint\">" + manualHint + "</div>"
          "</div>"

          "<div class=\"card\">"
          "<label style=\"margin-top:0\">" + netTypeLabel + "</label>"
          "<div class=\"radio-row\">"
          "<input type=\"radio\" id=\"typePersonal\" name=\"nettype\" value=\"personal\" checked "
          "onchange=\"toggleNetType()\">"
          "<label for=\"typePersonal\">" + personalOpt + "</label>"
          "</div>"
          "<div class=\"radio-row\">"
          "<input type=\"radio\" id=\"typeEnterprise\" name=\"nettype\" value=\"enterprise\" "
          "onchange=\"toggleNetType()\">"
          "<label for=\"typeEnterprise\">" + enterpriseOpt + "</label>"
          "</div>"

          "<div id=\"personalFields\">"
          "<label>" + passLabel + "</label>"
          "<input type=\"password\" name=\"pass\" id=\"passFld\">"
          "<div class=\"hint\">" + passHint + "</div>"
          "<div class=\"chk-row\">"
          "<input type=\"checkbox\" id=\"openChk\" name=\"open\" value=\"1\" "
          "onchange=\"document.getElementById('passFld').disabled=this.checked;\">"
          "<label for=\"openChk\">" + openLabel + "</label>"
          "</div>"
          "</div>"

          "<div id=\"entFields\">"
          "<input type=\"hidden\" id=\"entHidden\" name=\"enterprise\" value=\"\">"
          "<label>" + identityLabel + "</label>"
          "<input type=\"text\" name=\"identity\">"
          "<div class=\"hint\">" + identityHint + "</div>"
          "<label>" + usernameLabel + "</label>"
          "<input type=\"text\" name=\"eapuser\">"
          "<label>" + entPassLabel + "</label>"
          "<input type=\"password\" name=\"eappass\">"
          "<div class=\"hint\">" + entPassHint + "</div>"
          "</div>"
          "</div>"

          "<div class=\"card\">"
          "<label style=\"margin-top:0\">" + portalPassLabel + "</label>"
          "<input type=\"password\" name=\"portalpass\">"
          "<div class=\"hint\">" + portalPassHint2 + "</div>"
          "</div>"

          "<div class=\"card\">"
          "<button type=\"submit\">" + saveBtn + "</button>"
          "</div>"
          "</form>"

          "<script>"
          "function toggleNetType(){"
          "var ent = document.getElementById('typeEnterprise').checked;"
          "document.getElementById('entFields').style.display = ent ? 'block' : 'none';"
          "document.getElementById('personalFields').style.display = ent ? 'none' : 'block';"
          "document.getElementById('entHidden').value = ent ? '1' : '';"
          "}"
          "function pickNet(ssid){"
          "document.getElementById('ssidField').value = ssid;"
          "}"
          "function scanNets(btn){"
          "btn.disabled = true; var oldTxt = btn.textContent; btn.textContent = " + jsStrLit(scanning) + ";"
          "fetch('/api/wifi-scan').then(function(r){return r.json()}).then(function(nets){"
          "nets.sort(function(a,b){return b.rssi-a.rssi});"
          "var box = document.getElementById('netList');"
          "box.innerHTML = '';"
          "nets.forEach(function(n){"
          "var row = document.createElement('div');"
          "row.className = 'net-row';"
          "row.onclick = function(){ pickNet(n.ssid); };"
          "var left = document.createElement('span');"
          "left.textContent = n.ssid + (n.secure ? ' \\uD83D\\uDD12' : '');"
          "var right = document.createElement('span');"
          "right.className = 'net-rssi';"
          "right.textContent = n.rssi + ' dBm';"
          "row.appendChild(left); row.appendChild(right);"
          "box.appendChild(row);"
          "});"
          "document.getElementById('scanHint').style.display = 'none';"
          "btn.disabled = false; btn.textContent = oldTxt;"
          "}).catch(function(){ btn.disabled = false; btn.textContent = oldTxt; });"
          "}"
          "</script>"
          "</div></body></html>";
  return html;
}

// Небольшой помощник: превращает произвольную строку в безопасный JS-литерал
// (нужен для подстановки переведённых UI-строк прямо в тело <script>).
String jsStrLit(const String &s) {
  String out = "'";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\'' || c == '\\') out += '\\';
    out += c;
  }
  out += "'";
  return out;
}

// Статус-страница показывается вместо формы настройки, когда плата уже
// подключена к WiFi — портал теперь работает постоянно, не только в
// режиме первоначальной настройки.
// Отдаёт живые данные для дашборда в формате JSON — сама страница
// (см. buildDashboardHtml) статична и просто периодически дёргает этот
// адрес через JS, обновляя цифры без перезагрузки страницы.
// Экранирует спецсимволы (кавычки, переносы строк, обратный слэш),
// прежде чем подставить произвольный текст в JSON-строку. Без этого
// сырой PJL-ответ вида DISPLAY="Non HP supplyin use" с кавычками и
// переносами строк ломает синтаксис JSON, и fetch(...).json() в
// браузере падает с ошибкой парсинга — страница застревает на
// плейсхолдерах "—", ничего не обновляя.
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c < 0x20) { /* прочие управляющие символы пропускаем */ }
    else out += c;
  }
  return out;
}

void handleApiStatus() {
  if (!requirePortalAuth()) return;
  String printerState;
  if (!printerReady) printerState = "disconnected";
  else if (printerHasIssue) printerState = "issue";
  else if (printerIsBusyPrinting) printerState = "printing";
  else printerState = "ready";

  String j = "{";
  j += "\"printer\":{";
  j += "\"state\":\"" + printerState + "\",";
  j += "\"manufacturer\":\"" + jsonEscape(printerManufacturer) + "\",";
  j += "\"product\":\"" + jsonEscape(printerProduct) + "\",";
  j += "\"serial\":\"" + jsonEscape(printerSerial) + "\",";
  j += "\"pjlRaw\":\"" + jsonEscape(lastPrinterStatusRaw) + "\"";
  j += "},";
  j += "\"usb\":{";
  j += "\"attached\":" + String(printerReady ? "true" : "false") + ",";
  j += "\"address\":" + String((int)printerAddress) + ",";
  j += "\"epOut\":\"0x" + String(usb.vendorOutEndpoint(printerAddress), HEX) + "\",";
  j += "\"epIn\":\"0x" + String(usb.vendorInEndpoint(printerAddress), HEX) + "\",";
  j += "\"mps\":" + String(usb.vendorOutPacketSize(printerAddress));
  j += "},";
  j += "\"wifi\":{";
  j += "\"ssid\":\"" + jsonEscape(savedSsid) + "\",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  j += "\"mac\":\"" + WiFi.macAddress() + "\",";
  j += "\"rssi\":" + String(WiFi.RSSI());
  j += "},";
  j += "\"system\":{";
  j += "\"uptime\":" + String(millis() / 1000) + ",";
  j += "\"freeHeap\":" + String(ESP.getFreeHeap());
  j += "}";
  j += "}";

  configServer.send(200, "application/json; charset=utf-8", j);
}

// Карточный дашборд в духе современных веб-панелей: статичная разметка
// со стилями, данные подгружаются через /api/status раз в 2 секунды
// без перезагрузки страницы. WiFi-опрос статуса и мигание светодиода
// на самом принтере (реальный физический опрос по USB, см.
// checkPrinterStatusPeriodic) при этом никак не меняются — здесь
// только визуальное отображение уже собранных данных.
String buildDashboardHtml() {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="__HTMLLANG__">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>ESP32 Print Server</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
     background:#0f172a;color:#e2e8f0;min-height:100vh}
.wrap{max-width:900px;margin:0 auto;padding:20px}
.topbar{display:flex;justify-content:space-between;align-items:flex-start;gap:10px}
h1{font-size:1.35rem;font-weight:600;margin-bottom:4px}
.sub{color:#64748b;font-size:.82rem;margin-bottom:18px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}
.card{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:16px 18px}
.card h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.06em;color:#94a3b8;margin-bottom:12px}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:7px 0;border-bottom:1px solid #293548}
.row:last-child{border-bottom:none}
.k{color:#94a3b8;font-size:.85rem;white-space:nowrap}
.v{font-weight:500;font-size:.85rem;text-align:right;word-break:break-all}
.badge{display:inline-block;padding:2px 10px;border-radius:999px;font-size:.75rem;font-weight:600}
.b-green{background:#064e3b;color:#6ee7b7}
.b-red{background:#7f1d1d;color:#fca5a5}
.b-yellow{background:#713f12;color:#fde68a}
.b-gray{background:#334155;color:#cbd5e1}
.btn{padding:10px 16px;border:none;border-radius:8px;font-size:.9rem;font-weight:600;cursor:pointer;width:100%;margin-top:12px}
.btn-danger{background:#991b1b;color:#fca5a5}.btn-danger:hover{background:#7f1d1d}
.btn-primary{background:#3b82f6;color:#fff}.btn-primary:hover{background:#2563eb}
.btn-lang{background:#991b1b;color:#fff;padding:8px 16px;border:none;border-radius:8px;
          font-size:.85rem;font-weight:600;cursor:pointer;white-space:nowrap}
.btn-lang:hover{background:#7f1d1d}
.raw{margin-top:8px;color:#64748b;font-size:.78rem;line-height:1.5;word-break:break-all;white-space:pre-wrap}
.two{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:620px){.two{grid-template-columns:1fr}.topbar{flex-direction:column}}
ol{margin:0;padding-left:18px;color:#94a3b8;font-size:.82rem;line-height:1.75}
ol b{color:#e2e8f0}
.foot{text-align:center;color:#475569;font-size:.76rem;margin:22px 0 8px}
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <div>
      <h1>&#x1F5A8; ESP32 Print Server</h1>
      <div class="sub"><span data-i18n="subtitle">-</span> &middot; <span id="ipaddr">-</span></div>
    </div>
    <form method="POST" action="/set-lang">
      <input type="hidden" name="lang" value="__NEXTLANG__">
      <button type="submit" class="btn-lang">__NEXTLANGLABEL__</button>
    </form>
  </div>

  <div class="grid">
    <div class="card">
      <h2 data-i18n="printer">-</h2>
      <div class="row"><span class="k" data-i18n="state">-</span><span id="p-state" class="badge b-gray">—</span></div>
      <div class="row"><span class="k" data-i18n="manufacturer">-</span><span class="v" id="p-mfg">—</span></div>
      <div class="row"><span class="k" data-i18n="model">-</span><span class="v" id="p-mdl">—</span></div>
      <div class="row"><span class="k" data-i18n="serial">-</span><span class="v" id="p-sn">—</span></div>
      <div class="raw" id="p-raw"></div>
    </div>

    <div class="card">
      <h2 data-i18n="usbConn">-</h2>
      <div class="row"><span class="k" data-i18n="state">-</span><span id="u-conn" class="badge b-gray">—</span></div>
      <div class="row"><span class="k" data-i18n="devAddr">-</span><span class="v" id="u-addr">—</span></div>
      <div class="row"><span class="k">Bulk OUT</span><span class="v" id="u-epout">—</span></div>
      <div class="row"><span class="k">Bulk IN</span><span class="v" id="u-epin">—</span></div>
      <div class="row"><span class="k" data-i18n="pktSize">-</span><span class="v" id="u-mps">—</span></div>
    </div>

    <div class="card">
      <h2>WI-FI</h2>
      <div class="row"><span class="k" data-i18n="network">-</span><span class="v" id="w-ssid">—</span></div>
      <div class="row"><span class="k" data-i18n="ipAddr">-</span><span class="v" id="w-ip">—</span></div>
      <div class="row"><span class="k">MAC</span><span class="v" id="w-mac">—</span></div>
      <div class="row"><span class="k" data-i18n="signal">-</span><span class="v" id="w-rssi">—</span></div>
      <form action="/reset-wifi" method="POST" onsubmit="return confirm(T[LANG].confirmReset);">
        <button class="btn btn-danger" type="submit" data-i18n="resetWifi">-</button>
      </form>
    </div>

    <div class="card">
      <h2 data-i18n="system">-</h2>
      <div class="row"><span class="k" data-i18n="uptime">-</span><span class="v" id="s-up">—</span></div>
      <div class="row"><span class="k" data-i18n="freeMem">-</span><span class="v" id="s-heap">—</span></div>
      <form action="/set-portal-pass" method="POST" style="margin-top:14px;padding-top:14px;border-top:1px solid #293548">
        <span class="k" data-i18n="portalPass" style="display:block;margin-bottom:6px">-</span>
        <input type="password" name="portalpass" placeholder="••••••••"
               style="width:100%;padding:9px 11px;background:#0f172a;border:1px solid #475569;
                      border-radius:8px;color:#e2e8f0;font-size:.9rem;box-sizing:border-box">
        <div class="raw" style="margin-top:6px" data-i18n="portalPassHint">-</div>
        <button class="btn btn-primary" type="submit" data-i18n="portalPassBtn">-</button>
      </form>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2 data-i18n="setupTitle">-</h2>
    <div class="two">
      <div>
        <p class="k" style="margin-bottom:6px">Windows 10 / 11</p>
        <ol data-i18n-html="setupWin"></ol>
      </div>
      <div>
        <p class="k" style="margin-bottom:6px">macOS</p>
        <ol data-i18n-html="setupMac"></ol>
      </div>
    </div>
    <div class="raw" style="margin-top:12px" data-i18n="setupNote">-</div>
  </div>

  <div class="foot">ESP32 Print Server</div>
</div>

<script>
var LANG = "__LANG__";
var T = {
  ru: {
    subtitle: 'WiFi \u2192 USB \u043c\u043e\u0441\u0442',
    printer: '\u041f\u0420\u0418\u041d\u0422\u0415\u0420',
    usbConn: '\u041f\u041e\u0414\u041a\u041b\u042e\u0427\u0415\u041d\u0418\u0415 \u041f\u041e USB',
    state: '\u0421\u043e\u0441\u0442\u043e\u044f\u043d\u0438\u0435',
    manufacturer: '\u041f\u0440\u043e\u0438\u0437\u0432\u043e\u0434\u0438\u0442\u0435\u043b\u044c',
    model: '\u041c\u043e\u0434\u0435\u043b\u044c',
    serial: '\u0421\u0435\u0440\u0438\u0439\u043d\u044b\u0439 \u043d\u043e\u043c\u0435\u0440',
    devAddr: '\u0410\u0434\u0440\u0435\u0441 \u0443\u0441\u0442\u0440\u043e\u0439\u0441\u0442\u0432\u0430',
    pktSize: '\u0420\u0430\u0437\u043c\u0435\u0440 \u043f\u0430\u043a\u0435\u0442\u0430',
    network: '\u0421\u0435\u0442\u044c',
    ipAddr: 'IP-\u0430\u0434\u0440\u0435\u0441',
    signal: '\u0421\u0438\u0433\u043d\u0430\u043b',
    system: '\u0421\u0438\u0441\u0442\u0435\u043c\u0430',
    uptime: '\u0412\u0440\u0435\u043c\u044f \u0440\u0430\u0431\u043e\u0442\u044b',
    freeMem: '\u0414\u043e\u0441\u0442\u0443\u043f\u043d\u0430\u044f \u043f\u0430\u043c\u044f\u0442\u044c',
    resetWifi: '\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c WiFi-\u043d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0438',
    confirmReset: '\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c \u0441\u043e\u0445\u0440\u0430\u043d\u0451\u043d\u043d\u044b\u0435 WiFi-\u043d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0438 \u0438 \u043f\u0435\u0440\u0435\u0437\u0430\u0433\u0440\u0443\u0437\u0438\u0442\u044c \u043f\u043b\u0430\u0442\u0443?',
    portalPass: '\u041f\u0430\u0440\u043e\u043b\u044c \u043f\u043e\u0440\u0442\u0430\u043b\u0430',
    portalPassHint: '\u041e\u0441\u0442\u0430\u0432\u044c\u0442\u0435 \u043f\u0443\u0441\u0442\u044b\u043c \u0438 \u0441\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u0435, \u0447\u0442\u043e\u0431\u044b \u0441\u043d\u044f\u0442\u044c \u0437\u0430\u0449\u0438\u0442\u0443. \u041f\u043e\u0441\u043b\u0435 \u0443\u0441\u0442\u0430\u043d\u043e\u0432\u043a\u0438 \u0431\u0440\u0430\u0443\u0437\u0435\u0440 \u0441\u043f\u0440\u043e\u0441\u0438\u0442 \u043b\u043e\u0433\u0438\u043d/\u043f\u0430\u0440\u043e\u043b\u044c \u043f\u0440\u0438 \u0441\u043b\u0435\u0434\u0443\u044e\u0449\u0435\u043c \u0437\u0430\u0445\u043e\u0434\u0435 \u043d\u0430 \u043f\u043e\u0440\u0442\u0430\u043b (\u043b\u043e\u0433\u0438\u043d: admin).',
    portalPassBtn: '\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0430\u0440\u043e\u043b\u044c',
    setupTitle: '\u0414\u043e\u0431\u0430\u0432\u043b\u0435\u043d\u0438\u0435 \u043f\u0440\u0438\u043d\u0442\u0435\u0440\u0430 \u043d\u0430 \u043a\u043e\u043c\u043f\u044c\u044e\u0442\u0435\u0440\u0435',
    setupWin: '<li>\u0421\u043d\u0430\u0447\u0430\u043b\u0430 \u0443\u0441\u0442\u0430\u043d\u043e\u0432\u0438\u0442\u0435 \u043e\u0444\u0438\u0446\u0438\u0430\u043b\u044c\u043d\u044b\u0439 \u0434\u0440\u0430\u0439\u0432\u0435\u0440 <b><span class="pmdl">\u044d\u0442\u043e\u0433\u043e \u043f\u0440\u0438\u043d\u0442\u0435\u0440\u0430</span></b></li><li>\u041f\u0430\u0440\u0430\u043c\u0435\u0442\u0440\u044b \u2192 Bluetooth \u0438 \u0434\u0440\u0443\u0433\u0438\u0435 \u0443\u0441\u0442\u0440\u043e\u0439\u0441\u0442\u0432\u0430 \u2192 \u041f\u0440\u0438\u043d\u0442\u0435\u0440\u044b \u0438 \u0441\u043a\u0430\u043d\u0435\u0440\u044b \u2192 <b>\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c \u0443\u0441\u0442\u0440\u043e\u0439\u0441\u0442\u0432\u043e</b></li><li>\u041d\u0430\u0436\u043c\u0438\u0442\u0435 \u00ab<b>\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c \u0432\u0440\u0443\u0447\u043d\u0443\u044e</b>\u00bb \u2192 \u00ab<b>\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c \u043f\u0440\u0438\u043d\u0442\u0435\u0440 \u043f\u043e TCP/IP-\u0430\u0434\u0440\u0435\u0441\u0443 \u0438\u043b\u0438 \u0438\u043c\u0435\u043d\u0438 \u0443\u0437\u043b\u0430</b>\u00bb</li><li>\u0422\u0438\u043f \u0443\u0441\u0442\u0440\u043e\u0439\u0441\u0442\u0432\u0430 \u2014 <b>TCP/IP-\u0443\u0441\u0442\u0440\u043e\u0439\u0441\u0442\u0432\u043e</b>, \u0438\u043c\u044f \u0443\u0437\u043b\u0430/IP-\u0430\u0434\u0440\u0435\u0441 \u2014 <b><span class="ipv">\u2014</span></b></li><li>\u041f\u0440\u043e\u0442\u043e\u043a\u043e\u043b \u2014 \u043b\u0438\u0431\u043e <b>Raw</b> (\u043d\u043e\u043c\u0435\u0440 \u043f\u043e\u0440\u0442\u0430 <b>9100</b>), \u043b\u0438\u0431\u043e <b>LPR</b> (\u0438\u043c\u044f \u043e\u0447\u0435\u0440\u0435\u0434\u0438 \u2014 \u043b\u044e\u0431\u043e\u0435, \u043f\u043e\u0440\u0442 515)</li><li>\u0415\u0441\u043b\u0438 Windows \u043d\u0435 \u0434\u0430\u0451\u0442 \u0432\u044b\u0431\u0440\u0430\u0442\u044c LPR \u043d\u0430\u043f\u0440\u044f\u043c\u0443\u044e \u2014 \u0434\u043e\u0431\u0430\u0432\u044c\u0442\u0435 \u0447\u0435\u0440\u0435\u0437 \u00ab\u041e\u0441\u043e\u0431\u044b\u0439\u00bb, \u043f\u0440\u043e\u0442\u043e\u043a\u043e\u043b <b>LPR</b></li><li><b>\u0421\u043d\u0438\u043c\u0438\u0442\u0435 \u0433\u0430\u043b\u043e\u0447\u043a\u0443 \u00abSNMP\u00bb</b> \u2014 \u043f\u0440\u0438\u043d\u0442\u0435\u0440 \u0435\u0451 \u043d\u0435 \u043f\u043e\u0434\u0434\u0435\u0440\u0436\u0438\u0432\u0430\u0435\u0442</li><li>\u0412 \u043a\u0430\u0447\u0435\u0441\u0442\u0432\u0435 \u0434\u0440\u0430\u0439\u0432\u0435\u0440\u0430 \u0432\u044b\u0431\u0435\u0440\u0438\u0442\u0435 <b><span class="pmdl">\u044d\u0442\u043e\u0442 \u043f\u0440\u0438\u043d\u0442\u0435\u0440</span></b></li>',
    setupMac: '<li>\u0421\u043d\u0430\u0447\u0430\u043b\u0430 \u0443\u0441\u0442\u0430\u043d\u043e\u0432\u0438\u0442\u0435 \u043e\u0444\u0438\u0446\u0438\u0430\u043b\u044c\u043d\u044b\u0439 \u0434\u0440\u0430\u0439\u0432\u0435\u0440 <span class="pmdl">\u044d\u0442\u043e\u0433\u043e \u043f\u0440\u0438\u043d\u0442\u0435\u0440\u0430</span></li><li>\u0421\u0438\u0441\u0442\u0435\u043c\u043d\u044b\u0435 \u043d\u0430\u0441\u0442\u0440\u043e\u0439\u043a\u0438 \u2192 \u041f\u0440\u0438\u043d\u0442\u0435\u0440\u044b \u0438 \u0441\u043a\u0430\u043d\u0435\u0440\u044b \u2192 <b>\u0414\u043e\u0431\u0430\u0432\u0438\u0442\u044c \u043f\u0440\u0438\u043d\u0442\u0435\u0440</b></li><li>\u0412\u043a\u043b\u0430\u0434\u043a\u0430 <b>IP</b></li><li>\u041f\u0440\u043e\u0442\u043e\u043a\u043e\u043b \u2014 <b>HP Jetdirect - Socket</b> (Raw, 9100) \u043b\u0438\u0431\u043e <b>LPD</b> (LPR, 515)</li><li>\u0410\u0434\u0440\u0435\u0441 \u2014 <b><span class="ipv">\u2014</span></b>; \u0434\u043b\u044f LPD \u0432 \u043f\u043e\u043b\u0435 \u00ab\u041e\u0447\u0435\u0440\u0435\u0434\u044c\u00bb \u0432\u043f\u0438\u0448\u0438\u0442\u0435 \u043b\u044e\u0431\u043e\u0435 \u0438\u043c\u044f</li><li>\u0412\u044b\u0431\u0435\u0440\u0438\u0442\u0435 \u041f\u041e \u2192 \u043d\u0430\u0439\u0434\u0438\u0442\u0435 \u043c\u043e\u0434\u0435\u043b\u044c <span class="pmdl">\u044d\u0442\u043e\u0433\u043e \u043f\u0440\u0438\u043d\u0442\u0435\u0440\u0430</span></li>',
    setupNote: '\u041e\u0431\u0430 \u043f\u0440\u043e\u0442\u043e\u043a\u043e\u043b\u0430 (Raw \u0438 LPR) \u0440\u0430\u0431\u043e\u0442\u0430\u044e\u0442 \u043e\u0434\u043d\u043e\u0432\u0440\u0435\u043c\u0435\u043d\u043d\u043e \u2014 \u0440\u0430\u0437\u043d\u0438\u0446\u044b \u0432 \u043a\u0430\u0447\u0435\u0441\u0442\u0432\u0435 \u043d\u0435\u0442.',
    stReady: '\u0413\u043e\u0442\u043e\u0432', stPrinting: '\u0418\u0434\u0451\u0442 \u043f\u0435\u0447\u0430\u0442\u044c',
    stIssue: '\u0415\u0441\u0442\u044c \u043f\u0440\u043e\u0431\u043b\u0435\u043c\u0430', stDisc: '\u041d\u0435 \u043f\u043e\u0434\u043a\u043b\u044e\u0447\u0451\u043d',
    conn: '\u041f\u043e\u0434\u043a\u043b\u044e\u0447\u0435\u043d\u043e', notConn: '\u041d\u0435 \u043f\u043e\u0434\u043a\u043b\u044e\u0447\u0435\u043d\u043e',
    bytes: '\u0431\u0430\u0439\u0442', dbm: '\u0434\u0411\u043c', kb: '\u041a\u0411',
    h: '\u0447 ', m: '\u043c ', s: '\u0441', defPrinter: '\u044d\u0442\u043e\u0433\u043e \u043f\u0440\u0438\u043d\u0442\u0435\u0440\u0430'
  },
  en: {
    subtitle: 'WiFi \u2192 USB bridge',
    printer: 'PRINTER', usbConn: 'USB CONNECTION', state: 'Status',
    manufacturer: 'Manufacturer', model: 'Model', serial: 'Serial number',
    devAddr: 'Device address', pktSize: 'Packet size', network: 'Network',
    ipAddr: 'IP address', signal: 'Signal', system: 'System',
    uptime: 'Uptime', freeMem: 'Free memory', resetWifi: 'Reset WiFi settings',
    confirmReset: 'Reset saved WiFi settings and reboot the board?',
    portalPass: 'Portal password',
    portalPassHint: 'Leave blank and save to remove protection. After setting it, the browser will '
                    + 'ask for a login/password the next time you open the portal (login: admin).',
    portalPassBtn: 'Save password',
    setupTitle: 'Adding the printer on a computer',
    setupWin: '<li>First install the official driver for <b><span class="pmdl">this printer</span></b></li><li>Settings &rarr; Bluetooth &amp; devices &rarr; Printers &amp; scanners &rarr; <b>Add device</b></li><li>Click \u201c<b>Add manually</b>\u201d &rarr; \u201c<b>Add a printer using a TCP/IP address or hostname</b>\u201d</li><li>Device type \u2014 <b>TCP/IP Device</b>, hostname/IP \u2014 <b><span class="ipv">\u2014</span></b></li><li>Protocol \u2014 either <b>Raw</b> (port <b>9100</b>) or <b>LPR</b> (any queue name, port 515)</li><li>If Windows won\u2019t let you pick LPR directly \u2014 add it via \u201cSpecial\u201d, protocol <b>LPR</b></li><li><b>Make sure \u201cSNMP Status Enabled\u201d is unchecked</b> \u2014 the printer doesn\u2019t support it</li><li>For the driver, pick the already-installed <b><span class="pmdl">this printer</span></b></li>',
    setupMac: '<li>First install the official driver for <span class="pmdl">this printer</span></li><li>System Settings &rarr; Printers &amp; Scanners &rarr; <b>Add Printer</b></li><li>Switch to the <b>IP</b> tab</li><li>Protocol \u2014 <b>HP Jetdirect - Socket</b> (Raw, 9100) or <b>LPD</b> (LPR, 515)</li><li>Address \u2014 <b><span class="ipv">\u2014</span></b>; for LPD, type any name into the \u201cQueue\u201d field</li><li>Under Use, pick <b>Select Software</b> &rarr; find <span class="pmdl">this printer</span></li>',
    setupNote: 'Both protocols (Raw and LPR) work at the same time on this board \u2014 there is no difference in print quality.',
    stReady: 'Ready', stPrinting: 'Printing', stIssue: 'Issue', stDisc: 'Disconnected',
    conn: 'Connected', notConn: 'Not connected',
    bytes: 'bytes', dbm: 'dBm', kb: 'KB',
    h: 'h ', m: 'm ', s: 's', defPrinter: 'this printer'
  }
};

function $(id){return document.getElementById(id)}
function ups(sec){var h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),x=sec%60;
  return h+T[LANG].h+m+T[LANG].m+x+T[LANG].s}
function setB(id,cls,txt){var e=$(id);e.className='badge '+cls;e.textContent=txt}

function applyI18n(){
  document.querySelectorAll('[data-i18n]').forEach(function(el){
    var k = el.getAttribute('data-i18n');
    if (T[LANG][k]) el.textContent = T[LANG][k];
  });
  document.querySelectorAll('[data-i18n-html]').forEach(function(el){
    var k = el.getAttribute('data-i18n-html');
    if (T[LANG][k]) el.innerHTML = T[LANG][k];
  });
}

function update(){
  fetch('/api/status').then(function(r){return r.json()}).then(function(d){
    $('ipaddr').textContent = d.wifi.ip;

    var st=d.printer.state;
    if(st==='ready') setB('p-state','b-green',T[LANG].stReady);
    else if(st==='printing') setB('p-state','b-green',T[LANG].stPrinting);
    else if(st==='issue') setB('p-state','b-yellow',T[LANG].stIssue);
    else setB('p-state','b-red',T[LANG].stDisc);

    $('p-mfg').textContent = d.printer.manufacturer || '\u2014';
    $('p-mdl').textContent = d.printer.product || '\u2014';
    $('p-sn').textContent  = d.printer.serial || '\u2014';
    $('p-raw').textContent = d.printer.pjlRaw || '';

    setB('u-conn', d.usb.attached?'b-green':'b-red', d.usb.attached?T[LANG].conn:T[LANG].notConn);
    $('u-addr').textContent = d.usb.attached ? d.usb.address : '\u2014';
    $('u-epout').textContent = d.usb.attached ? d.usb.epOut : '\u2014';
    $('u-epin').textContent  = d.usb.attached ? d.usb.epIn  : '\u2014';
    $('u-mps').textContent   = d.usb.attached ? (d.usb.mps+' '+T[LANG].bytes) : '\u2014';

    $('w-ssid').textContent = d.wifi.ssid || '\u2014';
    $('w-ip').textContent = d.wifi.ip;
    $('w-mac').textContent = d.wifi.mac;
    $('w-rssi').textContent = d.wifi.rssi + ' ' + T[LANG].dbm;

    $('s-up').textContent = ups(d.system.uptime);
    $('s-heap').textContent = (d.system.freeHeap/1024).toFixed(0)+' '+T[LANG].kb;

    var mdlName = d.printer.product || T[LANG].defPrinter;
    var pm = document.querySelectorAll('.pmdl');
    for (var i = 0; i < pm.length; i++) pm[i].textContent = mdlName;
    var ipv = document.querySelectorAll('.ipv');
    for (var i2 = 0; i2 < ipv.length; i2++) ipv[i2].textContent = d.wifi.ip;
  }).catch(function(e){ console.error(e) });
}
applyI18n();
update();
setInterval(update, 2000);
</script>
</body>
</html>
)HTML";

  html.replace("__LANG__", uiLanguage);
  html.replace("__HTMLLANG__", uiLanguage);
  if (uiLanguage == "en") {
    html.replace("__NEXTLANG__", "ru");
    html.replace("__NEXTLANGLABEL__", "\u0420\u0443\u0441\u0441\u043a\u0438\u0439"); // "Русский"
  } else {
    html.replace("__NEXTLANG__", "en");
    html.replace("__NEXTLANGLABEL__", "English");
  }
  return html;
}

void handleConfigRoot() {
  if (!requirePortalAuth()) return;
  if (WiFi.status() == WL_CONNECTED) {
    configServer.send(200, "text/html; charset=utf-8", buildDashboardHtml());
    return;
  }
  configServer.send(200, "text/html; charset=utf-8", buildWifiSetupHtml());
}

// Сканирование доступных сетей для страницы первоначальной настройки.
// Синхронный WiFi.scanNetworks() блокирует на ~1-3 секунды, что здесь
// не проблема — этот запрос происходит только по явному нажатию кнопки
// пользователем, пока плата и так просто ждёт ввода в форме, а не
// занята приёмом задания печати.
void handleWifiScanApi() {
  if (!requirePortalAuth()) return;
  int n = WiFi.scanNetworks();
  String j = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) j += ",";
    j += "{\"ssid\":\"" + jsonEscape(WiFi.SSID(i)) + "\",";
    j += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    j += "\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
  }
  j += "]";
  WiFi.scanDelete();
  configServer.send(200, "application/json; charset=utf-8", j);
}

void handleConfigSave() {
  if (!requirePortalAuth()) return;
  String ssid = configServer.arg("ssid");
  String pass = configServer.arg("pass");
  // ВАЖНО: поле "enterprise" — скрытый <input type="hidden">, которым
  // управляет JS (переключение радиокнопок "Обычная"/"WPA2-Enterprise"),
  // а не чекбокс. Скрытое поле всегда присутствует в отправленных
  // данных формы, даже когда его значение — пустая строка (обычная
  // сеть) — поэтому hasArg("enterprise") здесь был бы true ВСЕГДА,
  // независимо от реального выбора. Нужно явно сравнивать значение.
  bool enterprise = (configServer.arg("enterprise") == "1");
  String identity = configServer.arg("identity");
  String eapUser = configServer.arg("eapuser");
  String eapPass = configServer.arg("eappass");
  // А вот "open" — настоящий чекбокс, при снятой галочке браузер вообще
  // не включает его в отправку, так что hasArg() тут работает корректно.
  bool openNetwork = configServer.hasArg("open");
  String portalPass = configServer.arg("portalpass");
  bool en = (uiLanguage == "en");

  if (ssid.length() == 0) {
    configServer.send(400, "text/plain; charset=utf-8", en ? "SSID cannot be empty" : "SSID не может быть пустым");
    return;
  }

  // Пароль портала на этой же странице — необязательное поле: заполнили
  // при первой настройке — задаём его тут же, оставили пустым — портал
  // остаётся как был (не защищённым, если ещё не защищали, либо со
  // старым паролем, если защита уже когда-то была настроена с дашборда).
  if (portalPass.length() > 0) {
    savePortalPassword(portalPass);
  }

  // Пустое поле означает "оставить как было сохранено" — так не нужно
  // вслепую перепечатывать длинный пароль/Identity, если меняется что-то
  // одно (например, просто выбрали другую сеть из списка). Явное
  // намерение задать ИМЕННО пустой пароль для открытой сети выражается
  // отдельной галочкой "Это открытая сеть".
  if (enterprise) {
    String finalIdentity = identity.length() > 0 ? identity : savedEapIdentity;
    String finalUser = eapUser.length() > 0 ? eapUser : savedEapUsername;
    String finalPass = eapPass.length() > 0 ? eapPass : savedPassword;

    if (finalUser.length() == 0) {
      configServer.send(400, "text/plain; charset=utf-8",
        en ? "Username is required for a WPA2-Enterprise network"
           : "Для сети WPA2-Enterprise нужно указать Username");
      return;
    }
    saveWifiCredentials(ssid, finalPass, true, finalIdentity, finalUser);
  } else {
    String finalPass = openNetwork ? "" : (pass.length() > 0 ? pass : savedPassword);
    saveWifiCredentials(ssid, finalPass, false, "", "");
  }

  String msg = en
    ? "<h3>Saved. Rebooting...</h3><p>If the network is available, the board will "
      "connect to it. The access point will shut down.</p>"
    : "<h3>Сохранено. Перезагрузка...</h3><p>Если сеть доступна, плата подключится к ней. "
      "Точка доступа отключится.</p>";

  configServer.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html lang=\"" + uiLanguage + "\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
    "background:#0f172a;color:#e2e8f0;min-height:100vh;display:flex;align-items:center;"
    "justify-content:center;text-align:center;padding:20px}"
    "div{max-width:360px}h3{margin-bottom:10px}p{color:#94a3b8;font-size:.9rem}</style>"
    "</head><body><div>" + msg + "</div></body></html>");

  delay(1000);
  ESP.restart();
}

// Позволяет сбросить сохранённые WiFi-настройки прямо со статус-страницы
// в браузере, без необходимости физически держать кнопку 30 секунд.
void handleResetWifiRequest() {
  if (!requirePortalAuth()) return;
  bool en = (uiLanguage == "en");
  String msg = en ? "<h3>WiFi settings reset. Rebooting...</h3>"
                   : "<h3>WiFi-настройки сброшены. Перезагрузка...</h3>";
  configServer.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html lang=\"" + uiLanguage + "\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
    "background:#0f172a;color:#e2e8f0;min-height:100vh;display:flex;align-items:center;"
    "justify-content:center;text-align:center;padding:20px}</style>"
    "</head><body>" + msg + "</body></html>");
  delay(500);
  clearWifiCredentials();
  delay(500);
  ESP.restart();
}

// Переключение языка интерфейса — без перезагрузки платы. Сохраняем в
// NVS и сразу редиректим обратно на "/", чтобы страница перерисовалась
// уже в новом языке.
void handleSetLanguage() {
  if (!requirePortalAuth()) return;
  String lang = configServer.arg("lang");
  saveUiLanguage(lang);
  configServer.sendHeader("Location", "/");
  configServer.send(303);
}

void handleConfigNotFound() {
  handleConfigRoot();
}

// Установка/снятие пароля портала. Сама эта операция тоже защищена
// requirePortalAuth() — если пароль уже задан, для его смены нужно
// сначала пройти аутентификацию текущим. Если пароль ещё не задан —
// пускает без проверки (иначе некому было бы задать самый первый пароль).
void handlePortalPasswordSave() {
  if (!requirePortalAuth()) return;
  String newPass = configServer.arg("portalpass");
  savePortalPassword(newPass);
  configServer.sendHeader("Location", "/");
  configServer.send(303);
}

// Регистрирует маршруты портала и запускает сервер. Вызывается из
// connectWiFi() (успешное STA-подключение) и startConfigPortal()
// (режим AP) — но реально выполняется только один раз благодаря
// флагу portalStarted, т.к. WiFi может переподключаться много раз
// за время работы платы, а маршруты регистрировать повторно незачем.
void ensurePortalRunning() {
  if (portalStarted) return;

  configServer.on("/", HTTP_GET, handleConfigRoot);
  configServer.on("/save", HTTP_POST, handleConfigSave);
  configServer.on("/reset-wifi", HTTP_POST, handleResetWifiRequest);
  configServer.on("/set-lang", HTTP_POST, handleSetLanguage);
  configServer.on("/set-portal-pass", HTTP_POST, handlePortalPasswordSave);
  configServer.on("/api/wifi-scan", HTTP_GET, handleWifiScanApi);
  configServer.on("/api/status", HTTP_GET, handleApiStatus);
  configServer.on(UPNP_DESCRIPTION_PATH, HTTP_GET, handleUpnpDescription);
  configServer.onNotFound(handleConfigNotFound);
  configServer.begin();

  portalStarted = true;
  Serial.println("[Portal] Веб-портал запущен (работает постоянно).");
}

// Поднимает точку доступа и HTTP-портал, блокирует выполнение,
// пока пользователь не введёт данные (после чего плата перезагрузится).
void startConfigPortal() {
  Serial.println("========================================");
  Serial.printf("[AP] Не удалось подключиться к WiFi. Поднимаю точку доступа.\n");
  Serial.printf("[AP] SSID: %s   Пароль: %s\n", AP_SSID, AP_PASSWORD);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.printf("[AP] MAC-адрес платы (Wi-Fi): %s\n", WiFi.softAPmacAddress().c_str());
  Serial.printf("[AP] Откройте http://%s в браузере, подключившись к сети \"%s\".\n",
                AP_IP.toString().c_str(), AP_SSID);

  ensurePortalRunning();

  setLedState(LED_STATE_AP_CONFIG);

  while (true) {
    configServer.handleClient();
    checkResetButton(); // также обновляет LED каждую итерацию
    checkPrinterStatusPeriodic(); // статус принтера опрашивается и без домашнего WiFi
    servePrintClients();          // печать работает и через собственную точку доступа 192.168.4.1
#if ENABLE_OLED_DISPLAY
    updateOledIp(WiFi.softAPIP().toString());
#endif
    delay(2);
  }
}

// ==================== UPnP: генерация и хранение UUID устройства ====================

String loadOrCreateDeviceUuid() {
  prefs.begin("device", false);
  String uuid = prefs.getString("uuid", "");
  if (uuid.length() == 0) {
    // Не настоящий криптографически случайный UUID, а стабильный
    // идентификатор на основе MAC-адреса чипа — этого достаточно для
    // целей UPnP UDN (главное — стабильность и уникальность).
    uint64_t mac = ESP.getEfuseMac();
    char buf[40];
    snprintf(buf, sizeof(buf), "%08x-%04x-%04x-%04x-%012llx",
             (uint32_t)(mac >> 32), (uint16_t)(mac >> 16), (uint16_t)(mac),
             (uint16_t)(mac >> 8), (unsigned long long)mac);
    uuid = String(buf);
    prefs.putString("uuid", uuid);
    Serial.printf("[UPnP] Сгенерирован новый device UUID: %s\n", uuid.c_str());
  }
  prefs.end();
  return uuid;
}

// ==================== UPnP: XML-описание устройства ====================
//
// Отдаётся по адресу UPNP_DESCRIPTION_PATH. Ключевое поле —
// <presentationURL>: именно на него ведёт ссылка "Просмотр веб-страницы
// устройства" в Проводнике Windows ("Сеть"). У нас это корень "/", тот
// же самый постоянно работающий портал (статус-страница/настройка WiFi).
String buildUpnpDescriptionXml() {
  String ip = WiFi.localIP().toString();
  return
    "<?xml version=\"1.0\"?>"
    "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
    "<specVersion><major>1</major><minor>0</minor></specVersion>"
    "<URLBase>http://" + ip + "/</URLBase>"
    "<device>"
    "<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>"
    "<friendlyName>ESP32 Print Server</friendlyName>"
    "<manufacturer>ESP32 DIY</manufacturer>"
    "<manufacturerURL>http://" + ip + "/</manufacturerURL>"
    "<modelDescription>WiFi -&gt; USB print bridge</modelDescription>"
    "<modelName>ESP32 Print Server</modelName>"
    "<modelNumber>1</modelNumber>"
    "<UDN>uuid:" + deviceUuid + "</UDN>"
    "<presentationURL>/</presentationURL>"
    "</device>"
    "</root>";
}

void handleUpnpDescription() {
  configServer.send(200, "text/xml", buildUpnpDescriptionXml());
}

// ==================== SSDP: анонс и ответ на поиск ====================

void sendSsdpNotifyAlive() {
  if (WiFi.status() != WL_CONNECTED) return;

  String location = "http://" + WiFi.localIP().toString() + String(UPNP_DESCRIPTION_PATH);
  String uuidUsn = "uuid:" + deviceUuid;

  // Два отдельных уведомления — под upnp:rootdevice (по этому ключу
  // Windows обычно строит список устройств в "Сети") и "голый" UDN.
  const char *ntList[2]    = {"upnp:rootdevice", uuidUsn.c_str()};
  String usnList[2] = {uuidUsn + "::upnp:rootdevice", uuidUsn};

  for (int i = 0; i < 2; i++) {
    String msg =
      "NOTIFY * HTTP/1.1\r\n"
      "HOST: 239.255.255.250:1900\r\n"
      "CACHE-CONTROL: max-age=1800\r\n"
      "LOCATION: " + location + "\r\n"
      "NT: " + String(ntList[i]) + "\r\n"
      "NTS: ssdp:alive\r\n"
      "SERVER: ESP32/1.0 UPnP/1.0 ESP32-Print-Server/1.0\r\n"
      "USN: " + usnList[i] + "\r\n"
      "\r\n";
    ssdpUdp.beginPacket(SSDP_MULTICAST_IP, SSDP_PORT);
    ssdpUdp.write((const uint8_t *)msg.c_str(), msg.length());
    ssdpUdp.endPacket();
  }

  Serial.println("[SSDP] Отправлен NOTIFY ssdp:alive.");
}

void setupSsdp() {
  if (!ssdpUdp.beginMulticast(SSDP_MULTICAST_IP, SSDP_PORT)) {
    Serial.println("[SSDP] Не удалось начать multicast-прослушивание.");
  } else {
    Serial.printf("[SSDP] Слушаю multicast %s:%u, UUID=%s\n",
                  SSDP_MULTICAST_IP.toString().c_str(), SSDP_PORT, deviceUuid.c_str());
  }
  sendSsdpNotifyAlive();
  lastSsdpAnnounceMs = millis();
}

// Вызывать часто из loop(). Неблокирующая: проверяет входящие M-SEARCH
// и периодически (раз в SSDP_REANNOUNCE_INTERVAL_MS) шлёт повторный
// ssdp:alive, как того требует протокол (объявления имеют TTL/max-age
// и должны обновляться, иначе клиенты сочтут устройство пропавшим).
void loopSsdp() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastSsdpAnnounceMs > SSDP_REANNOUNCE_INTERVAL_MS) {
    sendSsdpNotifyAlive();
    lastSsdpAnnounceMs = millis();
  }

  int packetSize = ssdpUdp.parsePacket();
  if (packetSize <= 0) return;

  IPAddress remoteIp = ssdpUdp.remoteIP();
  uint16_t remotePort = ssdpUdp.remotePort();

  int len = ssdpUdp.read(ssdpRecvBuf, sizeof(ssdpRecvBuf) - 1);
  if (len <= 0) return;
  ssdpRecvBuf[len] = '\0';

  if (strncmp(ssdpRecvBuf, "M-SEARCH", 8) != 0) return; // не поисковый запрос — игнорируем

  // Простой построчный разбор заголовка ST: (Search Target).
  String st = "";
  char *stLine = strstr(ssdpRecvBuf, "ST:");
  if (!stLine) stLine = strstr(ssdpRecvBuf, "st:");
  if (stLine) {
    stLine += 3;
    while (*stLine == ' ') stLine++;
    char *lineEnd = strstr(stLine, "\r\n");
    st = lineEnd ? String(stLine).substring(0, lineEnd - stLine) : String(stLine);
    st.trim();
  }

  String myUuidSt = "uuid:" + deviceUuid;
  bool interested = st == "ssdp:all" || st == "upnp:rootdevice" ||
                    st.indexOf("device:Basic") >= 0 || st == myUuidSt;
  if (!interested) return;

  Serial.printf("[SSDP] M-SEARCH от %s:%u, ST=\"%s\" -> отвечаю\n",
                remoteIp.toString().c_str(), remotePort, st.c_str());

  String location = "http://" + WiFi.localIP().toString() + String(UPNP_DESCRIPTION_PATH);
  String usn = (st == myUuidSt) ? myUuidSt : (myUuidSt + "::" + st);

  String response =
    "HTTP/1.1 200 OK\r\n"
    "CACHE-CONTROL: max-age=1800\r\n"
    "EXT:\r\n"
    "LOCATION: " + location + "\r\n"
    "SERVER: ESP32/1.0 UPnP/1.0 ESP32-Print-Server/1.0\r\n"
    "ST: " + st + "\r\n"
    "USN: " + usn + "\r\n"
    "\r\n";

  ssdpUdp.beginPacket(remoteIp, remotePort);
  ssdpUdp.write((const uint8_t *)response.c_str(), response.length());
  ssdpUdp.endPacket();
}


// ==================== USB CALLBACKS ====================

void onUsbDeviceConnected(const EspUsbHostDeviceInfo &device) {
  Serial.println("========================================");
  Serial.println("[USB] Обнаружено устройство:");
  espUsbHostPrint(device);

  if (!device.supported) {
    Serial.println("[USB] Устройство не поддерживается библиотекой на уровне enumeration.");
  }

  if (SCAN_ONLY_MODE) {
    Serial.println("[USB] SCAN_ONLY_MODE включен -> подробный дамп интерфейсов:");
    usb.printAllDeviceInfo(Serial);
    Serial.println("[USB] Найдите строку с bInterfaceClass = 0x07 (Printer),");
    Serial.println("      впишите её номер интерфейса в PRINTER_INTERFACE_NUMBER.");
    return;
  }

  bool ok = usb.vendorOpen(device.address, PRINTER_INTERFACE_NUMBER,
                            ESP_USB_HOST_VENDOR_READ_ON_DEMAND);

  if (ok) {
    printerAddress = device.address;
    printerReady = true;
    printerHasIssue = false; // сбрасываем — реальный статус узнаем при первом опросе
    lastPrinterStatusRaw = "";
    lastLoggedPjlCode = -999; // чтобы первый же опрос после подключения точно попал в лог

    // Настоящие строковые дескрипторы USB — те же самые, что видно в
    // самом первом логе "Обнаружено устройство" (espUsbHostPrint) и в
    // демо-скетче EspUsbHostDeviceInfo.ino. В отличие от PJL DISPLAY
    // (который у HP склеивает слова без пробелов) — это чистое,
    // читаемое имя прямо от производителя, без единого сетевого
    // запроса к принтеру.
    printerManufacturer = String(device.manufacturer);
    printerProduct = String(device.product);
    printerSerial = String(device.serial);

    // ВАЖНО: bulk OUT transfer, чья длина кратна максимальному размеру
    // пакета эндпоинта (MPS), не завершается сама по себе без
    // дополнительного zero-length пакета (ZLP) — так работает сам
    // протокол USB. У нас CHUNK_SIZE=4096 при MPS=64 — это ровно
    // кратно (4096 = 64*64), то есть КАЖДЫЙ полный чанк данных попадает
    // в эту пограничную ситуацию. Включаем автоматическую отправку ZLP,
    // чтобы библиотека сама решала эту неоднозначность.
    usb.vendorSetAutoZlp(true);

    // ВАЖНО: простой синхронный vendorWrite() внутри себя ждёт
    // подтверждения ровно 1000мс и это никак не настроить через его
    // сигнатуру. На крупных заданиях (особенно с изображениями)
    // принтер иногда "задумывается" на внутреннюю обработку дольше
    // секунды — и раньше это выглядело как "ошибка записи", хотя
    // принтер был всего лишь занят. Переходим на асинхронную очередь
    // записи библиотеки — у неё таймаут ожидания свободного слота
    // задаётся явно (см. sendToPrinter), что даёт принтеру гораздо
    // больше времени "отдышаться", не рискуя при этом задвоить уже
    // ушедшие на шину данные (в отличие от прежнего retry с нуля).
    if (!usb.vendorWriteQueueBegin(4, CHUNK_SIZE, device.address)) {
      Serial.println("[USB] Не удалось создать очередь записи — печать может быть менее устойчивой "
                      "к медленным ответам принтера.");
    }
    lastKnownWriteErrors = 0;

    uint8_t inEp = usb.vendorInEndpoint(device.address);
    Serial.printf("[USB] Принтер готов. address=%u interface=%u OUT EP=0x%02X (MPS=%u) "
                  "IN EP=0x%02X (MPS=%u)\n",
                  device.address, PRINTER_INTERFACE_NUMBER,
                  usb.vendorOutEndpoint(device.address),
                  usb.vendorOutPacketSize(device.address),
                  inEp, usb.vendorInPacketSize(device.address));
    if (inEp == 0) {
      Serial.println("[USB] !! У интерфейса нет bulk IN эндпоинта — чтение статуса "
                      "(PJL, vendorReadSync) физически невозможно, только отправка данных.");
    }
  } else {
    Serial.println("[USB] Не удалось открыть интерфейс принтера.");
    Serial.println("      Проверьте PRINTER_INTERFACE_NUMBER (включите SCAN_ONLY_MODE).");
  }

  recomputeIdleLedState();
}

void onUsbDeviceDisconnected(const EspUsbHostDeviceInfo &device) {
  Serial.println("[USB] Устройство отключено.");
  if (device.address == printerAddress) {
    usb.vendorWriteQueueEnd(printerAddress);
    printerReady = false;
    printerAddress = 0;
    printerHasIssue = false;
    printerIsBusyPrinting = false;
    printerManufacturer = "";
    printerProduct = "";
    printerSerial = "";
    recomputeIdleLedState();
  }
}

bool sendToPrinter(const uint8_t *data, size_t len) {
  // ВАЖНО: раньше здесь было "return true", т.е. при отсутствующем
  // принтере данные молча отбрасывались, а вызывающий код (и,
  // соответственно, ОС) думал, что всё прошло успешно — из-за этого
  // Windows показывала задание как "Напечатано", хотя физически оно
  // никуда не дошло. Теперь честно возвращаем false: RAW/LPR-обработчики
  // уже проверяют результат этого вызова и прерывают job/рвут
  // соединение при ошибке — так ОС увидит обрыв передачи и, как при
  // реально отключённом сетевом принтере, не должна пометить задание
  // как успешно напечатанное.
  if (!printerReady) return false;
  if (len == 0) return true;

  // ВАЖНО (пересмотрено дважды):
  // 1) Раньше здесь был retry поверх простого vendorWrite() — при
  //    неудаче мы несколько раз слепо повторяли отправку ТОГО ЖЕ
  //    чанка целиком. Но при таймауте часть данных чанка МОГЛА уже
  //    физически уйти на устройство на уровне USB-пакетов — "false"
  //    от vendorWrite() означает лишь "мы не дождались подтверждения",
  //    а не "ничего не долетело". Повтор с начала в таком случае
  //    склеивал на входе принтера задвоенный, испорченный кусок прямо
  //    посреди потока — задание выглядело успешным, но печатался
  //    битый файл (так ломались JPEG-картинки в PCL XL заданиях, без
  //    единой видимой ошибки в логе).
  // 2) Простой vendorWrite() имеет зашитый внутри таймаут ожидания
  //    подтверждения ровно 1000мс, не настраиваемый через его
  //    сигнатуру — принтер, задумавшийся дольше секунды на разбор
  //    растра/JPEG, воспринимался как "ошибка", хотя был всего лишь
  //    занят. Переходим на асинхронную очередь записи библиотеки
  //    (vendorWriteQueueBegin/Acquire/Submit) — у неё ожидание
  //    свободного слота задаётся явным таймаутом, и submit() не
  //    блокирует поток, так что задвоение исключено в принципе:
  //    каждый чанк копируется в СВОЙ собственный буфер очереди и
  //    отправляется ровно один раз.
  const uint32_t ACQUIRE_TIMEOUT_MS = 10000;

  size_t offset = 0;
  while (offset < len) {
    size_t capacity = 0;
    uint8_t *buf = usb.vendorWriteAcquire(&capacity, ACQUIRE_TIMEOUT_MS, printerAddress);
    if (!buf) {
      Serial.println("[USB] Таймаут ожидания свободного слота очереди записи — прерываем задание.");
      return false;
    }

    size_t n = (len - offset) < capacity ? (len - offset) : capacity;
    memcpy(buf, data + offset, n);

    if (!usb.vendorWriteSubmit(buf, n, printerAddress)) {
      Serial.println("[USB] Не удалось поставить чанк в очередь записи — прерываем задание.");
      usb.vendorWriteRelease(buf, printerAddress);
      return false;
    }
    offset += n;

    // submit() не ждёт завершения передачи, поэтому реальный сбой шины
    // (устройство ответило ошибкой уже ПОСЛЕ того как мы отправили
    // следующий чанк) виден только здесь — по росту счётчика ошибок
    // относительно последней известной нам базовой линии.
    EspUsbHostVendorWriteStats stats = usb.vendorWriteStats(printerAddress);
    if (stats.errors > lastKnownWriteErrors) {
      Serial.printf("[USB] Обнаружена ошибка на шине (errors=%u) — прерываем задание.\n",
                    (unsigned)stats.errors);
      lastKnownWriteErrors = stats.errors;
      return false;
    }
  }
  return true;
}

// Дожидается, пока ранее поставленные в очередь передачи реально
// завершатся на шине, и сверяет итоговый счётчик ошибок. Обязательно
// вызывать в конце КАЖДОГО задания печати (после того как клиент
// прислал все данные) — иначе можно посчитать задание успешным, пока
// хвост данных ещё физически летит по USB или вот-вот провалится.
bool flushPrinterQueue(uint32_t timeoutMs) {
  if (!printerReady) return false;
  bool flushed = usb.vendorWriteFlush(timeoutMs, printerAddress);
  EspUsbHostVendorWriteStats stats = usb.vendorWriteStats(printerAddress);
  bool hadNewErrors = stats.errors > lastKnownWriteErrors;
  lastKnownWriteErrors = stats.errors;
  if (!flushed) {
    Serial.println("[USB] Таймаут ожидания завершения очереди записи в конце задания.");
  }
  if (hadNewErrors) {
    Serial.printf("[USB] По завершении задания в очереди обнаружены ошибки (errors=%u).\n",
                  (unsigned)stats.errors);
  }
  return flushed && !hadNewErrors;
}

// Можно ли сейчас принимать новое задание печати: принтер должен быть
// подключён, без известных проблем (бумага/замятие/статус не определён)
// и не занят печатью другого задания (по последнему PJL-опросу).
bool printerCanAcceptJob() {
  return printerReady && !printerHasIssue && !printerIsBusyPrinting;
}

// ==================== PJL: опрос статуса принтера ====================
//
// ВНИМАНИЕ: формат ответа @PJL INFO STATUS проверен вживую на конкретно
// используемом при разработке PJL-принтере — реальные примеры: "CODE=0 DISPLAY=\"Non HP supplyin
// use\" ONLINE=TRUE" (норма), "CODE=41900 DISPLAY=\"Load paper\"",
// "CODE=40021 DISPLAY=\"Door open\"", "CODE=10023 DISPLAY=\"Printingdocument\"".
// Решение "можно ли принять новое задание" опирается ИСКЛЮЧИТЕЛЬНО на
// числовой CODE: 0 = всё в порядке, любое другое значение — нет.
const uint32_t PRINTER_STATUS_INTERVAL_MS = 2000; // опрашивать примерно раз в 2 секунды
uint32_t lastPrinterStatusCheckMs = 0;

bool queryPrinterStatus(String &statusOut) {
  if (!printerReady) return false;

  if (usb.vendorInEndpoint(printerAddress) == 0) {
    Serial.println("[PJL] Пропускаю опрос: у интерфейса нет bulk IN эндпоинта.");
    return false;
  }

  const char pjlQuery[] = "\x1b%-12345X@PJL INFO STATUS\r\n\x1b%-12345X";
  bool wrote = usb.vendorWrite((const uint8_t *)pjlQuery, strlen(pjlQuery), printerAddress);
  if (!wrote) {
    Serial.println("[PJL] Ошибка vendorWrite при отправке запроса статуса.");
    return false;
  }

  uint8_t resp[512];
  size_t len = 0;
  bool ok = usb.vendorReadSync(resp, sizeof(resp), &len, 1000, printerAddress);
  if (!ok || len == 0) return false;

  statusOut = String((const char *)resp, len);
  return true;
}

// Достаёт числовое значение CODE=... из ответа @PJL INFO STATUS.
// CODE=0 — всё в порядке; любое другое значение — что-то мешает печати.
// Возвращает -1, если CODE вообще не нашёлся (трактуем как "не определён").
long extractPjlStatusCode(const String &status) {
  int idx = status.indexOf("CODE=");
  if (idx < 0) return -1;
  idx += strlen("CODE=");
  int end = idx;
  while (end < (int)status.length() && isDigit(status[end])) end++;
  if (end == idx) return -1;
  return status.substring(idx, end).toInt();
}

// "Занят печатью" определяем по DISPLAY-тексту статуса — принтер
// склеивает слова без пробелов ("Printingdocument"), поэтому ищем
// именно "PRINTING" как подстроку, без пробела после неё. Используется
// только для более информативного лога/различения "занят" vs "ошибка" —
// на решение "можно ли принять задание" не влияет (там важен сам факт
// CODE != 0, независимо от причины).
bool statusTextIndicatesBusy(const String &status) {
  String s = status;
  s.toUpperCase();
  return s.indexOf("PRINTING") >= 0;
}

// Вызывать периодически из loop(). Сам решает, не рано ли ещё опрашивать
// (см. PRINTER_STATUS_INTERVAL_MS), поэтому дёргать можно на каждой
// итерации без опасений. В лог пишет только при смене CODE — иначе при
// опросе раз в 2 секунды Serial захлёбывался бы повторами одного и
// того же значения.
void checkPrinterStatusPeriodic() {
  if (!printerReady) return;
  if (millis() - lastPrinterStatusCheckMs < PRINTER_STATUS_INTERVAL_MS) return;
  lastPrinterStatusCheckMs = millis();

  String status;
  bool ok = queryPrinterStatus(status);

  if (!ok) {
    // -2 — отдельный "код" для "нет ответа", отличный от -1 (CODE не
    // нашёлся в реально полученном тексте) и от любого настоящего CODE.
    if (lastLoggedPjlCode != -2) {
      Serial.println("[PJL] Нет ответа на @PJL INFO STATUS — статус не определён.");
      lastLoggedPjlCode = -2;
    }
    if (!printerHasIssue) {
      printerHasIssue = true;
      recomputeIdleLedState();
    }
    return;
  }

  lastPrinterStatusRaw = status;
  long code = extractPjlStatusCode(status);
  bool issue = (code != 0); // CODE=0 — единственное значение, означающее "всё в порядке"
  bool busy = statusTextIndicatesBusy(status);

  if (code != lastLoggedPjlCode) {
    Serial.printf("[PJL] CODE=%ld -> %s%s | Сырой ответ (%u байт): %s\n",
                  code, issue ? "ЕСТЬ ПРОБЛЕМА" : "в норме",
                  busy ? " (печатает)" : "", status.length(), status.c_str());
    lastLoggedPjlCode = code;
  }

  printerIsBusyPrinting = busy;

  if (issue != printerHasIssue) {
    printerHasIssue = issue;
    recomputeIdleLedState();
  }
}

// ==================== WiFi (штатное подключение) ====================

// Пытается подключиться сохранёнными данными. true = успех.
bool connectWiFi() {
  if (savedSsid.length() == 0) {
    Serial.println("[WiFi] Сохранённых настроек нет.");
    return false;
  }

  Serial.printf("[WiFi] Подключение к \"%s\" ...\n", savedSsid.c_str());
  setLedState(LED_STATE_WIFI_CONNECTING);
  WiFi.disconnect(true); // сбрасываем любое предыдущее состояние (в т.ч. EAP), прежде чем настраивать заново
  WiFi.mode(WIFI_STA);
  delay(100); // без паузы radio ещё не готово, macAddress() вернёт нули
  Serial.printf("[WiFi] MAC-адрес платы: %s\n", WiFi.macAddress().c_str());

  if (savedIsEnterprise) {
    // WPA2-Enterprise (PEAP/TTLS + MSCHAPv2) — тот же механизм, что и у
    // корпоративных/университетских сетей вроде eduroam. Пароль от точки
    // доступа тут не нужен, WiFi.begin() вызывается без него — вся
    // аутентификация идёт через отдельно настроенный EAP-клиент.
    String identity = savedEapIdentity.length() > 0 ? savedEapIdentity : savedEapUsername;
    esp_eap_client_set_identity((const uint8_t *)identity.c_str(), identity.length());
    esp_eap_client_set_username((const uint8_t *)savedEapUsername.c_str(), savedEapUsername.length());
    esp_eap_client_set_password((const uint8_t *)savedPassword.c_str(), savedPassword.length());
    esp_err_t entErr = esp_wifi_sta_enterprise_enable();
    if (entErr != ESP_OK) {
      Serial.printf("[WiFi] esp_wifi_sta_enterprise_enable() вернул ошибку: %s\n", esp_err_to_name(entErr));
    }
    WiFi.begin(savedSsid.c_str());
  } else {
    // Обычная домашняя сеть WPA2-Personal — на случай, если до этого
    // подключались к Enterprise-сети, явно выключаем EAP-клиент, иначе
    // его состояние может помешать обычной PSK-аутентификации.
    esp_wifi_sta_enterprise_disable();
    WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
  }

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    checkResetButton(); // также обновляет LED
    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Таймаут подключения.");
      return false;
    }
  }

  Serial.println();
  Serial.printf("[WiFi] Подключено. IP: %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("jetdirect", "tcp", RAW_PORT);
    MDNS.addService("printer", "tcp", LPR_PORT);
    Serial.printf("[mDNS] Доступно как %s.local\n", MDNS_HOSTNAME);
  } else {
    Serial.println("[mDNS] Не удалось запустить mDNS (не критично).");
  }

  ensurePortalRunning();
  setupSsdp();

  recomputeIdleLedState();
  return true;
}

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ЧТЕНИЯ (RAW/LPR) ====================

bool readExact(WiFiClient &client, uint8_t *buf, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len) {
    if (!client.connected() && client.available() == 0) return false;
    int avail = client.available();
    if (avail > 0) {
      int toRead = (size_t)avail > (len - got) ? (int)(len - got) : avail;
      int n = client.read(buf + got, toRead);
      if (n > 0) {
        got += n;
        start = millis();
        continue;
      }
    }
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

int readLine(WiFiClient &client, char *buf, size_t maxLen, uint32_t timeoutMs) {
  size_t pos = 0;
  uint32_t start = millis();
  while (true) {
    if (!client.connected() && client.available() == 0) return -1;
    int avail = client.available();
    if (avail > 0) {
      int c = client.read();
      if (c < 0) continue;
      if (c == '\n') {
        buf[pos] = '\0';
        return (int)pos;
      }
      if (pos < maxLen - 1) {
        buf[pos++] = (char)c;
      }
      start = millis();
    } else {
      if (millis() - start > timeoutMs) return -1;
      delay(1);
    }
  }
}

void sendAck(WiFiClient &client) {
  uint8_t zero = 0x00;
  client.write(&zero, 1);
}

// ==================== RAW / JetDirect (порт 9100) ====================

void handleRawClient(WiFiClient &client) {
  Serial.printf("[RAW] Клиент подключился: %s\n", client.remoteIP().toString().c_str());
  setLedState(LED_STATE_PRINTING);

  uint32_t jobBytes = 0;
  uint32_t lastDataMs = millis();

  while (client.connected()) {
    int avail = client.available();
    if (avail > 0) {
      int toRead = avail > (int)CHUNK_SIZE ? (int)CHUNK_SIZE : avail;
      int n = client.read(chunkBuffer, toRead);
      if (n > 0) {
        lastDataMs = millis();
        jobBytes += n;
        if (!sendToPrinter(chunkBuffer, n)) {
          Serial.println("[USB] Ошибка записи в принтер, прерываем job.");
          break;
        }
      }
    } else {
      if (millis() - lastDataMs > 15000) {
        Serial.println("[RAW] Таймаут простоя соединения, закрываем.");
        break;
      }
      delay(2);
    }
    updateLed();
  }

  client.stop();

  // Ждём реального завершения всех уже отправленных в очередь передач
  // и сверяем итоговый статус ошибок — до этого момента "jobBytes"
  // отражает только то, что мы прочитали из TCP, а не то, что точно
  // без ошибок доехало до принтера по USB.
  bool flushOk = flushPrinterQueue(10000);
  Serial.printf("[RAW] Клиент отключился. Job: %u байт. %s\n", jobBytes,
                flushOk ? "Очередь USB подтвердила успешную отправку." :
                          "Внимание: очередь USB сообщила об ошибке/таймауте на хвосте задания.");
  recomputeIdleLedState();
}

// ==================== LPR / LPD, RFC 1179 (порт 515) ====================

void lprHandleReceiveJob(WiFiClient &client) {
  char lineBuf[256];

  int qlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
  if (qlen < 0) {
    Serial.println("[LPR] Таймаут при чтении имени очереди.");
    return;
  }
  Serial.printf("[LPR] Очередь: \"%s\"\n", lineBuf);
  sendAck(client);

  uint32_t jobBytes = 0;

  while (client.connected() || client.available() > 0) {
    uint8_t subcmd = 0;
    if (!readExact(client, &subcmd, 1, IO_TIMEOUT_MS)) {
      break;
    }

    if (subcmd == 0x01) {
      Serial.println("[LPR] Получена команда Abort job.");
      sendAck(client);
      break;

    } else if (subcmd == 0x02) {
      int hlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
      if (hlen < 0) { Serial.println("[LPR] Таймаут заголовка control file."); break; }

      long size = atol(lineBuf);
      Serial.printf("[LPR] Control file, размер=%ld: %s\n", size, lineBuf);
      sendAck(client);

      long remaining = size;
      while (remaining > 0) {
        size_t toRead = remaining > (long)CHUNK_SIZE ? CHUNK_SIZE : (size_t)remaining;
        if (!readExact(client, chunkBuffer, toRead, IO_TIMEOUT_MS)) {
          Serial.println("[LPR] Обрыв при чтении control file.");
          client.stop();
          return;
        }
        remaining -= toRead;
        updateLed();
      }

      uint8_t term;
      readExact(client, &term, 1, IO_TIMEOUT_MS);
      sendAck(client);

    } else if (subcmd == 0x03) {
      int hlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
      if (hlen < 0) { Serial.println("[LPR] Таймаут заголовка data file."); break; }

      long size = atol(lineBuf);
      Serial.printf("[LPR] Data file (задание печати), размер=%ld: %s\n", size, lineBuf);
      sendAck(client);
      setLedState(LED_STATE_PRINTING);

      // ВНИМАНИЕ: встроенный LPR-клиент Windows (LPR Port Monitor) известен
      // тем, что не знает реальный размер job'а заранее и часто присылает
      // здесь неправдоподобно огромное/мусорное число, просто стримя данные
      // и закрывая соединение по завершении. Поэтому заявленному размеру
      // доверяем только если он выглядит разумно; иначе переходим в
      // потоковый режим — читаем и сразу форвардим в принтер, пока клиент
      // не закроет соединение (или не истечёт таймаут простоя).
      const long LPR_SANE_SIZE_LIMIT = 200L * 1024 * 1024; // 200 МБ
      bool sizeIsSane = (size > 0 && size <= LPR_SANE_SIZE_LIMIT);

      if (!sizeIsSane) {
        Serial.println("[LPR] Заявленный размер выглядит некорректным (типичная особенность "
                        "Windows LPR-клиента) — переключаюсь в потоковый режим.");
      }

      uint32_t fileBytes = 0;
      bool ok = true;

      if (sizeIsSane) {
        long remaining = size;
        while (remaining > 0 && ok) {
          int avail = client.available();
          if (avail > 0) {
            size_t toRead = (size_t)avail > (size_t)remaining ? (size_t)remaining : (size_t)avail;
            if (toRead > CHUNK_SIZE) toRead = CHUNK_SIZE;
            int n = client.read(chunkBuffer, toRead);
            if (n > 0) {
              if (!sendToPrinter(chunkBuffer, n)) {
                Serial.println("[USB] Ошибка записи в принтер, прерываем job.");
                ok = false;
                break;
              }
              remaining -= n;
              fileBytes += n;
            }
          } else {
            if (!client.connected()) {
              Serial.println("[LPR] Соединение закрыто раньше заявленного размера — "
                              "печатаем то, что успели получить.");
              break;
            }
            delay(1);
          }
          updateLed();
        }
        // Заявленный терминатор 0x00 ждём с коротким таймаутом — если его
        // не будет (клиент уже закрыл сокет), это не считаем ошибкой.
        uint8_t term;
        readExact(client, &term, 1, 1000);
        sendAck(client);
      } else {
        // Потоковый режим: льём всё, что приходит, пока клиент не отключится
        // или не наступит пауза дольше IO_TIMEOUT_MS.
        uint32_t lastDataMs = millis();
        while (client.connected() || client.available() > 0) {
          int avail = client.available();
          if (avail > 0) {
            int toRead = avail > (int)CHUNK_SIZE ? (int)CHUNK_SIZE : avail;
            int n = client.read(chunkBuffer, toRead);
            if (n > 0) {
              lastDataMs = millis();
              fileBytes += n;
              if (!sendToPrinter(chunkBuffer, n)) {
                Serial.println("[USB] Ошибка записи в принтер, прерываем job.");
                ok = false;
                break;
              }
            }
          } else {
            if (millis() - lastDataMs > IO_TIMEOUT_MS) {
              Serial.println("[LPR] Таймаут простоя в потоковом режиме, завершаем job.");
              break;
            }
            delay(2);
          }
          updateLed();
        }
        // Терминатор/ACK в этом режиме не гарантирован протоколом клиента —
        // не настаиваем на нём.
      }

      jobBytes += fileBytes;
      Serial.printf("[LPR] Data file принят: %u байт.\n", fileBytes);

      if (!ok) break;

    } else {
      Serial.printf("[LPR] Неизвестная подкоманда 0x%02X, закрываем.\n", subcmd);
      break;
    }
  }

  Serial.printf("[LPR] Job завершён. Байт задания печати: %u\n", jobBytes);

  // Ждём реального завершения всех уже отправленных в очередь передач
  // и сверяем итоговый статус ошибок — до этого момента jobBytes
  // отражает только то, что мы прочитали из сети, а не то, что точно
  // без ошибок доехало до принтера по USB.
  if (jobBytes > 0) {
    bool flushOk = flushPrinterQueue(10000);
    Serial.println(flushOk ? "[LPR] Очередь USB подтвердила успешную отправку." :
                              "[LPR] Внимание: очередь USB сообщила об ошибке/таймауте на хвосте задания.");
  }
}

void handleLprClient(WiFiClient &client) {
  Serial.printf("[LPR] Клиент подключился: %s\n", client.remoteIP().toString().c_str());

  uint8_t cmd = 0;
  if (!readExact(client, &cmd, 1, IO_TIMEOUT_MS)) {
    Serial.println("[LPR] Таймаут при чтении команды.");
    client.stop();
    return;
  }

  if (cmd == 0x02) {
    lprHandleReceiveJob(client);
  } else {
    Serial.printf("[LPR] Команда 0x%02X не поддерживается, закрываем соединение.\n", cmd);
  }

  client.stop();
  Serial.println("[LPR] Соединение закрыто.");
  recomputeIdleLedState();
}

// ==================== SETUP / LOOP ====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32 Print Server (WiFi -> USB) ===");

  pixel.begin();
  pixel.setBrightness(RGB_BRIGHTNESS);
  pixel.show(); // выключен по умолчанию

#if ENABLE_OLED_DISPLAY
  initOled();
#endif

  setupResetButton();

  usb.onDeviceConnected(onUsbDeviceConnected);
  usb.onDeviceDisconnected(onUsbDeviceDisconnected);

  if (!usb.begin()) {
    Serial.printf("[USB] usb.begin() failed: %s\n", usb.lastErrorName());
  } else {
    Serial.println("[USB] USB Host запущен, ждём подключения принтера...");
  }

  loadWifiCredentials();
  loadUiLanguage(); // читаем сохранённый язык интерфейса (по умолчанию — русский)
  loadPortalPassword(); // читаем сохранённый пароль портала (по умолчанию — пусто, защита выключена)
  deviceUuid = loadOrCreateDeviceUuid(); // нужен и для UPnP UDN, и ни от чего сетевого не зависит

  // ВАЖНО: WiFi.mode() должен быть вызван хотя бы раз, прежде чем
  // WiFiServer::begin() сможет создать слушающий сокет — иначе
  // внутренний сетевой стек (lwIP/FreeRTOS-очереди, на которых он
  // держится) ещё не инициализирован, и begin() падает с
  // "assert failed: xQueueSemaphoreTake" прямо на старте. Включаем
  // сразу оба режима (AP+STA) — это не мешает дальнейшей логике
  // (connectWiFi() и startConfigPortal() ниже переключают режим
  // повторно под свои нужды), зато гарантирует, что стек уже жив
  // к моменту запуска серверов печати.
  WiFi.mode(WIFI_AP_STA);

  // Поднимаем серверы печати ДО попытки подключения к домашнему WiFi —
  // так печать через Raw/LPR работает и в собственной точке доступа
  // платы (192.168.4.1), даже если домашняя сеть ещё не настроена
  // (или недоступна) и startConfigPortal() ниже уйдёт в свой
  // блокирующий цикл на неопределённое время.
  rawServer.begin();
  rawServer.setNoDelay(true);
  Serial.printf("[TCP] RAW print server слушает порт %u\n", RAW_PORT);

  lprServer.begin();
  lprServer.setNoDelay(true);
  Serial.printf("[TCP] LPR/LPD print server слушает порт %u\n", LPR_PORT);

  bool connected = connectWiFi();
  if (!connected) {
    // Блокирующий вызов — вернётся только через ESP.restart()
    // после успешного сохранения новых настроек. Печать при этом уже
    // работает через собственную точку доступа — см. servePrintClients()
    // внутри цикла startConfigPortal().
    startConfigPortal();
  }
}

// Общий приём и обработка клиентов RAW/LPR — вызывается и из обычного
// loop() (когда подключены к домашней сети), и из блокирующего цикла
// startConfigPortal() (когда WiFi ещё не настроен и плата сидит в
// собственной точке доступа 192.168.4.1). Так печать работает в любом
// режиме — не только после успешного подключения к чужой сети.
void servePrintClients() {
  WiFiClient rawClient = rawServer.available();
  if (rawClient) {
    if (printerCanAcceptJob()) {
      handleRawClient(rawClient);
    } else {
      Serial.println("[RAW] Принтер не готов принять задание (ошибка/занят/не подключён) — отклоняю соединение.");
      rawClient.stop();
    }
  }

  WiFiClient lprClient = lprServer.available();
  if (lprClient) {
    if (printerCanAcceptJob()) {
      handleLprClient(lprClient);
    } else {
      Serial.println("[LPR] Принтер не готов принять задание (ошибка/занят/не подключён) — отклоняю соединение.");
      lprClient.stop();
    }
  }
}

void loop() {
  checkResetButton(); // также обновляет LED каждую итерацию

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Соединение потеряно, переподключаемся...");
    if (!connectWiFi()) {
      startConfigPortal(); // не вернётся, кроме как через reboot
    }
  }

  configServer.handleClient(); // портал теперь работает постоянно, не только в AP-режиме
  loopSsdp();                  // SSDP: отвечает на M-SEARCH, периодически шлёт ssdp:alive

  checkPrinterStatusPeriodic(); // раз в PRINTER_STATUS_INTERVAL_MS опрашивает @PJL INFO STATUS

  servePrintClients();

#if ENABLE_OLED_DISPLAY
  updateOledIp(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
#endif

  delay(5);
}
