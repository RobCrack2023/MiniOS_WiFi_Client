/*
 * MiniOS WiFi Client
 * Versión ligera para conectar al backend centralizado
 *
 * Características:
 * - Conexión WebSocket al backend
 * - Identificación por MAC Address
 * - Control GPIO remoto
 * - Sensores DHT e I2C (AHT20, BMP280)
 * - Actualizaciones OTA
 * - Soporte multi-plataforma: ESP32, ESP32-S3, ESP32-C3
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

// Incluir NeoPixel solo si el board lo soporta
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
  #include <Adafruit_NeoPixel.h>
  #define HAS_RGB_LED true
#else
  #define HAS_RGB_LED false
#endif

// ============================================
// DETECCIÓN AUTOMÁTICA DE PLATAFORMA
// ============================================

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define BOARD_MODEL "ESP32-S3"
  #define BOARD_FAMILY "ESP32-S3"
  #define LED_RGB_PIN 48
  #define I2C_SDA_DEFAULT 21
  #define I2C_SCL_DEFAULT 22
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define BOARD_MODEL "ESP32-C3"
  #define BOARD_FAMILY "ESP32-C3"
  #define LED_RGB_PIN -1
  #define I2C_SDA_DEFAULT 8
  #define I2C_SCL_DEFAULT 9
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  #define BOARD_MODEL "ESP32-S2"
  #define BOARD_FAMILY "ESP32-S2"
  #define LED_RGB_PIN -1
  #define I2C_SDA_DEFAULT 21
  #define I2C_SCL_DEFAULT 22
#else
  #define BOARD_MODEL "ESP32"
  #define BOARD_FAMILY "ESP32"
  #define LED_RGB_PIN -1
  #define I2C_SDA_DEFAULT 21
  #define I2C_SCL_DEFAULT 22
#endif

// ============================================
// CONFIGURACIÓN
// ============================================

#define FIRMWARE_VERSION "2.0.0"

// Reglas horarias locales en formato POSIX (no IANA: newlib no trae tzdata).
// Chile continental: UTC-4, con horario de verano UTC-3 desde el primer sabado
// de septiembre hasta el primer sabado de abril.
#define POSIX_TZ "<-04>4<-03>,M9.1.6/24,M4.1.6/24"

// WiFi: NO poner las credenciales aqui. Este archivo esta en un repositorio
// publico y el historial de git conserva para siempre lo que se escriba.
// Se configuran una vez por el puerto serie y quedan guardadas en NVS:
//     wifi <ssid> <password>
String WIFI_SSID = "";
String WIFI_PASS = "";

// Backend
String BACKEND_HOST = "minios.iot-robotics.cl";  // Dominio del backend
int BACKEND_PORT = 443;  // Puerto 80 con Nginx, 443 con SSL

// Token compartido que exige el backend cuando tiene DEVICE_TOKEN configurado.
// Vacio = no se envia (compatible con un backend que aun no lo exija).
// Se configura por serie con:  token <valor>
String DEVICE_TOKEN = "";

// Hardware
#define MAX_GPIOS 20
#define MAX_DHT_SENSORS 4
#define MAX_ULTRASONIC_SENSORS 4
#define MAX_I2C_SENSORS 4

// LED de estado. Depende de la placa, no solo del chip: la C3 SuperMini lleva
// un LED azul normal en GPIO 8 que se enciende con nivel BAJO, mientras que el
// GPIO 2 del define anterior es una entrada ADC y un pin de strapping.
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define LED_STATUS     8
  #define LED_ACTIVE_LOW 1
#else
  #define LED_STATUS     2
  #define LED_ACTIVE_LOW 0
#endif
#define LED_RGB_COUNT 1

// Intentos de conexion WiFi antes de pedir credenciales nuevas
#define WIFI_MAX_ATTEMPTS 3

// Cuanto se espera a que escribas credenciales antes de volver a dormir.
// Solo aplica con Deep Sleep activo: si esta desactivado se espera sin limite,
// porque se asume que hay alguien delante del monitor serie.
#define WIFI_CONFIG_WINDOW_MS 300000  // 5 minutos

// Ahorro de energía - Deep Sleep
#define DEEP_SLEEP_ENABLED true
#define SENSOR_READ_TIMEOUT 15000  // 15s timeout para leer sensores y enviar

// ============================================
// CLASIFICACIÓN DE GPIOs POR PLATAFORMA
// ============================================

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // ESP32-S3: 45 GPIOs disponibles
  const int ANALOG_GPIOS[] = {1, 2, 4, 5, 6, 7};
  const int ANALOG_GPIOS_COUNT = 6;
  const int DIGITAL_GPIOS[] = {0, 3, 14, 15, 16, 17, 18, 19, 20, 21, 36, 37, 38, 39, 40, 41, 42, 45, 46};
  const int DIGITAL_GPIOS_COUNT = 19;
  const int I2C_GPIOS[] = {8, 9};  // 8=SDA, 9=SCL
  const int I2C_GPIOS_COUNT = 2;
  const int SPI_GPIOS[] = {10, 11, 12, 13};
  const int SPI_GPIOS_COUNT = 4;

#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  // ESP32-C3: 13 GPIOs disponibles (0-10, 18-21)
  // GPIO 2, 8, 9 son de boot/strapping - usar con precaución
  const int ANALOG_GPIOS[] = {0, 1, 2, 3, 4};  // ADC1_CH0-4
  const int ANALOG_GPIOS_COUNT = 5;
  // 18 y 19 fuera: son D-/D+ del USB nativo. En placas como la C3 SuperMini,
  // que no llevan puente USB-serie, usarlos deja el equipo sin puerto serie.
  // 20 y 21 son UART0 y quedan libres si se compila con USB CDC On Boot.
  const int DIGITAL_GPIOS[] = {5, 6, 7, 10, 20, 21};
  const int DIGITAL_GPIOS_COUNT = 6;
  const int I2C_GPIOS[] = {8, 9};  // SDA=8, SCL=9 (default C3)
  const int I2C_GPIOS_COUNT = 2;
  const int SPI_GPIOS[] = {2, 6, 7, 10};  // MISO, MOSI, SCK, CS
  const int SPI_GPIOS_COUNT = 4;

#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  // ESP32-S2
  const int ANALOG_GPIOS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  const int ANALOG_GPIOS_COUNT = 10;
  const int DIGITAL_GPIOS[] = {0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};
  const int DIGITAL_GPIOS_COUNT = 20;
  const int I2C_GPIOS[] = {8, 9};
  const int I2C_GPIOS_COUNT = 2;
  const int SPI_GPIOS[] = {10, 11, 12, 13};
  const int SPI_GPIOS_COUNT = 4;

#else
  // ESP32 estándar
  const int ANALOG_GPIOS[] = {32, 33, 34, 35, 36, 39};
  const int ANALOG_GPIOS_COUNT = 6;
  const int DIGITAL_GPIOS[] = {0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27};
  const int DIGITAL_GPIOS_COUNT = 18;
  const int I2C_GPIOS[] = {21, 22};  // SDA=21, SCL=22 (default)
  const int I2C_GPIOS_COUNT = 2;
  const int SPI_GPIOS[] = {5, 18, 19, 23};
  const int SPI_GPIOS_COUNT = 4;
#endif

// ============================================
// ESTRUCTURAS
// ============================================

enum GPIOMode {
  MODE_OUTPUT,
  MODE_INPUT,
  MODE_INPUT_PULLUP,
  MODE_PWM
};

struct GPIOConfig {
  int pin;
  GPIOMode mode;
  int value;           // Valor digital (0/1) o analógico RAW (0-4095)
  String name;
  bool active;
  bool loopEnabled;
  unsigned long loopInterval;
  unsigned long lastLoop;
  bool isAnalog;       // True si es pin analógico (ADC)
  // Fórmulas de conversión para sensores analógicos
  bool formulaEnabled;
  String formulaType;  // "4-20mA", "0-10V", "0-3.3V", "custom"
  float formulaMin;    // Valor mínimo de la escala
  float formulaMax;    // Valor máximo de la escala
  float convertedValue; // Valor después de aplicar fórmula
  String unit;         // Unidad (°C, %, PSI, etc.)
};

struct DHTConfig {
  int pin;
  DHT* sensor;
  String name;
  String type;
  float temperature;
  float humidity;
  bool active;
  unsigned long readInterval;
  unsigned long lastRead;
};

struct I2CConfig {
  int id;
  String name;
  String sensorType;     // "AHT20", "BMP280", "BME280"
  uint8_t i2cAddress;    // 0x38 (AHT20), 0x76/0x77 (BMP280)
  bool active;
  unsigned long readInterval;
  unsigned long lastRead;
  // Datos del sensor
  float temperature;
  float humidity;        // Solo AHT20/BME280
  float pressure;        // Solo BMP280/BME280 (hPa)
  float altitude;        // Solo BMP280/BME280 (m)
  // Punteros a objetos sensor
  Adafruit_AHTX0* aht;
  Adafruit_BMP280* bmp;
  Adafruit_BME280* bme;
};

struct UltrasonicConfig {
  int id;
  int trigPin;
  int echoPin;
  String name;
  int maxDistance;          // cm
  bool detectionEnabled;
  int triggerDistance;      // cm
  int triggerGpioPin;
  int triggerGpioValue;
  int triggerDuration;      // ms (0 = mantener)
  bool active;
  unsigned long readInterval;
  unsigned long lastRead;
  float lastDistance;
  bool triggered;
  unsigned long triggerStartTime;
  // Buffer para detección de movimiento
  float distanceBuffer[10];  // Últimas 10 lecturas para mejor análisis
  int bufferIndex;
  bool bufferFull;
  // Estado de detección actual
  unsigned long detectionStartTime;
  float currentSpeed;
};

// ============================================
// VARIABLES GLOBALES
// ============================================

WebSocketsClient webSocket;
Preferences preferences;

// LED RGB solo en plataformas compatibles
#if HAS_RGB_LED
  Adafruit_NeoPixel rgbLed(LED_RGB_COUNT, LED_RGB_PIN, NEO_GRB + NEO_KHZ800);
#endif

GPIOConfig gpios[MAX_GPIOS];
int gpioCount = 0;

DHTConfig dhtSensors[MAX_DHT_SENSORS];
int dhtCount = 0;

I2CConfig i2cSensors[MAX_I2C_SENSORS];
int i2cCount = 0;

UltrasonicConfig ultrasonicSensors[MAX_ULTRASONIC_SENSORS];
int ultrasonicCount = 0;

String deviceMac;
int deviceId = 0;
bool isRegistered = false;

// Sincronización de tiempo
String serverTimezone = "America/Santiago";
bool timeSync = false;
unsigned long lastTimeSync = 0;
const unsigned long TIME_SYNC_INTERVAL = 3600000;  // Re-sincronizar cada hora

// Control de Deep Sleep
bool disableDeepSleep = false;         // Flag para desactivar deep sleep (útil para OTA/debugging)
int wifiFailCount = 0;                 // Fallos de conexión WiFi consecutivos
unsigned long deepSleepDuration = 60;  // Segundos entre ciclos (configurable desde backend)

unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 5000;

unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 5000;

// OTA
bool otaInProgress = false;
int otaId = 0;
String otaFilename = "";
int otaFilesize = 0;
String otaChecksum = "";

// ============================================
// PROTOTIPOS DE FUNCIONES
// ============================================

void connectWiFi();
void waitForWifiCredentials();
const char* wifiStatusText(wl_status_t status);
void connectWebSocket();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void registerDevice();
void syncTimeWithBackend();
void handleWebSocketMessage(const char* payload);
void handleConfig(JsonDocument& doc);
void handleCommand(JsonDocument& doc);
void sendSensorData();
void readDHTSensors();
void readI2CSensors();
void processGpioLoops();
void startOTA(int id, String filename, int filesize, String checksum);
void reportOTAStatus(const char* status, String error);
void loadConfig();
void saveConfig();
void handleSerial();

// Funciones de validación GPIO
bool GPIO_InList(int pin, const int* list, int count);
bool GPIO_IsValid(int pin);
bool GPIO_IsAnalog(int pin);
bool GPIO_IsAppropriate(int pin, GPIOMode mode);
float applyFormula(int rawValue, GPIOConfig& gpio);

// Ultrasonic HC-SR04
float readUltrasonic(int trigPin, int echoPin, int maxDistance);
void readUltrasonicSensors();
void processUltrasonicTriggers();
float calculateMovementSpeed(int sensorIndex);
bool isObjectMoving(int sensorIndex);
// LED RGB
void blinkRGB(uint8_t r, uint8_t g, uint8_t b, int duration = 50);

// Ahorro de energía
void enterDeepSleep(unsigned long seconds);

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Mostrar razón de wake-up
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("⏰ Wake up: Timer (Deep Sleep)");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      Serial.println("🔌 Wake up: Power on / Reset");
      break;
  }

  Serial.println("\n========================================");
  Serial.println("MiniOS WiFi Client v" FIRMWARE_VERSION);
  Serial.print("Board: ");
  Serial.println(BOARD_MODEL);
  Serial.println("========================================");

  // LED de estado (apagado al arrancar)
  pinMode(LED_STATUS, OUTPUT);
  ledStatus(false);

  // LED RGB NeoPixel (solo si está disponible)
  #if HAS_RGB_LED
    rgbLed.begin();
    rgbLed.setBrightness(50);  // Brillo moderado (0-255)
    rgbLed.clear();
    rgbLed.show();
    Serial.println("✅ LED RGB inicializado");
  #endif

  // Inicializar bus I2C con pines correctos según plataforma
  Wire.begin(I2C_SDA_DEFAULT, I2C_SCL_DEFAULT);
  Wire.setClock(100000);  // 100kHz: más compatible con módulos AHT20+BMP280 combo
  delay(100);             // Dar tiempo a los sensores I2C para estabilizarse tras encendido
  Serial.print("✅ I2C inicializado 100kHz (SDA:");
  Serial.print(I2C_SDA_DEFAULT);
  Serial.print(", SCL:");
  Serial.print(I2C_SCL_DEFAULT);
  Serial.println(")");

  // Inicializar WiFi para obtener MAC
  WiFi.mode(WIFI_STA);

  // Obtener MAC
  deviceMac = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(deviceMac);

  if (deviceMac == "00:00:00:00:00:00" || deviceMac.length() == 0) {
    Serial.println("⚠️  Error obteniendo MAC, reintentando...");
    delay(100);
    deviceMac = WiFi.macAddress();
    Serial.print("MAC Address: ");
    Serial.println(deviceMac);
  }

  // Cargar configuración guardada
  loadConfig();

  // Mostrar comandos disponibles
  Serial.println("\nComandos disponibles:");
  Serial.println("  wifi <ssid> <pass> - Configurar WiFi");
  Serial.println("     SSID con espacios: wifi \"CASA ROJAS\" miclave");
  Serial.println("  server <host> <port> - Configurar backend");
  Serial.println("  token <valor> - Token de dispositivo ('-' para borrar)");
  Serial.println("  status - Ver estado");
  Serial.println("  reboot - Reiniciar");

  // Prompt para desactivar Deep Sleep (útil para subir firmware)
  #if DEEP_SLEEP_ENABLED
    Serial.println("\n⚠️  Deep Sleep ACTIVADO");
    Serial.println("Presiona cualquier tecla en 10 segundos para DESACTIVAR Deep Sleep");
    Serial.println("(útil para subir nuevo firmware vía USB)");

    unsigned long startPrompt = millis();
    while (millis() - startPrompt < 10000) {  // Esperar 10 segundos
      if (Serial.available()) {
        // Leer y descartar la entrada
        while (Serial.available()) Serial.read();
        disableDeepSleep = true;
        Serial.println("\n✅ Deep Sleep DESACTIVADO - Modo normal continuo");
        Serial.println("El dispositivo permanecerá despierto para programación OTA/USB");
        break;
      }
      delay(100);

      // Mostrar countdown cada segundo
      if ((millis() - startPrompt) % 1000 == 0) {
        Serial.print(".");
      }
    }

    if (!disableDeepSleep) {
      Serial.println("\n⏰ Continuando con Deep Sleep activado");
    }
  #endif

  // Conectar WiFi
  if (WIFI_SSID.length() > 0) {
    connectWiFi();
  } else {
    Serial.println("\n⚠️  WiFi no configurado. Usa: wifi <ssid> <pass>");
  }
}

// ============================================
// LOOP PRINCIPAL
// ============================================

void loop() {
  static bool taskCompleted = false;

  if (taskCompleted) {
    // Ya terminamos, esperar a que Deep Sleep se active
    delay(1000);
    return;
  }

  unsigned long startTime = millis();

  // 1. Procesar comandos Serial (solo por 2 segundos)
  while (millis() - startTime < 2000 && Serial.available()) {
    handleSerial();
    delay(10);
  }

  // 2. Verificar WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ Sin WiFi, reintentando...");
    connectWiFi();

    // Agotados los intentos, pedir credenciales en vez de seguir a ciegas
    if (WiFi.status() != WL_CONNECTED && wifiFailCount >= WIFI_MAX_ATTEMPTS) {
      waitForWifiCredentials();
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("❌ No se pudo conectar WiFi");
      if (DEEP_SLEEP_ENABLED && !disableDeepSleep) {
        enterDeepSleep(deepSleepDuration);
      }
      return;
    }
  }

  // 3. Conectar WebSocket si no está conectado
  if (!isRegistered) {
    connectWebSocket();

    // Esperar conexión (timeout 10s)
    unsigned long wsStart = millis();
    while (!isRegistered && millis() - wsStart < 10000) {
      webSocket.loop();
      delay(100);
    }

    if (!isRegistered) {
      Serial.println("⚠️ No se pudo registrar en backend");
    }
  }

  // 4. Re-sincronizar hora si es necesario (cada hora)
  if (timeSync && millis() - lastTimeSync > TIME_SYNC_INTERVAL) {
    syncTimeWithBackend();
  }

  // 5. Leer TODOS los sensores
  Serial.println("📊 Leyendo sensores...");
  readDHTSensors();
  readI2CSensors();
  readUltrasonicSensors();  // Lee y detecta
  processUltrasonicTriggers();  // Procesa triggers

  // 6. Enviar datos
  if (isRegistered) {
    Serial.println("📤 Enviando datos...");
    sendSensorData();

    // 6. Ventana de comandos: primero drenar mensajes pendientes del backend
    // (comandos encolados mientras el dispositivo estaba dormido llegan aquí primero)
    Serial.println("⏳ Procesando comandos pendientes...");
    unsigned long drainStart = millis();
    while (millis() - drainStart < 3000) {
      webSocket.loop();
      delay(50);
    }

    // Luego mantener ventana activa para comandos en tiempo real
    Serial.println("⏳ Ventana de comandos abierta (15 segundos)...");
    unsigned long commandWaitStart = millis();
    while (millis() - commandWaitStart < 15000) {
      webSocket.loop();
      delay(100);

      if (Serial.available()) {
        handleSerial();
      }
    }
    Serial.println("✅ Ventana de comandos cerrada.");
  }

  // 7. Marcar como completado
  taskCompleted = true;

  // 8. Entrar en Deep Sleep (solo si no está desactivado)
  if (DEEP_SLEEP_ENABLED && !disableDeepSleep) {
    enterDeepSleep(deepSleepDuration);
  } else {
    Serial.println("💡 Deep Sleep deshabilitado, esperando...");
    delay(deepSleepDuration * 1000);
    taskCompleted = false;  // Repetir ciclo
  }
}

// ============================================
// FUNCIONES DE VALIDACIÓN GPIO
// ============================================

// Verificar si un GPIO existe en una lista
bool GPIO_InList(int pin, const int* list, int count) {
  for (int i = 0; i < count; i++) {
    if (list[i] == pin) return true;
  }
  return false;
}

// Verificar si un GPIO es válido
bool GPIO_IsValid(int pin) {
  return GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT) ||
         GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT) ||
         GPIO_InList(pin, I2C_GPIOS, I2C_GPIOS_COUNT) ||
         GPIO_InList(pin, SPI_GPIOS, SPI_GPIOS_COUNT);
}

// Verificar si un GPIO es analógico
bool GPIO_IsAnalog(int pin) {
  return GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT);
}

// Verificar si un GPIO es apropiado para un modo específico
bool GPIO_IsAppropriate(int pin, GPIOMode mode) {
  bool isAnalog = GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT);
  bool isDigital = GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT);

  switch (mode) {
    case MODE_OUTPUT:
    case MODE_PWM:
      // OUTPUT y PWM: solo digitales (no analógicas)
      return isDigital;

    case MODE_INPUT:
    case MODE_INPUT_PULLUP:
      // INPUT: analógicas o digitales
      return isAnalog || isDigital;

    default:
      return false;
  }
}

// Aplicar fórmula de conversión a valor analógico
float applyFormula(int rawValue, GPIOConfig& gpio) {
  if (!gpio.formulaEnabled || !gpio.isAnalog) {
    return (float)rawValue;
  }

  float voltage = (rawValue * 3.3) / 4095.0;
  float normalized = 0;

  if (gpio.formulaType == "4-20mA") {
    // Sensor 4-20mA con resistencia 250Ω
    // Voltaje = Corriente * Resistencia
    // Corriente (mA) = Voltaje / 0.250
    float current = voltage / 0.250;  // mA
    // Normalizar: 4mA = 0%, 20mA = 100%
    normalized = (current - 4.0) / 16.0;
  }
  else if (gpio.formulaType == "0-10V") {
    // Sensor 0-10V con divisor de voltaje 1:4
    // Voltaje real = voltaje leído * 4
    float realVoltage = voltage * 4.0;
    // Normalizar: 0V = 0%, 10V = 100%
    normalized = realVoltage / 10.0;
  }
  else if (gpio.formulaType == "0-3.3V") {
    // Voltaje directo 0-3.3V
    normalized = voltage / 3.3;
  }
  else {
    // Custom o desconocido: usar valor RAW normalizado
    normalized = rawValue / 4095.0;
  }

  // Limitar entre 0 y 1
  if (normalized < 0) normalized = 0;
  if (normalized > 1) normalized = 1;

  // Escalar al rango min-max
  return gpio.formulaMin + (normalized * (gpio.formulaMax - gpio.formulaMin));
}

// ============================================
// LED RGB
// ============================================

void blinkRGB(uint8_t r, uint8_t g, uint8_t b, int duration) {
  #if HAS_RGB_LED
    rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
    rgbLed.show();
    delay(duration);
    rgbLed.clear();
    rgbLed.show();
  #endif
}

// ============================================
// WIFI
// ============================================

// Traduce el codigo de estado del WiFi a algo accionable: distingue "no veo esa
// red" (nombre mal escrito o fuera de alcance) de "la red me rechaza" (clave mal)
const char* wifiStatusText(wl_status_t status) {
  switch (status) {
    case WL_NO_SSID_AVAIL:   return "no se encuentra esa red (¿nombre mal escrito o fuera de alcance?)";
    case WL_CONNECT_FAILED:  return "la red rechazó la conexión (¿contraseña incorrecta?)";
    case WL_CONNECTION_LOST: return "conexión perdida";
    case WL_DISCONNECTED:    return "desconectado";
    case WL_IDLE_STATUS:     return "sin respuesta del punto de acceso";
    default:                 return "error desconocido";
  }
}

// Enciende o apaga el LED de estado respetando la polaridad de la placa
void ledStatus(bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(LED_STATUS, on ? LOW : HIGH);
#else
  digitalWrite(LED_STATUS, on ? HIGH : LOW);
#endif
}

void connectWiFi() {
  if (WIFI_SSID.length() == 0) {
    Serial.println("⚠️  WiFi sin configurar. Usa: wifi <ssid> <password>");
    return;
  }

  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiFailCount = 0;
    Serial.println("\n✅ WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // Habilitar WiFi sleep mode para ahorro de energía
    WiFi.setSleep(WIFI_PS_MIN_MODEM);  // Ahorro moderado, mantiene conexión
    Serial.println("💤 WiFi Sleep Mode habilitado");

    ledStatus(true);

    // Conectar al backend
    if (BACKEND_HOST.length() > 0) {
      connectWebSocket();
    } else {
      Serial.println("⚠️  Backend no configurado. Usa: server <host> <port>");
    }
  } else {
    wifiFailCount++;
    Serial.printf("\n❌ Error conectando WiFi (intento %d de %d): %s\n",
                  wifiFailCount, WIFI_MAX_ATTEMPTS, wifiStatusText(WiFi.status()));
    ledStatus(false);
  }
}

// Tras WIFI_MAX_ATTEMPTS fallos se deja de reintentar a ciegas y se pide que
// escribas credenciales nuevas. Antes el loop volvia a llamar a connectWiFi()
// sin parar y sus 10 s de bloqueo no dejaban hueco para teclear nada.
void waitForWifiCredentials() {
  Serial.println("\n========================================");
  Serial.printf("⚠️  %d intentos fallidos con SSID \"%s\"\n", wifiFailCount, WIFI_SSID.c_str());

  // Diagnostico: mirar si la red siquiera esta al alcance
  Serial.println("🔍 Buscando redes cercanas...");
  int n = WiFi.scanNetworks();
  bool found = false;

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      found = true;
      Serial.printf("   La red existe (señal %d dBm) → revisa la CONTRASEÑA\n", WiFi.RSSI(i));
      break;
    }
  }

  if (!found) {
    Serial.println("   Esa red NO aparece → revisa el NOMBRE. Redes visibles:");
    for (int i = 0; i < n && i < 12; i++) {
      Serial.printf("     [%s]  %d dBm\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    if (n == 0) Serial.println("     (ninguna)");
  }

  WiFi.scanDelete();

  Serial.println("\nEscribe las credenciales correctas:");
  Serial.println("   wifi <ssid> <password>");
  Serial.println("   wifi \"CASA ROJAS\" miclave     (si el nombre lleva espacios)");
  Serial.println("========================================\n");

  unsigned long start = millis();
  unsigned long lastReminder = millis();

  while (WiFi.status() != WL_CONNECTED) {
    handleSerial();
    delay(50);

    // Con Deep Sleep activo no se puede esperar indefinidamente: el equipo
    // puede ir a bateria y la red quizas solo este caida un rato.
    if (DEEP_SLEEP_ENABLED && !disableDeepSleep && millis() - start > WIFI_CONFIG_WINDOW_MS) {
      Serial.println("⏱️  Nadie respondió: se vuelve a dormir y se reintentará al despertar");
      wifiFailCount = 0;  // ciclo nuevo tras el sueño
      return;
    }

    if (millis() - lastReminder > 30000) {
      lastReminder = millis();
      Serial.println("⌛ Esperando:  wifi <ssid> <password>");
    }
  }
}

// ============================================
// WEBSOCKET
// ============================================

void connectWebSocket() {
  Serial.print("Conectando a backend: ");
  Serial.print(BACKEND_HOST);
  Serial.print(":");
  Serial.println(BACKEND_PORT);

  // El backend exige el token si tiene DEVICE_TOKEN configurado
  String path = "/ws/device";
  if (DEVICE_TOKEN.length() > 0) {
    path += "?token=" + DEVICE_TOKEN;
  }

  // Usar SSL para puerto 443, sin SSL para otros puertos
  if (BACKEND_PORT == 443) {
    Serial.println("Usando conexión SSL (wss://)");
    webSocket.beginSSL(BACKEND_HOST.c_str(), BACKEND_PORT, path.c_str());
  } else {
    Serial.println("Usando conexión sin SSL (ws://)");
    webSocket.begin(BACKEND_HOST.c_str(), BACKEND_PORT, path.c_str());
  }

  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // Heartbeat para mantener conexión viva y detectar desconexiones
  // Ping cada 15s, timeout 3s, 2 intentos antes de reconectar
  webSocket.enableHeartbeat(15000, 3000, 2);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("📴 WebSocket desconectado");
      isRegistered = false;
      break;

    case WStype_CONNECTED:
      Serial.println("🔌 WebSocket conectado");
      registerDevice();
      break;

    case WStype_TEXT:
      blinkRGB(0, 0, 255);  // Azul: mensaje recibido
      handleWebSocketMessage((char*)payload);
      break;

    case WStype_ERROR:
      Serial.println("❌ WebSocket error");
      break;

    default:
      break;
  }
}

void registerDevice() {
  StaticJsonDocument<512> doc;
  doc["type"] = "register";
  doc["mac_address"] = deviceMac;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["ip_address"] = WiFi.localIP().toString();
  doc["board_model"] = BOARD_MODEL;      // 🆕 Enviar modelo de placa
  doc["board_family"] = BOARD_FAMILY;    // 🆕 Enviar familia del chip

  String json;
  serializeJson(doc, json);

  // Debug: mostrar lo que se envía
  Serial.println("📱 Enviando registro al backend:");
  Serial.println(json);

  webSocket.sendTXT(json);
  blinkRGB(0, 255, 0);  // Verde: datos enviados
}

void syncTimeWithBackend() {
  Serial.println("🕐 Sincronizando hora con el servidor...");

  HTTPClient http;
  String url = String("http") + (BACKEND_PORT == 443 ? "s" : "") + "://" +
               BACKEND_HOST + ":" + String(BACKEND_PORT) + "/api/time";

  http.begin(url);
  http.setTimeout(5000);

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      unsigned long timestamp = doc["timestamp"];
      const char* timezone = doc["timezone"];

      if (timezone) {
        serverTimezone = String(timezone);
      }

      // El reloj se mantiene en UTC y la conversion a hora local se hace con
      // reglas POSIX. Antes habia un desfase fijo de -3 h, asi que medio ano
      // (abril-agosto, cuando Chile esta en UTC-4) la hora salia corrida.
      // newlib no trae la base de datos IANA, de modo que el nombre que envia
      // el backend no se puede usar directamente: se guarda solo para mostrarlo.
      setenv("TZ", POSIX_TZ, 1);
      tzset();

      // Establecer la hora directamente desde el timestamp del servidor (UTC)
      struct timeval tv;
      tv.tv_sec = timestamp;
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);

      timeSync = true;
      lastTimeSync = millis();

      Serial.printf("✅ Hora sincronizada: %lu (Timezone: %s)\n", timestamp, serverTimezone.c_str());

      // Mostrar hora actual
      time_t now = time(nullptr);
      Serial.print("📅 Fecha/Hora actual: ");
      Serial.println(ctime(&now));
    } else {
      Serial.println("❌ Error parseando respuesta de tiempo");
    }
  } else {
    Serial.printf("❌ Error obteniendo hora del servidor (HTTP %d)\n", httpCode);
  }

  http.end();
}

void handleWebSocketMessage(const char* payload) {
  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }

  const char* type = doc["type"];

  if (strcmp(type, "config") == 0) {
    handleConfig(doc);
  } else if (strcmp(type, "command") == 0) {
    handleCommand(doc);
  } else if (strcmp(type, "scan_i2c") == 0) {
    scanAndReportI2C();
  }
}

void handleConfig(JsonDocument& doc) {
  deviceId = doc["device_id"];
  isRegistered = true;

  Serial.print("✅ Registrado como dispositivo ID: ");
  Serial.println(deviceId);

  // Aplicar sleep_interval si el backend lo envía (viene en ms, convertir a segundos)
  if (doc.containsKey("sleep_interval")) {
    unsigned long newSleep = doc["sleep_interval"].as<unsigned long>() / 1000;
    if (newSleep >= 5) {
      deepSleepDuration = newSleep;
      preferences.begin("minios", false);
      preferences.putUInt("sleep_secs", (uint32_t)deepSleepDuration);
      preferences.end();
      Serial.printf("⏰ Sleep interval: %lu segundos\n", deepSleepDuration);
    }
  }

  // Sincronizar hora con el servidor
  if (!timeSync) {
    syncTimeWithBackend();
  }

  // Configurar GPIOs
  JsonArray gpioArray = doc["gpio"].as<JsonArray>();
  gpioCount = 0;
  for (JsonObject gpio : gpioArray) {
    if (gpioCount >= MAX_GPIOS) break;

    int pin = gpio["pin"];
    String modeStr = gpio["mode"].as<String>();

    // Determinar modo
    GPIOMode mode;
    if (modeStr == "OUTPUT") {
      mode = MODE_OUTPUT;
    } else if (modeStr == "INPUT") {
      mode = MODE_INPUT;
    } else if (modeStr == "INPUT_PULLUP") {
      mode = MODE_INPUT_PULLUP;
    } else if (modeStr == "PWM") {
      mode = MODE_PWM;
    } else {
      Serial.printf("[GPIO] Modo desconocido para pin %d\n", pin);
      continue;
    }

    // Validar pin
    if (!GPIO_IsValid(pin)) {
      Serial.printf("[GPIO] Pin %d no es válido\n", pin);
      continue;
    }

    if (!GPIO_IsAppropriate(pin, mode)) {
      Serial.printf("[GPIO] Pin %d no es apropiado para modo %s\n", pin, modeStr.c_str());
      if (GPIO_IsAnalog(pin)) {
        Serial.println("[GPIO] Este pin es analógico, solo puede usarse como INPUT");
      }
      continue;
    }

    gpios[gpioCount].pin = pin;
    gpios[gpioCount].name = gpio["name"].as<String>();
    gpios[gpioCount].value = gpio["value"];
    gpios[gpioCount].active = gpio["active"];
    gpios[gpioCount].loopEnabled = gpio["loop_enabled"];
    gpios[gpioCount].loopInterval = gpio["loop_interval"];
    gpios[gpioCount].lastLoop = 0;
    gpios[gpioCount].isAnalog = GPIO_IsAnalog(pin);
    gpios[gpioCount].mode = mode;

    // Configuración de fórmulas para sensores analógicos
    gpios[gpioCount].formulaEnabled = gpio["formula_enabled"] | false;
    gpios[gpioCount].formulaType = gpio["formula_type"].as<String>();
    gpios[gpioCount].formulaMin = gpio["formula_min"] | 0.0;
    gpios[gpioCount].formulaMax = gpio["formula_max"] | 100.0;
    gpios[gpioCount].unit = gpio["unit"].as<String>();
    gpios[gpioCount].convertedValue = 0;

    // Aplicar configuración física
    switch (mode) {
      case MODE_OUTPUT:
        pinMode(pin, OUTPUT);
        digitalWrite(pin, gpios[gpioCount].value);
        break;
      case MODE_INPUT:
        pinMode(pin, INPUT);
        break;
      case MODE_INPUT_PULLUP:
        pinMode(pin, INPUT_PULLUP);
        break;
      case MODE_PWM:
        ledcAttach(pin, 5000, 8);
        ledcWrite(pin, gpios[gpioCount].value);
        break;
    }

    gpioCount++;
  }

  Serial.print("GPIOs configurados: ");
  Serial.println(gpioCount);

  // Configurar DHT
  JsonArray dhtArray = doc["dht"].as<JsonArray>();

  // Limpiar sensores anteriores
  for (int i = 0; i < dhtCount; i++) {
    if (dhtSensors[i].sensor) {
      delete dhtSensors[i].sensor;
    }
  }
  dhtCount = 0;

  for (JsonObject dht : dhtArray) {
    if (dhtCount >= MAX_DHT_SENSORS) break;

    int pin = dht["pin"];
    String type = dht["sensor_type"].as<String>();

    // Los GPIO si se validaban, pero los DHT no: un pin de la flash interna
    // llegado del backend acababa en pinMode() y colgaba el chip.
    if (!GPIO_IsValid(pin) || !GPIO_IsAppropriate(pin, MODE_OUTPUT)) {
      Serial.printf("[DHT] Pin %d no es válido para un sensor DHT\n", pin);
      continue;
    }

    dhtSensors[dhtCount].pin = pin;
    dhtSensors[dhtCount].name = dht["name"].as<String>();
    dhtSensors[dhtCount].type = type;
    dhtSensors[dhtCount].active = dht["active"];
    dhtSensors[dhtCount].readInterval = dht["read_interval"] | 5000;
    dhtSensors[dhtCount].lastRead = 0;
    dhtSensors[dhtCount].temperature = 0;
    dhtSensors[dhtCount].humidity = 0;

    uint8_t dhtType = (type == "DHT22") ? DHT22 : DHT11;
    dhtSensors[dhtCount].sensor = new DHT(pin, dhtType);
    dhtSensors[dhtCount].sensor->begin();

    dhtCount++;
  }

  Serial.print("Sensores DHT configurados: ");
  Serial.println(dhtCount);

  // Configurar sensores I2C
  JsonArray i2cArray = doc["i2c"].as<JsonArray>();

  // Limpiar sensores anteriores
  for (int i = 0; i < i2cCount; i++) {
    if (i2cSensors[i].aht) delete i2cSensors[i].aht;
    if (i2cSensors[i].bmp) delete i2cSensors[i].bmp;
    if (i2cSensors[i].bme) delete i2cSensors[i].bme;
  }
  i2cCount = 0;

  for (JsonObject i2c : i2cArray) {
    if (i2cCount >= MAX_I2C_SENSORS) break;

    String sensorType = i2c["sensor_type"].as<String>();
    uint8_t address = i2c["i2c_address"] | 0x00;

    i2cSensors[i2cCount].id = i2c["id"];
    i2cSensors[i2cCount].name = i2c["name"].as<String>();
    i2cSensors[i2cCount].sensorType = sensorType;
    i2cSensors[i2cCount].i2cAddress = address;
    i2cSensors[i2cCount].active = i2c["active"] | true;
    i2cSensors[i2cCount].readInterval = i2c["read_interval"] | 5000;
    i2cSensors[i2cCount].lastRead = 0;
    i2cSensors[i2cCount].temperature = 0;
    i2cSensors[i2cCount].humidity = 0;
    i2cSensors[i2cCount].pressure = 0;
    i2cSensors[i2cCount].altitude = 0;
    i2cSensors[i2cCount].aht = nullptr;
    i2cSensors[i2cCount].bmp = nullptr;
    i2cSensors[i2cCount].bme = nullptr;

    // Inicializar sensor según tipo (con 3 reintentos y 100ms entre ellos)
    bool sensorOk = false;
    if (sensorType == "AHT20") {
      i2cSensors[i2cCount].aht = new Adafruit_AHTX0();
      for (int attempt = 0; attempt < 3 && !sensorOk; attempt++) {
        if (attempt > 0) delay(100);
        sensorOk = i2cSensors[i2cCount].aht->begin(&Wire, 0, address);
      }
      if (sensorOk) {
        Serial.printf("✅ AHT20 inicializado en 0x%02X\n", address);
      } else {
        Serial.printf("❌ Error inicializando AHT20 en 0x%02X (verifique cableado y dirección)\n", address);
        delete i2cSensors[i2cCount].aht;
        i2cSensors[i2cCount].aht = nullptr;
        continue;
      }
    }
    else if (sensorType == "BMP280") {
      i2cSensors[i2cCount].bmp = new Adafruit_BMP280();
      for (int attempt = 0; attempt < 3 && !sensorOk; attempt++) {
        if (attempt > 0) delay(100);
        sensorOk = i2cSensors[i2cCount].bmp->begin(address);
      }
      if (sensorOk) {
        Serial.printf("✅ BMP280 inicializado en 0x%02X\n", address);
        i2cSensors[i2cCount].bmp->setSampling(
          Adafruit_BMP280::MODE_NORMAL,
          Adafruit_BMP280::SAMPLING_X2,
          Adafruit_BMP280::SAMPLING_X16,
          Adafruit_BMP280::FILTER_X16,
          Adafruit_BMP280::STANDBY_MS_500
        );
      } else {
        Serial.printf("❌ Error inicializando BMP280 en 0x%02X (verifique cableado y dirección)\n", address);
        delete i2cSensors[i2cCount].bmp;
        i2cSensors[i2cCount].bmp = nullptr;
        continue;
      }
    }
    else if (sensorType == "BME280") {
      i2cSensors[i2cCount].bme = new Adafruit_BME280();
      for (int attempt = 0; attempt < 3 && !sensorOk; attempt++) {
        if (attempt > 0) delay(100);
        sensorOk = i2cSensors[i2cCount].bme->begin(address, &Wire);
      }
      if (sensorOk) {
        Serial.printf("✅ BME280 inicializado en 0x%02X\n", address);
        i2cSensors[i2cCount].bme->setSampling(
          Adafruit_BME280::MODE_NORMAL,
          Adafruit_BME280::SAMPLING_X2,
          Adafruit_BME280::SAMPLING_X16,
          Adafruit_BME280::SAMPLING_X16,
          Adafruit_BME280::FILTER_X16,
          Adafruit_BME280::STANDBY_MS_500
        );
      } else {
        Serial.printf("❌ Error inicializando BME280 en 0x%02X (verifique cableado y dirección)\n", address);
        delete i2cSensors[i2cCount].bme;
        i2cSensors[i2cCount].bme = nullptr;
        continue;
      }
    }

    i2cCount++;
  }

  Serial.print("Sensores I2C configurados: ");
  Serial.println(i2cCount);

  // Configurar sensores ultrasónicos
  JsonArray ultrasonicArray = doc["ultrasonic"].as<JsonArray>();
  ultrasonicCount = 0;

  for (JsonObject us : ultrasonicArray) {
    if (ultrasonicCount >= MAX_ULTRASONIC_SENSORS) break;

    int trigPin = us["trig_pin"];
    int echoPin = us["echo_pin"];

    // Mismo caso que los DHT: trig, echo y el GPIO de disparo iban directos a
    // pinMode() sin comprobar nada
    if (!GPIO_IsValid(trigPin) || !GPIO_IsAppropriate(trigPin, MODE_OUTPUT)) {
      Serial.printf("[US] trig_pin %d no es válido\n", trigPin);
      continue;
    }
    if (!GPIO_IsValid(echoPin) || !GPIO_IsAppropriate(echoPin, MODE_INPUT)) {
      Serial.printf("[US] echo_pin %d no es válido\n", echoPin);
      continue;
    }

    int trigGpio = us["trigger_gpio_pin"] | -1;
    if (trigGpio >= 0 && (!GPIO_IsValid(trigGpio) || !GPIO_IsAppropriate(trigGpio, MODE_OUTPUT))) {
      Serial.printf("[US] trigger_gpio_pin %d no es válido, se ignora\n", trigGpio);
      trigGpio = -1;
    }

    ultrasonicSensors[ultrasonicCount].id = us["id"];
    ultrasonicSensors[ultrasonicCount].trigPin = trigPin;
    ultrasonicSensors[ultrasonicCount].echoPin = echoPin;
    ultrasonicSensors[ultrasonicCount].name = us["name"].as<String>();
    ultrasonicSensors[ultrasonicCount].maxDistance = us["max_distance"] | 400;
    ultrasonicSensors[ultrasonicCount].detectionEnabled = us["detection_enabled"] | true;
    ultrasonicSensors[ultrasonicCount].triggerDistance = us["trigger_distance"] | 50;
    ultrasonicSensors[ultrasonicCount].triggerGpioPin = trigGpio;
    ultrasonicSensors[ultrasonicCount].triggerGpioValue = us["trigger_gpio_value"] | 1;
    ultrasonicSensors[ultrasonicCount].triggerDuration = us["trigger_duration"] | 1000;
    ultrasonicSensors[ultrasonicCount].active = us["active"] | true;
    ultrasonicSensors[ultrasonicCount].readInterval = us["read_interval"] | 100;
    ultrasonicSensors[ultrasonicCount].lastRead = 0;
    ultrasonicSensors[ultrasonicCount].lastDistance = -1;
    ultrasonicSensors[ultrasonicCount].triggered = false;
    ultrasonicSensors[ultrasonicCount].triggerStartTime = 0;

    // Inicializar buffer de movimiento
    ultrasonicSensors[ultrasonicCount].bufferIndex = 0;
    ultrasonicSensors[ultrasonicCount].bufferFull = false;
    for (int j = 0; j < 10; j++) {
      ultrasonicSensors[ultrasonicCount].distanceBuffer[j] = -1;
    }

    // Estado de detección
    ultrasonicSensors[ultrasonicCount].detectionStartTime = 0;
    ultrasonicSensors[ultrasonicCount].currentSpeed = 0;

    // Configurar pines
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);

    // Configurar GPIO de trigger si existe
    if (ultrasonicSensors[ultrasonicCount].triggerGpioPin >= 0) {
      pinMode(ultrasonicSensors[ultrasonicCount].triggerGpioPin, OUTPUT);
      digitalWrite(ultrasonicSensors[ultrasonicCount].triggerGpioPin,
                   !ultrasonicSensors[ultrasonicCount].triggerGpioValue); // Estado inicial opuesto
    }

    ultrasonicCount++;
  }

  Serial.print("Sensores ultrasónicos configurados: ");
  Serial.println(ultrasonicCount);

  // Verificar OTA pendiente
  if (!doc["ota"].isNull()) {
    JsonObject ota = doc["ota"];
    startOTA(
      ota["id"],
      ota["filename"].as<String>(),
      ota["filesize"],
      ota["checksum"].as<String>()
    );
  }
}

void handleCommand(JsonDocument& doc) {
  const char* action = doc["action"];

  if (strcmp(action, "set_gpio") == 0) {
    int pin = doc["pin"];
    int value = doc["value"];

    for (int i = 0; i < gpioCount; i++) {
      if (gpios[i].pin == pin) {
        gpios[i].value = value;

        if (gpios[i].mode == MODE_OUTPUT) {
          digitalWrite(pin, value);
        } else if (gpios[i].mode == MODE_PWM) {
          ledcWrite(pin, value);
        }

        Serial.print("GPIO ");
        Serial.print(pin);
        Serial.print(" = ");
        Serial.println(value);
        break;
      }
    }
  }
  else if (strcmp(action, "update_gpio") == 0) {
    // Reconfigurar GPIOs
    Serial.println("📥 Recibida actualización de GPIO");

    JsonArray gpioArray = doc["gpio"].as<JsonArray>();
    gpioCount = 0;

    for (JsonObject gpio : gpioArray) {
      if (gpioCount >= MAX_GPIOS) break;

      int pin = gpio["pin"];
      String modeStr = gpio["mode"].as<String>();

      GPIOMode mode;
      if (modeStr == "OUTPUT") mode = MODE_OUTPUT;
      else if (modeStr == "INPUT") mode = MODE_INPUT;
      else if (modeStr == "INPUT_PULLUP") mode = MODE_INPUT_PULLUP;
      else if (modeStr == "PWM") mode = MODE_PWM;
      else continue;

      if (!GPIO_IsValid(pin) || !GPIO_IsAppropriate(pin, mode)) {
        Serial.printf("[GPIO] Pin %d no válido para modo %s\n", pin, modeStr.c_str());
        continue;
      }

      gpios[gpioCount].pin = pin;
      gpios[gpioCount].name = gpio["name"].as<String>();
      gpios[gpioCount].value = gpio["value"];
      gpios[gpioCount].active = gpio["active"];
      gpios[gpioCount].loopEnabled = gpio["loop_enabled"];
      gpios[gpioCount].loopInterval = gpio["loop_interval"];
      gpios[gpioCount].lastLoop = 0;
      gpios[gpioCount].isAnalog = GPIO_IsAnalog(pin);
      gpios[gpioCount].mode = mode;

      // Configuración de fórmulas para sensores analógicos
      gpios[gpioCount].formulaEnabled = gpio["formula_enabled"] | false;
      gpios[gpioCount].formulaType = gpio["formula_type"].as<String>();
      gpios[gpioCount].formulaMin = gpio["formula_min"] | 0.0;
      gpios[gpioCount].formulaMax = gpio["formula_max"] | 100.0;
      gpios[gpioCount].unit = gpio["unit"].as<String>();
      gpios[gpioCount].convertedValue = 0;

      switch (mode) {
        case MODE_OUTPUT:
          pinMode(pin, OUTPUT);
          digitalWrite(pin, gpios[gpioCount].value);
          break;
        case MODE_INPUT:
          pinMode(pin, INPUT);
          break;
        case MODE_INPUT_PULLUP:
          pinMode(pin, INPUT_PULLUP);
          break;
        case MODE_PWM:
          ledcAttach(pin, 5000, 8);
          ledcWrite(pin, gpios[gpioCount].value);
          break;
      }

      gpioCount++;
    }

    Serial.printf("GPIOs actualizados: %d\n", gpioCount);
  }
  else if (strcmp(action, "update_dht") == 0) {
    // Reconfigurar sensores DHT
    Serial.println("📥 Recibida actualización de DHT");

    JsonArray dhtArray = doc["dht"].as<JsonArray>();

    // Limpiar sensores anteriores
    for (int i = 0; i < dhtCount; i++) {
      if (dhtSensors[i].sensor) {
        delete dhtSensors[i].sensor;
      }
    }
    dhtCount = 0;

    for (JsonObject dht : dhtArray) {
      if (dhtCount >= MAX_DHT_SENSORS) break;

      int pin = dht["pin"];
      String type = dht["sensor_type"].as<String>();

      dhtSensors[dhtCount].pin = pin;
      dhtSensors[dhtCount].name = dht["name"].as<String>();
      dhtSensors[dhtCount].type = type;
      dhtSensors[dhtCount].active = dht["active"];
      dhtSensors[dhtCount].readInterval = dht["read_interval"] | 5000;
      dhtSensors[dhtCount].lastRead = 0;
      dhtSensors[dhtCount].temperature = 0;
      dhtSensors[dhtCount].humidity = 0;

      uint8_t dhtType = (type == "DHT22") ? DHT22 : DHT11;
      dhtSensors[dhtCount].sensor = new DHT(pin, dhtType);
      dhtSensors[dhtCount].sensor->begin();

      dhtCount++;
    }

    Serial.printf("Sensores DHT actualizados: %d\n", dhtCount);
  }
  else if (strcmp(action, "update_ultrasonic") == 0) {
    // Reconfigurar sensores ultrasónicos
    Serial.println("📥 Recibida actualización de Ultrasonic");

    JsonArray ultrasonicArray = doc["ultrasonic"].as<JsonArray>();
    ultrasonicCount = 0;

    for (JsonObject us : ultrasonicArray) {
      if (ultrasonicCount >= MAX_ULTRASONIC_SENSORS) break;

      int trigPin = us["trig_pin"];
      int echoPin = us["echo_pin"];

      ultrasonicSensors[ultrasonicCount].id = us["id"];
      ultrasonicSensors[ultrasonicCount].trigPin = trigPin;
      ultrasonicSensors[ultrasonicCount].echoPin = echoPin;
      ultrasonicSensors[ultrasonicCount].name = us["name"].as<String>();
      ultrasonicSensors[ultrasonicCount].maxDistance = us["max_distance"] | 400;
      ultrasonicSensors[ultrasonicCount].detectionEnabled = us["detection_enabled"] | true;
      ultrasonicSensors[ultrasonicCount].triggerDistance = us["trigger_distance"] | 50;
      ultrasonicSensors[ultrasonicCount].triggerGpioPin = us["trigger_gpio_pin"] | -1;
      ultrasonicSensors[ultrasonicCount].triggerGpioValue = us["trigger_gpio_value"] | 1;
      ultrasonicSensors[ultrasonicCount].triggerDuration = us["trigger_duration"] | 1000;
      ultrasonicSensors[ultrasonicCount].active = us["active"] | true;
      ultrasonicSensors[ultrasonicCount].readInterval = us["read_interval"] | 100;
      ultrasonicSensors[ultrasonicCount].lastRead = 0;
      ultrasonicSensors[ultrasonicCount].lastDistance = -1;
      ultrasonicSensors[ultrasonicCount].triggered = false;
      ultrasonicSensors[ultrasonicCount].triggerStartTime = 0;

      // Inicializar buffer de movimiento
      ultrasonicSensors[ultrasonicCount].bufferIndex = 0;
      ultrasonicSensors[ultrasonicCount].bufferFull = false;
      for (int j = 0; j < 10; j++) {
        ultrasonicSensors[ultrasonicCount].distanceBuffer[j] = -1;
      }

      // Estado de detección
      ultrasonicSensors[ultrasonicCount].detectionStartTime = 0;
      ultrasonicSensors[ultrasonicCount].currentSpeed = 0;

      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      digitalWrite(trigPin, LOW);

      if (ultrasonicSensors[ultrasonicCount].triggerGpioPin >= 0) {
        pinMode(ultrasonicSensors[ultrasonicCount].triggerGpioPin, OUTPUT);
        digitalWrite(ultrasonicSensors[ultrasonicCount].triggerGpioPin,
                     !ultrasonicSensors[ultrasonicCount].triggerGpioValue);
      }

      ultrasonicCount++;
    }

    Serial.printf("Sensores ultrasónicos actualizados: %d\n", ultrasonicCount);
  }
  else if (strcmp(action, "remove_gpio") == 0) {
    int pin = doc["pin"];
    for (int i = 0; i < gpioCount; i++) {
      if (gpios[i].pin == pin) {
        for (int j = i; j < gpioCount - 1; j++) gpios[j] = gpios[j + 1];
        gpioCount--;
        Serial.printf("🗑️ GPIO pin %d eliminado\n", pin);
        break;
      }
    }
  }
  else if (strcmp(action, "remove_dht") == 0) {
    int pin = doc["pin"];
    for (int i = 0; i < dhtCount; i++) {
      if (dhtSensors[i].pin == pin) {
        if (dhtSensors[i].sensor) { delete dhtSensors[i].sensor; dhtSensors[i].sensor = nullptr; }
        for (int j = i; j < dhtCount - 1; j++) dhtSensors[j] = dhtSensors[j + 1];
        dhtCount--;
        Serial.printf("🗑️ DHT pin %d eliminado\n", pin);
        break;
      }
    }
  }
  else if (strcmp(action, "remove_i2c") == 0) {
    int address = doc["i2c_address"];
    for (int i = 0; i < i2cCount; i++) {
      if (i2cSensors[i].i2cAddress == address) {
        if (i2cSensors[i].aht) { delete i2cSensors[i].aht; i2cSensors[i].aht = nullptr; }
        if (i2cSensors[i].bmp) { delete i2cSensors[i].bmp; i2cSensors[i].bmp = nullptr; }
        if (i2cSensors[i].bme) { delete i2cSensors[i].bme; i2cSensors[i].bme = nullptr; }
        for (int j = i; j < i2cCount - 1; j++) i2cSensors[j] = i2cSensors[j + 1];
        i2cCount--;
        Serial.printf("🗑️ Sensor I2C 0x%02X eliminado\n", address);
        break;
      }
    }
  }
  else if (strcmp(action, "update_i2c") == 0) {
    Serial.println("📥 Recibida actualización de I2C");

    // Liberar sensores anteriores
    for (int i = 0; i < i2cCount; i++) {
      if (i2cSensors[i].aht) { delete i2cSensors[i].aht; i2cSensors[i].aht = nullptr; }
      if (i2cSensors[i].bmp) { delete i2cSensors[i].bmp; i2cSensors[i].bmp = nullptr; }
      if (i2cSensors[i].bme) { delete i2cSensors[i].bme; i2cSensors[i].bme = nullptr; }
    }
    i2cCount = 0;

    JsonArray i2cArray = doc["i2c"].as<JsonArray>();
    for (JsonObject i2c : i2cArray) {
      if (i2cCount >= MAX_I2C_SENSORS) break;

      String sensorType = i2c["sensor_type"].as<String>();
      uint8_t address   = i2c["i2c_address"] | 0x00;

      i2cSensors[i2cCount].id          = i2c["id"];
      i2cSensors[i2cCount].name        = i2c["name"].as<String>();
      i2cSensors[i2cCount].sensorType  = sensorType;
      i2cSensors[i2cCount].i2cAddress  = address;
      i2cSensors[i2cCount].active      = i2c["active"] | true;
      i2cSensors[i2cCount].readInterval = i2c["read_interval"] | 5000;
      i2cSensors[i2cCount].lastRead    = 0;
      i2cSensors[i2cCount].temperature = 0;
      i2cSensors[i2cCount].humidity    = 0;
      i2cSensors[i2cCount].pressure    = 0;
      i2cSensors[i2cCount].altitude    = 0;
      i2cSensors[i2cCount].aht         = nullptr;
      i2cSensors[i2cCount].bmp         = nullptr;
      i2cSensors[i2cCount].bme         = nullptr;

      bool sensorOk2 = false;
      if (sensorType == "AHT20") {
        i2cSensors[i2cCount].aht = new Adafruit_AHTX0();
        for (int attempt = 0; attempt < 3 && !sensorOk2; attempt++) {
          if (attempt > 0) delay(100);
          sensorOk2 = i2cSensors[i2cCount].aht->begin(&Wire, 0, address);
        }
        if (sensorOk2) {
          Serial.printf("✅ AHT20 inicializado en 0x%02X\n", address);
        } else {
          Serial.printf("❌ Error inicializando AHT20 en 0x%02X\n", address);
          delete i2cSensors[i2cCount].aht;
          i2cSensors[i2cCount].aht = nullptr;
          continue;
        }
      }
      else if (sensorType == "BMP280") {
        i2cSensors[i2cCount].bmp = new Adafruit_BMP280();
        for (int attempt = 0; attempt < 3 && !sensorOk2; attempt++) {
          if (attempt > 0) delay(100);
          sensorOk2 = i2cSensors[i2cCount].bmp->begin(address);
        }
        if (sensorOk2) {
          i2cSensors[i2cCount].bmp->setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_500
          );
          Serial.printf("✅ BMP280 inicializado en 0x%02X\n", address);
        } else {
          Serial.printf("❌ Error inicializando BMP280 en 0x%02X\n", address);
          delete i2cSensors[i2cCount].bmp;
          i2cSensors[i2cCount].bmp = nullptr;
          continue;
        }
      }
      else if (sensorType == "BME280") {
        i2cSensors[i2cCount].bme = new Adafruit_BME280();
        for (int attempt = 0; attempt < 3 && !sensorOk2; attempt++) {
          if (attempt > 0) delay(100);
          sensorOk2 = i2cSensors[i2cCount].bme->begin(address, &Wire);
        }
        if (sensorOk2) {
          i2cSensors[i2cCount].bme->setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X16,
            Adafruit_BME280::SAMPLING_X16,
            Adafruit_BME280::FILTER_X16,
            Adafruit_BME280::STANDBY_MS_500
          );
          Serial.printf("✅ BME280 inicializado en 0x%02X\n", address);
        } else {
          Serial.printf("❌ Error inicializando BME280 en 0x%02X\n", address);
          delete i2cSensors[i2cCount].bme;
          i2cSensors[i2cCount].bme = nullptr;
          continue;
        }
      }

      i2cCount++;
    }
    Serial.printf("Sensores I2C actualizados: %d\n", i2cCount);
  }
  else if (strcmp(action, "scan_i2c") == 0) {
    Serial.println("🔍 Escaneo I2C solicitado por backend");
    scanAndReportI2C();
  }
  else if (strcmp(action, "set_sleep_interval") == 0) {
    unsigned long newSleep = doc["sleep_interval"].as<unsigned long>() / 1000;
    if (newSleep >= 5) {
      deepSleepDuration = newSleep;
      preferences.begin("minios", false);
      preferences.putUInt("sleep_secs", (uint32_t)deepSleepDuration);
      preferences.end();
      Serial.printf("⏰ Sleep interval actualizado a %lu segundos (próximo ciclo)\n", deepSleepDuration);
    } else {
      Serial.println("⚠️ Sleep interval mínimo es 5 segundos");
    }
  }
  else if (strcmp(action, "reboot") == 0) {
    Serial.println("🔄 Reiniciando...");
    delay(1000);
    ESP.restart();
  }
  else if (strcmp(action, "ota_update") == 0) {
    startOTA(
      doc["ota_id"],
      doc["filename"].as<String>(),
      doc["filesize"],
      doc["checksum"].as<String>()
    );
  }
}

// ============================================
// ENVÍO DE DATOS
// ============================================

void sendSensorData() {
  StaticJsonDocument<2048> doc;
  doc["type"] = "data";
  doc["mac_address"] = deviceMac;

  // Agregar timestamp Unix si el tiempo está sincronizado
  if (timeSync) {
    time_t now = time(nullptr);
    doc["timestamp"] = now;
  }

  JsonObject payload = doc.createNestedObject("payload");

  // Agregar TODOS los sensores DHT
  if (dhtCount > 0) {
    JsonArray dhtArray = payload.createNestedArray("dht");
    for (int i = 0; i < dhtCount; i++) {
      if (dhtSensors[i].active) {
        JsonObject d = dhtArray.createNestedObject();
        d["pin"] = dhtSensors[i].pin;
        d["name"] = dhtSensors[i].name;
        d["temperature"] = dhtSensors[i].temperature;
        d["humidity"] = dhtSensors[i].humidity;

        Serial.printf("DHT[%d] pin:%d temp:%.1f hum:%.1f\n",
          i, dhtSensors[i].pin, dhtSensors[i].temperature, dhtSensors[i].humidity);
      }
    }
  }

  // Agregar sensores I2C (AHT20, BMP280, etc.)
  if (i2cCount > 0) {
    JsonArray i2cArray = payload.createNestedArray("i2c");
    for (int i = 0; i < i2cCount; i++) {
      if (i2cSensors[i].active) {
        JsonObject s = i2cArray.createNestedObject();
        s["id"] = i2cSensors[i].id;
        s["name"] = i2cSensors[i].name;
        s["sensor_type"] = i2cSensors[i].sensorType;
        s["i2c_address"] = i2cSensors[i].i2cAddress;

        if (i2cSensors[i].sensorType == "AHT20") {
          s["temperature"] = i2cSensors[i].temperature;
          s["humidity"] = i2cSensors[i].humidity;
          Serial.printf("I2C[%d] %s temp:%.1f hum:%.1f\n",
            i, i2cSensors[i].name.c_str(), i2cSensors[i].temperature, i2cSensors[i].humidity);
        }
        else if (i2cSensors[i].sensorType == "BMP280") {
          s["temperature"] = i2cSensors[i].temperature;
          s["pressure"] = i2cSensors[i].pressure;
          s["altitude"] = i2cSensors[i].altitude;
          Serial.printf("I2C[%d] %s temp:%.1f pres:%.1f alt:%.1f\n",
            i, i2cSensors[i].name.c_str(), i2cSensors[i].temperature,
            i2cSensors[i].pressure, i2cSensors[i].altitude);
        }
        else if (i2cSensors[i].sensorType == "BME280") {
          s["temperature"] = i2cSensors[i].temperature;
          s["humidity"]    = i2cSensors[i].humidity;
          s["pressure"]    = i2cSensors[i].pressure;
          s["altitude"]    = i2cSensors[i].altitude;
          Serial.printf("I2C[%d] %s temp:%.1f hum:%.1f pres:%.1f alt:%.1f\n",
            i, i2cSensors[i].name.c_str(), i2cSensors[i].temperature,
            i2cSensors[i].humidity, i2cSensors[i].pressure, i2cSensors[i].altitude);
        }
      }
    }
  }

  // Agregar estados GPIO
  JsonArray gpioArray = payload.createNestedArray("gpio");
  for (int i = 0; i < gpioCount; i++) {
    if (gpios[i].active) {
      JsonObject g = gpioArray.createNestedObject();
      g["pin"] = gpios[i].pin;
      g["name"] = gpios[i].name;

      // Leer valor según tipo de pin
      if (gpios[i].mode == MODE_INPUT || gpios[i].mode == MODE_INPUT_PULLUP) {
        if (gpios[i].isAnalog) {
          // Lectura analógica (0-4095)
          gpios[i].value = analogRead(gpios[i].pin);
          g["analog"] = true;
          g["raw"] = gpios[i].value;

          // Aplicar fórmula si está habilitada
          if (gpios[i].formulaEnabled) {
            gpios[i].convertedValue = applyFormula(gpios[i].value, gpios[i]);
            g["converted"] = gpios[i].convertedValue;
            g["unit"] = gpios[i].unit;
            g["formula_type"] = gpios[i].formulaType;

            Serial.printf("GPIO[%d] pin:%d raw:%d -> %.2f %s\n",
              i, gpios[i].pin, gpios[i].value, gpios[i].convertedValue, gpios[i].unit.c_str());
          }
        } else {
          // Lectura digital (0/1)
          gpios[i].value = digitalRead(gpios[i].pin);
          g["analog"] = false;
        }
      }
      g["value"] = gpios[i].value;
    }
  }

  // Agregar datos ultrasónicos con análisis LOCAL
  if (ultrasonicCount > 0) {
    JsonArray ultrasonicArray = payload.createNestedArray("ultrasonic");
    for (int i = 0; i < ultrasonicCount; i++) {
      if (ultrasonicSensors[i].active && ultrasonicSensors[i].lastDistance >= 0) {
        JsonObject u = ultrasonicArray.createNestedObject();
        u["id"] = ultrasonicSensors[i].id;
        u["trig_pin"] = ultrasonicSensors[i].trigPin;
        u["echo_pin"] = ultrasonicSensors[i].echoPin;
        u["name"] = ultrasonicSensors[i].name;
        u["distance"] = ultrasonicSensors[i].lastDistance;
        u["triggered"] = ultrasonicSensors[i].triggered;

        Serial.printf("Ultrasonic[%d] dist:%.1f cm vel:%.1f cm/s\n",
          i, ultrasonicSensors[i].lastDistance, ultrasonicSensors[i].currentSpeed);
      }
    }
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
  blinkRGB(0, 255, 0);  // Verde: datos enviados
}

// ============================================
// SENSORES DHT
// ============================================

void readDHTSensors() {
  for (int i = 0; i < dhtCount; i++) {
    if (!dhtSensors[i].active || !dhtSensors[i].sensor) continue;

    if (millis() - dhtSensors[i].lastRead >= dhtSensors[i].readInterval) {
      dhtSensors[i].lastRead = millis();

      float h = dhtSensors[i].sensor->readHumidity();
      float t = dhtSensors[i].sensor->readTemperature();

      if (!isnan(h) && !isnan(t)) {
        dhtSensors[i].temperature = t;
        dhtSensors[i].humidity = h;
      } else {
        Serial.printf("⚠️ Error leyendo DHT pin %d\n", dhtSensors[i].pin);
      }
    }
  }
}

// ============================================
// SENSORES I2C
// ============================================

void readI2CSensors() {
  for (int i = 0; i < i2cCount; i++) {
    if (!i2cSensors[i].active) continue;

    if (millis() - i2cSensors[i].lastRead >= i2cSensors[i].readInterval) {
      i2cSensors[i].lastRead = millis();

      if (i2cSensors[i].sensorType == "AHT20" && i2cSensors[i].aht) {
        sensors_event_t humidity, temp;
        if (i2cSensors[i].aht->getEvent(&humidity, &temp)) {
          i2cSensors[i].temperature = temp.temperature;
          i2cSensors[i].humidity = humidity.relative_humidity;
        } else {
          Serial.printf("⚠️ Error leyendo AHT20 [%s]\n", i2cSensors[i].name.c_str());
        }
      }
      else if (i2cSensors[i].sensorType == "BMP280" && i2cSensors[i].bmp) {
        i2cSensors[i].temperature = i2cSensors[i].bmp->readTemperature();
        float pressure_pa = i2cSensors[i].bmp->readPressure();
        i2cSensors[i].pressure = pressure_pa / 100.0f;
        i2cSensors[i].altitude = i2cSensors[i].bmp->readAltitude(1013.25);

        if (isnan(i2cSensors[i].temperature) || isnan(i2cSensors[i].pressure)) {
          Serial.printf("⚠️ Error leyendo BMP280 [%s]\n", i2cSensors[i].name.c_str());
        }
      }
      else if (i2cSensors[i].sensorType == "BME280" && i2cSensors[i].bme) {
        i2cSensors[i].temperature = i2cSensors[i].bme->readTemperature();
        i2cSensors[i].humidity    = i2cSensors[i].bme->readHumidity();
        float pressure_pa = i2cSensors[i].bme->readPressure();
        i2cSensors[i].pressure = pressure_pa / 100.0f;
        i2cSensors[i].altitude = i2cSensors[i].bme->readAltitude(1013.25);

        if (isnan(i2cSensors[i].temperature)) {
          Serial.printf("⚠️ Error leyendo BME280 [%s]\n", i2cSensors[i].name.c_str());
        }
      }
    }
  }
}

void scanAndReportI2C() {
  StaticJsonDocument<512> doc;
  doc["type"] = "i2c_scan_result";
  JsonArray devices = doc.createNestedArray("devices");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      JsonObject dev = devices.createNestedObject();
      dev["address"] = addr;

      const char* type = "Unknown";
      if (addr == 0x38) type = "AHT20";
      else if (addr == 0x76 || addr == 0x77) {
        // Leer chip ID (registro 0xD0): BMP280=0x58, BME280=0x60
        Wire.beginTransmission(addr);
        Wire.write(0xD0);
        Wire.endTransmission(false);
        Wire.requestFrom(addr, (uint8_t)1);
        uint8_t chipId = Wire.available() ? Wire.read() : 0;
        type = (chipId == 0x60) ? "BME280" : "BMP280";
      }
      else if (addr == 0x23) type = "BH1750";
      else if (addr == 0x48) type = "ADS1115";

      dev["sensor_type"] = type;
    }
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
}

// ============================================
// GPIO LOOPS
// ============================================

void processGpioLoops() {
  for (int i = 0; i < gpioCount; i++) {
    if (!gpios[i].loopEnabled || gpios[i].mode != MODE_OUTPUT) continue;

    if (millis() - gpios[i].lastLoop >= gpios[i].loopInterval) {
      gpios[i].lastLoop = millis();
      gpios[i].value = !gpios[i].value;
      digitalWrite(gpios[i].pin, gpios[i].value);
    }
  }
}

// ============================================
// SENSORES ULTRASÓNICOS HC-SR04
// ============================================

float readUltrasonic(int trigPin, int echoPin, int maxDistance) {
  // Generar pulso de trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Medir tiempo de respuesta con timeout
  unsigned long timeout = maxDistance * 58 + 1000; // timeout en microsegundos
  unsigned long duration = pulseIn(echoPin, HIGH, timeout);

  if (duration == 0) {
    return -1; // Sin respuesta (fuera de rango)
  }

  // Calcular distancia en cm
  // Velocidad del sonido = 343 m/s = 0.0343 cm/µs
  // Distancia = (tiempo * velocidad) / 2
  float distance = duration * 0.0343 / 2.0;

  // Limitar al rango máximo
  if (distance > maxDistance) {
    return maxDistance;
  }

  return distance;
}

// Calcula la velocidad de movimiento basada en el buffer de lecturas
// Retorna velocidad en cm/s (cambio de distancia por tiempo)
float calculateMovementSpeed(int sensorIndex) {
  UltrasonicConfig& sensor = ultrasonicSensors[sensorIndex];

  if (!sensor.bufferFull && sensor.bufferIndex < 3) {
    return 0; // No hay suficientes lecturas
  }

  float totalChange = 0;
  int count = sensor.bufferFull ? 10 : sensor.bufferIndex;
  int validReadings = 0;

  for (int i = 1; i < count; i++) {
    int prevIdx = (sensor.bufferIndex - count + i + 10) % 10;
    int currIdx = (sensor.bufferIndex - count + i + 1 + 10) % 10;

    if (sensor.distanceBuffer[prevIdx] >= 0 && sensor.distanceBuffer[currIdx] >= 0) {
      totalChange += abs(sensor.distanceBuffer[currIdx] - sensor.distanceBuffer[prevIdx]);
      validReadings++;
    }
  }

  if (validReadings == 0) return 0;

  // Tiempo entre lecturas
  float timeSpan = validReadings * sensor.readInterval / 1000.0; // segundos

  return timeSpan > 0 ? totalChange / timeSpan : 0; // cm/s
}

// Verifica si hay movimiento real (no objeto estático)
// Umbral mínimo: 5 cm/s
bool isObjectMoving(int sensorIndex) {
  const float MIN_MOVEMENT_SPEED = 5.0; // cm/s
  float speed = calculateMovementSpeed(sensorIndex);
  return speed >= MIN_MOVEMENT_SPEED;
}

void readUltrasonicSensors() {
  for (int i = 0; i < ultrasonicCount; i++) {
    if (!ultrasonicSensors[i].active) continue;

    if (millis() - ultrasonicSensors[i].lastRead >= ultrasonicSensors[i].readInterval) {
      ultrasonicSensors[i].lastRead = millis();

      float distance = readUltrasonic(
        ultrasonicSensors[i].trigPin,
        ultrasonicSensors[i].echoPin,
        ultrasonicSensors[i].maxDistance
      );

      ultrasonicSensors[i].lastDistance = distance;

      // Agregar lectura al buffer de movimiento
      if (distance >= 0) {
        ultrasonicSensors[i].distanceBuffer[ultrasonicSensors[i].bufferIndex] = distance;
        ultrasonicSensors[i].bufferIndex = (ultrasonicSensors[i].bufferIndex + 1) % 10;
        if (ultrasonicSensors[i].bufferIndex == 0) {
          ultrasonicSensors[i].bufferFull = true;
        }
      }

      // Verificar detección con lógica inteligente LOCAL
      if (ultrasonicSensors[i].detectionEnabled && distance >= 0) {
        bool objectInRange = distance <= ultrasonicSensors[i].triggerDistance;
        bool objectMoving = isObjectMoving(i);

        // Calcular velocidad actual
        ultrasonicSensors[i].currentSpeed = calculateMovementSpeed(i);

        if (objectInRange && objectMoving) {
          // Objeto en movimiento detectado - iniciar timer si es nuevo
          if (ultrasonicSensors[i].detectionStartTime == 0) {
            ultrasonicSensors[i].detectionStartTime = millis();
            Serial.printf("🔍 Objeto en movimiento detectado a %.1f cm\n", distance);
          }

          // Activar GPIO trigger si corresponde
          if (!ultrasonicSensors[i].triggered && ultrasonicSensors[i].detectionEnabled) {
            ultrasonicSensors[i].triggered = true;
            ultrasonicSensors[i].triggerStartTime = millis();

            if (ultrasonicSensors[i].triggerGpioPin >= 0) {
              digitalWrite(ultrasonicSensors[i].triggerGpioPin,
                          ultrasonicSensors[i].triggerGpioValue);
              Serial.printf("🎯 Movimiento detectado a %.1f cm (vel: %.1f cm/s) - GPIO %d activado\n",
                distance, ultrasonicSensors[i].currentSpeed,
                ultrasonicSensors[i].triggerGpioPin);
            }
          }
        }
        else if (!objectInRange) {
          // Objeto salió del rango - resetear detección
          ultrasonicSensors[i].detectionStartTime = 0;

          if (ultrasonicSensors[i].triggered && ultrasonicSensors[i].triggerDuration == 0) {
            ultrasonicSensors[i].triggered = false;
            if (ultrasonicSensors[i].triggerGpioPin >= 0) {
              digitalWrite(ultrasonicSensors[i].triggerGpioPin,
                          !ultrasonicSensors[i].triggerGpioValue);
              Serial.printf("📤 GPIO %d desactivado\n", ultrasonicSensors[i].triggerGpioPin);
            }
          }
        }
        else if (objectInRange && !objectMoving) {
          ultrasonicSensors[i].currentSpeed = 0;
        }
      }
    }
  }
}

void processUltrasonicTriggers() {
  for (int i = 0; i < ultrasonicCount; i++) {
    if (!ultrasonicSensors[i].triggered) continue;
    if (ultrasonicSensors[i].triggerDuration == 0) continue; // Mantener indefinidamente

    // Verificar timeout del trigger
    if (millis() - ultrasonicSensors[i].triggerStartTime >= ultrasonicSensors[i].triggerDuration) {
      ultrasonicSensors[i].triggered = false;

      if (ultrasonicSensors[i].triggerGpioPin >= 0) {
        digitalWrite(ultrasonicSensors[i].triggerGpioPin,
                    !ultrasonicSensors[i].triggerGpioValue);
        Serial.printf("⏱️ Trigger timeout - GPIO %d desactivado\n",
          ultrasonicSensors[i].triggerGpioPin);
      }
    }
  }
}

// ============================================
// OTA
// ============================================

void startOTA(int id, String filename, int filesize, String checksum) {
  Serial.println("\n========================================");
  Serial.println("📦 Iniciando actualización OTA");
  Serial.print("Versión: ");
  Serial.println(filename);
  Serial.print("Tamaño: ");
  Serial.println(filesize);
  Serial.println("========================================\n");

  otaInProgress = true;
  otaId = id;
  otaFilename = filename;
  otaFilesize = filesize;
  otaChecksum = checksum;

  // Descargar firmware.
  // El esquema iba fijo a http:// aunque BACKEND_PORT fuera 443, asi que con SSL
  // la descarga fallaba siempre. syncTimeWithBackend() ya lo hacia bien.
  String url = String("http") + (BACKEND_PORT == 443 ? "s" : "") + "://" +
               BACKEND_HOST + ":" + String(BACKEND_PORT) +
               "/api/ota/download/" + filename;

  if (DEVICE_TOKEN.length() > 0) {
    url += "?token=" + DEVICE_TOKEN;
  }

  HTTPClient http;
  http.begin(url);
  http.setTimeout(30000);

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();

    if (contentLength > 0) {
      bool canBegin = Update.begin(contentLength);

      if (canBegin) {
        // El backend manda el MD5 del binario y hasta ahora se guardaba sin
        // comprobarlo: se grababa cualquier cosa que llegara por la red.
        if (otaChecksum.length() > 0) {
          if (!Update.setMD5(otaChecksum.c_str())) {
            Serial.println("❌ Checksum con formato inválido");
            Update.abort();
            reportOTAStatus("failed", "Invalid checksum format");
            http.end();
            otaInProgress = false;
            return;
          }
        } else {
          Serial.println("⚠️  El backend no envió checksum: no se puede verificar");
        }

        Serial.println("Descargando firmware...");

        WiFiClient* stream = http.getStreamPtr();
        size_t written = Update.writeStream(*stream);

        // Antes se avisaba del descuadre y se continuaba igualmente a Update.end()
        if (written != (size_t)contentLength) {
          Serial.printf("❌ Descarga incompleta: %u de %d bytes\n",
                        (unsigned)written, contentLength);
          Update.abort();
          reportOTAStatus("failed", "Incomplete download");
          http.end();
          otaInProgress = false;
          return;
        }

        Serial.println("Firmware descargado correctamente");

        // Update.end() valida aqui el MD5 fijado arriba
        if (Update.end(true)) {
          if (Update.isFinished()) {
            Serial.println("✅ OTA completado exitosamente");
            reportOTAStatus("success", "");

            Serial.println("Reiniciando...");
            delay(1000);
            ESP.restart();
          } else {
            reportOTAStatus("failed", "Update not finished");
          }
        } else {
          String error = String(Update.getError());
          Serial.print("❌ Error OTA: ");
          Serial.println(error);
          reportOTAStatus("failed", error);
        }
      } else {
        reportOTAStatus("failed", "Not enough space");
      }
    }
  } else {
    String error = "HTTP error: " + String(httpCode);
    Serial.println(error);
    reportOTAStatus("failed", error);
  }

  http.end();
  otaInProgress = false;
}

void reportOTAStatus(const char* status, String error) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ota_status";
  doc["mac_address"] = deviceMac;
  doc["ota_id"] = otaId;
  doc["status"] = status;
  if (error.length() > 0) {
    doc["error"] = error;
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
}

// ============================================
// CONFIGURACIÓN PERSISTENTE
// ============================================

void loadConfig() {
  preferences.begin("minios", true);

  // Usar valores hardcodeados como default si no hay nada en NVS
  String savedSSID = preferences.getString("wifi_ssid", "");
  String savedPass = preferences.getString("wifi_pass", "");
  String savedHost = preferences.getString("backend_host", "");
  String savedToken = preferences.getString("device_token", "");
  int savedPort = preferences.getInt("backend_port", 0);
  uint32_t savedSleep = preferences.getUInt("sleep_secs", 0);

  // Solo sobrescribir si hay valores guardados
  if (savedSSID.length() > 0) {
    WIFI_SSID = savedSSID;
    WIFI_PASS = savedPass;
  }
  if (savedHost.length() > 0) {
    BACKEND_HOST = savedHost;
    BACKEND_PORT = savedPort;
  }
  if (savedToken.length() > 0) {
    DEVICE_TOKEN = savedToken;
  }
  if (savedSleep >= 5) {
    deepSleepDuration = savedSleep;
  }

  preferences.end();

  Serial.println("Configuración cargada");
  Serial.print("WiFi SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Backend: ");
  Serial.print(BACKEND_HOST);
  Serial.print(":");
  Serial.println(BACKEND_PORT);
  Serial.print("Token de dispositivo: ");
  Serial.println(DEVICE_TOKEN.length() > 0 ? "configurado" : "(ninguno)");
}

void saveConfig() {
  preferences.begin("minios", false);

  preferences.putString("wifi_ssid", WIFI_SSID);
  preferences.putString("wifi_pass", WIFI_PASS);
  preferences.putString("backend_host", BACKEND_HOST);
  preferences.putString("device_token", DEVICE_TOKEN);
  preferences.putInt("backend_port", BACKEND_PORT);
  preferences.putUInt("sleep_secs", (uint32_t)deepSleepDuration);

  preferences.end();

  Serial.println("✅ Configuración guardada");
}

// ============================================
// COMANDOS SERIAL
// ============================================

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) return;

  // Parsear comando
  int space1 = input.indexOf(' ');
  String cmd = (space1 > 0) ? input.substring(0, space1) : input;
  String args = (space1 > 0) ? input.substring(space1 + 1) : "";

  if (cmd == "wifi") {
    // El corte se hacia por el PRIMER espacio, asi que un SSID como
    // "CASA ROJAS" se partia en SSID="CASA" y contrasena="ROJAS <clave>".
    // Ahora: con comillas se delimita el SSID; sin ellas se corta por el
    // ULTIMO espacio, de modo que el SSID puede llevar espacios.
    String ssid = "";
    String pass = "";
    bool ok = false;

    if (args.startsWith("\"")) {
      int endQuote = args.indexOf('"', 1);
      if (endQuote > 1) {
        ssid = args.substring(1, endQuote);
        pass = args.substring(endQuote + 1);
        pass.trim();
        ok = true;
      }
    } else {
      int lastSpace = args.lastIndexOf(' ');
      if (lastSpace > 0) {
        ssid = args.substring(0, lastSpace);
        pass = args.substring(lastSpace + 1);
        ssid.trim();
        ok = true;
      }
    }

    if (ok && ssid.length() > 0 && pass.length() > 0) {
      WIFI_SSID = ssid;
      WIFI_PASS = pass;

      // Eco para poder verificar el corte sin exponer la contrasena
      Serial.print("SSID: [");
      Serial.print(WIFI_SSID);
      Serial.print("]  contraseña: ");
      Serial.print(WIFI_PASS.length());
      Serial.println(" caracteres");

      saveConfig();
      wifiFailCount = 0;  // credenciales nuevas: se empieza a contar de cero
      connectWiFi();
    } else {
      Serial.println("Uso: wifi <ssid> <password>");
      Serial.println("  Si el SSID lleva espacios, entre comillas:");
      Serial.println("  wifi \"CASA ROJAS\" miclave");
      Serial.println("  (sin comillas se toma como contraseña lo que va tras el último espacio)");
    }
  }
  else if (cmd == "server") {
    int space2 = args.indexOf(' ');
    if (space2 > 0) {
      BACKEND_HOST = args.substring(0, space2);
      BACKEND_PORT = args.substring(space2 + 1).toInt();
      saveConfig();

      if (WiFi.status() == WL_CONNECTED) {
        connectWebSocket();
      }
    } else {
      Serial.println("Uso: server <host> <port>");
    }
  }
  else if (cmd == "token") {
    if (args.length() > 0) {
      DEVICE_TOKEN = (args == "-") ? "" : args;
      saveConfig();
      Serial.println(DEVICE_TOKEN.length() > 0
                     ? "✅ Token guardado"
                     : "✅ Token borrado");
      if (WiFi.status() == WL_CONNECTED) {
        connectWebSocket();
      }
    } else {
      Serial.println("Uso: token <valor>   (usa '-' para borrarlo)");
    }
  }
  else if (cmd == "status") {
    Serial.println("\n--- Estado ---");
    Serial.print("MAC: ");
    Serial.println(deviceMac);
    Serial.print("WiFi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Backend: ");
    Serial.print(BACKEND_HOST);
    Serial.print(":");
    Serial.println(BACKEND_PORT);
    Serial.print("Token: ");
    Serial.println(DEVICE_TOKEN.length() > 0 ? "configurado" : "(ninguno)");
    Serial.print("Registrado: ");
    Serial.println(isRegistered ? "Sí" : "No");
    Serial.print("GPIOs: ");
    Serial.println(gpioCount);
    Serial.print("DHT: ");
    Serial.println(dhtCount);
    Serial.println("--------------\n");
  }
  else if (cmd == "reboot") {
    Serial.println("Reiniciando...");
    delay(500);
    ESP.restart();
  }
  else {
    Serial.println("Comando no reconocido");
  }
}

// ============================================
// AHORRO DE ENERGÍA - DEEP SLEEP
// ============================================

void enterDeepSleep(unsigned long seconds) {
  Serial.println("\n========================================");
  Serial.printf("💤 Entrando en Deep Sleep por %lu segundos\n", seconds);
  Serial.println("========================================\n");

  // Guardar cualquier dato pendiente
  Serial.flush();
  delay(100);

  // Configurar timer para despertar
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);  // microsegundos

  // Apagar LED
  digitalWrite(LED_STATUS, LOW);
  #if HAS_RGB_LED
    rgbLed.clear();
    rgbLed.show();
  #endif

  // Entrar en Deep Sleep
  Serial.println("😴 Durmiendo...");
  delay(100);
  esp_deep_sleep_start();

  // Esta línea nunca se ejecuta (el ESP32 se reinicia al despertar)
}
