/*
 * MiniOS WiFi Client
 * Versión ligera para conectar al backend centralizado
 *
 * Características:
 * - Conexión WebSocket al backend
 * - Identificación por MAC Address
 * - Control GPIO remoto
 * - Sensores DHT
 * - Actualizaciones OTA
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <DHT.h>

// ============================================
// CONFIGURACIÓN
// ============================================

#define FIRMWARE_VERSION "1.0.0"

// WiFi - Configurar aquí o vía Serial
String WIFI_SSID = "CASA ROJAS";
String WIFI_PASS = "STD2024....";

// Backend
String BACKEND_HOST = "minios.iot-robotics.cl";  // Dominio del backend
int BACKEND_PORT = 443;  // Puerto 80 con Nginx, 443 con SSL

// Hardware
#define MAX_GPIOS 20
#define MAX_DHT_SENSORS 4
#define LED_STATUS 2  // LED integrado

// ============================================
// CLASIFICACIÓN DE GPIOs ESP32-S3
// ============================================

// GPIOs Analógicas (ADC)
const int ANALOG_GPIOS[] = {1, 2, 4, 5, 6, 7};
const int ANALOG_GPIOS_COUNT = 6;

// GPIOs Digitales (uso general)
const int DIGITAL_GPIOS[] = {0, 3, 14, 15, 16, 17, 18, 19, 20, 21, 36, 37, 38, 39, 40, 41, 42, 45, 46};
const int DIGITAL_GPIOS_COUNT = 19;

// GPIOs I2C (por defecto)
const int I2C_GPIOS[] = {8, 9};  // 8=SDA, 9=SCL
const int I2C_GPIOS_COUNT = 2;

// GPIOs SPI (usar con precaución)
const int SPI_GPIOS[] = {10, 11, 12, 13};
const int SPI_GPIOS_COUNT = 4;

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

// ============================================
// VARIABLES GLOBALES
// ============================================

WebSocketsClient webSocket;
Preferences preferences;

GPIOConfig gpios[MAX_GPIOS];
int gpioCount = 0;

DHTConfig dhtSensors[MAX_DHT_SENSORS];
int dhtCount = 0;

String deviceMac;
int deviceId = 0;
bool isRegistered = false;

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
void connectWebSocket();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void registerDevice();
void handleWebSocketMessage(const char* payload);
void handleConfig(JsonDocument& doc);
void handleCommand(JsonDocument& doc);
void sendSensorData();
void readDHTSensors();
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

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("MiniOS WiFi Client v" FIRMWARE_VERSION);
  Serial.println("========================================");

  // LED de estado
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

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
  Serial.println("  server <host> <port> - Configurar backend");
  Serial.println("  status - Ver estado");
  Serial.println("  reboot - Reiniciar");

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
  // Procesar comandos Serial
  handleSerial();

  // WebSocket loop
  webSocket.loop();

  // Verificar conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnect > RECONNECT_INTERVAL) {
      lastReconnect = millis();
      connectWiFi();
    }
    return;
  }

  // Leer sensores DHT
  readDHTSensors();

  // Procesar loops GPIO
  processGpioLoops();

  // Enviar datos periódicamente
  if (isRegistered && millis() - lastDataSend > DATA_SEND_INTERVAL) {
    lastDataSend = millis();
    sendSensorData();
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
// WIFI
// ============================================

void connectWiFi() {
  if (WIFI_SSID.length() == 0) return;

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
    Serial.println("\n✅ WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_STATUS, HIGH);

    // Conectar al backend
    if (BACKEND_HOST.length() > 0) {
      connectWebSocket();
    } else {
      Serial.println("⚠️  Backend no configurado. Usa: server <host> <port>");
    }
  } else {
    Serial.println("\n❌ Error conectando WiFi");
    digitalWrite(LED_STATUS, LOW);
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

  // Usar SSL para puerto 443, sin SSL para otros puertos
  if (BACKEND_PORT == 443) {
    Serial.println("Usando conexión SSL (wss://)");
    webSocket.beginSSL(BACKEND_HOST.c_str(), BACKEND_PORT, "/ws/device");
  } else {
    Serial.println("Usando conexión sin SSL (ws://)");
    webSocket.begin(BACKEND_HOST.c_str(), BACKEND_PORT, "/ws/device");
  }

  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
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

  String json;
  serializeJson(doc, json);

  // Debug: mostrar lo que se envía
  Serial.println("📱 Enviando registro al backend:");
  Serial.println(json);

  webSocket.sendTXT(json);
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
  }
}

void handleConfig(JsonDocument& doc) {
  deviceId = doc["device_id"];
  isRegistered = true;

  Serial.print("✅ Registrado como dispositivo ID: ");
  Serial.println(deviceId);

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

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
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

  // Descargar firmware
  String url = "http://" + BACKEND_HOST + ":" + String(BACKEND_PORT) + "/api/ota/download/" + filename;

  HTTPClient http;
  http.begin(url);
  http.setTimeout(30000);

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();

    if (contentLength > 0) {
      bool canBegin = Update.begin(contentLength);

      if (canBegin) {
        Serial.println("Descargando firmware...");

        WiFiClient* stream = http.getStreamPtr();
        size_t written = Update.writeStream(*stream);

        if (written == contentLength) {
          Serial.println("Firmware descargado correctamente");
        } else {
          Serial.print("Error: solo se escribieron ");
          Serial.print(written);
          Serial.print(" de ");
          Serial.println(contentLength);
        }

        if (Update.end()) {
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
  int savedPort = preferences.getInt("backend_port", 0);

  // Solo sobrescribir si hay valores guardados
  if (savedSSID.length() > 0) {
    WIFI_SSID = savedSSID;
    WIFI_PASS = savedPass;
  }
  if (savedHost.length() > 0) {
    BACKEND_HOST = savedHost;
    BACKEND_PORT = savedPort;
  }

  preferences.end();

  Serial.println("Configuración cargada");
  Serial.print("WiFi SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Backend: ");
  Serial.print(BACKEND_HOST);
  Serial.print(":");
  Serial.println(BACKEND_PORT);
}

void saveConfig() {
  preferences.begin("minios", false);

  preferences.putString("wifi_ssid", WIFI_SSID);
  preferences.putString("wifi_pass", WIFI_PASS);
  preferences.putString("backend_host", BACKEND_HOST);
  preferences.putInt("backend_port", BACKEND_PORT);

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
    int space2 = args.indexOf(' ');
    if (space2 > 0) {
      WIFI_SSID = args.substring(0, space2);
      WIFI_PASS = args.substring(space2 + 1);
      saveConfig();
      connectWiFi();
    } else {
      Serial.println("Uso: wifi <ssid> <password>");
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
