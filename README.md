# MiniOS WiFi Client

Firmware ligero para ESP32 que se conecta al backend centralizado MiniOS.

## Diferencias con MiniOS_WiFi Original

| Característica | Original | Client |
|----------------|----------|--------|
| Servidor Web | Sí (HTML embebido) | No |
| Tamaño Flash | ~800KB | ~300KB |
| Configuración | Local via web | Remota via backend |
| Acceso | Solo red local | Desde cualquier lugar |

## Librerías Requeridas

Instalar desde el Library Manager del IDE (o con `arduino-cli lib install`):

1. **WebSockets** — Markus Sattler (probado con 2.7.2)
2. **ArduinoJson** — Benoit Blanchon (probado con 7.4.3; el código usa la API v6,
   que sigue funcionando en v7 aunque avise de obsolescencia)
3. **DHT sensor library** — Adafruit (1.4.7) + **Adafruit Unified Sensor**
4. **Adafruit AHTX0** (2.0.6)
5. **Adafruit BMP280 Library** (3.0.0)
6. **Adafruit BME280 Library** (2.3.0)
7. **Adafruit NeoPixel** (1.15.5) — solo se usa en ESP32 y ESP32-S3

## Opciones de compilación

El firmware **no cabe** con el esquema de particiones por defecto: hay que
elegir uno con más espacio de aplicación que conserve las dos particiones OTA.

| Opción del IDE | Valor |
|----------------|-------|
| Partition Scheme | **Minimal SPIFFS (1.9MB APP with OTA)** |
| USB CDC On Boot | **Enabled** (si no, los logs salen por UART0 y no por el USB) |

Con `arduino-cli`:

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs \
  MiniOS_WiFi_Client
```

Tamaños verificados: **ESP32-C3 1,32 MB (67%)** · **ESP32-S3 1,21 MB (61%)**.

## Configuración Inicial

> **Las credenciales no van en el código.** El archivo `.ino` está en un
> repositorio público y el historial de git conserva para siempre lo que se
> escriba en él. `WIFI_SSID` y `WIFI_PASS` se dejan vacíos y se configuran por
> el puerto serie: quedan guardados en NVS y sobreviven a los reinicios.

### Via Serial (115200 baud)

1. Configurar WiFi:
```
wifi MiRed MiPassword
```

2. Configurar backend:
```
server 192.168.1.100 3000
```

O si tienes dominio:
```
server mi-servidor.com 3000
```

3. Ver estado:
```
status
```

4. Token de dispositivo (solo si el backend tiene `DEVICE_TOKEN` configurado):
```
token el-token-del-servidor
```
Se envía en la URL del WebSocket y en las descargas OTA. Con `token -` se borra.

5. Reiniciar:
```
reboot
```

## Funcionamiento

1. El ESP32 se conecta al WiFi configurado
2. Establece conexión WebSocket con el backend
3. Se registra automáticamente usando su MAC Address
4. Recibe configuraciones (GPIO, DHT) del backend
5. Envía datos de sensores cada 5 segundos
6. Ejecuta comandos recibidos (set GPIO, reiniciar, OTA)

## Actualizaciones OTA

1. Subir archivo .bin al backend desde el dashboard
2. Seleccionar "Enviar a todos" o actualizar dispositivo específico
3. El ESP32 descarga e instala automáticamente
4. Se reinicia con el nuevo firmware

## Estructura de Mensajes

### Registro (ESP32 → Backend)
```json
{
  "type": "register",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "firmware_version": "1.0.0",
  "ip_address": "192.168.1.50"
}
```

### Datos de Sensores (ESP32 → Backend)
```json
{
  "type": "data",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "payload": {
    "temperature": 25.5,
    "humidity": 60,
    "gpio": [
      {"pin": 2, "value": 1},
      {"pin": 4, "value": 0}
    ]
  }
}
```

### Comando GPIO (Backend → ESP32)
```json
{
  "type": "command",
  "action": "set_gpio",
  "pin": 2,
  "value": 1
}
```

### Comando OTA (Backend → ESP32)
```json
{
  "type": "command",
  "action": "ota_update",
  "ota_id": 1,
  "filename": "firmware_1.0.1.bin",
  "filesize": 512000,
  "checksum": "abc123..."
}
```

## Compilación

### Arduino IDE

1. Board: "ESP32S3 Dev Module"
2. Partition Scheme: "Default 4MB with spiffs"
3. Upload Speed: 921600

### PlatformIO

```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps =
    links2004/WebSockets@^2.4.0
    bblanchon/ArduinoJson@^6.21.0
    adafruit/DHT sensor library@^1.4.4
```

## Troubleshooting

### No conecta al WiFi
- Verificar SSID y contraseña
- El ESP32 debe estar cerca del router

### No conecta al backend
- Verificar IP/puerto del servidor
- El backend debe estar corriendo
- Verificar firewall del VPS

### OTA falla
- Verificar espacio en flash
- El archivo .bin debe ser válido
- Conexión estable durante descarga

## Pines por Defecto

- **LED Status**: GPIO 2 (LED integrado)
- Los demás GPIOs se configuran desde el dashboard
