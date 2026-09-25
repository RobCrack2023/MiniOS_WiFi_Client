/*
 * Prueba de 2 micrófonos INMP441 (I2S) en estéreo
 *
 * Sketch independiente para comprobar el cableado antes de integrar los
 * micrófonos en MiniOS_WiFi_Client. Imprime el nivel de sonido de cada canal
 * en dB SPL aproximados, en formato compatible con el Serial Plotter.
 *
 * Cableado: los dos micrófonos comparten el mismo bus I2S.
 *
 *   INMP441    Mic A (izq.)   Mic B (der.)
 *   VDD        3V3            3V3
 *   GND        GND            GND
 *   SCK        PIN_SCK        PIN_SCK
 *   WS         PIN_WS         PIN_WS
 *   SD         PIN_SD         PIN_SD
 *   L/R        GND            3V3        <- esto es lo que los separa
 *
 * El INMP441 deja su pin SD en alta impedancia mientras le toca hablar al otro
 * canal, por eso los dos pueden compartir la misma línea de datos.
 *
 * Requiere el core ESP32 de Arduino 3.x (librería ESP_I2S incluida en el core).
 */

#include <Arduino.h>
#include <ESP_I2S.h>

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define PIN_SCK 6
  #define PIN_WS  7
  #define PIN_SD  5
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #define PIN_SCK 15
  #define PIN_WS  16
  #define PIN_SD  17
#else  // ESP32 clásico
  #define PIN_SCK 26
  #define PIN_WS  25
  #define PIN_SD  33
#endif

// 1 = solo el micrófono con L/R a GND (canal izquierdo); 2 = los dos
#define NUM_MICS      1

#define SAMPLE_RATE   16000
#define WINDOW_MS     125   // Ventana de medición: 125 ms = "fast" de un sonómetro
#define FRAMES        (SAMPLE_RATE * WINDOW_MS / 1000)

// Sensibilidad del INMP441 según hoja de datos: 94 dB SPL -> -26 dBFS.
// dB SPL = dBFS + 94 + 26. Sin calibrar, espera un error de unos ±3 dB.
#define DBFS_TO_SPL   120.0f

I2SClass i2s;
int32_t frameBuf[FRAMES * 2];  // Intercalado: izq, der, izq, der...

// Nivel RMS de un canal en dBFS, sin la componente continua que trae el micrófono
float channelDbfs(int channel) {
  double sum = 0;
  for (int i = 0; i < FRAMES; i++) {
    sum += frameBuf[i * 2 + channel] >> 8;  // Los 24 bits útiles van alineados arriba
  }
  double mean = sum / FRAMES;

  double sq = 0;
  for (int i = 0; i < FRAMES; i++) {
    double s = (frameBuf[i * 2 + channel] >> 8) - mean;
    sq += s * s;
  }
  double rms = sqrt(sq / FRAMES);
  if (rms < 1) rms = 1;  // Evita log10(0) si el canal está mudo
  return 20.0f * log10(rms / 8388608.0);  // 2^23 = fondo de escala de 24 bits
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\nINMP441 x2 - SCK:%d WS:%d SD:%d @ %d Hz\n", PIN_SCK, PIN_WS, PIN_SD, SAMPLE_RATE);

  i2s.setPins(PIN_SCK, PIN_WS, -1, PIN_SD);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("❌ No se pudo iniciar I2S");
    while (true) delay(1000);
  }

  // El INMP441 entrega ceros o basura durante ~85 ms tras recibir reloj
  delay(200);
  i2s.readBytes((char*)frameBuf, sizeof(frameBuf));
  Serial.println("✅ I2S listo. Abre el Serial Plotter para ver las curvas.");
}

void loop() {
  size_t got = i2s.readBytes((char*)frameBuf, sizeof(frameBuf));
  if (got != sizeof(frameBuf)) {
    Serial.printf("⚠️ Lectura incompleta: %u de %u bytes\n", got, sizeof(frameBuf));
    return;
  }

  float left = channelDbfs(0) + DBFS_TO_SPL;

  // Un canal clavado en ~0-20 dB suele ser un micrófono sin alimentar, con el
  // L/R mal puesto o con SD desconectado: el pin queda flotando en cero.
#if NUM_MICS == 2
  float right = channelDbfs(1) + DBFS_TO_SPL;
  Serial.printf("Izq_dB:%.1f Der_dB:%.1f\n", left, right);
#else
  Serial.printf("Mic_dB:%.1f\n", left);
#endif
}
