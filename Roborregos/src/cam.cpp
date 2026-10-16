#include <Arduino.h>
#include "esp_camera.h"
#include "aruco_dict_4x4_50.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Pines LIBRES en la ESP32-CAM.
const int PIN_TX_A_DEVKIT = 14; // sale de la CAM hacia el RX del DevKit
const int PIN_RX_DESDE_DEVKIT = 13; // entra a la CAM desde el TX del DevKit (no se usa por ahora, pero queda listo)


void setup() {
  Serial.begin(115200);

  // Monitor serial para que la camara le mande datos a el DevKit
  Serial2.begin(115200, SERIAL_8N1, PIN_RX_DESDE_DEVKIT, PIN_TX_A_DEVKIT);

camera_config_t config;
config.pin_pwdn = CAM_PIN_PWDN;
config.pin_reset = CAM_PIN_RESET;
config.pin_xclk = CAM_PIN_XCLK;
config.pin_sscb_sda = CAM_PIN_SIOD;
config.pin_sscb_scl = CAM_PIN_SIOC;
config.pin_d7 = CAM_PIN_D7;
config.pin_d6 = CAM_PIN_D6;
config.pin_d5 = CAM_PIN_D5;
config.pin_d4 = CAM_PIN_D4;
config.pin_d3 = CAM_PIN_D3;
config.pin_d2 = CAM_PIN_D2;
config.pin_d1 = CAM_PIN_D1;
config.pin_d0 = CAM_PIN_D0;
config.pin_vsync = CAM_PIN_VSYNC;
config.pin_href = CAM_PIN_HREF;
config.pin_pclk = CAM_PIN_PCLK;
config.xclk_freq_hz = 20000000;
config.ledc_timer = LEDC_TIMER_0;
config.ledc_channel = LEDC_CHANNEL_0;
config.pixel_format = PIXFORMAT_RGB565;
config.frame_size = FRAMESIZE_QQVGA;
config.fb_count = 1;

// esta funcion checa si la camara esta conectada y si no lo esta, imprime un mensaje de error en el monitor serial
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("La camara no esta disponible");
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);

  Serial.println("Camara conectada");
}

// esta funcion rectangulo es para limitar el cuadro de imagen que analizara para no teneer que analizar tantos pixeles y asi ahorrar tiempo de procesamiento
struct Rectangulo {
  int xMin, xMax, yMin, yMax;
  bool encontrado;
};

// esta funcion obtiene el valor de un pixel del buffer de la camara 
uint16_t obtenerPixel(camera_fb_t *fb, int x, int y) {
  uint16_t *pixels = (uint16_t *)fb->buf;
  uint16_t pixel = pixels[y * fb->width + x];
  pixel = (pixel >> 8) | (pixel << 8);  
  return pixel;
}

//  Separa un pixel en el formato RGB565 en sus 3 canales, normalizados a 0-255. Este formato tiene 5 bits para rojo, 6 bits para verde y 5 bits para azul
void extraerRGB(uint16_t pixel, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t r5 = (pixel >> 11) & 0x1F;
  uint8_t g6 = (pixel >> 5) & 0x3F;
  uint8_t b5 = pixel & 0x1F;

  r = (r5 * 255) / 31;
  g = (g6 * 255) / 63;
  b = (b5 * 255) / 31;
}

// calcula brillo a partir del color, para que el ArUco funcione sin grises, se calcula el brillo promedio de los 3 canales de color
uint8_t obtenerBrillo(camera_fb_t *fb, int x, int y) {
  uint16_t pixel = obtenerPixel(fb, x, y);
  uint8_t r, g, b;
  extraerRGB(pixel, r, g, b);
  return (r + g + b) / 3;
}

// Aqui se checa si el color es naranja
bool esNaranja(uint8_t r, uint8_t g, uint8_t b) {
  bool rojoAlto = r > 200;
  bool diferenciaRB = (r - b) > 100;
  return rojoAlto && diferenciaRB;
}

// Encuentra el rectángulo que envuelve la mancha naranja que detecte la camara
Rectangulo encontrarRegionNaranja(camera_fb_t *fb) {
  int xMin = fb->width, xMax = 0;
  int yMin = fb->height, yMax = 0;
  bool encontrado = false;

  for (int y = 0; y < fb->height; y++) {
    for (int x = 0; x < fb->width; x++) {
      uint16_t pixel = obtenerPixel(fb, x, y);
      uint8_t r, g, b;
      extraerRGB(pixel, r, g, b);

      if (esNaranja(r, g, b)) {
        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;
        if (y < yMin) yMin = y;
        if (y > yMax) yMax = y;
        encontrado = true;
      }
    }
  }
  return {xMin, xMax, yMin, yMax, encontrado};
}

// Encuentra la seccion rectangular que envuelve la region oscura (para ArUco)
Rectangulo encontrarRegionOscura(camera_fb_t *fb, uint8_t umbral) {
  int xMin = fb->width, xMax = 0;
  int yMin = fb->height, yMax = 0;
  bool encontrado = false;

  for (int y = 0; y < fb->height; y++) {
    for (int x = 0; x < fb->width; x++) {
      uint8_t brillo = obtenerBrillo(fb, x, y);
      if (brillo < umbral) {
        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;
        if (y < yMin) yMin = y;
        if (y > yMax) yMax = y;
        encontrado = true;
      }
    }
  }
  return {xMin, xMax, yMin, yMax, encontrado};
}

// esta funcion calcula el brillo promedio de toda la imagen, para poder calcular un umbral de deteccion de ArUco
uint32_t brilloPromedio(camera_fb_t *fb) {
  uint32_t suma = 0;
  for (int y = 0; y < fb->height; y++) {
    for (int x = 0; x < fb->width; x++) {
      suma += obtenerBrillo(fb, x, y);
    }
  }
  return suma / (fb->width * fb->height);
}

uint16_t leerPatronBits(camera_fb_t *fb, Rectangulo r, uint8_t umbral) {
  float anchoCelda = (float)(r.xMax - r.xMin) / 6.0;
  float altoCelda = (float)(r.yMax - r.yMin) / 6.0;

  uint16_t patron = 0;

  for (int fila = 0; fila < 4; fila++) {
    for (int col = 0; col < 4; col++) {
      float px = r.xMin + (col + 1 + 0.5) * anchoCelda;
      float py = r.yMin + (fila + 1 + 0.5) * altoCelda;

      uint8_t brillo = obtenerBrillo(fb, (int)px, (int)py);
      uint8_t bit = (brillo > umbral) ? 1 : 0;

      patron = (patron << 1) | bit;
    }
  }
  return patron;
}

uint16_t rotar90(uint16_t patron) {
  uint16_t rotado = 0;
  for (int fila = 0; fila < 4; fila++) {
    for (int col = 0; col < 4; col++) {
      int bitOriginal = (patron >> (15 - (fila * 4 + col))) & 1;
      int nuevaFila = col;
      int nuevaCol = 3 - fila;
      int posDestino = 15 - (nuevaFila * 4 + nuevaCol);
      rotado |= (bitOriginal << posDestino);
    }
  }
  return rotado;
}

int identificarMarcador(uint16_t patronLeido) {
  uint16_t patronRotado = patronLeido;
  for (int rotacion = 0; rotacion < 4; rotacion++) {
    for (int id = 0; id < 50; id++) {
      if (patronRotado == codigosArUco[id]) {
        return id;
      }
    }
    patronRotado = rotar90(patronRotado);
  }
  return -1;
}

const float TAMANO_REAL_MM = 42.0;           // diámetro real de la pelota de golf
const float CONSTANTE_CALIBRACION = 1000.0;  // <-- VALOR TEMPORAL, hay que recalcularlo

float calcularDistanciaMM(int anchoEnPixeles) {
  if (anchoEnPixeles <= 0) return -1;  // evita división entre cero
  return (TAMANO_REAL_MM * CONSTANTE_CALIBRACION) / anchoEnPixeles;
}


// Aqui se realiza la detección de la pelota y el cálculo de la distancia
// en el Serial Monitor de la CAM. 
bool pelotaCercaParaCerrar(float distanciaMM, float umbralMM) {
  if (distanciaMM <= 0) return false;
  return distanciaMM <= umbralMM;
}


void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Fallo al capturar");
    return;
  }

  // deteccion de ArUco 
  uint32_t brillo = brilloPromedio(fb);
  uint8_t umbral = brillo * 0.6;
  Rectangulo r = encontrarRegionOscura(fb, umbral);

  if (r.encontrado) {
    uint16_t patron = leerPatronBits(fb, r, umbral);
    int id = identificarMarcador(patron);
    if (id != -1) {
      Serial.printf(">>> ARUCO: ID %d <<<\n", id);
    }
  }
// aqui termina la parte de ArUco y empieza la parte de deteccion de la pelota

  Rectangulo naranja = encontrarRegionNaranja(fb);
  float distanciaMM = -1;

  if (naranja.encontrado) {
    int anchoPx = naranja.xMax - naranja.xMin;
    distanciaMM = calcularDistanciaMM(anchoPx);
    Serial.printf("Naranja: ancho=%dpx, distancia estimada=%.1fmm (%.1fcm)\n",
                  anchoPx, distanciaMM, distanciaMM / 10.0);
  }
// esta funcion ayuda a enviar la distancia de la pelota al DevKit para que decida si cerrar la garra
  Serial2.printf("%.1f\n", distanciaMM);

 
  const float UMBRAL_DEBUG_MM = 300.0;
  bool cerrarDebug = pelotaCercaParaCerrar(distanciaMM, UMBRAL_DEBUG_MM);
  Serial.printf("-> Enviado a DevKit: %.1fmm (cerraria=%s)\n",
                distanciaMM, cerrarDebug ? "SI" : "NO");


  esp_camera_fb_return(fb);
  delay(500);
}