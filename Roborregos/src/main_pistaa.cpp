// pista A: laberinto 5x5
// explora todo con BFS, va a la meta y regresa por el mismo camino (bonus 1)
// la camara manda "A<id>" cuando ve un ArUco y se ensena en la pantalla (bonus 2)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_SSD1306.h>

Adafruit_MPU6050 mpu;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_SSD1306 pantalla(128, 64, &Wire, -1);

//declaracion de pines
const int PWMA = 25, AIN1 = 26, AIN2 = 27;   // motor derecho
const int PWMB = 13, BIN1 = 32, BIN2 = 33;   // motor izquierdo
const int STBY = 14;
const int ENC_DER_A = 34, ENC_DER_B = 35;
const int ENC_IZQ_A = 36, ENC_IZQ_B = 39;
const int TRIG = 5;                           // compartido
const int ECHO_FRENTE = 19, ECHO_IZQ = 18, ECHO_DER = 23;
const int CAM_RX = 16, CAM_TX = 17;

// Calibraciones
const int   FRECUENCIA_PWM   = 20000; // 20 kHz para que los motores no zumben o hagan algun ruido
const int   VELOCIDAD        = 175;
const int   VELOCIDAD_RAMPA  = 230;   // subida y escalones, falta calibrar
const int   VELOCIDAD_BAJADA = 130;   // falta calibrar
const float KP_RECTO         = 2.0;   // falta calibrar
const float CELDA_CM         = 30.0;
const float LLANTA_CM        = 23.5;  // circunferencia, falta calibrar
const int   PULSOS_VUELTA    = 360;   // falta calibrar
const float PARED_CM         = 12.0;  // falta calibrar
const float ANGULO_GIRO      = 88.0;  // falta calibrar
const float ANGULO_INCLINADO = 8.0;   // positivo = subiendo, revisar el signo con MODO_PRUEBA
const float SOUND_SPEED      = 0.034;
const bool  MODO_PRUEBA      = false; // true = solo imprime los sensores

// vueltas que da la llanta en una celda x pulsos por vuelta formula hecha con apoyo de ia y fuentes
const long PULSOS_CELDA = (long)(CELDA_CM / LLANTA_CM * PULSOS_VUELTA);

// laberinto, se cambia en la calibracion de cada ronda
const int N = 5;
const int START_X = 0, START_Y = 0, START_DIR = 0;   // 0 norte, 1 este, 2 sur, 3 oeste
const int GOAL_X = 4, GOAL_Y = 0;
const int MAX_PASOS = 150;
const unsigned long TIEMPO_EXPLORAR_MS = 150000;

enum Color { C_NADA, C_BLANCO, C_AZUL, C_AMARILLO, C_NARANJA, C_ROSA, C_VERDE, C_ROJO };
const char* NOMBRES[] = { "?", "BLANCO", "AZUL CLARO", "AMARILLO", "NARANJA", "ROSA", "VERDE", "ROJO" };
// proporcion r, g, b de cada color, falta medirlos con MODO_PRUEBA para asi saber si el sensor de color esta bien calibrado para los colores de la pista
const float REF[8][3] = { {0, 0, 0}, {.31, .35, .34}, {.17, .39, .44}, {.44, .40, .16},
                          {.55, .27, .18}, {.45, .22, .33}, {.22, .48, .30}, {.60, .20, .20} };

const int dx[4] = { 0, 1, 0, -1 };   // N, E, S, O
const int dy[4] = { 1, 0, -1, 0 };

volatile long posIzq = 0, posDer = 0;
float gzOffset = 0;
int curX = START_X, curY = START_Y, currentDir = START_DIR;
bool visited[N][N], wallMap[N][N][4], colorVisto[N][N];
int ruta[MAX_PASOS + 1][2];   // celdas por las que paso, para el bonus 1
int largoRuta = 0;
int arucoVisto = -1;

// encoder: cuando sube el canal A, el canal B dice si va adelante o atras (fuente: Curio Res)
void IRAM_ATTR readEncoderIzq() {
    int b = digitalRead(ENC_IZQ_B);
    if (b > 0) posIzq = posIzq + 1;
    else       posIzq = posIzq - 1;
}
void IRAM_ATTR readEncoderDer() {
    int b = digitalRead(ENC_DER_B);
    if (b > 0) posDer = posDer + 1;
    else       posDer = posDer - 1;
}

// mueve un motor: dir 1 adelante, -1 atras, 0 parado (fuente: Curio Res)
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal);
    if (dir == 1)       { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
    else if (dir == -1) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
    else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
}

// velocidad de -255 a 255 para cada lado
void moverMotores(int izq, int der) {
    setMotor(izq > 0 ? 1 : (izq < 0 ? -1 : 0), min(abs(izq), 255), PWMB, BIN1, BIN2);
    setMotor(der > 0 ? 1 : (der < 0 ? -1 : 0), min(abs(der), 255), PWMA, AIN1, AIN2);
}
void stopMotors() { moverMotores(0, 0); }

// vueltas hechas sin importar el sentido con apoyo de fuentes e ia
long avance() { return (abs(posIzq) + abs(posDer)) / 2; }
void reiniciarEncoders() { posIzq = 0; posDer = 0; }

// si una llanta lleva mas pulsos se le baja y a la otra se le sube (fuente: SparkFun)
void manejarRecto(int vel) {
    long error = abs(posIzq) - abs(posDer);
    int correccion = KP_RECTO * error;
    if (vel < 0) correccion = -correccion;   // de reversa se corrige al reves
    moverMotores(vel - correccion, vel + correccion);
}

// distancia en cm, espera a los otros ecos porque el TRIG es compartido (fuente: Random Nerd Tutorials)
float leerDistanciaCM(int echoPin) {
    unsigned long t = millis();
    while ((digitalRead(ECHO_FRENTE) || digitalRead(ECHO_IZQ) || digitalRead(ECHO_DER)) && millis() - t < 40) {}
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999;
    return duration * SOUND_SPEED / 2;
}

// enfrente mas preciso: la de en medio de 3 lecturas
float distanciaFrente() {
    float a = leerDistanciaCM(ECHO_FRENTE), b = leerDistanciaCM(ECHO_FRENTE), c = leerDistanciaCM(ECHO_FRENTE);
    return max(min(a, b), min(max(a, b), c));
}

// inclinacion en grados con el acelerometro (fuente: How To Mechatronics)
float leerInclinacion() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return atan(-a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
}

// escribe 2 renglones en la pantalla (fuente: Adafruit)
void mostrarTexto(const char* linea1, const char* linea2) {
    pantalla.clearDisplay();
    pantalla.setTextSize(2);
    pantalla.setTextColor(SSD1306_WHITE);
    pantalla.setCursor(0, 8);
    pantalla.println(linea1);
    pantalla.setCursor(0, 40);
    pantalla.println(linea2);
    pantalla.display();
}

// si la camara mando "A<id>" la primera vez se para y lo ensena 3.5 s (fuente: Random Nerd Tutorials, UART)
bool revisarAruco() {
    while (Serial2.available()) {
        String linea = Serial2.readStringUntil('\n');
        if (linea[0] == 'A' && arucoVisto < 0) {
            arucoVisto = linea.substring(1).toInt();
            stopMotors();
            mostrarTexto("ARUCO ID:", String(arucoVisto).c_str());
            delay(3500);   // el reglamento pide minimo 3 s
            return true;
        }
    }
    return false;
}

// avanza una celda: subiendo le da mas fuerza, bajando va lento y si se atora en un escalon empuja
bool avanzarCelda() {
    reiniciarEncoders();
    bool rampa = false;
    long antes = 0;
    unsigned long tMovio = millis(), inicio = millis();
    while (avance() < PULSOS_CELDA && millis() - inicio < 8000) {
        float ang = leerInclinacion();
        if (abs(ang) > ANGULO_INCLINADO) rampa = true;
        int vel = VELOCIDAD;
        if (ang > ANGULO_INCLINADO)       vel = VELOCIDAD_RAMPA;    // subiendo
        else if (ang < -ANGULO_INCLINADO) vel = VELOCIDAD_BAJADA;   // bajando
        if (avance() != antes) { antes = avance(); tMovio = millis(); }
        else if (millis() - tMovio > 1500) break;                  // no se mueve ni empujando
        else if (millis() - tMovio > 300) vel = VELOCIDAD_RAMPA;   // atorado en escalon o speedbump
        manejarRecto(vel);
        if (revisarAruco()) tMovio = millis();
        if (!rampa && leerDistanciaCM(ECHO_FRENTE) < 5) break;
        delay(5);
    }
    stopMotors();
    return avance() > PULSOS_CELDA * 0.8;   // solo cuenta lo que de verdad avanzo
}

// se regresa los pulsos que le digas
void regresar(long pulsos) {
    reiniciarEncoders();
    unsigned long t = millis();
    while (avance() < pulsos && millis() - t < 3000) manejarRecto(-VELOCIDAD);
    stopMotors();
}

// gira 90 grados sumando lo que mide el giroscopio, yaw = yaw + GyroZ * elapsedTime (fuente: How To Mechatronics)
void girar(bool izquierda) {
    float yaw = 0;
    unsigned long previousTime = millis(), inicio = millis();
    if (izquierda) moverMotores(-VELOCIDAD, VELOCIDAD);
    else           moverMotores(VELOCIDAD, -VELOCIDAD);
    while (abs(yaw) < ANGULO_GIRO && millis() - inicio < 2000) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float elapsedTime = (millis() - previousTime) / 1000.0;
        previousTime = millis();
        yaw = yaw + (g.gyro.z - gzOffset) * 180 / PI * elapsedTime;
        delay(5);
    }
    stopMotors();
}

// voltea hacia N, E, S u O (fuente: repo rahulraghavendrar)
void faceDirection(int targetDir) {
    int diff = (targetDir - currentDir + 4) % 4;
    if (diff == 1)      girar(false);
    else if (diff == 3) girar(true);
    else if (diff == 2) { girar(false); girar(false); }
    currentDir = targetDir;
}

// el color de la tabla que mas se parece a lo que lee el sensor, 2 veces igual para estar seguro (fuente: Adafruit)
Color leerColor() {
    int anterior = -1;   // ningun color, asi la primera lectura nunca cuenta como confirmada
    for (int intento = 0; intento < 5; intento++) {
        uint16_t r, g, b, c;
        tcs.getRawData(&r, &g, &b, &c);
        float suma = r + g + b + 1;
        Color mejor = C_NADA;
        float menor = 0.08 * 0.08;   // si no se parece a ninguno queda C_NADA
        for (int i = 1; i < 8; i++) {
            float d = sq(r / suma - REF[i][0]) + sq(g / suma - REF[i][1]) + sq(b / suma - REF[i][2]);
            if (d < menor) { menor = d; mejor = (Color)i; }
        }
        if (mejor == anterior) return mejor;
        anterior = mejor;
    }
    return C_NADA;
}

// si la celda tiene uno de los 4 colores lo ensena en la pantalla
void revisarColor(int x, int y) {
    Color c = leerColor();
    if (!colorVisto[x][y] && c >= C_AZUL && c <= C_ROSA) {
        colorVisto[x][y] = true;
        mostrarTexto("COLOR:", NOMBRES[c]);
        delay(1500);   // quieto para que el juez lo vea
    }
}

// guarda la pared en la celda y en su vecina
void marcarPared(int x, int y, int d, bool hay) {
    wallMap[x][y][d] = hay;
    int nx = x + dx[d], ny = y + dy[d];
    if (nx >= 0 && ny >= 0 && nx < N && ny < N) wallMap[nx][ny][(d + 2) % 4] = hay;
}

// paredes enfrente, izquierda y derecha (fuente: repo rahulraghavendrar)
void sensarParedes() {
    marcarPared(curX, curY, currentDir, distanciaFrente() < PARED_CM);
    marcarPared(curX, curY, (currentDir + 3) % 4, leerDistanciaCM(ECHO_IZQ) < PARED_CM);
    marcarPared(curX, curY, (currentDir + 1) % 4, leerDistanciaCM(ECHO_DER) < PARED_CM);
}

int direccionHacia(int x, int y, int nx, int ny) {
    for (int d = 0; d < 4; d++) if (dx[d] == nx - x && dy[d] == ny - y) return d;
    return -1;
}

// camino mas corto con BFS, paredes no vistas cuentan como abiertas (fuente: repo rahulraghavendrar)
// con gx = -1 busca la celda sin visitar mas cercana y no pasa por la meta,
// porque al llegar a la meta se termina la ronda (reglamento, seccion 3)
int solveMazeBFS(int sx, int sy, int gx, int gy, int path[][2]) {
    bool explorar = gx < 0;
    bool bfsVisited[N][N] = {};
    int parentX[N][N], parentY[N][N];
    int queueX[N * N], queueY[N * N];
    int qHead = 0, qTail = 0;

    queueX[qTail] = sx; queueY[qTail] = sy; qTail++;
    bfsVisited[sx][sy] = true;
    parentX[sx][sy] = -1; parentY[sx][sy] = -1;

    bool found = false;
    while (qHead < qTail) {
        int x = queueX[qHead], y = queueY[qHead]; qHead++;
        if ((!explorar && x == gx && y == gy) || (explorar && !visited[x][y])) {
            found = true; gx = x; gy = y; break;
        }
        for (int d = 0; d < 4; d++) {
            if (wallMap[x][y][d]) continue;
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N || bfsVisited[nx][ny]) continue;
            if (explorar && nx == GOAL_X && ny == GOAL_Y) continue;
            bfsVisited[nx][ny] = true;
            parentX[nx][ny] = x; parentY[nx][ny] = y;
            queueX[qTail] = nx; queueY[qTail] = ny; qTail++;
        }
    }
    if (!found) return -1;

    // arma el camino de atras para adelante y lo voltea
    int tempPath[N * N][2];
    int len = 0, x = gx, y = gy;
    while (x != -1) {
        tempPath[len][0] = x; tempPath[len][1] = y; len++;
        int px = parentX[x][y], py = parentY[x][y];
        x = px; y = py;
    }
    for (int i = 0; i < len; i++) {
        path[i][0] = tempPath[len - 1 - i][0];
        path[i][1] = tempPath[len - 1 - i][1];
    }
    return len;
}

// va celda por celda hasta (gx, gy), con gx = -1 explora
bool caminarHacia(int gx, int gy) {
    int camino[N * N][2];
    unsigned long inicio = millis();
    while (!(curX == gx && curY == gy)) {
        if (largoRuta >= MAX_PASOS || (gx < 0 && millis() - inicio > TIEMPO_EXPLORAR_MS)) return false;
        sensarParedes();
        if (solveMazeBFS(curX, curY, gx, gy, camino) < 2) return false;

        int nx = camino[1][0], ny = camino[1][1];
        int d = direccionHacia(curX, curY, nx, ny);
        faceDirection(d);
        if (distanciaFrente() < PARED_CM) { marcarPared(curX, curY, d, true); continue; }   // pared que antes estaba atras
        if (!avanzarCelda()) { regresar(avance()); marcarPared(curX, curY, d, true); continue; }

        curX = nx; curY = ny;
        visited[curX][curY] = true;
        ruta[largoRuta][0] = curX; ruta[largoRuta][1] = curY; largoRuta++;
        revisarColor(curX, curY);
    }
    return true;
}

// recorre un camino celda por celda (fuente: repo rahulraghavendrar)
void drivePath(int path[][2], int len) {
    for (int i = 1; i < len; i++) {
        faceDirection(direccionHacia(path[i - 1][0], path[i - 1][1], path[i][0], path[i][1]));
        if (!avanzarCelda()) break;   // si no avanzo ya no sabe donde esta
    }
}

// promedia 200 lecturas del giroscopio quieto (fuente: How To Mechatronics)
void calibrarGiroscopio() {
    for (int i = 0; i < 200; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gzOffset += g.gyro.z / 200;
        delay(5);
    }
}

// imprime en hexadecimal las direcciones I2C que contestan (fuente: Random Nerd Tutorials)
void escanearI2C() {
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) Serial.printf("I2C en 0x%02X\n", address);
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);   // camara (fuente: Random Nerd Tutorials)
    Serial2.setTimeout(20);
    Wire.begin(21, 22);
    escanearI2C();   // MPU6050 = 0x68, TCS34725 = 0x29, pantalla = 0x3C

    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    analogWriteFrequency(FRECUENCIA_PWM);   // antes del primer analogWrite
    pinMode(ENC_IZQ_B, INPUT); pinMode(ENC_DER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), readEncoderIzq, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_DER_A), readEncoderDer, RISING);
    pinMode(TRIG, OUTPUT);

    pantalla.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    tcs.begin();
    mpu.begin();
    calibrarGiroscopio();
}

void loop() {
    if (MODO_PRUEBA) {
        revisarAruco();
        Serial.printf("enc izq=%ld der=%ld | frente=%.1f izq=%.1f der=%.1f | color=%s | incl=%.1f | aruco=%d\n",
                      posIzq, posDer, leerDistanciaCM(ECHO_FRENTE), leerDistanciaCM(ECHO_IZQ),
                      leerDistanciaCM(ECHO_DER), NOMBRES[leerColor()], leerInclinacion(), arucoVisto);
        delay(300);
        return;
    }

    delay(2000);   // tiempo para soltar el robot
    digitalWrite(STBY, HIGH);

    ruta[0][0] = curX; ruta[0][1] = curY; largoRuta = 1;
    visited[curX][curY] = true;
    revisarColor(curX, curY);

    mostrarTexto("EXPLORANDO", "");
    caminarHacia(-1, -1);
    mostrarTexto("A LA META", "");
    if (caminarHacia(GOAL_X, GOAL_Y)) {
        // bonus 1: la ruta al reves
        int alReves[MAX_PASOS + 1][2];
        for (int i = 0; i < largoRuta; i++) {
            alReves[i][0] = ruta[largoRuta - 1 - i][0];
            alReves[i][1] = ruta[largoRuta - 1 - i][1];
        }
        drivePath(alReves, largoRuta);
    }
    mostrarTexto("FIN", "");
    stopMotors();
    digitalWrite(STBY, LOW);
    while (true) delay(1000);   // se queda quieto
}