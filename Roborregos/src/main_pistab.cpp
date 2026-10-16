// pista B: 1 agarrar la pelota, 2 cruzar sin pisar lineas blancas, 3 seguir colores
// la camara manda la distancia de la pelota en mm y con eso se cierra la garra sola

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_TCS34725.h>

Adafruit_MPU6050 mpu;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Servo garra;
Preferences memoria;   // guarda el ultimo checkpoint aunque se apague

//declaracion de pines
const int PWMA = 25, AIN1 = 26, AIN2 = 27;   // motor derecho
const int PWMB = 13, BIN1 = 32, BIN2 = 33;   // motor izquierdo
const int STBY = 14;
const int ENC_DER_A = 34, ENC_DER_B = 35;
const int ENC_IZQ_A = 36, ENC_IZQ_B = 39;
const int TRIG = 5;                           // compartido
const int ECHO_FRENTE = 19, ECHO_IZQ = 18, ECHO_DER = 23;
const int PIN_SERVO = 4;
const int CAM_RX = 16, CAM_TX = 17;

// Calibraciones
const int   FRECUENCIA_PWM  = 20000; // 20 kHz para que los motores no zumben
const int   VELOCIDAD       = 175;
const int   VELOCIDAD_LENTA = 110;   // falta calibrar
const float KP_RECTO        = 2.0;   // falta calibrar
const float CELDA_CM        = 30.0;
const float LLANTA_CM       = 23.5;  // circunferencia, falta calibrar
const int   PULSOS_VUELTA   = 360;   // falta calibrar
const float ANGULO_GIRO     = 88.0;  // falta calibrar
const float SOUND_SPEED     = 0.034;
const int   GARRA_ABIERTA   = 0;     // falta calibrar
const int   GARRA_CERRADA   = 90;    // falta calibrar
const float CERRAR_MM       = 80;    // falta calibrar
const bool  REVISAR_POSESION = true; // false si la camara no ve la pelota cuando ya esta en la garra
const uint16_t LUZ_BLANCO   = 150;   // luz de la linea con lectura rapida, falta calibrar
const float SENSOR_ADELANTE_CM = 8;  // distancia del sensor de color al eje de las llantas, medir
const bool  MODO_PRUEBA     = false; // true = solo imprime los sensores

// vueltas que da la llanta en una celda x pulsos por vuelta formula hecha con apoyo de ia y fuentes
const long PULSOS_CELDA = (long)(CELDA_CM / LLANTA_CM * PULSOS_VUELTA);
long cmAPulsos(float cm) { return (long)(cm / LLANTA_CM * PULSOS_VUELTA); }

// seccion 1: 3x3 con la pelota en (1,1), x a la derecha y y hacia arriba, se cambia en la calibracion
const int S1_INICIO_X = 0, S1_INICIO_Y = -1;
const int S1_CHECK_X = 3, S1_CHECK_Y = 1;
// seccion 2: 2 filas x 4 columnas, los checkpoints quedan entre las 2 filas
const int S2_FILA_INICIAL = 0;   // 0 abajo, 1 arriba

enum Color { C_NADA, C_BLANCO, C_AZUL, C_AMARILLO, C_NARANJA, C_ROSA, C_VERDE, C_ROJO };
// proporcion r, g, b de cada color, falta medirlos con MODO_PRUEBA
const float REF[8][3] = { {0, 0, 0}, {.31, .35, .34}, {.17, .39, .44}, {.44, .40, .16},
                          {.55, .27, .18}, {.45, .22, .33}, {.22, .48, .30}, {.60, .20, .20} };

enum Direccion { NORTE = 0, ESTE = 1, SUR = 2, OESTE = 3 };
const int dx[4] = { 0, 1, 0, -1 };
const int dy[4] = { 1, 0, -1, 0 };

volatile long posIzq = 0, posDer = 0;
float gzOffset = 0;
int curX = 0, curY = 0, currentDir = NORTE;
float distanciaPelotaMM = -1;
bool garraCerrada = false;

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

// lee la distancia que manda la camara y cierra la garra si la pelota ya esta adentro (fuente: Random Nerd Tutorials)
// una vez cerrada ya no se abre, para no soltar la pelota
void atenderCamara() {
    while (Serial2.available()) {
        float d = Serial2.readStringUntil('\n').toFloat();
        if (d != 0) distanciaPelotaMM = d;   // 0 = linea con basura
    }
    if (!garraCerrada && distanciaPelotaMM > 0 && distanciaPelotaMM <= CERRAR_MM) {
        garra.write(GARRA_CERRADA);
        garraCerrada = true;
    }
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

// avanza los pulsos que le digas (negativo = de reversa), se frena si hay algo enfrente
void avanzar(long pulsos, int vel) {
    reiniciarEncoders();
    unsigned long t = millis();
    while (avance() < abs(pulsos) && millis() - t < 5000) {
        manejarRecto(pulsos > 0 ? vel : -vel);
        atenderCamara();
        if (pulsos > 0 && leerDistanciaCM(ECHO_FRENTE) < 5) break;
        delay(1);
    }
    stopMotors();
}

// gira 90 grados sumando lo que mide el giroscopio, yaw = yaw + GyroZ * elapsedTime (fuente: How To Mechatronics)
void girar(bool izquierda) {
    float yaw = 0;
    unsigned long previousTime = millis(), inicio = millis();
    if (izquierda) moverMotores(-VELOCIDAD, VELOCIDAD);
    else           moverMotores(VELOCIDAD, -VELOCIDAD);
    while (abs(yaw) < ANGULO_GIRO && millis() - inicio < 2000) {
        atenderCamara();
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
        float menor = 0.08 * 0.08;
        for (int i = 1; i < 8; i++) {
            float d = sq(r / suma - REF[i][0]) + sq(g / suma - REF[i][1]) + sq(b / suma - REF[i][2]);
            if (d < menor) { menor = d; mejor = (Color)i; }
        }
        if (mejor == anterior) return mejor;
        anterior = mejor;
    }
    return C_NADA;
}

// true si el sensor de color ve la linea blanca
bool hayLineaBlanca() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    return c > LUZ_BLANCO;
}

// true si la garra esta cerrada y la camara ve la pelota pegada
bool tienePelota() {
    if (!REVISAR_POSESION) return garraCerrada;
    bool cerca = false;
    unsigned long t = millis();
    while (millis() - t < 400) {
        atenderCamara();
        if (distanciaPelotaMM > 0 && distanciaPelotaMM <= CERRAR_MM) cerca = true;
        delay(10);
    }
    return garraCerrada && cerca;
}

// si se le cayo la pelota gira buscandola, va por ella y regresa a donde estaba
void recuperarPelota(long maxPulsos) {
    if (tienePelota()) return;
    garra.write(GARRA_ABIERTA);
    garraCerrada = false;
    int dirOriginal = currentDir;
    for (int i = 0; i < 4 && !garraCerrada; i++) {
        delay(300);
        atenderCamara();
        if (distanciaPelotaMM > 0 && distanciaPelotaMM < 300) {
            reiniciarEncoders();
            while (avance() < maxPulsos && !garraCerrada) {
                manejarRecto(VELOCIDAD_LENTA);
                atenderCamara();
                delay(1);
            }
            stopMotors();
            avanzar(-avance(), VELOCIDAD);
            break;
        }
        faceDirection((currentDir + 1) % 4);
    }
    faceDirection(dirOriginal);
}

//  Seccion 1

// celdas por donde puede pasar (todo menos la caja del centro)
bool celdaLibre(int x, int y) {
    if ((x == S1_INICIO_X && y == S1_INICIO_Y) || (x == S1_CHECK_X && y == S1_CHECK_Y)) return true;
    return x >= 0 && y >= 0 && x <= 2 && y <= 2 && !(x == 1 && y == 1);
}

// camino mas corto en la seccion 1, los indices llevan +1 porque hay coordenadas en -1 (fuente: repo rahulraghavendrar)
int bfsSeccion1(int sx, int sy, int gx, int gy, int path[][2]) {
    bool visto[5][5] = {};
    int parentX[5][5], parentY[5][5];
    int queueX[25], queueY[25];
    int qHead = 0, qTail = 0;
    queueX[qTail] = sx; queueY[qTail] = sy; qTail++;
    visto[sx + 1][sy + 1] = true;

    bool found = false;
    while (qHead < qTail) {
        int x = queueX[qHead], y = queueY[qHead]; qHead++;
        if (x == gx && y == gy) { found = true; break; }
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (!celdaLibre(nx, ny) || visto[nx + 1][ny + 1]) continue;
            visto[nx + 1][ny + 1] = true;
            parentX[nx + 1][ny + 1] = x; parentY[nx + 1][ny + 1] = y;
            queueX[qTail] = nx; queueY[qTail] = ny; qTail++;
        }
    }
    if (!found) return -1;

    // arma el camino de atras para adelante y lo voltea
    int tempPath[25][2];
    int len = 0, x = gx, y = gy;
    while (true) {
        tempPath[len][0] = x; tempPath[len][1] = y; len++;
        if (x == sx && y == sy) break;
        int px = parentX[x + 1][y + 1], py = parentY[x + 1][y + 1];
        x = px; y = py;
    }
    for (int i = 0; i < len; i++) {
        path[i][0] = tempPath[len - 1 - i][0];
        path[i][1] = tempPath[len - 1 - i][1];
    }
    return len;
}

// va a una celda por el camino mas corto (fuente: drivePath del repo rahulraghavendrar)
void irA(int gx, int gy) {
    int path[25][2];
    int len = bfsSeccion1(curX, curY, gx, gy, path);
    for (int i = 1; i < len; i++) {
        for (int d = 0; d < 4; d++) if (dx[d] == path[i][0] - curX && dy[d] == path[i][1] - curY) faceDirection(d);
        avanzar(PULSOS_CELDA, VELOCIDAD);
        curX = path[i][0]; curY = path[i][1];
    }
}

// rodea la caja, entra por el lado abierto hasta que se cierra la garra y se va al checkpoint 1
// regresa true si llego al checkpoint 1 con la pelota
bool seccion1() {
    curX = S1_INICIO_X; curY = S1_INICIO_Y; currentDir = NORTE;
    // celdas pegadas a la caja y hacia donde voltear para verla
    const int lados[4][3] = { {1, 0, NORTE}, {0, 1, ESTE}, {1, 2, SUR}, {2, 1, OESTE} };
    bool revisado[4] = {};

    for (int intento = 0; intento < 4; intento++) {
        int mejor = -1, menorLargo = 99, path[25][2];
        for (int i = 0; i < 4; i++) {   // el lado sin revisar mas cercano
            int len = bfsSeccion1(curX, curY, lados[i][0], lados[i][1], path);
            if (!revisado[i] && len > 0 && len < menorLargo) { menorLargo = len; mejor = i; }
        }
        if (mejor < 0) break;   // ya no hay lados que se puedan alcanzar
        revisado[mejor] = true;
        irA(lados[mejor][0], lados[mejor][1]);
        faceDirection(lados[mejor][2]);
        delay(200);
        atenderCamara();

        // abierto si el ultrasonico ve lejos o la camara ve la pelota
        if (leerDistanciaCM(ECHO_FRENTE) > 25 || distanciaPelotaMM > 0) {
            reiniciarEncoders();
            while (avance() < PULSOS_CELDA && !garraCerrada && leerDistanciaCM(ECHO_FRENTE) > 4) {
                manejarRecto(VELOCIDAD_LENTA);
                atenderCamara();
            }
            stopMotors();
            avanzar(-avance(), VELOCIDAD);   // sale de reversa
            recuperarPelota(PULSOS_CELDA);   // si no la agarro o se le cayo
            break;
        }
    }
    irA(S1_CHECK_X, S1_CHECK_Y);
    faceDirection(ESTE);
    return garraCerrada && curX == S1_CHECK_X && curY == S1_CHECK_Y;
}

//  Seccion 2

// pasa a la otra fila, las lineas son verticales asi que no pisa ninguna
void cambiarDeFila(int &fila) {
    faceDirection(fila == 0 ? NORTE : SUR);
    avanzar(PULSOS_CELDA, VELOCIDAD);
    faceDirection(ESTE);
    fila = 1 - fila;
}

// se acomoda en una fila, avanza viendo la linea blanca y si la ve se cambia de fila
bool seccion2() {
    currentDir = ESTE;
    recuperarPelota(PULSOS_CELDA / 2);   // tras un lack of progress la ponen enfrente (imagen 3.1)
    int fila = S2_FILA_INICIAL;
    faceDirection(fila == 1 ? NORTE : SUR);   // media celda hacia la fila
    avanzar(PULSOS_CELDA / 2, VELOCIDAD);
    faceDirection(ESTE);

    tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);   // lectura rapida para no brincarse la linea
    long total = PULSOS_CELDA * 5;                 // de centro de checkpoint a centro de checkpoint
    long ventana = cmAPulsos(5);                   // cerca de la separacion blanca de los checkpoints no se revisa
    long recorrido = 0;
    for (int cambios = 0; recorrido < total; cambios++) {
        if (cambios > 6) return false;
        reiniciarEncoders();
        bool vioLinea = false;
        while (recorrido + avance() < total) {
            manejarRecto(VELOCIDAD_LENTA);
            atenderCamara();
            long sensor = recorrido + avance() + cmAPulsos(SENSOR_ADELANTE_CM);
            bool borde = abs(sensor - PULSOS_CELDA / 2) < ventana || abs(sensor - (total - PULSOS_CELDA / 2)) < ventana;
            if (!borde && hayLineaBlanca()) { vioLinea = true; break; }
            delay(1);
        }
        stopMotors();
        recorrido += avance();
        if (!vioLinea) break;
        avanzar(-cmAPulsos(6), VELOCIDAD);   // se aleja de la linea
        recorrido -= avance();
        cambiarDeFila(fila);
    }
    tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);

    faceDirection(fila == 1 ? SUR : NORTE);   // media celda al centro del checkpoint 2
    avanzar(PULSOS_CELDA / 2, VELOCIDAD);
    faceDirection(ESTE);
    return true;
}

//  Seccion 3

// lee el color de cada unidad y va hacia donde dice hasta el verde (imagen 2.6 del reglamento)
// "arriba" es hacia donde mira el robot al salir del checkpoint 2
void seccion3() {
    currentDir = NORTE;
    recuperarPelota(PULSOS_CELDA / 2);
    avanzar(PULSOS_CELDA, VELOCIDAD);
    for (int paso = 0; paso < 12; paso++) {
        Color c = leerColor();
        if (c == C_NADA || c == C_BLANCO) {   // cayo en la orilla blanca, se hace atras y vuelve a leer
            avanzar(-cmAPulsos(5), VELOCIDAD_LENTA);
            c = leerColor();
        }
        if (c == C_VERDE) break;                          // fin
        if      (c == C_AZUL)     faceDirection(ESTE);    // derecha
        else if (c == C_AMARILLO) faceDirection(OESTE);   // izquierda
        else if (c == C_NARANJA)  faceDirection(NORTE);   // arriba
        else if (c == C_ROSA)     faceDirection(SUR);     // abajo
        // si no reconoce el color sigue derecho, hacia donde ya iba
        avanzar(PULSOS_CELDA, VELOCIDAD);
    }
    stopMotors();
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
    Serial2.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);   // (fuente: Random Nerd Tutorials)
    Serial2.setTimeout(20);
    Wire.begin(21, 22);
    escanearI2C();   // MPU6050 = 0x68, TCS34725 = 0x29

    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    analogWriteFrequency(FRECUENCIA_PWM);   // antes del primer analogWrite
    pinMode(ENC_IZQ_B, INPUT); pinMode(ENC_DER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), readEncoderIzq, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_DER_A), readEncoderDer, RISING);
    pinMode(TRIG, OUTPUT);

    // garra (fuente: ejemplo Sweep de ESP32Servo)
    ESP32PWM::allocateTimer(0);
    garra.setPeriodHertz(50);
    garra.attach(PIN_SERVO, 500, 2400);
    garra.write(GARRA_ABIERTA);

    tcs.begin();
    mpu.begin();
    calibrarGiroscopio();
}

void loop() {
    if (MODO_PRUEBA) {
        uint16_t r, g, b, c;
        tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);
        tcs.getRawData(&r, &g, &b, &c);
        tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);
        atenderCamara();
        Serial.printf("enc izq=%ld der=%ld | frente=%.1f izq=%.1f der=%.1f | color=%d | luz rapida=%u | cam=%.1f\n",
                      posIzq, posDer, leerDistanciaCM(ECHO_FRENTE), leerDistanciaCM(ECHO_IZQ),
                      leerDistanciaCM(ECHO_DER), leerColor(), c, distanciaPelotaMM);
        delay(300);
        return;
    }

    // en verde empieza en la seccion 1, en rojo sigue en la que tenia guardada (fuente: Random Nerd Tutorials)
    memoria.begin("pistaB", false);
    int etapa = memoria.getInt("etapa", 1);
    int rojos = 0;   // 2 de 3 lecturas en rojo para creer que esta en un checkpoint
    for (int i = 0; i < 3; i++) if (leerColor() == C_ROJO) rojos++;
    if (rojos < 2) etapa = 1;

    delay(2000);   // tiempo para soltar el robot
    digitalWrite(STBY, HIGH);

    if (etapa == 1 && seccion1()) { memoria.putInt("etapa", 2); etapa = 2; }
    if (etapa == 2 && seccion2()) { memoria.putInt("etapa", 3); etapa = 3; }
    if (etapa == 3) { seccion3(); memoria.putInt("etapa", 1); }

    stopMotors();
    digitalWrite(STBY, LOW);
    while (true) delay(1000);   // se queda quieto
}