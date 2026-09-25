#include <Arduino.h> //Declaramos la libreria de arduino para que nos deje usar la variable tipo byte


#define ENCODER_A   4//Aqui se definen los pines a los que estan conectados los encoders
#define ENCODER_B   5//Aqui se definen los pines a los que estan conectados los encoders


//Definimos los pines de control shield
// Sirven para habilitar los motores y el sentido de giro de los mostores
const int E1PIN = 10;
const int M1PIN = 12;
const int E2PIN = 11;
const int M2PIN = 13;


// Configuración de Hardware PWM para ESP32 (Periférico LEDC)
const int CANAL_PWM_IZQ = 0;  // Canal LEDC 0
const int CANAL_PWM_DER = 1;  // Canal LEDC 1
const int FREC_PWM = 5000;    // Frecuencia en Hz y es 5 kHz ya que es la ideal para los motores y evitar movimientos bruscos
const int RES_PWM = 8;        // Resolución de 8 bits (Valores de 0 a 255)


struct Motor{
    byte enPIN;
    byte directionPIN;
};


// Creamos la variable Motor con la estructura motor
// Creamos los dos objetos motores del tipo Motor porque son 2 motores XD con llaves {}
const Motor motorIzq = {E1PIN, M1PIN};  
const Motor motorDer = {E2PIN, M2PIN};


const int HaciaAdelante = LOW;
const int HaciaAtras = HIGH;


//Definimos funciones para mover los motores a diferentes velocidades,direcciones y detenerlos
void moverMotores(int dirIzq, int velIzq, int dirDer, int velDer);
void detenerMotores();


void setup(){
    //Hacemos una comunicacion serial de 115200 baudios
    Serial.begin(115200);
    //Configuramos los encoders como entradas
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);


    pinMode(motorIzq.directionPIN, OUTPUT);
    pinMode(motorDer.directionPIN, OUTPUT);


    //Configuramos los motores con la configuraacion de canales PWM en ESP32
    // 1. Configuramos el canal (Canal, Frecuencia, Resolución)
    ledcSetup(CANAL_PWM_IZQ, FREC_PWM, RES_PWM);
    ledcSetup(CANAL_PWM_DER, FREC_PWM, RES_PWM);


    // 2. Asignamos el pin al canal correspondiente (Pin, Canal)
    ledcAttachPin(motorIzq.enPIN, CANAL_PWM_IZQ);
    ledcAttachPin(motorDer.enPIN, CANAL_PWM_DER);


    detenerMotores();
}






void loop(){
    Serial.print("¿Que movimiento quieres hacer?");
    Serial.println("Opciones: [a]vanzar, [r]etroceder, [d]erecha, [i]zquierda, [p]arar");
    Serial.print("Ingresa una opcion: ");
    while (Serial.available() == 0) {
        // Bloquea la ejecución hasta que llegue un dato
    }


    // 3. Leer el texto que ponga el usuario
    String direccion = Serial.readStringUntil('\n');
    direccion.trim(); // Elimina espacios en blanco y saltos de línea (\r, \n)


    Serial.println(direccion); // Muestra lo que el usuario escribió


    // 4. Evaluar la acción y controlar los motores
    if (direccion == "a" || direccion == "avanzar") {
        Serial.println(" Moviendo hacia adelante");
        moverMotores(HaciaAdelante, 200, HaciaAdelante, 200);
    }
    else if (direccion == "r" || direccion == "retroceder") {
        Serial.println(" Moviendo hacia atras");
        moverMotores(HaciaAtras, 200, HaciaAtras, 200);
    }
    else if (direccion == "i" || direccion == "izquierda") {
        Serial.println(" Girando a la izquierda");
        // Giro sobre su eje: Motor izquierdo atrás, derecho adelante
        moverMotores(HaciaAtras, 200, HaciaAdelante, 200);
    }
    else if (direccion == "d" || direccion == "derecha") {
        Serial.println(" Girando a la derecha");
        // Giro sobre su eje: Motor izquierdo adelante, derecho atrás
        moverMotores(HaciaAdelante, 200, HaciaAtras, 200);
    }
    else if (direccion == "p" || direccion == "parar") {
        Serial.println(" Motores detenidos");
        detenerMotores();
    }
    else {
        Serial.println(" Opción no disponible. Intenta de nuevo.");
    }


    delay(1000); // Pausa antes de volver a preguntar al usuario
}


// Funcion para dar el sentido y velocidad (PWM) de ambos motores
void moverMotores(int dirIzq, int velIzq, int dirDer, int velDer) {
    // Dirección del motor (HIGH / LOW)
    digitalWrite(motorIzq.directionPIN, dirIzq);
    digitalWrite(motorDer.directionPIN, dirDer);


    // Velocidad PWM a través del periférico LEDC de ESP32
    ledcWrite(CANAL_PWM_IZQ, velIzq);
    ledcWrite(CANAL_PWM_DER, velDer);
}


//Definimos en la funcion detenerMotores que la velocidad de ambos motores es 0 para que se detengan
void detenerMotores() {
    ledcWrite(CANAL_PWM_IZQ, 0);
    ledcWrite(CANAL_PWM_DER, 0);
}
