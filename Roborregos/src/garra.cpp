#include <Servo.h>

Servo Servol;
Servo Servoi;

int MotorIA = 4;//Pin del motor izquierdo adelante
int MotorIR = 5;//Pin del motor izquierdo reversa
int MotorDA = 6;//Pin del motor derecho adelante
int MotorDR = 7;//Pin del motor derecho adelante

// ==================== Cierre automático de la garra por distancia ====================
float distanciaPelota = -1; // TODO: aquí debe quedar tu lectura real de distancia

// TODO: reemplaza esto por tu forma real de medir la distancia
// (sensor ultrasónico en este mismo ESP32, dato recibido de la ESP32-CAM, etc.)
float leerDistanciaPelota() {
  return -1;
}
// ==================== FIN ====================

void setup() {
  Serial.begin(9600);

  pinMode(MotorIA, OUTPUT);
  pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT);
  pinMode(MotorDR, OUTPUT);

  Servol.attach(11);
  Servoi.attach(10);
  Servol.write(0);
  Servoi.write(0);

  stop(); // motores apagados al iniciar
}

void loop() {


  // ==================== Garra automática por distancia ====================
  distanciaPelota = leerDistanciaPelota();

  if (distanciaPelota <= 20) {
    Servol.write(90);   // cerrar garra
  } else {
    Servol.write(0);    // abrir garra
  }
  // ==================== FIN ====================

}

//funcion de Paro de motores.
void stop(){
    digitalWrite(MotorIA,LOW);
    digitalWrite(MotorDA,LOW);
    digitalWrite(MotorIR,LOW);
    digitalWrite(MotorDR,LOW);
  }