#include <Servo.h>

Servo Servol;
Servo Servoi;

int MotorIA = 4;//Pin del motor izquierdo adelante
int MotorIR = 5;//Pin del motor izquierdo reversa
int MotorDA = 6;//Pin del motor derecho adelante
int MotorDR = 7;//Pin del motor derecho adelante


float distanciaPelota = -1; // Inicializamos distancia

//Funcion para definir la distancia de la pelota por definir
float leerDistanciaPelota() {
  return -1;
}


void setup(){

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


  // Funcion donde si la distancia de la pelota esta a la necesaria, se cierra la garra o se acciona el servo
  distanciaPelota = leerDistanciaPelota();

  if (distanciaPelota <= 20) {
    Servol.write(90);   // cerrar garra
  } else {
    Servol.write(0);    // abrir garra
  }


}

//funcion de Paro de motores.
void stop(){
    digitalWrite(MotorIA,LOW);
    digitalWrite(MotorDA,LOW);
    digitalWrite(MotorIR,LOW);
    digitalWrite(MotorDR,LOW);
  }