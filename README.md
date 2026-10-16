<div align="center">
  <img src="Banner-APG.jpeg" alt="APG Banner" width="100%">
</div>

<div align="center">
  <h1>APG Project</h1> 
  <p><b>RoBorregos Candidates 2026 | Tecnológico de Monterrey</b></p>
</div>

<br>

## 📌 About The Project

We are **APG**, a development and robotics team taking part in the **Candidates 2026** competition organized by **RoBorregos** at Tecnológico de Monterrey. Our goal is to design, build, and program a fully autonomous robot that can solve challenges in logic, navigation, image processing, and system control.

This repository contains the full documentation of our project:
* **Software:** Navigation algorithms, color detection, and computer vision.
* **Electronics:** Schematic designs, PCB development, and power management.
* **Mechanics:** 3D CAD models, assemblies, and structural analysis of the chassis and gripping systems.

<br>

## 🎯 The Challenge

The competition evaluates the performance of our autonomous robot on two main tracks:
* **Track A (MAZE):** Autonomous maze exploration and navigation using color sensors, obstacle traversal (ramps, stairs, and speed bumps), and ArUco marker detection via computer vision.
* **Track B (Levels):** Object manipulation (locating and carrying a ball), line-avoiding path following, and real-time instruction execution based on colored floor tiles.

<br>

<div align="center">
  <h1>Team Members</h1> 
</div>

* Abdiel Gongora Muñoz
* Joshua Gongora Muñoz
* Pablo Ramirez Zavala
* Juan Miguel Puentes de la Rosa

<br>
<br>

## 🛠️ Our Progress

### 📄 Full documentation of the final code

Documentation of the robot's final code (Track A, Track B, camera, and ArUco), with an explanation of each function, datasheets, and the sources consulted.

📄 **[View the full documentation](https://docs.google.com/document/d/1cKvGaSNZqk7vZrx0yYfnXA9aFo_hxbGD/edit?usp=sharing&ouid=114002012716191553194&rtpof=true&sd=true)**

<br>

### 🚗 Basic movement and tests (ESP32)

As a first control test, we implemented direction and speed control for a pair of DC motors with encoders using an ESP32. The system receives text commands through the Serial Monitor/Terminal to move forward, move backward, turn in place, or stop. This served as an initial validation before integrating the autonomous logic.

Unlike Arduino (which uses `analogWrite()`), the ESP32 generates PWM signals through its hardware peripheral **LEDC (LED Control)**. During development we found that `ledcAttachChannel()` only exists from `arduino-esp32` v3.0 onward. Since we were working with v2.x (Legacy), we used `ledcSetup()`, `ledcAttachPin()`, and `ledcWrite(channel, duty)` instead.

```cpp
#include <Arduino.h> // Include the Arduino library so we can use the byte type

#define ENCODER_A 4 // Pins the encoders are connected to
#define ENCODER_B 5 // Pins the encoders are connected to

// Control shield pins
// They enable the motors and set their direction of rotation
const int E1PIN = 10;
const int M1PIN = 12;
const int E2PIN = 11;
const int M2PIN = 13;

// Hardware PWM configuration for ESP32 (LEDC peripheral)
const int CANAL_PWM_IZQ = 0; // LEDC channel 0
const int CANAL_PWM_DER = 1; // LEDC channel 1
const int FREC_PWM = 5000; // Frequency in Hz; 5 kHz is ideal for the motors and avoids jerky movement
const int RES_PWM = 8; // 8-bit resolution (values from 0 to 255)

struct Motor{
  byte enPIN;
  byte directionPIN;
};

// Create the motor variables with the Motor structure
// We create two motor objects of type Motor because there are 2 motors, using braces {}
const Motor motorIzq = {E1PIN, M1PIN};
const Motor motorDer = {E2PIN, M2PIN};

const int HaciaAdelante = LOW;
const int HaciaAtras = HIGH;

// Functions to move the motors at different speeds and directions, and to stop them
void moverMotores(int dirIzq, int velIzq, int dirDer, int velDer);
void detenerMotores();

void setup(){
  // Serial communication at 115200 baud
  Serial.begin(115200);

  // Configure the encoders as inputs
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(motorIzq.directionPIN, OUTPUT);
  pinMode(motorDer.directionPIN, OUTPUT);

  // Configure the motors with the PWM channel setup for ESP32
  // 1. Configure the channel (Channel, Frequency, Resolution)
  ledcSetup(CANAL_PWM_IZQ, FREC_PWM, RES_PWM);
  ledcSetup(CANAL_PWM_DER, FREC_PWM, RES_PWM);
  // 2. Attach the pin to its channel (Pin, Channel)
  ledcAttachPin(motorIzq.enPIN, CANAL_PWM_IZQ);
  ledcAttachPin(motorDer.enPIN, CANAL_PWM_DER);

  detenerMotores();
}

void loop(){
  Serial.print("What movement do you want to make?");
  Serial.println("Options: [a]dvance, [r]everse, [d] right, [i] left, [p] pause");
  Serial.print("Enter an option: ");

  while (Serial.available() == 0) {
    // Block execution until data arrives
  }

  // 3. Read the text the user typed
  String direccion = Serial.readStringUntil('\n');
  direccion.trim(); // Remove spaces and line breaks (\r, \n)
  Serial.println(direccion); // Show what the user typed

  // 4. Evaluate the action and control the motors
  if (direccion == "a" || direccion == "avanzar") {
    Serial.println("Moving forward");
    moverMotores(HaciaAdelante, 200, HaciaAdelante, 200);
  }
  else if (direccion == "r" || direccion == "retroceder") {
    Serial.println("Moving backward");
    moverMotores(HaciaAtras, 200, HaciaAtras, 200);
  }
  else if (direccion == "i" || direccion == "izquierda") {
    Serial.println("Turning left");
    // Turn in place: left motor backward, right motor forward
    moverMotores(HaciaAtras, 200, HaciaAdelante, 200);
  }
  else if (direccion == "d" || direccion == "derecha") {
    Serial.println("Turning right");
    // Turn in place: left motor forward, right motor backward
    moverMotores(HaciaAdelante, 200, HaciaAtras, 200);
  }
  else if (direccion == "p" || direccion == "parar") {
    Serial.println("Motors stopped");
    detenerMotores();
  }
  else {
    Serial.println("Option not available. Try again.");
  }

  delay(1000); // Pause before asking the user again
}

// Function that sets the direction and speed (PWM) of both motors
void moverMotores(int dirIzq, int velIzq, int dirDer, int velDer) {
  // Motor direction (HIGH / LOW)
  digitalWrite(motorIzq.directionPIN, dirIzq);
  digitalWrite(motorDer.directionPIN, dirDer);
  // PWM speed through the ESP32 LEDC peripheral
  ledcWrite(CANAL_PWM_IZQ, velIzq);
  ledcWrite(CANAL_PWM_DER, velDer);
}

// Set the speed of both motors to 0 so they stop
void detenerMotores() {
  ledcWrite(CANAL_PWM_IZQ, 0);
  ledcWrite(CANAL_PWM_DER, 0);
}
```

📄 **Full documentation:** [View the detailed document](https://docs.google.com/document/d/1L9G4jsoBlEPP43StgHrCMGgcGLqekIltsTy_N-n6iCA/edit?usp=sharing)

**Technical references consulted:**
* [Espressif Arduino ESP32 migration guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html) — comparison table of Legacy (2.0) vs 3.0 functions.
* [Random Nerd Tutorials - ESP32 PWM with Arduino IDE](https://randomnerdtutorials.com/esp32-pwm-arduino-ide/) — channel, frequency, and resolution concepts.
* [DFRobot Wiki (DRI0017 - Motor Shield Reference)](https://wiki.dfrobot.com/dri0017/docs/18141) — enable pin assignments.
* DC Motors and Logic Control tutorial: https://www.youtube.com/watch?v=DYMvm6qV1uQ
* Serial environment setup and data structures: https://www.youtube.com/watch?v=ADPpHxj5Nbg
* Technical consultation with Gemini (Google AI) to understand the differences between ESP32 PWM function versions and fix compilation errors.

<br>

<br>