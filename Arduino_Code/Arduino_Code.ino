/*
 * Arduino_Code.c
 * Created: 02/11/2024 05:48:52 p. m.
 * Author : Team_1
 */

/*
* Programa del Cartesian Robot
* Este programa controla:
* - 4 drivers DRV8833
* - 1 Servomotor Mg995
* - 1 Sensor de color TCS3200
* - 2 Sensores ultrasónicos HC-SR04
* - 3 limit-sitches
* - Botonera Industrial
* 
* Este programa automatiza las clasificacion de cubos por color.
*/

//Librerias
#include <Servo.h>

//Protitipos de funciones
void portsInit(void);
void homeX(void);
void homeY(void);
void homeZ(void);


//Definicion de pines
//Step-Motors
#define dirPinY   2  
#define stepPinY  3 
#define dirPinZ   4  
#define stepPinZ  5
#define dirPinX   6 
#define stepPinX  7

//Limit Switches
#define limitX    9   
#define limitY    10 
#define limitZ    11  

//Servomotor
#define servoPin  8

//Inicializacion del objeto Servo
Servo Gripper; 


//Variables Globales
//Servomotor
int pos = 0;    // Variable para almacenar la posición del servomotor

// Variables para controlar los pasos de cada eje
const int stepsY = 200; // Pasos del eje Y
const int stepsZ = 150; // Pasos del eje Z
const int stepsX = 100; // Pasos del eje X

// Variables de retardo específicas para cada eje (en microsegundos)
int stepDelayY = 1000; // Velocidad del eje Y
int stepDelayZ = 800;  // Velocidad del eje Z
int stepDelayX = 800; // Velocidad del eje X




void setup() {
  portsInit();
}

void loop() {
   homeX();  // HOME EN EJE X
   homeY();  // HOME EN EJE Y
   homeZ();
   SERVO();  // Controla el movimiento del servomotor

   // Movimientos para cada eje en ambas direcciones con sus respectivos pasos y velocidad
   // HIGH = DERECHA   LOW = IZQUIERDA
 
   // moverEje(dirPinY, stepPinY, HIGH, stepsY, stepDelayY); // Mueve el eje Y en dirección positiva
   // delay(1000);

   // moverEje(dirPinY, stepPinY, LOW, stepsY, stepDelayY);  // Mueve el eje Y en dirección negativa
   // delay(1000);

   // moverEje(dirPinZ, stepPinZ, HIGH, stepsZ, stepDelayZ); // Mueve el eje Z en dirección positiva
   // delay(1000);

   // moverEje(dirPinZ, stepPinZ, LOW, stepsZ, stepDelayZ);  // Mueve el eje Z en dirección negativa
   // delay(1000);

   // moverEje(dirPinX, stepPinX, HIGH, stepsX, stepDelayX); // Mueve el eje X en dirección positiva
   // delay(1000);

   // moverEje(dirPinX, stepPinX, LOW, stepsX, stepDelayX);  // Mueve el eje X en dirección negativa
   // delay(1000);
}




void portsInit(void) {
  //Step-Motors
  pinMode(dirPinY, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinZ, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinX, OUTPUT);
  //Limit sitches
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);
  //Servomotor
  Gripper.attach(servoPin);
}




void homeX(void) {
  // Homing para el eje X hasta encontrar el límite
  while (digitalRead(limitX) == LOW) {
    digitalWrite(dirPinX, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX); // Usa el retardo específico del eje X
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }
}


void homeY(void) {
  // Homing para el eje Y hasta encontrar el límite
  while (digitalRead(limitY) == LOW) {
    digitalWrite(dirPinY, HIGH);
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY); // Usa el retardo específico del eje Y
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }
}

void homeZ(void) {
  // Homing para el eje Y hasta encontrar el límite
  while (digitalRead(limitZ) == LOW) {
    digitalWrite(dirPinZ, HIGH);
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ); // Usa el retardo específico del eje Z
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}



void moverEje(int dirPin, int stepPin, bool direccion, int pasos, int stepDelay) {
  // Función genérica para mover cualquier eje en una dirección y cantidad de pasos especificada
  digitalWrite(dirPin, direccion);
  for (int i = 0; i < pasos; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);  // Usa el retardo específico del eje
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}





void SERVO() {
  // Movimiento del servomotor
  // ABRIR
  for (pos = 0; pos <= 100; pos += 1) {
    Gripper.write(pos);
    delay(10);
  }
  delay(1000);
  
  // CERRAR
  for (pos = 100; pos >= 0; pos -= 1) {
    Gripper.write(pos);
    delay(3);
  }
  delay(1000);
}

