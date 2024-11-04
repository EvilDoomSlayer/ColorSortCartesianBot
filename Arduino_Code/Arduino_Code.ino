#include <Servo.h>
Servo myservo;  // Crea un objeto servo para controlar el servomotor

int pos = 0;    // Variable para almacenar la posición del servomotor
const int dirPinY = 2;  
const int stepPinY = 3; 
const int dirPinZ = 4;  
const int stepPinZ = 5; 
const int dirPinX = 6;  
const int stepPinX = 7; 

const int limitX = 9;   
const int limitY = 10; 
const int limitZ = 11;  

// Variables para controlar los pasos de cada eje
const int stepsY = 200; // Pasos del eje Y
const int stepsZ = 150; // Pasos del eje Z
const int stepsX = 100; // Pasos del eje X

// Variables de retardo específicas para cada eje (en microsegundos)
int stepDelayY = 1000; // Velocidad del eje Y
int stepDelayZ = 800;  // Velocidad del eje Z
int stepDelayX = 800; // Velocidad del eje X

void setup() {
   myservo.attach(8);
   pinMode(dirPinY, OUTPUT);
   pinMode(stepPinY, OUTPUT);
   pinMode(dirPinZ, OUTPUT);
   pinMode(stepPinZ, OUTPUT);
   pinMode(dirPinX, OUTPUT);
   pinMode(stepPinX, OUTPUT);
   pinMode(limitX, INPUT);
   pinMode(limitY, INPUT);
}

void loop() {
   HOMEX();  // HOME EN EJE X
   HOMEY();  // HOME EN EJE Y
   HOMEZ();
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

void HOMEX() {
  // Homing para el eje X hasta encontrar el límite
  while (digitalRead(limitX) == LOW) {
    digitalWrite(dirPinX, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX); // Usa el retardo específico del eje X
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }
}


void HOMEY() {
  // Homing para el eje Y hasta encontrar el límite
  while (digitalRead(limitY) == LOW) {
    digitalWrite(dirPinY, HIGH);
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY); // Usa el retardo específico del eje Y
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }
}

void HOMEZ() {
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
    myservo.write(pos);
    delay(10);
  }
  delay(1000);
  
  // CERRAR
  for (pos = 100; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(3);
  }
  delay(1000);
}

