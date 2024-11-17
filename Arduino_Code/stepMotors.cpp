#include "stepMotors.h"

void stepMotorsInit(void) {
  //Step Motors
  pinMode(dirPinY, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinZ, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinX, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  //Limit sitches
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);
}


void homeX(void) {
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  // Homing para el eje X hasta encontrar el límite
  while (digitalRead(limitX) == LOW) {
    digitalWrite(dirPinX, LOW);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX); // Usa el retardo específico del eje X
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }
}


void homeY(void) {
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
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
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  // Homing para el eje Y hasta encontrar el límite
  while (digitalRead(limitZ) == LOW) {
    digitalWrite(dirPinZ, LOW);
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ); // Usa el retardo específico del eje Z
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}


void homeAllAxes(void) {
  homeX();
  homeY();
  homeZ();
}


void moveXSteps (int steps, int dir) {
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  if(dir == 1) { //Derecha
      digitalWrite(dirPinX, HIGH);
  }

  else if(dir == 0) {  //Izquierda
      digitalWrite(dirPinX, LOW);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX);  // Usa el retardo específico del eje
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }
}


void moveYSteps (int steps, int dir) {
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  if(dir == 1) {
      digitalWrite(dirPinY, LOW); //Derecha
  }

  else if(dir == 0) { //Izquierda
      digitalWrite(dirPinY, HIGH);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);  // Usa el retardo específico del eje
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }
}


void moveZSteps (int steps, int dir) {
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  if(dir == 1) {
      digitalWrite(dirPinZ, HIGH); // Abajo
  }

  else if(dir == 0) { // Arriba
      digitalWrite(dirPinZ, LOW);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);  // Usa el retardo específico del eje
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}

void moveToSteps(int pos[6]) {
  //Convierte las coordenadas deseadas a pasos
  int x_steps = pos[0];
  int y_steps = pos[1];
  int z_steps = pos[2];
  int dir_x = pos[3];
  int dir_y = pos[4];
  int dir_z = pos[5];

  // Función para mover eje Y una cantidad de pasos especificada
  moveYSteps(y_steps, dir_y);

  // Función para mover eje X una cantidad de pasos especificada
  moveXSteps(x_steps, dir_x);

  // Función para mover eje Z una cantidad de pasos especificada
  moveZSteps(z_steps, dir_z);
}


void moveXMicroSteps (int steps, int dir) {
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  if(dir == 1) { // Derecha
      digitalWrite(dirPinX, HIGH);
  }

  else if(dir == 0) { // Izquierda
      digitalWrite(dirPinX, LOW);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX);  // Usa el retardo específico del eje
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }
}


void moveYMicroSteps (int steps, int dir) {  
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  if(dir == 1) { //Derecha
      digitalWrite(dirPinY, LOW);
  }

  else if(dir == 0) { //Izquierda
      digitalWrite(dirPinY, HIGH);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);  // Usa el retardo específico del eje
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }
}


void moveZMicroSteps (int steps, int dir) { 
  //Activa Microsteps para 1/4 de paso
  digitalWrite(M0, LOW);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  if(dir == 1) { // Abajo
      digitalWrite(dirPinZ, HIGH);
  }

  else if(dir == 0) {  //Arriba
      digitalWrite(dirPinZ, LOW);
  }
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);  // Usa el retardo específico del eje
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}


void moveToMicroSteps(int pos[6]) {
  //Convierte las coordenadas deseadas a pasos
  int x_steps = pos[0];
  int y_steps = pos[1];
  int z_steps = pos[2];
  int dir_x = pos[3];
  int dir_y = pos[4];
  int dir_z = pos[5];

  // Función para mover eje Y una cantidad de micro pasos especificada
  moveYMicroSteps(y_steps, dir_y);

  // Función para mover eje X una cantidad de micro pasos especificada
  moveXMicroSteps(x_steps, dir_x);

  // Función para mover eje Z una cantidad de micro pasos especificada
  moveZMicroSteps(z_steps, dir_z);
}