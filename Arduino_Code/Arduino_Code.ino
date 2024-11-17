/*
 * Arduino_Code.c
 * Created: 02/11/2024 05:48:52 p. m.
 * Author : Team_1
 */

/*
* Programa del Cartesian Robot
* Este programa controla:
* - 4 drivers DRV8825
* - 1 Servomotor Mg995
* - 1 Sensor de color TCS3200
* - 1 LDR
* - 3 limit-sitches
* - Botonera Industrial
* 
* Este programa automatiza las clasificacion de cubos por color.
*/

//Librerias
#include <Servo.h>
#include "stepMotors.h"
#include "colorSensor.h"
#include "ldr.h"


//Protótipos de funciones
void portsInit(void);
void openGripper (void);
void closeGripper (void);
void goToBlock(void);
void goToSensor(void);
void goToWhite(void);
void goToBlack(void);
void goToGreen(void);
void goToDiscard(void);

//Definicion de pines
//Servomotor
#define servoPin  32

//Bontonera
#define onPin 2
#define buttonGnd 3

//Inicializacion del objeto Servo
Servo Gripper; 


//Variables Globales
//Maquina de estados
enum states { INIT, HOME, BLOQUE, SENSOR, NEGRO, BLANCO, VERDE, ERROR, PAUSE} state; // Enumeración para los estados del sistema
int lastState;

//Servomotor
int pos = 0;    // Variable para almacenar la posición del servomotor

// Posiciones 

int blockPos[6] = {495, 712, 0, RIGHT, RIGHT, DOWN};
int sensorPos[6] = {511, 500, 330, RIGHT, LEFT, DOWN};
int whitePos[6] = {0, 488, 745, RIGHT, RIGHT, DOWN};
int blackPos[6] = {0, 488, 745, RIGHT, RIGHT, DOWN};
int greenPos[6] = {0, 488, 745, RIGHT, RIGHT, DOWN};
int discardPos[6] = {300, 488, 745, RIGHT, RIGHT, DOWN};

int blockPosMicroSteps[6] = {1980, 2848, 0, RIGHT, RIGHT, DOWN};
int sensorPosMicroSteps[6] = {2045, 2000, 345, RIGHT, LEFT, DOWN};
int whitePosMicroSteps[6] = {0, 1954, 745, RIGHT, RIGHT, DOWN};
int blackPosMicroSteps[6] = {0, 1954, 745, RIGHT, RIGHT, DOWN};
int greenPosMicroSteps[6] = {0, 1954, 745, RIGHT, RIGHT, DOWN};
int discardPosMicroSteps[6] = {2000, 1954, 745, RIGHT, RIGHT, DOWN};


void setup() {
  portsInit();
  state = INIT;
}

//MAIN
void loop() {
    switch (state) {
        case INIT:
            if (digitalRead(onPin) == 1) {
              state = HOME;
            }
            break;

        case HOME:
            homeAllAxes();
            Gripper.write(0); //Abre el gripper
            if (digitalRead(onPin) == 0) { 
              state = INIT;
            }
            else if (deteccionBloque() == 1) {  
              state = BLOQUE;
              delay(3000);
            }
            break;

        case BLOQUE:
            if (digitalRead(onPin) == 0) {
              lastState = state;  // Guardar el estado actual antes de cambiar
              state = PAUSE;
            } else {
              goToBlockMicroSteps();
              if (deteccionBloque() == 0) {  
                state = HOME;
              }
              else if (deteccionBloque() == 1) {
                moveZSteps(2258, DOWN);
                delay(500);
                Gripper.write(50); //Cierra el gripper
                delay(500);
                moveZSteps(1000, UP);
                state = SENSOR;
              }
            }
            break;

        case SENSOR:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
                goToSensorMicroSteps();
                delay(1000);
                detectarColor();
                delay(500);
                switch (color) {
                  case WHITE:
                      state = BLANCO;
                      break;

                  case BLACK:
                      state = NEGRO;
                      break;

                  case GREEN:
                      state = VERDE;
                      break;

                  case OTHER:
                      state = ERROR;
                      break;
                  }
            }
            break;

        case BLANCO:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToWhiteMicroSteps();
              state = HOME;
            }
            break;

        case NEGRO:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToBlackMicroSteps();
              state = HOME;
            }
            break;

        case VERDE:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToGreenMicroSteps();
              state = HOME;
            }
            break;

        case ERROR:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToDiscardMicroSteps();
              state = HOME;
            }
            break;

        case PAUSE:
            delay(5000);
            if (digitalRead(onPin) == 1) {
              state = lastState;
            } else {
              state = HOME;
            }
            break;
    }
}


void portsInit(void) {
  //Mototres a pasos
  stepMotorsInit();
  //Servomotor
  Gripper.attach(servoPin);
  //Sensor de color
	colorSensorInit();
  //Boton
  pinMode(onPin, INPUT);
  pinMode(buttonGnd, OUTPUT);
  digitalWrite(buttonGnd,LOW);
  //Deteccion bloque
  ldrInit();
}


void goToBlock(void) {
    Gripper.write(0); //Abre el gripper
    moveToSteps(blockPos);
    delay(500);
    Gripper.write(50); //Cierra el gripper
    delay(500);
    moveZSteps(1000,UP);
}

void goToSensor(void) {
    moveToSteps(sensorPos);
}

void goToWhite(void) {
    moveZSteps(330,UP);
    moveToSteps(whitePos);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300,UP);
}

void goToBlack(void) {
    moveZSteps(330,UP);
    moveToSteps(blackPos);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300,UP);
}

void goToGreen(void) {
    moveZSteps(330,UP);
    moveToSteps(greenPos);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(330,UP);
}

void goToDiscard(void) {
    moveZSteps(330,UP);
    moveToSteps(whitePos);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300,UP);
}



void goToBlockMicroSteps(void) {
    Gripper.write(0); //Abre el gripper
    moveToMicroSteps(blockPosMicroSteps);
}

void goToSensorMicroSteps(void) {
    moveToMicroSteps(sensorPosMicroSteps);
}

void goToWhiteMicroSteps(void) {
    moveZSteps(345,UP);
    moveToMicroSteps(whitePosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToBlackMicroSteps(void) {
    moveZSteps(345,UP);
    moveToMicroSteps(blackPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToGreenMicroSteps(void) {
    moveZSteps(345,UP);
    moveToMicroSteps(greenPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToDiscardMicroSteps(void) {
    moveZSteps(345,UP);
    moveToMicroSteps(discardPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

