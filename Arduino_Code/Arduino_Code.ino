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
int blockPos[6] = {500, 500, 2700, RIGHT, RIGHT, DOWN};
int sensorPos[6] = {500, 0, 300, RIGHT, RIGHT, DOWN};
int whitePos[6] = {0, 200, 450, RIGHT, RIGHT, DOWN};
int blackPos[6] = {0, 300, 450, RIGHT, RIGHT, DOWN};
int greenPos[6] = {0, 400, 450, RIGHT, RIGHT, DOWN};
int discardPos[6] = {500, 450, 450, RIGHT, RIGHT, DOWN};

int blockPosMicroSteps[6] = {2000, 2000, 10800, RIGHT, RIGHT, DOWN};
int sensorPosMicroSteps[6] = {2000, 0, 1200, RIGHT, RIGHT, DOWN};
int whitePosMicroSteps[6] = {0, 800, 1800, RIGHT, RIGHT, DOWN};
int blackPosMicroSteps[6] = {0, 1200, 1800, RIGHT, RIGHT, DOWN};
int greenPosMicroSteps[6] = {0, 1600, 1800, RIGHT, RIGHT, DOWN};
int discardPosMicroSteps[6] = {2000, 1800, 1800, RIGHT, RIGHT, DOWN};


void setup() {
  portsInit();
  state = INIT;
}

//MAIN
void loop() {
    switch (state) {
        case INIT:
            if (digitalRead(onPin) == 1) {
              lastState = state; // Guardar el estado actual antes de cambiar
              state = HOME;
            }
            break;

        case HOME:
            homeAllAxes();
            Gripper.write(0); //Abre el gripper
            if (deteccionBloque() == 1) {
              lastState = state;
              state = BLOQUE;
            }
            break;

        case BLOQUE:
            goToBlock();
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              lastState = state;
              state = SENSOR;
            }
            break;

        case SENSOR:
            goToSensor();
            detectarColor();
            delay(2000);
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
                switch (color) {
                  case WHITE:
                      lastState = state;
                      state = BLANCO;
                      break;

                  case BLACK:
                      lastState = state;
                      state = NEGRO;
                      break;

                  case GREEN:
                      lastState = state;
                      state = VERDE;
                      break;

                  case OTHER:
                      lastState = state;
                      state = ERROR;
                      break;
                  }
            }
            break;

        case BLANCO:
            goToWhite();
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              lastState = state;
              state = HOME;
            }
            break;

        case NEGRO:
            goToBlack();
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              lastState = state;
              state = HOME;
            }
            break;

        case VERDE:
            goToGreen();
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              lastState = state;
              state = HOME;
            }
            break;

        case ERROR:
            goToDiscard();
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              lastState = state;
              state = HOME;
            }
            break;

        case PAUSE:
            delay(5000);
            if (digitalRead(onPin) == 1) {
              switch (lastState) {
                  case BLOQUE:
                      state = SENSOR;
                      break;

                  case SENSOR:
                      switch (color) {
                        case WHITE:
                            lastState = state;
                            state = BLANCO;
                            break;

                        case BLACK:
                            lastState = state;
                            state = NEGRO;
                            break;

                        case GREEN:
                            lastState = state;
                            state = VERDE;
                            break;

                        case OTHER:
                            lastState = state;
                            state = ERROR;
                            break;
                        }
                      break;

                    case NEGRO:
                    case BLANCO:
                    case VERDE:
                    case ERROR:
                        state = HOME;
                        break;
                }
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
    delay(1000);
    openGripper();
    moveToSteps(blockPos);
    delay(3000);
    closeGripper();
    moveZSteps(950,UP);
}

void goToSensor(void) {
    delay(1000);
    moveToSteps(sensorPos);
    delay(1000);
    moveZSteps(300,UP);
}

void goToWhite(void) {
    delay(1000);
    moveToSteps(whitePos);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}

void goToBlack(void) {
    delay(1000);
    moveToSteps(blackPos);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}

void goToGreen(void) {
    delay(1000);
    moveToSteps(greenPos);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}

void goToDiscard(void) {
    delay(1000);
    moveToSteps(whitePos);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}



void goToBlockMicroSteps(void) {
    delay(1000);
    openGripper();
    moveToMicroSteps(blockPosMicroSteps);
    delay(3000);
    closeGripper();
    moveZMicroSteps(3800, UP);
}

void goToSensorMicroSteps(void) {
    delay(1000);
    moveToMicroSteps(sensorPosMicroSteps);
    delay(1000);
    moveZMicroSteps(1200, UP);
}

void goToWhiteMicroSteps(void) {
    delay(1000);
    moveToMicroSteps(whitePosMicroSteps);
    delay(1000);
    openGripper();
    delay(1000);
    moveZMicroSteps(1200, UP);
}

void goToBlackMicroSteps(void) {
    delay(1000);
    moveToMicroSteps(blackPosMicroSteps);
    delay(1000);
    openGripper();
    delay(1000);
    moveZMicroSteps(1200, UP);
}

void goToGreenMicroSteps(void) {
    delay(1000);
    moveToMicroSteps(greenPosMicroSteps);
    delay(1000);
    openGripper();
    delay(1000);
    moveZMicroSteps(1200, UP);
}

void goToDiscardMicroSteps(void) {
    delay(1000);
    moveToMicroSteps(whitePosMicroSteps);
    delay(1000);
    openGripper();
    delay(1000);
    moveZMicroSteps(1200, UP);
}


void closeGripper (void) {
  for (pos = 0; pos <= 100; pos += 1) {
    Gripper.write(pos);
    delay(20);
  }
  Gripper.write(100);
}

void openGripper (void) {
  for (pos = 100; pos >= 0; pos -= 1) {
    Gripper.write(pos);
    delay(20);
  }
  Gripper.write(0);
}
