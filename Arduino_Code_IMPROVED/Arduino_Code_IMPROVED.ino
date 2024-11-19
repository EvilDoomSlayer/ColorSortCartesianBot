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
* - 1 LDR 2MOhms
* - 3 limit-sitches
* - 2 Reles 5V
* - 2 leds 12V 
* - Botonera Industrial
* 
* Este programa automatiza las clasificacion de cubos por color.
*/

//Librerias
#include <Servo.h>
#include "stepMotors.h"
#include "colorSensor.h"
#include "ldr.h"

//Definicion de pines
//Servomotor
#define servoPin  32

//Bontonera
#define onPin 2
#define buttonGnd 3

//Leds with relees
#define ledBlue 7
#define ledGreen 8

//Inicializacion del objeto Servo
Servo Gripper; 

//Variables Globales
//Maquina de estados
enum states { INIT, HOME, BLOQUE, SENSOR, NEGRO, BLANCO, VERDE, ERROR, PAUSE} state; // Enumeración para los estados del sistema
int lastState;
unsigned long pauseStartTime = 0;

//Bloques
#define blocksPerContainers 1  // Blocks -1
uint8_t whiteBlocks = 0;
uint8_t blackBlocks = 0;
uint8_t greenBlocks = 0;

// Servo
#define gripperPositionOpen 0
#define gripperPositionClosed 50

// Posiciones
// Pasos
const int blockPos[6] = {495, 712, 0, RIGHT, RIGHT, DOWN};
const int sensorPos[6] = {511, 500, 340, RIGHT, LEFT, DOWN};
const int whitePos[6] = {0, 488, 745, RIGHT, RIGHT, DOWN};
const int blackPos[6] = {0, 598, 745, RIGHT, RIGHT, DOWN};
const int greenPos[6] = {0, 707, 745, RIGHT, RIGHT, DOWN};
const int discardPos[6] = {300, 488, 0, RIGHT, RIGHT, DOWN};

//Micropasos
const int blockPosMicroSteps[6] = {1980, 2848, 0, RIGHT, RIGHT, DOWN};
const int sensorPosMicroSteps[6] = {2045, 2000, 340, RIGHT, LEFT, DOWN};
const int whitePosMicroSteps[6] = {0, 1954, 745, RIGHT, RIGHT, DOWN};
const int blackPosMicroSteps[6] = {0, 2391, 745, RIGHT, RIGHT, DOWN};
const int greenPosMicroSteps[6] = {0, 2829, 745, RIGHT, RIGHT, DOWN};
const int discardPosMicroSteps[6] = {2000, 1954, 0, RIGHT, RIGHT, DOWN};

//Protótipos de funciones
void portsInit(void);
void goToBlock(void);
void goToSensor(void);
void goToDiscard(void);
void gripBlock(void);
void processColor(uint8_t* blockCounter, int* positionMicroSteps, states nextState);
states getStateFromColor(int color);
void ledsHomming(void);
void ledsWorking(void);


void setup() {
  portsInit();
  state = INIT;
}

//MAIN
void loop() {
    switch (state) {
        case INIT:
            Gripper.write(gripperPositionOpen);
            if (digitalRead(onPin) == 1) {
              state = HOME;
            }
            break;

        case HOME:
            ledsHomming();
            homeAllAxes();
            Gripper.write(gripperPositionOpen);
            if (digitalRead(onPin) == 0) { 
              state = INIT;
            }
            else if (deteccionBloque() == 1) {  
              state = BLOQUE;
              ledsWorking();
              delay(3000);
            }
            break;

        case BLOQUE:
            if (digitalRead(onPin) == 0) {
              lastState = state;  // Guardar el estado actual antes de cambiar
              state = PAUSE;
            } else {
              goToBlock();
              if (deteccionBloque() == 0) {  // Comprueba que el bloque que siga ahi
                state = HOME;
              }
              else if (deteccionBloque() == 1) {
                gripBlock();
                state = SENSOR;
              }
            }
            break;

        case SENSOR:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
                goToSensor();
                delay(100);
                int color = detectarColor();
                delay(100);
                state = getStateFromColor(color);
                if (state == ERROR) {
                  color = detectarColor(); // Segunda revisión
                  state = getStateFromColor(color);
                }
            }
            break;

        case BLANCO:
            processColor(&whiteBlocks, whitePosMicroSteps, BLANCO);
            break;

        case NEGRO:
            processColor(&blackBlocks, blackPosMicroSteps, NEGRO);
            break;

        case VERDE:
            processColor(&greenBlocks, greenPosMicroSteps, VERDE);
            break;

        case ERROR:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToDiscard();
              ledsHomming();
              state = HOME;
            }
            break;

        case PAUSE:
            if (pauseStartTime == 0) pauseStartTime = millis();
            if (millis() - pauseStartTime >= 5000) {
              state = (digitalRead(onPin) == 1) ? lastState : HOME;
              pauseStartTime = 0;
            }
            break;
    }
}

// Inicializacion de puertos
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
  //Leds
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  //Deteccion bloque
  ldrInit();
}

// Funciones de posicicionamiento
void goToBlock(void) {
    Gripper.write(gripperPositionOpen);
    moveToMicroSteps(blockPosMicroSteps);
}

void goToSensor(void) {
    moveToMicroSteps(sensorPosMicroSteps);
}

void goToDiscard(void) {
    moveZSteps(340,UP);
    moveToMicroSteps(discardPosMicroSteps);
    Gripper.write(gripperPositionOpen);
    delay(500);
}

void gripBlock(void) {
    moveZSteps(2258, DOWN);
    delay(500);
    Gripper.write(gripperPositionOpen);
    delay(500);
    moveZSteps(1000, UP);
}


//Colores
void processColor(uint8_t* blockCounter, int* positionMicroSteps, states nextState) {
    if (digitalRead(onPin) == 0) {
        lastState = state;
        state = PAUSE;
    } else {
        moveZSteps(340, UP);
        moveToMicroSteps(positionMicroSteps);
        Gripper.write(gripperPositionOpen);
        delay(500);
        moveZSteps(300, UP);
        (*blockCounter)++;
        ledsHomming();
        state = HOME;
    }
}

states getStateFromColor(int color) {
    switch (color) {
        case 0: return (whiteBlocks <= blocksPerContainers) ? BLANCO : HOME;
        case 1: return (blackBlocks <= blocksPerContainers) ? NEGRO : HOME;
        case 2: return (greenBlocks <= blocksPerContainers) ? VERDE : HOME;
        default: return ERROR;
    }
}


//Leds
void ledsHomming(void) {
    digitalWrite(ledGreen,LOW); // Enciende el led verde
    digitalWrite(ledBlue,HIGH); // Apaga el led azul
}

void ledsWorking(void) {
    digitalWrite(ledBlue,LOW); // Enciende el led azul
    digitalWrite(ledGreen,HIGH); // Apaga el led verde
}
