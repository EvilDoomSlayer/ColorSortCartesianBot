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


//Protótipos de funciones
void portsInit(void);
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

//Leds with relees
#define ledBlue 7
#define ledGreen 8

//Inicializacion del objeto Servo
Servo Gripper; 


//Variables Globales
//Maquina de estados
enum states { INIT, HOME, BLOQUE, SENSOR, NEGRO, BLANCO, VERDE, ERROR, PAUSE} state; // Enumeración para los estados del sistema
int lastState;

//Bloques
#define blocksPerContainers 1  // Blocks -1
uint8_t whiteBlocks = 0;
uint8_t blackBlocks = 0;
uint8_t greenBlocks = 0;

// Posiciones
// Pasos
int blockPos[6] = {495, 712, 0, RIGHT, RIGHT, DOWN};
int sensorPos[6] = {511, 500, 340, RIGHT, LEFT, DOWN};
int whitePos[6] = {0, 488, 745, RIGHT, RIGHT, DOWN};
int blackPos[6] = {0, 598, 745, RIGHT, RIGHT, DOWN};
int greenPos[6] = {0, 707, 745, RIGHT, RIGHT, DOWN};
int discardPos[6] = {300, 488, 0, RIGHT, RIGHT, DOWN};

//Micropasos
int blockPosMicroSteps[6] = {1980, 2848, 0, RIGHT, RIGHT, DOWN};
int sensorPosMicroSteps[6] = {2045, 2000, 340, RIGHT, LEFT, DOWN};
int whitePosMicroSteps[6] = {0, 1954, 745, RIGHT, RIGHT, DOWN};
int blackPosMicroSteps[6] = {0, 2391, 745, RIGHT, RIGHT, DOWN};
int greenPosMicroSteps[6] = {0, 2829, 745, RIGHT, RIGHT, DOWN};
int discardPosMicroSteps[6] = {2000, 1954, 0, RIGHT, RIGHT, DOWN};

/*
int u =0;
// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
*/

void setup() {
  portsInit();
  state = INIT;
  // Setup Serial Monitor
	Serial.begin(9600);
}

//MAIN
void loop() {
  /*
    if (u==0){
      u++;
      homeAllAxes();
      Gripper.write(0); //Abre el gripper
      goToBlock();
      moveZSteps(2258, DOWN);
      delay(500);
      Gripper.write(50); //Cierra el gripper
      delay(500);
      moveZSteps(1000, UP);
      goToSensor();
    }
    // Read Red Pulse Width
	redPW = getRedPW();
	// Delay to stabilize sensor
	delay(100);

	// Read Green Pulse Width
	greenPW = getGreenPW();
	// Delay to stabilize sensor
	delay(100);

	// Read Blue Pulse Width
	bluePW = getBluePW();
	// Delay to stabilize sensor
	delay(100);

  
	// Print output to Serial Monitor
	Serial.print("Red PW = ");
	Serial.print(redPW);
	Serial.print(" - Green PW = ");
	Serial.print(greenPW);
	Serial.print(" - Blue PW = ");
	Serial.println(bluePW);
  
  int color = detectarColor();

  switch (color) {
      case 0:
         Serial.println("Blanco");
         break;

      case 1:
         Serial.println("Negro");
         break;

      case 2:
          Serial.println("Verde");
          break;

      case 3:
          Serial.println("otro");
          break;
  }
  */
    switch (state) {
        case INIT:
            Gripper.write(0); //Abre el gripper
            if (digitalRead(onPin) == 1) {
              state = HOME;
            }
            break;

        case HOME:
            ledsHomming();
            homeAllAxes();
            Gripper.write(0); //Abre el gripper
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
              // Antes de bajar poe el bloque realiza una revision de que siga ahi
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
                goToSensor();
                delay(500);
                int color = detectarColor();
                delay(500);
                switch (color) {
                  case 0: // Blanco
                      if (whiteBlocks <= blocksPerContainers) {
                        state = BLANCO;
                        whiteBlocks++;
                      } else {
                        state = HOME;
                      }
                      break;

                  case 1: // Negro
                      if (blackBlocks <= blocksPerContainers) {
                        state = NEGRO;
                        blackBlocks++;
                      } else {
                        state = HOME;
                      }
                      break;

                  case 2: // Verde
                      if (greenBlocks <= blocksPerContainers) {
                        state = VERDE;
                        greenBlocks++;
                      } else {
                        state = HOME;
                      }
                      break;

                  case 3: // Otro
                      // Realiza una segunda revision de color como seguridad
                      int color = detectarColor();
                      switch (color) {
                      case 0: // Blanco
                          if (whiteBlocks <= blocksPerContainers) {
                            state = BLANCO;
                            whiteBlocks++;
                          } else {
                            state = HOME;
                          }
                          break;

                      case 1: // Negro
                          if (blackBlocks <= blocksPerContainers) {
                            state = NEGRO;
                            blackBlocks++;
                          } else {
                            state = HOME;
                          }
                          break;

                      case 2: // Verde
                          if (greenBlocks <= blocksPerContainers) {
                            state = VERDE;
                            greenBlocks++;
                          } else {
                            state = HOME;
                          }
                          break;

                      case 3: // Otro
                          state = ERROR;
                          break;

                      }
                      break;
                  }
            }
            break;

        case BLANCO:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToWhite();
              ledsHomming();
              state = HOME;
            }
            break;

        case NEGRO:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToBlack();
              ledsHomming();
              state = HOME;
            }
            break;

        case VERDE:
            if (digitalRead(onPin) == 0) {
              lastState = state;
              state = PAUSE;
            } else {
              goToGreen();
              ledsHomming();
              state = HOME;
            }
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
            delay(5000);
            /*if (digitalRead(onPin) == 1) {
              state = lastState;
            } else {
              state = HOME;
              ledsHomming();
            }
            */
            state = HOME;
            {ledsHomming();
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
    Gripper.write(0); //Abre el gripper
    moveToMicroSteps(blockPosMicroSteps);
}

void goToSensor(void) {
    moveToMicroSteps(sensorPosMicroSteps);
}

void goToWhite(void) {
    moveZSteps(340,UP);
    moveToMicroSteps(whitePosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToBlack(void) {
    moveZSteps(340,UP);
    moveToMicroSteps(blackPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToGreen(void) {
    moveZSteps(340,UP);
    moveToMicroSteps(greenPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
    moveZSteps(300, UP);
}

void goToDiscard(void) {
    moveZSteps(340,UP);
    moveToMicroSteps(discardPosMicroSteps);
    Gripper.write(0); //Abre el gripper
    delay(500);
}

void ledsHomming(void) {
    digitalWrite(ledGreen,LOW); // Enciende el led verde
    digitalWrite(ledBlue,HIGH); // Apaga el led azul
}

void ledsWorking(void) {
    digitalWrite(ledBlue,LOW); // Enciende el led azul
    digitalWrite(ledGreen,HIGH); // Apaga el led verde
}
