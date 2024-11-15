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
#include <avr/io.h>
#include <avr/interrupt.h>

//Protótipos de funciones
void portsInit(void);
void homeX(void);
void homeY(void);
void homeZ(void);
void homeAllAxes(void);
void moveXSteps (int steps, int dir);
void moveYSteps (int steps, int dir);
void moveZSteps (int steps, int dir);
void moveXMicroSteps (int steps, int dir);
void moveYMicroSteps (int steps, int dir);
void moveZMicroSteps (int steps, int dir);
void moveToSteps(int pos[5]);
void moveToMicroSteps(int pos[5]);
void openGripper (void);
void closeGripper (void);
int getRedPW(void);
int getGreenPW(void);
int getBluePW(void);
void detectarColor(int (*readRed)(), int (*readGreen)(), int (*readBlue)());
bool deteccionBloque(void);
void goToBlock(void);
void goToSensor(void);
void goToWhite(void);
void goToBlack(void);
void goToGreen(void);
void goToDiscard(void);

//Definicion de pines
//Step-Motors
#define dirPinX   22 
#define stepPinX  24
#define dirPinY   9  
#define stepPinY  10 
#define dirPinZ   11  
#define stepPinZ  12

#define M0        6
#define M1        5
#define M2        4

//Limit Switches
#define limitX    28   
#define limitY    30 
#define limitZ    26  

//Servomotor
#define servoPin  32

//Color sensor
#define S0 40
#define S1 41
#define S2 42
#define S3 43
#define sensorOut 44

//Bontonera
#define onPin 2
#define buttonGnd 3

// Fotoresistencia
#define ldrPin 7

//Direcciones
#define RIGHT 1
#define LEFT  0
#define DOWN  1
#define UP    0


//Inicializacion del objeto Servo
Servo Gripper; 


//Variables Globales
//Maquina de estados
enum states { INIT, HOME, BLOQUE, SENSOR, NEGRO, BLANCO, VERDE, ERROR, PAUSE} state; // Enumeración para los estados del sistema
int lastState;

//Servomotor
int pos = 0;    // Variable para almacenar la posición del servomotor

// Variables de retardo específicas para cada eje (en microsegundos)
#define stepDelayY 1100 // Velocidad del eje Y
#define stepDelayZ 600  // Velocidad del eje Z
#define stepDelayX 1100 // Velocidad del eje X

// Posiciones 
int pos1[6] = {300, 300, 100, 1, 1, 1};
int pos2[6] = {200, 200, 0, 1, 1, 1};
int pos3[6] = {300, 300, 100, 0, 0, 0};


// Sensor de color
//Valores RGB para detectar el color blanco
#define whiteRed 30
#define whiteGreen 30
#define whiteBlue 25
//Valores RGB para detectar el color negro
#define blackRed 137
#define blackGreen 152
#define blackBlue 126
//Valores RGB para detectar el color verde
#define greenRed 156
#define greenGreen 102
#define greenBlue 98
//Tolerancia +- del sesnsor de color
#define tolerancia 15
// Enumeración para los posibles colores del sistema
enum colors { WHITE, BLACK, GREEN, OTHER} color; 


// LDR
// Voltaje mínimo requerido para detectar el bloque
#define nivelLuzMinimo 3.5           


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
            openGripper();
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
            detectarColor(getRedPW(), getGreenPW(), getBluePW());
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
  //Step-Motors
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
  //Servomotor
  Gripper.attach(servoPin);
  //Sensor de color
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
	pinMode(sensorOut, INPUT);
	// Set Frequency scaling to 20%
	digitalWrite(S0,HIGH);
	digitalWrite(S1,LOW);
  //Boton
  pinMode(onPin, INPUT);
  pinMode(buttonGnd, OUTPUT);
  digitalWrite(buttonGnd,LOW);
  //Deteccion bloque
  pinMode(ldrPin, INPUT);
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
  digitalWrite(M1, HIGH);
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

  // Función para mover eje X una cantidad de pasos especificada
  moveXSteps(x_steps, dir_x);

  // Función para mover eje Y una cantidad de pasos especificada
  moveYSteps(y_steps, dir_y);

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

  // Función para mover eje X una cantidad de micro pasos especificada
  moveXMicroSteps(x_steps, dir_x);

  // Función para mover eje Y una cantidad de micro pasos especificada
  moveYMicroSteps(y_steps, dir_y);

  // Función para mover eje Z una cantidad de micro pasos especificada
  moveZMicroSteps(z_steps, dir_z);
}


void goToBlock(void) {
    delay(1000);
    openGripper();
    delay(1000);
    moveYSteps(500,RIGHT);
    delay(1000);
    moveXSteps(500,RIGHT);
    delay(1000);
    //BAJA POR BLOQUE
    moveZSteps(2700,DOWN);
    delay(10000);
    closeGripper();
}
void goToSensor(void) {
    delay(1000);
    moveZSteps(950,UP);
    delay(1000);
    moveXSteps(500,RIGHT);
    delay(1000);
    //DETECTA COLOR
    moveZSteps(300,DOWN);
    delay(1000);
    moveZSteps(300,UP);
}

void goToWhite(void) {
    delay(1000);
    moveYSteps(200,RIGHT);
    delay(1000);
    moveZSteps(450,DOWN);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}

void goToBlack(void) {
    delay(1000);
    moveYSteps(300,RIGHT);
    delay(1000);
    moveZSteps(450,DOWN);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}
void goToGreen(void) {
    delay(1000);
    moveYSteps(400,RIGHT);
    delay(1000);
    moveZSteps(450,DOWN);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
}
void goToDiscard(void) {
    delay(1000);
    moveYSteps(700,RIGHT);
    delay(1000);
    moveZSteps(450,DOWN);
    delay(1000);
    openGripper();
    delay(1000);
    moveZSteps(300,UP);
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



// Function to read Red Pulse Widths
int getRedPW(void) {
	// Set sensor to read Red only
	digitalWrite(S2,LOW);
	digitalWrite(S3,LOW);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
int getGreenPW(void) {
	// Set sensor to read Green only
	digitalWrite(S2,HIGH);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
int getBluePW(void) {
	// Set sensor to read Blue only
	digitalWrite(S2,LOW);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

void detectarColor(int (*readRed)(), int (*readGreen)(), int (*readBlue)()) {
    int red = readRed();
    delay(200);
    int green = readGreen();
    delay(200);
    int blue = readBlue();
    delay(200);
    if ((red+tolerancia) < whiteRed && (green+tolerancia) < whiteGreen && (blue+tolerancia) < whiteBlue) {
      color = WHITE;
    }
    if ((red-tolerancia) > blackRed && (green-tolerancia) > blackGreen && (blue-tolerancia) > blackBlue) {
      color = BLACK;
    }
    if ((red-tolerancia) > greenRed && (green+tolerancia) < greenGreen && (blue-tolerancia) > greenBlue) {
      color = GREEN;
    }
    else color = OTHER;
}

bool deteccionBloque(void) {
  
  float ldrRead = analogRead(ldrPin);
  float voltage = 5.0 - ((ldrRead / 1023.0) * 5);
  
  if (voltage >= nivelLuzMinimo) { // Si el voltaje es mayor o igual al nivel minimo, hay un bloque
      return 1;
  } else { // Si el voltaje es menor al nivel mínimo, no hay un bloque
      return 0;
  }
}





