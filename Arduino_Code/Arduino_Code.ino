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
void homeAllAxes(void);
void moveToSteps(int pos[3]);
void moveToMicroSteps(int pos[3]);
int getRedPW(void);
int getGreenPW(void);
int getBluePW(void);
String detectarColor(int (*readRed)(), int (*readGreeen)(), int (*readBlue)());


//Definicion de pines
//Step-Motors
#define dirPinY   2  
#define stepPinY  3 
#define dirPinZ   4  
#define stepPinZ  5
#define dirPinX   6 
#define stepPinX  7
#define M0        1
#define M1        13
#define M2        12

//Limit Switches
#define limitX    9   
#define limitY    10 
#define limitZ    11  

//Servomotor
#define servoPin  8

//Color sensor
#define S0 40
#define S1 41
#define S2 42
#define S3 43
#define sensorOut 44

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

// Posiciones 
int pos1[3] = {300, 300, 300};


// Sensor de color
//Valores RGB para detectar el color blanco
#define whiteRed 70
#define whiteGreen 70
#define whiteBlue 70
//Valores RGB para detectar el color negro
#define blackRed 495
#define blackGreen 490
#define blackBlue 400
//Valores RGB para detectar el color verde
#define greenRed 380
#define greenGreen 320
#define greenBlue 300

void setup() {
  portsInit();
}

void loop() {
   homeAllAxes();
   moveToSteps(pos1);
   moveToMicroSteps(pos1);
   delay(3000);
   homeAllAxes();
   SERVO();  // Controla el movimiento del servomotor
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
}




void homeX(void) {
  //Desactiva Microsteps para un movimiento mas rapido
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
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
  //Desactiva Microsteps para un movimiento mas rapido
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
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
  //Desactiva Microsteps para un movimiento mas rapido
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  // Homing para el eje Y hasta encontrar el límite
  while (digitalRead(limitZ) == LOW) {
    digitalWrite(dirPinZ, HIGH);
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ); // Usa el retardo específico del eje Z
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}

/*
void homeAllAxes(void) {
  //Desactiva Microsteps para un movimiento mas rapido
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  // Variables para controlar cuándo cada motor llega a su home
  bool xHomed = false;
  bool yHomed = false;
  bool zHomed = false;

  // Bucle que sigue ejecutándose hasta que todos los ejes están en home
  while (!xHomed || !yHomed || !zHomed) {
    
    // Homing del eje X
    if (!xHomed) {  // Si el eje X aún no ha llegado a su límite
      if (digitalRead(limitX) == LOW) {
        digitalWrite(dirPinX, HIGH);
        digitalWrite(stepPinX, HIGH);
        delayMicroseconds(stepDelayX);
        digitalWrite(stepPinX, LOW);
        delayMicroseconds(stepDelayX);
      } else {
        xHomed = true;  // Marca el eje X como homed si alcanza el límite
      }
    }

    // Homing del eje Y
    if (!yHomed) {  // Si el eje Y aún no ha llegado a su límite
      if (digitalRead(limitY) == LOW) {
        digitalWrite(dirPinY, HIGH);
        digitalWrite(stepPinY, HIGH);
        delayMicroseconds(stepDelayY);
        digitalWrite(stepPinY, LOW);
        delayMicroseconds(stepDelayY);
      } else {
        yHomed = true;  // Marca el eje Y como homed si alcanza el límite
      }
    }

    // Homing del eje Z
    if (!zHomed) {  // Si el eje Z aún no ha llegado a su límite
      if (digitalRead(limitZ) == LOW) {
        digitalWrite(dirPinZ, HIGH);
        digitalWrite(stepPinZ, HIGH);
        delayMicroseconds(stepDelayZ);
        digitalWrite(stepPinZ, LOW);
        delayMicroseconds(stepDelayZ);
      } else {
        zHomed = true;  // Marca el eje Z como homed si alcanza el límite
      }
    }
  }
}
*/

void homeAllAxes(void) {
  homeY();
  homeZ();
  homeZ();
}





void moveToSteps(int pos[3]) {
  //Desacriva Microsteps para un movimiento mas preciso
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  //Convierte las coordenadas deseadas a pasos
  int x_steps = pos[1];
  int y_steps = pos[2];
  int z_steps = pos[3];
  
 // Función para mover eje Y una cantidad de pasos especificada
  digitalWrite(dirPinY, LOW);
  for (int i = 0; i < y_steps; i++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);  // Usa el retardo específico del eje
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }

// Función para mover eje X una cantidad de pasos especificada
  digitalWrite(dirPinX, LOW);
  for (int i = 0; i < x_steps; i++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX);  // Usa el retardo específico del eje
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }

// Función para mover eje Z una cantidad de pasos especificada
  digitalWrite(dirPinZ, LOW);
  for (int i = 0; i < z_steps; i++) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);  // Usa el retardo específico del eje
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayZ);
  }
}


void moveToMicroSteps(int pos[3]) {
  //Activa Microsteps para un movimiento mas preciso
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  //Convierte las coordenadas deseadas a pasos
  int x_steps = pos[1];
  int y_steps = pos[2];
  int z_steps = pos[3];
  
 // Función para mover eje Y una cantidad de pasos especificada
  digitalWrite(dirPinY, LOW);
  for (int i = 0; i < y_steps; i++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayY);  // Usa el retardo específico del eje
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayY);
  }

// Función para mover eje X una cantidad de pasos especificada
  digitalWrite(dirPinX, LOW);
  for (int i = 0; i < x_steps; i++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX);  // Usa el retardo específico del eje
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);
  }

// Función para mover eje Z una cantidad de pasos especificada
  digitalWrite(dirPinZ, LOW);
  for (int i = 0; i < z_steps; i++) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayZ);  // Usa el retardo específico del eje
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

String detectarColor(int (*readRed)(), int (*readGreen)(), int (*readBlue)()) {
    int red = readRed();
    delay(200);
    int green = readGreen();
    delay(200);
    int blue = readBlue();
    delay(200);
    if (red < whiteRed && green < whiteGreen && blue < whiteBlue) {
      return "White";
    }
    if (red > blackRed && green > blackGreen && blue > blackBlue) {
      return "Black";
    }
    if (red > greenRed && green < greenGreen && blue > greenBlue) {
      return "Green";
    }
    else return "NA";
}