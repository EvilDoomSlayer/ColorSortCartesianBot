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
String detectarColor(int (*readRed)(), int (*readGreen)(), int (*readBlue)());


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


//Direcciones
#define RIGHT 1
#define LEFT  0
#define DOWN  1
#define UP    0


//Inicializacion del objeto Servo
Servo Gripper; 


//Variables Globales
//Maquina de estados
enum states { ON, OFF} state; // Enumeración para los estados del sistema
bool stateChange = 0;

//Servomotor
int pos = 0;    // Variable para almacenar la posición del servomotor

// Variables de retardo específicas para cada eje (en microsegundos)
int stepDelayY = 1100; // Velocidad del eje Y
int stepDelayZ = 1500;  // Velocidad del eje Z
int stepDelayX = 1100; // Velocidad del eje X

// Posiciones 
int pos1[6] = {300, 300, 100, 1, 1, 1};
int pos2[6] = {200, 200, 0, 1, 1, 1};
int pos3[6] = {300, 300, 100, 0, 0, 0};




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
  init_interrupt_E4();
  state = OFF;
}

//MAIN
void loop() {
    switch (state) {
      case OFF:
          
      break;

      case ON:
          homeAllAxes();
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
          delay(1000);
          //SUBE CON BLOQUE AGARRADO
          moveZSteps(950,UP);
          delay(1000);
          moveXSteps(500,RIGHT);
          delay(1000);
          //DETECTA COLOR
          moveZSteps(300,DOWN);
          delay(1000);
          moveZSteps(300,UP);
          delay(1000);
          moveYSteps(200,RIGHT);
          delay(1000);
          //DEJA BLOQUE EN RECIPIENTE
          moveZSteps(450,DOWN);
          delay(1000);
          openGripper();
          delay(1000);
          //TEMINA DE DEJARLO
          moveZSteps(300,UP);
          delay(1000);
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
}

void init_interrupt_E4(void) {
    // Configurar el pin PE4 como entrada
    DDRE &= ~(1 << PE4);

    // Habilitar la interrupción en cambio de nivel (rising edge) para PE4
    // INT4 está asociado a PE4 en el ATmega2560
    EICRB |= (1 << ISC40);  // Configura para flanco de cambio (subida o bajada)
    
    // Habilitar interrupción INT4
    EIMSK |= (1 << INT4);
    
    // Habilitar interrupciones globales
    sei();
}

ISR(INT4_vect) {
    if (PINE & (1 << PE4)) {
        state = ON;  // Si PE4 está en HIGH, cambia estado a 1
        //stateChange=1;
    } else {
        state = OFF;  // Si PE4 está en LOW, cambia estado a 0
    }
}



void homeX(void) {
  //Desactiva Microsteps para un movimiento mas rapido
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
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
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

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
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

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
  //Desactiva Microsteps para un movimiento mas preciso
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  if(dir == 1) { // Abajo
      digitalWrite(dirPinZ, LOW);
  }

  else if(dir == 0) {  //Arriba
      digitalWrite(dirPinZ, HIGH);
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
