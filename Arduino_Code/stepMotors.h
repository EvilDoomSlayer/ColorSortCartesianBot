#ifndef STEPMOTORS_H
#define STEPMOTORS_H

#include <Arduino.h>

// Pin Definition
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


// Retardo específico para cada eje (en microsegundos)
#define stepDelayY 600 // Velocidad del eje Y
#define stepDelayZ 600  // Velocidad del eje Z
#define stepDelayX 600 // Velocidad del eje X


//Direcciones
#define RIGHT 1
#define LEFT  0
#define DOWN  1
#define UP    0


// Conversion
#define microStepsPerMil 19.178

// Function Prototypes
void stepMotorsInit(void);
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

#endif