#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

// Pin Definition
#define S0 40
#define S1 42
#define S2 44
#define S3 46
#define sensorOut 48


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
enum colors { WHITE, BLACK, GREEN, OTHER};
extern enum colors color;  // Declarar como externa


// Function Prototypes
void colorSensorInit(void);
int getRedPW(void);
int getGreenPW(void);
int getBluePW(void);
void detectarColor(void);


#endif
