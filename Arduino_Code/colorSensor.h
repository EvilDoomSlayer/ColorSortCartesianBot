#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

// Pin Definition
#define S0 40
#define S1 42
#define S2 38
#define S3 46
#define sensorOut 48


//Valores RGB para detectar el color blanco
#define whiteRed 26
#define whiteGreen 24
#define whiteBlue 24
//Valores RGB para detectar el color negro
#define blackRed 200
#define blackGreen 190
#define blackBlue 160
//Valores RGB para detectar el color verde
#define greenRed 140
#define greenGreen 100
#define greenBlue 101
//Tolerancia +- del sesnsor de color
#define tolerancia 40
//Iteraciones
#define iteraciones 70

// Function Prototypes
void colorSensorInit(void);
int getRedPW(void);
int getGreenPW(void);
int getBluePW(void);
int detectarColor(void);



#endif
