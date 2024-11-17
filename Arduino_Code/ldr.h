#ifndef LDR_H
#define LDR_H

#include <Arduino.h>

// Pin Definition
#define ldrPin 34

// Voltaje mínimo requerido para detectar el bloque
#define nivelLuzMinimo 4.85   

// Function Prototypes
void ldrInit(void);
bool deteccionBloque(void);

#endif