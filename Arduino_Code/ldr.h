#ifndef LDR_H
#define LDR_H

#include <Arduino.h>

// Pin Definition
#define ldrPin 7

// Voltaje mínimo requerido para detectar el bloque
#define nivelLuzMinimo 3.5   

// Function Prototypes
void ldrInit(void);
bool deteccionBloque(void);

#endif