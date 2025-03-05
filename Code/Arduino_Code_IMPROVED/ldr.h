#ifndef LDR_H
#define LDR_H

#include <Arduino.h>

// Pin Definition
#define ldrPin A0

// Voltaje mínimo requerido para detectar el bloque
#define nivelLuzMinimo 4.4   

// Function Prototypes
void ldrInit(void);
bool deteccionBloque(void);

#endif