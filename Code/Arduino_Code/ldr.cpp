#include "ldr.h"

void ldrInit(void) {
  pinMode(ldrPin, INPUT);
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
