#include "colorSensor.h"

void colorSensorInit(void) {
  pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
	pinMode(sensorOut, INPUT);
	// Set Frequency scaling to 20%
	digitalWrite(S0,HIGH);
	digitalWrite(S1,LOW);
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


int detectarColor(void) {
  int redMax = 0;
  int greenMax = 0;
  int blueMax = 0;

  for (int i = 0; i < iteraciones; i++) {
    redMax = max(redMax, getRedPW());
    greenMax = max(greenMax, getGreenPW());
    blueMax = max(blueMax, getBluePW());
    delay(10); // Reducido para evitar pausas prolongadas
  }

    // Detectar color basándonos en los rangos definidos
  if (abs(redMax - whiteRed) <= tolerancia &&
      abs(greenMax - whiteGreen) <= tolerancia &&
      abs(blueMax - whiteBlue) <= tolerancia) {
    return 0; // Blanco
  }

  if (abs(redMax - blackRed) <= tolerancia &&
      abs(greenMax - blackGreen) <= tolerancia &&
      abs(blueMax - blackBlue) <= tolerancia) {
    return 1; // Negro
  }

  if (abs(redMax - greenRed) <= tolerancia &&
      abs(greenMax - greenGreen) <= tolerancia &&
      abs(blueMax - greenBlue) <= tolerancia) {
    return 2; // Verde
  }

  return 3; // Otro color
}
