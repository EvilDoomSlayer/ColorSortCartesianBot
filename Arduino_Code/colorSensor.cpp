#include "colorSensor.h"

enum colors color;  // Definir la variable



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


#include "colorSensor.h"

void detectarColor(void) {
    int red = getRedPW();
    delay(200);
    int green = getGreenPW();
    delay(200);
    int blue = getBluePW();
    delay(200);
    if ((red + tolerancia) < whiteRed && (green + tolerancia) < whiteGreen && (blue + tolerancia) < whiteBlue) {
        color = WHITE;
    }
    if ((red - tolerancia) > blackRed && (green - tolerancia) > blackGreen && (blue - tolerancia) > blackBlue) {
        color = BLACK;
    }
    if ((red - tolerancia) > greenRed && (green + tolerancia) < greenGreen && (blue - tolerancia) > greenBlue) {
        color = GREEN;
    } else {
        color = OTHER;
    }
}
