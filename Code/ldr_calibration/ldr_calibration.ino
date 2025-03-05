#define ldrPin A0

void setup() {
  pinMode(ldrPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  float ldrRead = analogRead(ldrPin);
  float voltage = 5.0 - ((ldrRead / 1023.0) * 5);
  Serial.println(voltage);
}
