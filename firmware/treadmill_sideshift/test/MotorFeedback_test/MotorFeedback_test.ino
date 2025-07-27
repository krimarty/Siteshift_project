#include "MotorFeedback.h"

MotorFeedback feedback;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection
  Serial.println("Starting MotorFeedback test...");

  feedback.begin();  // Initialize the analog input pin
}

void loop() {
  int raw = feedback.readRaw();
  float percent = feedback.readPercent();

  Serial.print("Raw ADC value: ");
  Serial.print(raw);
  Serial.print(" | Smoothed percent: ");
  Serial.print(percent, 2);
  Serial.println(" %");

  delay(500);
}
