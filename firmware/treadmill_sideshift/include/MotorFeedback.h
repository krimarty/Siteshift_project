#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H
#include <Arduino.h>

constexpr int PIN_MOTOR_FEEDBACK = 28;
constexpr int SAMPLE_SIZE = 10;

class MotorFeedback {
private:
  int readings[SAMPLE_SIZE];
  int readIndex = 0;      
  int total = 0;             

public:
  MotorFeedback() {
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      readings[i] = 0;
    }
  }

  void begin() {
    pinMode(PIN_MOTOR_FEEDBACK, INPUT);
  }

  int readRaw() {
    int currentReading = analogRead(PIN_MOTOR_FEEDBACK);
    total -= readings[readIndex];        
    readings[readIndex] = currentReading; 
    total += currentReading;          
    readIndex = (readIndex + 1) % SAMPLE_SIZE;

    return total / SAMPLE_SIZE;
  }

  float readPercent(float vref = 3.3) {
    int value = readRaw();
    return (value / 4095.0) * 100;
  }
};

#endif
