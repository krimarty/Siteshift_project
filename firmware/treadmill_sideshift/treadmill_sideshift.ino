#include "include/Motor.h"
#include "include/UserInterface.h"
#include "include/MotorFeedback.h"
#include "include/I2CCommunication.h"
#include "include/UARTCommunication.h"


ICommunication* comm;
Motor motor;
UserInterface ui;


void setup() {
    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    // Choose communication method based on the mode
    comm = new UARTCommunication();
    // comm = new I2CCommunication(isMaster);

    comm->begin();
    motor.begin();
    ui.setupInterrupts();
}

void loop() {
    motor.dutyUpdate();
    ui.readFilteredPotPercent();
    ui.processInputs();
}