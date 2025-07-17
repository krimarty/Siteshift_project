#include "include/Communication.h"
#include "include/Motor.h"
#include "include/UserInterface.h"
#include "include/MotorFeedback.h"


Communication comm;
Motor motor;
UserInterface ui;


void setup() {
    comm.begin();
    motor.begin();
    ui.setupInterrupts();
}

void loop() {
    motor.dutyUpdate();
    ui.readFilteredPotPercent();
    ui.processInputs();
}