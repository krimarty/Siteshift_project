#include "include/Communication.h"
#include "include/Motor.h"
#include "include/UserInterface.h"
#include "include/MotorFeedback.h"


Communication comm;
Motor motor;
UserInterface ui;

// I2C slave event handler
void onReceiveEvent(int numBytes) {
    String msg = "";
    while (Wire.available()) {
        char c = Wire.read();
        msg += c;
    }
    // zpracuj zprávu, ovládej motor apod.
}

void setup() {
    comm.begin();
    motor.begin();
    ui.setupInterrupts();

#ifdef USE_I2C
    if (!comm.isMaster) {
        Wire.onReceive(onReceiveEvent);
    }
#endif
    // případně další inicializace
}

void loop() {
    // UART komunikace
#ifdef USE_UART
    if (comm.available()) {
        String msg = comm.receiveMessage();
        // zpracuj zprávu, ovládej motor apod.
    }
#endif

    // rampu motoru aktualizuj pravidelně
    motor.dutyUpdate();
    ui.readFilteredPotPercent();
    ui.processInputs();
}