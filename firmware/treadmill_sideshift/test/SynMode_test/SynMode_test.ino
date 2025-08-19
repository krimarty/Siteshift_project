#include "MotorFeedback.h"
#include "I2CCommunication.h"
#include "UARTCommunication.h"
#include "PositionCalculator.h"
#include "OtherDevice.h"


enum class State {
    CALIB_LEFT,
    CALIB_RIGHT,
    SYN,
    MANUAL,
    RANDOM,
    SEQUENCE
};

ICommunication* comm;
MotorFeedback feedback;
PositionCalculator positionCalculator(0.0, 130.0);
OtherDevice otherDevice;


void setup() {
    Serial.begin(115200);
    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    // Choose communication method based on the mode
    comm = new UARTCommunication();
    //comm = new I2CCommunication(isMaster);

    comm->begin();

    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH); 
}

void loop() {
    delay(10);

    // Second board communication
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 50) {
        lastTime = millis();
        //Receive
        if (comm->available()) {otherDevice.update(comm->receiveMessage());}
        // Send
        comm->sendMessage(otherDevice.send(positionCalculator.computePosition(feedback.readRaw()), 0, 1));

        Serial.print("pos: ");
        Serial.println(positionCalculator.computePosition(feedback.readRaw()));

        Serial.print("other device pos: ");
        Serial.println(otherDevice.getPistonPos());
    }

}