#include <Arduino.h>
#include "I2CCommunication.h"
#include "UARTCommunication.h"
#include "OtherDevice.h"

ICommunication* comm;
OtherDevice otherDevice;

void setup() {
    Serial.begin(115200);
    delay(1000); 

    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    Serial.println(isMaster ? "Device is MASTER" : "Device is SLAVE");

    //comm = new UARTCommunication();
    comm = new I2CCommunication(isMaster);
    comm->begin();
    Serial.println("Communication initialized.");
}

void loop() {
    static unsigned long lastTime = 0;

    
    
    
    // Běží přibližně každé 2 sekundy
    if (millis() - lastTime > 2000) {
        lastTime = millis();

    // Second board communication
    //Receive
    if (comm->available()) {otherDevice.update(comm->receiveMessage());}
    Serial.print("[Receive] Message received: ");
    Serial.print(otherDevice.getPistonPos());
    Serial.print(",");
    Serial.print(otherDevice.getPotVal());
    Serial.print(",");
    Serial.println(otherDevice.getDirection());
    // Send
    comm->sendMessage(otherDevice.send(24, 25, 26));
    Serial.println("[Send] Message sent: 24, 25, 26");
    }
}
