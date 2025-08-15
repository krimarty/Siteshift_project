#include "Motor.h"


Motor motor;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting Motor class test...");

  motor.begin();
  motor.setInverse(false);  // Set to true to test inverse logic

  Serial.println("Motor initialized.");
}

void loop() {
  // === Forward ramp ===
  Serial.println("Turning RIGHT (ramping up)...");
  motor.right(100);  // Max duty
  for (int i = 0; i < 260; i++) {
    motor.dutyUpdate();  // Gradually ramps up
    delay(10);
  }

  delay(1000);  // Hold at full speed

  // === Stop ===
  Serial.println("Stopping...");
  motor.stop();
  while (true) {
    motor.dutyUpdate();
    if (motor.isFault()) {
      Serial.println("FAULT detected!");
      motor.resetFault();
    }
    if (analogRead(EN) == 0) break; // You can also check currentDuty if exposed
    delay(10);
  }

  delay(500);

  // === Reverse ramp ===
  Serial.println("Turning LEFT (ramping up)...");
  motor.left(100);
  for (int i = 0; i < 210; i++) {
    motor.dutyUpdate();
    delay(10);
  }

  delay(1000);  // Hold

  // === Sleep & Wake ===
  Serial.println("Putting motor to sleep...");
  motor.sleep();
  delay(1000);
  Serial.println("Waking motor up...");
  motor.wake();

  // === Disable & Enable ===
  Serial.println("Disabling motor...");
  motor.disable();
  delay(1000);
  Serial.println("Enabling motor...");
  motor.enable();

  delay(2000);  // Wait before next cycle
}
