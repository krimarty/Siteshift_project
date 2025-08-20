/* =============================================================
 * Project:   treadmill_sideshift
 * File:      treadmill_sideshift.ino
 * Author:    Martin Kriz
 * Company:   Ullmanna s.r.o.
 * Created:   2025-08-20
 * -------------------------------------------------------------
 * Description:
 *   This file is an Arduino IDE sketch for Raspberry Pi Pico 2.
 *   It is designed to control the LINAK LA14 linear actuator
 *   and specifically developed for the Sideshift Board v0.0.0.
 *
 * Notes:
 *   - All boards must use the SAME communication protocol.
 *   - For I2C, the master/slave role is selected via hardware jumper.
 *   - The PCB pins labeled "COMMUNICATION TX" and "COMMUNICATION RX"
 *     are shared by both UART and I2C:
 *         TX → SDA
 *         RX → SCL
 *   - WARNING: When switching between UART and I2C, the wiring
 *     must be crossed accordingly.
 * ============================================================= */

#include "include/Motor.h"
#include "include/UserInterface.h"
#include "include/MotorFeedback.h"
#include "include/I2CCommunication.h"
#include "include/UARTCommunication.h"
#include "include/Pid.h"
#include "include/PositionCalculator.h"
#include "include/OtherDevice.h"

// State machine
enum class State {
    CALIB_LEFT,     // left calibration
    CALIB_RIGHT,    // right calibration
    SYN,            // second motor following
    MANUAL,         // manual control
    RANDOM,         // going random
    SEQUENCE        // going left and right
};
State currentState = State::CALIB_LEFT;

// Motor spec
constexpr int  PISTON_LEN = 130; // [mm]

// Calibration parameters and variables
constexpr long unsigned int  DELAY_MS = 10; // [ms] speed of main loop
constexpr uint8_t calib_duty = 75;          // [%] calibration duty cycle
constexpr int stableThreshold = 2/(DELAY_MS/1000.0f);   // number of stable samples to detect min, max position
constexpr int changeTolerance = 2;          // tolerance of stable samples
int lastValue = 0;
int stableCount = 0;

// Communication variables
constexpr int  COMM_MS = 50;    // [ms] communication period
unsigned long lastMillis = 0;
bool comm_status = false;
unsigned long lastToggle = 0;
bool ledState = false; 

// Random mode variables
static int targetPos = 0;

// Sequention mode variables
bool goingLeft = true;

// Classes
ICommunication* comm;
Motor motor;
UserInterface ui;
MotorFeedback feedback;
Pid pidPos(8.5, 0.01, 0);
PositionCalculator positionCalculator(0.0, 130.0);
OtherDevice otherDevice;


State nextState(State state)
{
    if(ui.isSyncMode()) return State::SYN;
    else if(ui.getModeType() == ModeType::MANUAL) return State::MANUAL;
    else if(ui.getModeType() == ModeType::RANDOM) return State::RANDOM;
    else return State::SEQUENCE;
}

void setup() {
    Serial.begin(115200);
    pinMode(MS_PIN, INPUT_PULLUP);
    bool isMaster = digitalRead(MS_PIN) == LOW;

    // -------------------------------------------------------------
    // SELECT COMMUNICATION PROTOCOL
    // !!! IMPORTANT: all boards must use the SAME protocol !!!
    //
    // Uncomment exactly one line depending on the protocol you want:
    //
    //   UARTCommunication → communication via UART
    //   I2CCommunication  → communication via I2C 
    //                        (master/slave role is selected by HW jumper)
    //
    // -------------------------------------------------------------
    comm = new UARTCommunication();
    //comm = new I2CCommunication(isMaster);

    comm->begin();
    motor.begin();

    // Setup finished
    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH); 
}

void loop() {
    delay(DELAY_MS);
    unsigned long currentMillis = millis();
    unsigned long deltaMillis = currentMillis - lastMillis;

    // Periodical class updates
    motor.dutyUpdate();
    //ui.readFilteredPotPercent();

    // Second board communication
    static unsigned long lastTime_comm = 0;
    if (millis() - lastTime_comm > COMM_MS && currentState == State::SYN) {
        lastTime_comm = millis();
        comm_status = comm->isConnected();
        Serial.print("Comm status: ");
        Serial.println(comm_status);
        //Receive
        if (comm->available()) {otherDevice.update(comm->receiveMessage());}
        // Send
        comm->sendMessage(otherDevice.send(positionCalculator.computePosition(feedback.readRaw()), ui.readFilteredPotPercent(), ui.getDirectionNum()));
    }

    // LEDs 
    if (currentState != State::MANUAL)
    {
        ui.setLeftLed(false);
        ui.setRightLed(false);
    }
    if (currentState == State::SEQUENCE || currentState == State::RANDOM)
        ui.setSignalLed(true);
    else if (currentState == State::SYN)
    {
        if (comm_status)
            ui.setSignalLed(true);
        else
        {
            if (currentMillis - lastToggle >= 500)
            {
                ledState = !ledState;
                ui.setSignalLed(ledState ? HIGH : LOW);
                lastToggle = currentMillis;
            }
        }
    }
    else
        ui.setSignalLed(false);

    // Inverse mode settings
    if (currentState != State::CALIB_LEFT && currentState != State::CALIB_RIGHT)
    if(ui.isInverse()) {
        positionCalculator.setInverse(true);
        motor.setInverse(true);
    } else {
        positionCalculator.setInverse(false);
        motor.setInverse(false);
    }

    // State machine
    switch (currentState) {
        case State::CALIB_LEFT:{
            motor.left(calib_duty);
            int value = feedback.readRaw();
            if (abs(value - lastValue) < changeTolerance) {
                stableCount++;
            } else {
                stableCount = 0;
            }
            lastValue = value;

            if (stableCount >= stableThreshold) {
                positionCalculator.setAdcMaxLimit(value);
                currentState = State::CALIB_RIGHT;
                stableCount = 0;
            }
            break;
        }

        case State::CALIB_RIGHT:{
            motor.right(calib_duty);
            int value = feedback.readRaw();
            if (abs(value - lastValue) < changeTolerance) {
                stableCount++;
            } else {
                stableCount = 0;
            }
            lastValue = value;

            if (stableCount >= stableThreshold) {
                positionCalculator.setAdcMinLimit(value);
                currentState = nextState(currentState);
                stableCount = 0;
            }
            break;
        }

        case State::SYN:{
            float error = positionCalculator.computePosition(feedback.readRaw()) - otherDevice.getPistonPos();
            if (std::abs(error) < 1)
            {
                motor.stop();
            }
            else
            {
                float duty = pidPos.step(error, deltaMillis/1000.0f); 
                duty = std::abs(duty);
                if (duty > 100) duty = 100;
                else if (duty < 20) duty = 20;
                if (error < 0) {
                    motor.left(duty);
                } else {
                    motor.right(duty);
                }
            }
            currentState = nextState(currentState);
            break;
        }

        case State::MANUAL:{
            float duty = ui.readFilteredPotPercent();
            if (ui.getDirection() == Direction::LEFT) {
                if (positionCalculator.computePosition(feedback.readRaw()) > PISTON_LEN - 10)
                    duty = 20;
                motor.left(duty);
                ui.setLeftLed(true);
                ui.setRightLed(false);
            } else if (ui.getDirection() == Direction::RIGHT) {
                if (positionCalculator.computePosition(feedback.readRaw()) < 10)
                    duty = 20;
                motor.right(duty);
                ui.setRightLed(true);
                ui.setLeftLed(false);
            } else {
                motor.stop();
                ui.setLeftLed(true);
                ui.setRightLed(true);
            }
            currentState = nextState(currentState);
            break;
        }

        case State::RANDOM:{
            int currentPos = positionCalculator.computePosition(feedback.readRaw());

            if (targetPos == currentPos)
            {
                targetPos = random(5, 125);
            }
            
            float error =  currentPos - targetPos;
            float duty = pidPos.step(error, deltaMillis/1000.0f);
            duty = std::abs(duty);
            if (duty > ui.readFilteredPotPercent()) duty = ui.readFilteredPotPercent();
            else if (duty < 20) duty = 20;

            if (error < 0) {
                motor.left(duty);
            } else {
                motor.right(duty);
            }
            currentState = nextState(currentState);
            break;
        }

        case State::SEQUENCE:
          float duty = ui.readFilteredPotPercent();

          if (goingLeft) {
              if (positionCalculator.computePosition(feedback.readRaw()) > PISTON_LEN - 10)
                  duty = 20;
              motor.left(duty);
              if (positionCalculator.maxLimitReached(feedback.readRaw())) {
                  goingLeft = false;
              }
          } else {
              if (positionCalculator.computePosition(feedback.readRaw()) < 10)
                  duty = 20;
              motor.right(duty);
              if (positionCalculator.minLimitReached(feedback.readRaw())) {
                  goingLeft = true;
              }
          }
          currentState = nextState(currentState);
          break;
      }


  lastMillis = currentMillis;
}