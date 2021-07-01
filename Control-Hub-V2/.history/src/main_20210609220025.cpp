#include "../include/Calibration.h"
#include <Arduino.h>

Controller controller;
int limitSwitchPins[DOF] = LIMIT_SWITCH_PINS;
double movement[DOF] = { 15, 0, 0, 0, 0, 0 };

double pos1[DOF] = { 10, 10, 10, 10, 10, 10 };
double pos2[DOF] = { 190, 45, 140, 167, 75, 170 };

void setup()
{
    Serial.begin(BAUDRATE);
    delay(STARTUP_DELAY);
    Serial.println("\n--- Initialization STARTING ---");
    performFullCalibration();
    if (!home(&controller)) {
        while (true) { }
    }

    Serial.println("--- Initialization COMPLETE ---\n");
}

void loop()
{
    //delay(1000);
    /*
    if (Serial.available()) {
        String input = Serial.readString();
        if (input.compareTo("1\n") == 0) {
            Serial.println("Command Received! 1");
            controller.traverseStraightLine(pos1, HOMING_VELOCITY * 6, HOMING_ACCELERATION * 6, 0, 0, false, false);
            controller.traverseStraightLine(pos2, HOMING_VELOCITY * 6, HOMING_ACCELERATION * 6, 0, 0, false, false);
        } else if (input.compareTo("2\n") == 0) {
            Serial.println("Command Received! 2");
            while (controller.isActive()) {
                controller.update();
            }
            Serial.println("Command Received! 2 Finished");
        } else if (input.compareTo("3\n") == 0) {
            Serial.println("Command Received! 3");
            controller.savePosition();
        }
    }*/
    controller.update();
    //controller.traverseStraightLine(pos1, HOMING_VELOCITY * 10, HOMING_ACCELERATION*5, 0, 0, false, true);
    //controller.traverseStraightLine(pos2, HOMING_VELOCITY * 10, HOMING_ACCELERATION*5, 0, 0, false, true);

    //testLimitSwitchs(10000, 10, limitSwitchPins);
    //testEncoderPosition(controller.getEncoders());
}