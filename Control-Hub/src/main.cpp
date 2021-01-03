#include "../include/configuration.h"
#include "../include/controller.h"
#include "../include/limitSwitch.h"
#include "../include/stepper.h"

#include <AccelStepper.h>
#include <Arduino.h>
#include <SPI.h>

Controller controller;
int pins[DOF] = LIMIT_SWITCH_PINS;
Stepper* axis = controller.getSteppers();
uint32_t timer;

double startPos[DOF] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double finalPos1[DOF] = { 165.0, 45.0, 0.0, 0.0, 0.0, 0.0 };
double finalPos2[DOF] = { 165.0, 135.0, 0.0, 0.0, 0.0, 0.0 };

// Axis 1 MAX: 327.5 Degrees

void setup()
{
    Serial.begin(BAUDRATE);
    delay(8000);
    Serial.println("\n--- Initialization STARTING ---");
    controller.home();
    Serial.println("--- Initialization COMPLETE ---\n");
    delay(1000);
    controller.traverseStraightLine(startPos, finalPos1, 0.15e-3, 0.1e-9, 0.0, 0.0);
}

void loop()
{
    controller.traverseStraightLine(finalPos1, finalPos2, 0.15e-3, 0.1e-9, 0.0, 0.0);
    controller.traverseStraightLine(finalPos2, finalPos1, 0.15e-3, 0.1e-9, 0.0, 0.0);
    delay(5000);
    //controller.update();
    //testLimitSwitchs(10000, 10, pins);
}