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

void setup()
{
    Serial.begin(BAUDRATE);
    delay(8000);
    Serial.println("--- Initialization STARTING ---");
    controller.home();
    Serial.println("--- Initialization COMPLETE ---");

    axis[0].setRPM(20, false);
    axis[0].setAcceleration(0.95, 0.1);
    axis[0].setTargetPosition(90000, true, true, true);
    timer = micros();
    /*

    axis[1].setRPM(5, false);
    axis[1].setAccelerationDeceleration(0.2, 0.2, 1, 1);
    axis[1].setTargetPosition(15000, true, true, false);

    axis[2].setRPM(10, false);
    axis[2].setAccelerationDeceleration(0.2, 0.2, 1, 1);
    axis[2].setTargetPosition(85000, true, true, false);

    uint32_t startTime = micros();
    double value = sqrt(sin(0.25) * sin(0.45));
    Serial.print("Calculation Time: ");
    Serial.println(micros() - startTime);

    axis[1].setRPM(5, false);
    axis[1].setTargetPosition(9000, false, false); // 29700 for 0 Degrees
    axis[2].setRPM(5, false);
    axis[2].setTargetPosition(30000, false, false);
    */
}

void loop()
{
    controller.update();
    //testLimitSwitchs(10000, 10, pins);
    if (!controller.isActive()) {
        Serial.print("Total Movement Time: ");
        Serial.println(micros() - timer);
        timer = micros();
        delay(50);
        if (axis[0].getCurrentPosition() != 0) {
            axis[0].setTargetPosition(0, true, true, true);
        } else {
            axis[0].setTargetPosition(90000, true, true, true);
        }
    }
}