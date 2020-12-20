#include "../include/StepperSimulation.h"
#include <Arduino.h>

AccelStepper stepper;

void setup()
{
    Serial.begin(9600);
    delay(8000);

    stepper.setMaxSpeed(10000);
    stepper.setAcceleration(2000);
    stepper.moveTo(50000);

    uint32_t startTime = micros();
    Serial.println(stepper.simulate());
    Serial.print("Calculation Time: ");
    Serial.println(micros() - startTime);
}

void loop()
{
    //stepper.run();
}