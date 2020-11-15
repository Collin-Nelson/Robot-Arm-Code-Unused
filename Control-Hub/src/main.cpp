#include "../include/configuration.h"
#include "../include/controller.h"
#include "../include/stepper.h"

#include <AccelStepper.h>
#include <Arduino.h>
#include <SPI.h>

AXIS_1_INIT(axis1)
AXIS_2_INIT(axis2)
CONTROLLER_INIT(driver, axis1, axis2)

void setup()
{
    Serial.begin(BAUDRATE);
    //driver.homeAllAxis();
    axis2.enableAcceleration(false);
    axis2.setMaxRPM(1500);
    axis2.setAcceleration(10000);
    axis2.setTargetPosition(64000);
    axis1.setMaxRPM(15);
    axis1.setAcceleration(10000);
    axis1.setTargetPosition(64000);
    axis2.activate();
    //axis1.activate();
    Serial.println("Loaded information into Axis 2");
}

void loop()
{
    axis2.update();
    axis1.update();
}
