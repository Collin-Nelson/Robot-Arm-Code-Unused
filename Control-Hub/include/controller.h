#include "configuration.h"
#include "stepper.h"
#include <Arduino.h>

/**
 * This class is responsible for controlling everything in the robot
 */
class Controller {
protected:
    /** This is an array of all the steppers in the robot */
    Stepper axis[DOF];
    /** Limit switch pins */
    long limitSwitchPins[DOF] = LIMIT_SWITCH_PINS;
    /** Homing RPM for each axis */
    double homingRPM[DOF] = HOMING_RPM;
public:
    /**
     * Function for constructing the Controller. Takes an array of steppers as the parameter
     */
    Controller();

    /**
     * This method is used to home all of of the axis
     */
    void home();

    /**
     * This function designed to be run in constant loop in main.ino.
     * It is used to update all the motors when it is needed.
     */
    void update();

    Stepper* getSteppers() {
        return &this->axis[0];
    }

    bool isActive();
};