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
    /** This is the speed the motors will be traveling at (0-100%) */
    double speed;
    /** The starting speed (0-100%) of the motors */
    double initialSpeed;
    /** The final speed (0-100%) of the motors */
    double finalSpeed;
    /** The time (0-100%) that the motors will speed accelerating */
    double acceleration;
    /** The time (0-100%) that the motors will speed decelerating */
    double deceleration;

public:
    /**
     * Function for constructing the Controller. Takes an array of steppers as the parameter
     * 
     * @param axis the array of steppers
     * @param initialSpeed is the starting speed (0-100%) of the motors
     * @param finalSpeed is the final speed (0-100%) of the motors
     * @param acceleration is the time (0-100%) that the motors will speed accelerating
     * @param deceleration is the time (0-100%) that the motors will speed decelerating
     */
    Controller(Stepper axis[DOF], double initialSpeed, double finalSpeed, double acceleration, double deceleration);

    /**
     * This method is used to home all of of the axis
     */
    void homeAllAxis();

    void newMovement(double speed);

    /**
     * This function designed to be run in constant loop in main.ino.
     * It is used to update all the motors when it is needed.
     */
    void update(); 

    /**
     * This function is used to determine if any motors in the arm are currently moving
     * 
     * @return is true if they are moving and false if they are not
     */
    bool isActive()
    {
        for (int i = 0; i < DOF; i++) {
            if (this->axis[i].isActive())
                return true;
        }
        return false;
    }
};