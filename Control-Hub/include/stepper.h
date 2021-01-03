/**
 * This header file associated with stepper.cpp and stepperTMC.cpp. This file contains all of the
 * information nesseccary construct a stepper motor and all of it's functions. A stepper motor
 * object is responsible for keeping track of position and updating the stepper driver board as
 * needed. It should not handle and complicated processing other than acceleration and deceleration
 * All heavy processing should be handled by the program using these classes. 
 * 
 * @author Thomas Batchelder
 * @date    11/1/2020 - created stepper.h
 */

#pragma once
#include "../lib/arduino-Tools40-master/src/Filter.h"
#include "configuration.h"
#include "limitSwitch.h"
#include <Arduino.h>

/**
 * Main stepper motor class that contains all information to get the Arduino to move a stepper
 * using the controller. 
 */
class Stepper {
protected:
    /** Pin used to send step pulses to motor driver */
    int stepPin;
    /** Pin used to set the direction the motor will rotate */
    int dirPin;
    /** Pin used to enable/disable the stepper motor */
    int enaPin;
    /** Pin used to determine if the motor has reached its limit */
    int limPin;
    /** The amount of microsteps required to complete one rotation */
    long microstepping;
    /** The max RPM the motor alone can rotate at */
    double absoluteMaxRPM;
    /** The gear reduction associated with the motor (1 for no reduction) */
    double gearReduction;
    /** Maximum position that the motor can travel to */
    long maxPos;
    /** Used to reverse the motor direction for default */
    bool invertDir;
    /** The current position of the motor in steps */
    long currentPos;
    /** The current angle of the axis in degrees */
    double currentAngle;
    /** Current direction the axis is spinning (true for counterclosewise)*/
    bool counterClockwise;
    /** The amount of degrees the axis changes with each step */
    double degreeChangePerStep;

    /** The homing RPM of the motor */
    double homingRPM;
    /** Homing RPM of the motor in steptime (us) */
    long homingStepTime;
    /** Point in time when to send to the motor a pulse in microseconds */
    uint32_t processNextStepTime;

public:
    /**
     * This method is used to initialize the motor.
     * @param size              -Size of motor
     * @param amperage          -Amperage of motor
     * @param stepPin           -Step pin for motor
     * @param dirPin            -Direction pin for motor
     * @param enaPin            -Enable/disable pin for motor
     * @param microstepping     -Microstepping be used with the motor
     * @param absoluteMaxRPM    -The maximum RPM at which the motor can rotate
     * @param gearReduction     -Gear reduction being used by the Motors (1 for no reduction)
     * @param maxPosition       -Maximum amount of the steps the motor can travel past home (-1 is limitless)
     */
    Stepper(int stepPin, int dirPin, int enaPin, int limPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseHomeDir, double homingRPM);

    /**
     * Default contstructor for Stepper
     */
    Stepper() { }

    bool home(int config, double speed);

    /**
     * This function is used to send a pulse to the steppers step pin
     */
    bool pulseStepper();

    /**
     * This function is used to set the direction of the motor
     * @param counterClockwise is true if the motor is to spin counter clockwise
     */
    void setDirection(bool counterClockwise);

    /**
     * This function is used to get the direction of the motor
     * @return counterClockwise is true if the motor is to spin counter clockwise
     */
    bool getDirection() {
        return this->counterClockwise;
    }

    /**
     * This function is used to get the current position of the motor
     * @return is the current position of the motor
     */
    long getCurrentPosition()
    {
        return this->currentPos;
    }

    /**
     * This method is used to set the current position of the motor
     */
    void setCurrentPosition(long position)
    {
        this->currentPos = position;
    }

    /**
     * This function is used to get the current angle of the axis
     * @return is the current angle of the axis
     */
    double getCurrentAngle()
    {
        return this->currentAngle;
    }

    /**
     * This method is used to set the current angle of the axis
     */
    void setCurrentAngle(double angle)
    {
        this->currentAngle = angle;
    }

    double getDegreeChangePerStep()
    {
        return this->degreeChangePerStep;
    }
};