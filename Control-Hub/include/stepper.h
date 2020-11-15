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
#include "configuration.h"
#include <Arduino.h>

#define DISTANCE_TO_GO (this->targetPos - this->currentPos)

/**
 * Main stepper motor class that contains all information to get the Arduino to move a stepper
 * using the controller. 
 */
class Stepper {
protected:
    /** General size of the stepper motor (i.e. NEMA 17, NEMA 23, etc) */
    long size;
    /** Amperage that the stepper motor will be running on */
    double amperage;
    /** Pin used to send step pulses to motor driver */
    long stepPin;
    /** Pin used to set the direction the motor will rotate */
    long dirPin;
    /** Pin used to enable/disable the stepper motor */
    long enaPin;
    /** The amount of microsteps required to complete one rotation */
    long microstepping;
    /** The max RPM the motor alone can rotate at */
    double absoluteMaxRPM;
    /** The gear reduction associated with the motor (1 for no reduction) */
    double gearReduction;
    /** This pin is used to read in data from an end stop for homeing */
    long endStopPin;
    /** Current Direction */
    bool direction;
    /** Used to reverse the motor direction for default */
    bool reverseDir;

    /** The current RPM set for the motor */
    double RPM;
    /** Used in calculations for acceleration/deceleration */
    double stepsPerSecond;
    /** Accereration time frame (0 - 100)% */
    double acceleration;

    /** Bool used to determine if the motor is active or not */
    bool active;
    /** Used to enable/disble acceleration */
    bool enaAcceleration = true;
    /** Used to enable/disble deceleration */
    bool enaDeceleration = true;

    /** This is the maximum position that a motor is able to go to */
    long maxPos;
    /** Value used to calculate acceleration/deceleration */
    double cn;
    /** Value used to calculate initial acceleration/deceleration */
    double c0;
    /** Counter for keeping track of acceleration/deceleration */
    long n;
    /** This is min step time the motor will use to optain the set RPM */
    long cmin;
    /** This is the position that the motor is actively moving towards */
    double targetPos;
    /** This is the position is currently at */
    double currentPos;
    /** This is the current speed of the motor is steps per second */
    double speed;
    /** This is the max speed the motor will achieve is steps per second */
    double maxSpeed;
    /** This is the time between the current step and the next step in microseconds */
    uint32_t stepInterval;
    /** This is the point in time when the last step was performed */
    uint32_t lastStepTime;

public:
    /** Symbolic names for the direction the motor is turning */
    typedef enum {
        DIRECTION_CCW = 0, ///< Counter-Clockwise
        DIRECTION_CW = 1 ///< Clockwise
    } Direction;

    /**
     * This method is used to initialize the motor.
     * 
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
    Stepper(long size, double amperage, long stepPin, long dirPin, long enaPin, long endStopPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseHomeDir);

    /**
     * Default constructer
     */
    Stepper() { }

    void pulseMotor()
    {
        digitalWrite(this->stepPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(this->stepPin, LOW);
    }

    /**
     * This function is used to update the position of the stepper when needed and calculate
     * when the next step is.
     */
    void update();

    void setTargetPosition(long steps);

    void enableAcceleration(bool ena)
    {
        this->enaAcceleration = ena;
    }

    void enableDeceleration(bool ena)
    {
        this->enaDeceleration = ena;
    }

    /**
     * This method is used to determine if the motor is active
     * @return is true if the motor is moving and false if it is not
     */
    bool isActive()
    {
        return this->active;
    }

    /**
     * This function is used to start the motor once all data has been loaded into It
     */
    void activate()
    {
        this->active = true;
    }

    void setMaxRPM(double RPM);

    void calculateNewSpeed();

    void setAcceleration(double acceleration);

    boolean runSpeed();

    boolean run();
};