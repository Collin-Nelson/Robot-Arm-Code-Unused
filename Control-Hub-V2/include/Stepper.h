/**
 * This header file is associated with stepper.cpp. This file contains all of the
 * information nesseccary construct a stepper motor and all of it's functions. A stepper motor
 * object is responsible for keeping track of position and updating the stepper driver board as
 * needed. It should not handle and complicated processing such as acceleration and deceleration.
 * All heavy processing should be handled by the program using these classes. 
 * 
 * @author Thomas Batchelder
 * @date   1/19/2021 - created stepper.h
 */

#pragma once
#include "Configuration.h"
#include "Util.h"
#include <Arduino.h>
#include <Encoder.h>

/**
 * Main stepper motor class that contains all information to get the Arduino to move a stepper
 * using the controller. 
 */
class Stepper {
protected:
    /** Pin used to send step pulses to motor driver */
    uint8_t stepPin;
    /** Pin used to set the direction the motor will rotate */
    uint8_t dirPin;
    /** Pin used to determine if the motor has reached its limit */
    uint8_t limPin;

    /** The amount of microsteps required to complete one rotation */
    int32_t microsteping;
    /** The gear reduction associated with the motor (1 for no reduction) */
    double gearReduction;

    /** Maximum position that the motor can travel to in steps */
    int32_t maxMotorPosition;
    /** The current position of the motor in steps */
    int32_t currentPosition;
    /** The amount of degrees the axis changes with each step */
    double degreeChangePerStep;

    /** Used to reverse the motor direction for default */
    bool invertMotorDirection;
    /** Current direction the axis is spinning (true for counterclosewise)*/
    bool counterClockwise;

    /** Encoder used by the stepper motor to keep track of position */
    Encoder* encoder;
    /** The maximum allowable difference between motor position and encoder position */
    int32_t encoderThreshold;
    /** Used to determine if crash detection will be used with the motor */
    bool enableCrashDetection;

    /** This field can be used to disable the motor and won't allow for any movement */
    bool disable;

    /** This is the axis the motor is connected to */
    int axis;

public:
    /**
     * This method is used to initialize the motor.
     * @param stepPin           -Step pin for motor
     * @param dirPin            -Direction pin for motor
     * @param limPin            -Pin used for limit switch control
     * @param encoder           -A pointer to the encoder the motor will be using
     * @param microsteping      -Microstepping be used with the motor
     * @param gearReduction     -Gear reduction being used by the Motors (1 for no reduction)
     * @param maxPosition       -Maximum amount of the steps the motor can travel past home (-1 is limitless)
     * @param reverseDir        -Used to determine if the motor's direct needs to be reversed
     * @param encoderThresHold  -The maximum allowable difference between motor position and encoder position
     * @param axis              -The axis the motor is currently moving
     */
    Stepper(
        uint8_t stepPin,
        uint8_t dirPin,
        uint8_t limPin,
        Encoder* encoder,
        int32_t microsteping,
        double gearReduction,
        int32_t maxPosition,
        bool reverseDir,
        int32_t encoderThreshole,
        int axis,
        bool enableCrashDetection);

    /** Default contstructor for Stepper */
    Stepper() { }

    /**
     * This function is used to send a pulse to the steppers step pin. If the motor is at it's max
     * position or the motor is on the limit switch traveling counterclockwise then false is returned
     * @return is true if the motor able to successfully pulse the motor, otherwise false is returned
     * */
    bool pulse();

    /** This function is used to get information about the motor as String */
    String toString();

    /**
     * This function compares the position if the motor to the encoder to determine the motor has lost position
     * @return is true if the motor's position matches the encoder within thershold, otherwise false if returned
     */
    bool comparePositionToEncoder();

    /**
     * This function is used to read the current position of the encoder
     * @return is the current position of the encoder
     */
    int32_t readEncoderPosition();

    /**
     * This function is used to reset the encoder position to 0
     * @return is the position of the encoder before the reset
     */
    int32_t resetEncoderPosition();

    /**
     * This function is used to read the current position of the limit switch (true/false) 
     * This function does the limit switch filter to determine the state of the switch
     * @return is the current state of the limit switch
     */
    bool readLimitSwitch();

    /**
     * This function is used to get the current direction the motor is moving in
     * @return is true if the motor is traveling counter clockwise, other false is returned
     */
    bool getDirection();

    /**
     * This function is used to set the direction of the motor
     * @param counterClockwise is true if the motor is to spin counter clockwise
     */
    void setDirection(bool counterClockwise);

    /**
     * This function is used to get the current position of the motor in motor steps
     * @return is the current position of the motor in steps
     */
    int32_t getCurrentPositionSteps();

    /**
     * This function is used to get the current position of the motor in degrees
     * @return is the current position of the motor in degrees
     */
    double getCurrentPositionDegrees();
    
    /**
     * This function is used to get the maximum steps that the motor can travel
     * @return is the maximum number of steps the motor can travel
     */
    int32_t getMaximumPosition();

    /**
     * This method is used to set the current position of the motor in steps
     * @param position is the updated position of the motor
     * @return is true if the new position is set and false if it is not
     */
    bool setCurrentPosition(int32_t positionSteps);

    /** 
     * This function is used to get the degrees the axis changes by with each step
     * @param double is the angle the axis changes by with each step
     */
    double getDegreeChangePerStep();

    /**
     * This function gets the microstepping the motor is currently using
     * @return is the microstepping the motor is currently set to
     */
    int32_t getMicrosteping();

    /** 
     * This function can be used to set the current state of the motor
     * @param disable is true if the motor is be disabled, otherwise should be false
     */
    void setStatus(bool disable);

    /**
     * This function is used to determine if a motor is disabled
     * @return is true if the motor is disabled, otherwise false is returned
     */
    bool isDisabled();

    /**
     * This function is used to determine crash detection is enabled
     * @return is true if it is enable, otherwise false is returned
     */
    bool isCrashDetectionEnabled();
};