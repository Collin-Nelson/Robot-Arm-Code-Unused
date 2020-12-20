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

    /** Current RPM of the motor */
    double RPM;
    /** Step Time for set RPM */
    long stepTimeRPM;
    /** Next Step Time */
    uint32_t processNextStepTime = micros();
    /** Current position of the motor */
    long currentPos;
    /** Current direction the axis is spinning */
    bool counterClockwise;
    /** Position that the motor is trying to go to */
    long targetPos;

    /** Used to tell the motor if it needs to use acceleration */
    bool useAcc;
    /** Used to tell the motor if it needs to use deceleration */
    bool useDec;
    /** step when the motor is to stop accelerating */
    long endAcc;
    /** step when the motor is to stop decelerating */
    long startDec;
    /** Change in acceleration per step */
    double accDelta;
    /** Change in deceleration per step */
    double decDelta;
    /** Acceleration value (0 - 1) */
    double acc;
    /** Deceleration value (0 - 1) */
    double dec;
    /** initial speed of the axis */
    double accRPM;
    /** final speed of the axis */
    double decRPM;
    /** The amount of steps the motor will spend accelerating */
    long accStepChange;
    /** The amount of steps the motor will spend decelerating */
    long decStepChange;

    double RPMDelta;

    long stepChange;

    long decAbsStep;

    long startStepTime;
    long finalStepTime;

    /***************************************************/
    /*         Time Based Acceleration Variables       */
    /***************************************************/

    /** Total Amount of time spent accelerating and decelerating */
    long totalAccelerationTime;
    /** Total Amount of steps processed during calculation */
    long totalAccelerationSteps;
    /** The start time of the movement */
    uint32_t movementStartTime;
    uint32_t timeHolder;



    /** amount of time between each step in microseconds */
    double stepTime;

    long stepCounter = 0;

    void printDouble(double val, int precision)
    {
        // prints val with number of decimal places determine by precision
        // precision is a number from 0 to 6 indicating the desired decimial places
        // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

        Serial.print(int(val)); //prints the int part
        if (precision > 0) {
            Serial.print("."); // print the decimal point
            unsigned long frac;
            unsigned long mult = 1;
            int padding = precision - 1;
            while (precision--)
                mult *= 10;

            if (val >= 0)
                frac = (val - int(val)) * mult;
            else
                frac = (int(val) - val) * mult;
            unsigned long frac1 = frac;
            while (frac1 /= 10)
                padding--;
            while (padding--)
                Serial.print("0");
            Serial.print(frac, DEC);
        }
    }

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
    Stepper(int stepPin, int dirPin, int enaPin, int limPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseHomeDir);

    /**
     * Default contstructor for Stepper
     */
    Stepper() { }

    /**
     * This function sets the RPM of the motor
     * @param newRPM is the new RPM of the motor
     * @param motorRPM is used to determine if the RPM be set if the for the motor or axis
     * @return is true if the RPM is set and false if it is not
     */
    bool setRPM(double newRPM, bool motorRPM);

    /** 
     * This function is used to home the motor
     * @param setting is used to determine which mode of homing is occuring
     * @return true if the motor is home and false if it is not
     */
    bool home(int config);

    /**
     * This function is used to update the motor and determine if the motor needs to move
     */
    void update();

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
     * used to set the target position of the motor and the direction the motor will turn
     * @param pos is the new position of the motor
     * @param useAcc is used to determine if acceleration is going to be used
     * @param useDec is used to determine if deceleration is going to be used
     * @return is the true if the position is set and false if it is not
     */
    uint32_t setTargetPosition(long pos, bool useAcc, bool useDec, bool simulate);

    /**
     * This function is used to set the acceleration values, but does no calculations
     * @param acceleration is the percentage of time that the motor will be accelerating
     * @param deceleration is the percentage of time that the motor will be deceleration
     * @param startRPM is the inital RPM of the axis
     * @param finalRPM is the final RPM of the axis
     * @return is true if everything is set correctly and false if the values are outside of their bonds
     */
    bool setAcceleration(double acceleration, double startRPM)
    {
        if (acceleration > 1 || (startRPM * this->gearReduction) > this->absoluteMaxRPM) {
            return false;
        } else {
            this->acc = acceleration;
            this->dec = acceleration;
            this->accRPM = startRPM;
            this->decRPM = startRPM;
            return true;
        }
    }

    /**
     * This function is used to send a pulse to the steppers step pin
     */
    bool pulseStepper()
    {
        if (digitalReadFast(this->limPin) && this->counterClockwise)
            return false;
        if (this->counterClockwise) {
            this->currentPos--;
        } else {
            this->currentPos++;
        }
        digitalWrite(this->stepPin, HIGH);
        delayMicroseconds(3);
        digitalWrite(this->stepPin, LOW);
        return true;
    }

    /**
     * This function is used to set the direction of the motor
     * @param counterClockwise is true if the motor is to spin counter clockwise
     */
    void setDirection(bool counterClockwise)
    {
        this->counterClockwise = counterClockwise;
        if ((counterClockwise && invertDir) || (!counterClockwise && !invertDir))
            digitalWrite(this->dirPin, LOW);
        else
            digitalWrite(this->dirPin, HIGH);
    }

    /**
     * This function is used to determine if the stepper is currently moving
     * @return is true if the stepper is moving and false if it is not
     */
    bool isActive()
    {
        return (this->currentPos != this->targetPos);
    }

    double sinAcceleration()
    {
        if (this->useAcc && ((currentPos < this->endAcc && !this->counterClockwise) || (currentPos > this->endAcc && this->counterClockwise))) {
            if (this->endAcc > this->startDec) {
                return (-this->accDelta * sin((PI / (2 * this->startDec) * this->stepCounter)) + this->startStepTime);
            } else {
                return (-this->accDelta * sin((PI / (2 * this->endAcc) * this->stepCounter)) + this->startStepTime);
            }
        } else if (this->useDec && ((currentPos > this->startDec && !this->counterClockwise) || (currentPos < this->startDec && this->counterClockwise))) {
            if (this->endAcc > this->startDec) {
                return (-this->decDelta * cos((PI / (2 * this->startDec) * (this->stepCounter - (this->decAbsStep)))) + this->finalStepTime);
            } else {
                return (-this->decDelta * cos((PI / (2 * this->endAcc) * (this->stepCounter - (this->decAbsStep)))) + this->finalStepTime);
            }
        } else {
            return this->stepTimeRPM;
        }
    }

    double sinRPMAcceleration()
    {
        double calculatedRPM = this->RPM / this->gearReduction;
        if (this->useAcc && ((currentPos < this->endAcc && !this->counterClockwise) || (currentPos > this->endAcc && this->counterClockwise))) {
            calculatedRPM = RPMDelta * sq(sin(PI / (2 * this->accStepChange) * this->stepCounter)) + accRPM;
        } else if (this->useDec && ((currentPos > this->startDec && !this->counterClockwise) || (currentPos < this->startDec && this->counterClockwise))) {
            calculatedRPM = RPMDelta * sq(cos(PI / (2 * this->decStepChange) * this->stepCounter)) + accRPM;
        } else {
            this->stepCounter = 0;
        }
        return (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (calculatedRPM * this->gearReduction * this->microstepping);
    }

    double sinSquaredAcceleration()
    {
        if (this->useAcc && ((currentPos < this->endAcc && !this->counterClockwise) || (currentPos > this->endAcc && this->counterClockwise))) {
            if (this->endAcc > this->startDec) {
                return (-this->accDelta * sq(sin((PI / (2 * this->startDec) * this->stepCounter))) + this->startStepTime);
            } else {
                return (-this->accDelta * sq(sin((PI / (2 * this->endAcc) * this->stepCounter))) + this->startStepTime);
            }
        } else if (this->useDec && ((currentPos > this->startDec && !this->counterClockwise) || (currentPos < this->startDec && this->counterClockwise))) {
            if (this->endAcc > this->startDec) {
                return (-this->decDelta * sq(cos((PI / (2 * this->startDec) * (this->stepCounter - (this->decAbsStep))))) + this->finalStepTime);
            } else {
                return (-this->decDelta * sq(cos((PI / (2 * this->endAcc) * (this->stepCounter - (this->decAbsStep))))) + this->finalStepTime);
            }
        } else {
            return this->stepTimeRPM;
        }
    }

    double timeBasedAccelerationRPM(uint32_t currentTime) {
        double calculatedRPM = this->RPM / this->gearReduction;
        if (this->useAcc && currentTime <= this->totalAccelerationTime + this->movementStartTime && abs(this->currentPos - this->targetPos) >= this->stepChange / 2) {
            this->totalAccelerationSteps++;
            calculatedRPM = RPMDelta * sq(sin(PI / (2 * this->totalAccelerationTime) * (currentTime - this->movementStartTime))) + accRPM;
        } else if (this->useDec && abs(this->currentPos - this->targetPos) <= this->totalAccelerationSteps) {
            calculatedRPM = RPMDelta * sq(cos(PI / (2 * this->totalAccelerationTime) * (currentTime - this->timeHolder))) + accRPM;
        } else {
            this->timeHolder = currentTime;
        }
        return (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (calculatedRPM * this->gearReduction * this->microstepping);
    }

    double timeBasedAccelerationSteps(uint32_t currentTime) {
        double calculatedRPM = this->stepTimeRPM;
        if (this->useAcc && currentTime <= this->totalAccelerationTime + this->movementStartTime && abs(this->currentPos - this->targetPos) >= this->stepChange / 2) {
            this->totalAccelerationSteps++;
            calculatedRPM = -this->accDelta * sq(sin(PI / (2 * this->totalAccelerationTime) * (currentTime - this->movementStartTime))) + this->startStepTime;
        } else if (this->useDec && abs(this->currentPos - this->targetPos) <= this->totalAccelerationSteps) {
            calculatedRPM = -this->accDelta * sq(cos(PI / (2 * this->totalAccelerationTime) * (currentTime - this->timeHolder))) + this->startStepTime;
        } else {
            this->timeHolder = currentTime;
        }
        return calculatedRPM;
    }
};