/**
 * This file is associated with stepper.h. This file contains all of the
 * information nesseccary construct a stepper motor and all of it's functions. A stepper motor
 * object is responsible for keeping track of position and updating the stepper driver board as
 * needed. It should not handle and complicated processing such as acceleration and deceleration.
 * All heavy processing should be handled by the program using these classes. 
 * 
 * @author Thomas Batchelder
 * @date   1/19/2021 - created stepper.cpp
 */

#include "../include/Stepper.h"

Stepper::Stepper(
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
    bool enableCrashDetection)
{
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->limPin = limPin;
    this->microsteping = microsteping;
    this->gearReduction = gearReduction;
    this->maxMotorPosition = maxPosition;
    this->invertMotorDirection = reverseDir;
    this->encoderThreshold = encoderThreshold;
    this->degreeChangePerStep = DEGREES_PER_ROTATION / (microsteping * gearReduction);
    this->encoder = encoder;
    this->disable = false;
    this->axis = axis;
    this->enableCrashDetection = enableCrashDetection;

    pinMode(this->stepPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    pinMode(this->limPin, INPUT);

    setCurrentPosition(0);
    setDirection(CLOCKWISE);
}

bool Stepper::pulse()
{
    if (digitalReadFast(this->limPin) && this->counterClockwise)
        if (limitSwitchFilter(this->limPin, 20, 0.75))
            return false;
    if (this->counterClockwise) {
        if (!setCurrentPosition(getCurrentPositionSteps() - 1))
            return false;
    } else if (!setCurrentPosition(getCurrentPositionSteps() + 1)) {
        return false;
    }
    if (!this->disable) {
        digitalWrite(this->stepPin, HIGH);
        delayMicroseconds(3);
        digitalWrite(this->stepPin, LOW);
    }
    return true;
}

bool Stepper::comparePositionToEncoder()
{
    if (enableCrashDetection) {
        long value = abs(readEncoderPosition() - (int32_t)this->currentPosition);
        if (value > 300) {
            Serial.print("Difference: ");
            Serial.print(value);
            Serial.print(", ");
            Serial.println(200);
            return false;
        }
    }
    return true;
}

int32_t Stepper::readEncoderPosition()
{
    return (int32_t)(this->encoder->read() * (this->microsteping / ENCODER_CPR));
}

int32_t Stepper::resetEncoderPosition()
{
    return (int32_t)(this->encoder->readAndReset() * (this->microsteping / ENCODER_CPR));
}

bool Stepper::readLimitSwitch()
{
    return limitSwitchFilter(this->limPin, 20, 0.6);
}

bool Stepper::getDirection()
{
    return this->counterClockwise;
}

void Stepper::setDirection(bool counterClockwise)
{
    this->counterClockwise = counterClockwise;
    if ((counterClockwise && this->invertMotorDirection) || (!counterClockwise && !this->invertMotorDirection))
        digitalWrite(this->dirPin, LOW);
    else
        digitalWrite(this->dirPin, HIGH);
}

int32_t Stepper::getCurrentPositionSteps()
{
    return this->currentPosition;
}

double Stepper::getCurrentPositionDegrees()
{
    return this->currentPosition * this->degreeChangePerStep;
}

bool Stepper::setCurrentPosition(int32_t positionSteps)
{
    if (this->maxMotorPosition != -1 && (positionSteps > this->maxMotorPosition))
        return false;
    this->currentPosition = positionSteps;
    return true;
}

int32_t Stepper::getMaximumPosition()
{
    return this->maxMotorPosition;
}

double Stepper::getDegreeChangePerStep()
{
    return this->degreeChangePerStep;
}

int32_t Stepper::getMicrosteping()
{
    return this->microsteping;
}

void Stepper::setStatus(bool disable)
{
    this->disable = disable;
}

bool Stepper::isDisabled()
{
    return this->disable;
}

bool Stepper::isCrashDetectionEnabled() {
    return this->enableCrashDetection;
}

String Stepper::toString()
{
    String motorString = "Motor Info: Pins[ ";
    motorString = String(motorString + String(this->stepPin));
    motorString = String(motorString + ", ");
    motorString = String(motorString + String(this->dirPin));
    motorString = String(motorString + ", ");
    motorString = String(motorString + String(this->limPin));
    motorString = String(motorString + " ]\tPosition[ ");
    motorString = String(motorString + String(this->currentPosition));
    motorString = String(motorString + " Steps, \t");
    motorString = String(motorString + String(getCurrentPositionDegrees(), 5));
    motorString = String(motorString + " Degrees ]\tEncoder[ ");
    motorString = String(motorString + String(readEncoderPosition()));
    motorString = String(motorString + " ]    \tDirection[ ");
    if (this->counterClockwise) {
        motorString = String(motorString + "CNTCLKWS ]");
    } else {
        motorString = String(motorString + "CLKWS ]");
    }
    return motorString;
}