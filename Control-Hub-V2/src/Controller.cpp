/**
 * This file is used to control of the robots functions and talk to all of the sensors
 * 
 * @author Thomas Batchelder
 * @file Controller.cpp
 * @date 1/19/2021 - File created
 */

#include "../include/Controller.h"

Controller::Controller()
{
    // Getting pins for controlling the stepper motors
    uint8_t stepPins[DOF] = STEP_PINS;
    uint8_t dirPins[DOF] = DIR_PINS;
    uint8_t limPins[DOF] = LIMIT_SWITCH_PINS;

    // Getting motor information
    int32_t mircosteping[DOF] = MICROSTEPING;
    int32_t encoderThreshold[DOF] = ENCODER_THRESHOLD;
    int32_t maxPosition[DOF] = MAX_POSITION;
    double gearReduction[DOF] = GEAR_REDUCTION;
    bool invertDir[DOF] = INVERT_DIR;
    bool crashDetection[DOF] = CRASH_DETECTION;

    this->encoders[0] = &motorEncoder1;
    this->encoders[1] = &motorEncoder2;
    this->encoders[2] = &motorEncoder3;
    this->encoders[3] = &motorEncoder4;
    this->encoders[4] = &motorEncoder5;
    this->encoders[5] = &motorEncoder6;

    for (size_t i = 0; i < DOF; i++) {
        this->motors[i] = Stepper(
            stepPins[i],
            dirPins[i],
            limPins[i],
            this->encoders[i],
            mircosteping[i],
            gearReduction[i],
            maxPosition[i],
            invertDir[i],
            encoderThreshold[i],
            i, 
            crashDetection[i]);
    }

    this->eventQueue = EventQueue(this->motors);
    this->communication = Communication(&this->eventQueue);
}

void Controller::update()
{
    eventQueue.update();
    communication.update();
}

void Controller::traverseStraightLine(
    double* targetPosition,
    double velocity,
    double acceleration,
    double initVelocity,
    double finalVelocity,
    bool useEncoderPosition,
    bool blocking)
{
    this->eventQueue.addMovementEvent(targetPosition, velocity, acceleration, initVelocity, finalVelocity, useEncoderPosition);
    if (blocking) {
        while (this->eventQueue.getQueueSize() > 0) {
            this->update();
        }
    }
}

void Controller::savePosition()
{
    double currentPosition[DOF];
    for (int i = 0; i < DOF; i++) {
        currentPosition[i] = this->motors[i].readEncoderPosition() * this->motors[i].getDegreeChangePerStep();
    }
    traverseStraightLine(currentPosition, HOMING_VELOCITY * 3, HOMING_ACCELERATION * 3, 0, 0, false, false);
}

Stepper* Controller::getSteppers()
{
    return this->motors;
}

Encoder* Controller::getEncoders()
{
    return this->encoders[0];
}

EventQueue* Controller::getEventQueue()
{
    return &this->eventQueue;
}

bool Controller::isActive()
{
    return (this->eventQueue.getQueueSize() != 0);
}
