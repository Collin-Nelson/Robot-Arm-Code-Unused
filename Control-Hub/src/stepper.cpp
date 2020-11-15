#include "../include/stepper.h"

Stepper::Stepper(long size, double amperage, long stepPin, long dirPin, long enaPin, long endStopPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseDir)
{
    this->size = size;
    this->amperage = amperage;
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enaPin = enaPin;
    this->endStopPin = endStopPin;
    this->microstepping = microstepping;
    this->absoluteMaxRPM = absoluteMaxRPM;
    this->gearReduction = gearReduction;
    this->maxPos = maxPosition;
    this->reverseDir = reverseDir;

    pinMode(this->stepPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    pinMode(this->enaPin, OUTPUT);
    pinMode(this->endStopPin, INPUT_PULLUP);
}

void Stepper::update()
{
    if (this->active) {
        if (this->targetPos == START_HOME_MOVEMENT) {
        } else {
            run();
        }
    }
}

void Stepper::calculateNewSpeed()
{
    long distanceTo = DISTANCE_TO_GO; // +ve is clockwise from curent location

    long stepsToStop = (long)((this->speed * this->speed) / (2.0 * this->acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1) {
        // We are at the target and its time to stop
        this->stepInterval = 0;
        this->speed = 0.0;
        this->n = 0;
        this->active = false;
        return;
    }

    if (distanceTo > 0) {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (this->n > 0) {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || this->direction == Stepper::DIRECTION_CCW && this->enaDeceleration)
                this->n = -stepsToStop; // Start deceleration
        } else if (this->n < 0) {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && this->direction == Stepper::DIRECTION_CW && this->enaAcceleration);
                this->n = -this->n; // Start accceleration
        }
    } else if (distanceTo < 0) {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (this->n > 0) {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || this->direction == Stepper::DIRECTION_CW && this->enaDeceleration)
                this->n = -stepsToStop; // Start deceleration
        } else if (this->n < 0) {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && this->direction == Stepper::DIRECTION_CCW && this->enaAcceleration)
                this->n = -this->n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (this->n == 0) {
        // First step from stopped
        this->cn = this->c0;
        this->direction = (distanceTo > 0) ? Stepper::DIRECTION_CW : Stepper::DIRECTION_CCW;
    } else {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        this->cn = this->cn - ((2.0 * this->cn) / ((4.0 * this->n) + 1)); // Equation 13
        this->cn = max(this->cn, this->cmin);
    }
    this->n++;
    this->stepInterval = this->cn;
    this->speed = SECONDS_TO_MICROSECONDS / this->cn;
    if (this->direction == DIRECTION_CCW)
        this->speed = -this->speed;
}

void Stepper::setAcceleration(double acceleration)
{
    if (acceleration == 0.0)
        return;
    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (this->acceleration != acceleration) {
        // Recompute this->n per Equation 17
        this->n = this->n * (this->acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        this->c0 = 0.676 * sqrt(2.0 / acceleration) * SECONDS_TO_MICROSECONDS; // Equation 15
        this->acceleration = acceleration;
        calculateNewSpeed();
    }
}

void Stepper::setMaxRPM(double RPM)
{
    this->RPM = RPM;
    double speed = (RPM * this->microstepping * this->gearReduction) / MINUTES_TO_SECONDS;
    if (speed < 0.0)
        speed = -speed;
    if (this->maxSpeed != speed) {
        this->maxSpeed = speed;
        this->cmin = SECONDS_TO_MICROSECONDS / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (this->n > 0) {
            this->n = (long)((this->speed * this->speed) / (2.0 * this->acceleration)); // Equation 16
            calculateNewSpeed();
        }
    }
}

void Stepper::setTargetPosition(long steps)
{
    this->targetPos = steps;
    if (DISTANCE_TO_GO < 0) {
        digitalWrite(this->dirPin, HIGH);
    } else {
        digitalWrite(this->dirPin, LOW);
    }

    calculateNewSpeed();
}

boolean Stepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!this->stepInterval)
        return false;

    unsigned long time = micros();
    if (time - lastStepTime >= this->stepInterval) {
        if (this->direction == Stepper::DIRECTION_CW) {
            // Clockwise
            this->currentPos += 1;
        } else {
            // Anticlockwise
            this->currentPos -= 1;
        }
        pulseMotor();

        this->lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    } else {
        return false;
    }
}

boolean Stepper::run()
{
    if (runSpeed())
        calculateNewSpeed();
    return this->speed != 0.0 || DISTANCE_TO_GO != 0;
}
