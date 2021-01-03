#include "../include/stepper.h"

Stepper::Stepper(int stepPin, int dirPin, int enaPin, int limPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseDir, double homingRPM)
{
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enaPin = enaPin;
    this->limPin = limPin;
    this->microstepping = microstepping;
    this->absoluteMaxRPM = absoluteMaxRPM;
    this->gearReduction = gearReduction;
    this->maxPos = maxPosition;
    this->invertDir = reverseDir;
    this->currentPos = 0;
    this->currentAngle = 0.0;
    this->homingRPM = homingRPM;
    this->homingStepTime = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (this->homingRPM * this->microstepping * this->gearReduction);
    this->processNextStepTime = micros();

    pinMode(this->stepPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    pinMode(this->enaPin, OUTPUT);
    pinMode(this->limPin, INPUT);

    this->setDirection(CLOCKWISE);
    this->degreeChangePerStep = DEGREES_PER_ROTATION / (this->microstepping * this->gearReduction);
}

bool Stepper::home(int config, double speed)
{
    if (digitalRead(this->limPin) && config != 1) {
        if (limitSwitchFilter(this->limPin, 100, 0.9)) {
            return true;
        }
    }
    uint32_t currentTime = micros();
    if (this->processNextStepTime <= currentTime) {
        pulseStepper();
        this->processNextStepTime = currentTime + (1 / speed) * this->homingStepTime;
    }
    return false;
}

bool Stepper::pulseStepper()
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

void Stepper::setDirection(bool counterClockwise)
{
    this->counterClockwise = counterClockwise;
    /*
    if (this->counterClockwise == COUNTERCLOCKWISE) {
        Serial.println("Direction Updated: COUNTERCLOCKWISE");
    } else {
        Serial.println("Direction Updated: CLOCKWISE");
    }
    */
    if ((counterClockwise && invertDir) || (!counterClockwise && !invertDir))
        digitalWrite(this->dirPin, LOW);
    else
        digitalWrite(this->dirPin, HIGH);
}