#include "../include/stepper.h"

Stepper::Stepper(int stepPin, int dirPin, int enaPin, int limPin, long microstepping, double absoluteMaxRPM, double gearReduction, long maxPosition, bool reverseDir)
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

    pinMode(this->stepPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    pinMode(this->enaPin, OUTPUT);
    pinMode(this->limPin, INPUT);
}

bool Stepper::setRPM(double newRPM, bool motorRPM)
{
    if ((!motorRPM && newRPM * this->gearReduction >= this->absoluteMaxRPM) || (motorRPM && newRPM >= this->absoluteMaxRPM))
        return false;

    if (motorRPM) {
        this->RPM = newRPM;
    } else {
        this->RPM = newRPM * this->gearReduction;
    }
    this->stepTimeRPM = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (this->RPM * this->microstepping);
    return true;
}

bool Stepper::home(int config)
{
    if (digitalRead(this->limPin) && config != 1) {
        if (limitSwitchFilter(this->limPin, 50, 0.8)) {
            return true;
        }
    }
    uint32_t currentTime = micros();
    if (this->processNextStepTime <= currentTime) {
        pulseStepper();
        this->processNextStepTime = currentTime + this->stepTimeRPM;
    }
    return false;
}

void Stepper::update()
{
    if (this->targetPos != this->currentPos) {
        uint32_t currentTime = micros();
        if (this->processNextStepTime <= currentTime) {
            pulseStepper();
            this->stepCounter++;
            this->stepTime = timeBasedAccelerationRPM(currentTime);
            if (stepCounter % 10 == 0) {
                //Serial.println(stepTime);
            }
            processNextStepTime = currentTime + this->stepTime;
        }
    }
}

uint32_t Stepper::setTargetPosition(long pos, bool useAcc, bool useDec, bool simulate)
{
    if (pos > this->maxPos && this->maxPos != -1) {
        this->targetPos = this->currentPos;
        return -1;
    } else {
        this->useAcc = useAcc;
        this->useDec = useDec;
        this->targetPos = pos;
        this->stepCounter = 0;
        setDirection(this->currentPos > this->targetPos);
        if (this->useAcc || this->useDec) {
            this->stepChange = this->targetPos - this->currentPos; // Calculate the total amount of steps the motor will be moving
            this->endAcc = (stepChange * this->acc) + this->currentPos; // Calculate the step that the motor will step accelerating
            this->startDec = this->targetPos - (stepChange * this->dec); // Calculate the step that the motor will step decelerating
            this->startStepTime = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (this->accRPM * this->gearReduction * this->microstepping);
            this->finalStepTime = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (this->decRPM * this->gearReduction * this->microstepping);
            this->accDelta = abs(this->stepTimeRPM - this->startStepTime);
            this->decDelta = abs(this->stepTimeRPM - this->finalStepTime);
            this->decAbsStep = abs(this->startDec - stepChange);
            this->accStepChange = abs(stepChange * this->acc);
            this->decStepChange = abs(stepChange * this->dec);
            this->RPMDelta = (this->RPM / this->gearReduction) - this->accRPM;
            this->totalAccelerationTime = this->acc * (this->stepTimeRPM * abs(stepChange));
            this->totalAccelerationSteps = 0;
        }
        if (simulate) {

            long stepChange = abs(this->targetPos - this->currentPos);
            long accTotaltime = (this->accStepChange * (2 * this->accRPM + this->RPMDelta)) / 2;
            accTotaltime = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (accTotaltime * this->gearReduction * this->microstepping);
            long TopSpeedTotalTime = this->stepTimeRPM * (stepChange * (1 - (this->acc + this->dec)));
            long decTotaltime = (this->accStepChange * (2 * this->accRPM + this->RPMDelta)) / 2;
            accTotaltime = (SECONDS_TO_MICROSECONDS * MINUTES_TO_SECONDS) / (decTotaltime * this->gearReduction * this->microstepping);

            Serial.println("--- Simulating: STARTED ---");
            Serial.println("  --> Speed Calculations:");
            Serial.print("    --> Initial Step Time:       ");
            Serial.println(this->startStepTime);
            Serial.print("    --> Final Step Time:         ");
            Serial.println(this->finalStepTime);
            Serial.print("    --> Top Speed Step Time:     ");
            Serial.println(this->stepTimeRPM);
            Serial.print("    --> Acceleration Delta:      ");
            Serial.println(this->accStepChange);
            Serial.print("    --> Deceleration Delta:      ");
            Serial.println(this->decStepChange);
            Serial.print("    --> total Acceleration Time: ");
            Serial.println(this->totalAccelerationTime);

            Serial.println();

            Serial.println("  --> Step Calculations:");
            Serial.print("    --> Starting Step:           ");
            Serial.println(this->currentPos);
            Serial.print("    --> Target Step:             ");
            Serial.println(this->targetPos);
            Serial.print("    --> End Acceleration Step:   ");
            Serial.println(this->endAcc);
            Serial.print("    --> Start Deceleration Step: ");
            Serial.println(this->startDec);
            Serial.print("    --> Abs Deceleration Step:   ");
            Serial.println(this->decAbsStep);

            Serial.println();

            Serial.println("  --> Time Calculations:");
            Serial.print("    --> Total Acceleration Time: ");
            Serial.println(totalAccelerationTime);
            Serial.print("    --> Top Speed Time:          ");
            Serial.println(TopSpeedTotalTime);
            Serial.print("    --> Total Deceleration Time: ");
            Serial.println(totalAccelerationTime);
            Serial.print("    --> Total Time:              ");
            Serial.println(totalAccelerationTime + totalAccelerationTime);

            Serial.println();

            Serial.println("--- Simulating: COMPLETE ---");

            this->movementStartTime = micros();
            return totalAccelerationTime + TopSpeedTotalTime + totalAccelerationTime;
        }
        this->movementStartTime = micros();
        return 0;
    }
}