#include "../include/controller.h"

Controller::Controller()
{
    // Getting pins for controlling the stepper motors
    int stepPins[DOF] = STEP_PINS;
    int dirPins[DOF] = DIR_PINS;
    int enaPins[DOF] = ENA_PINS;
    int limPins[DOF] = LIMIT_SWITCH_PINS;

    // Getting motor information
    long mircostepping[DOF] = MICROSTEPPING;
    long absMaxRPM[DOF] = ABS_MAX_RPM;
    double gearReduction[DOF] = GEAR_REDUCTION;
    long maxPosition[DOF] = MAX_POSITION;
    bool invertDir[DOF] = INVERT_DIR;

    for (size_t i = 0; i < DOF; i++) {
        this->axis[i] = Stepper(stepPins[i], dirPins[i], enaPins[i], limPins[i], mircostepping[i], absMaxRPM[i], gearReduction[i], maxPosition[i], invertDir[i]);
    }
}

void Controller::home()
{
    Serial.println("  --> Homing All Axis: STARTED");
    for (int i = 0; i < DOF; i++) {
        this->axis[i].setRPM(this->homingRPM[i], false);
        this->axis[i].setDirection(true);
    }
    Serial.println("    --> Homing information written to steppers");
    Serial.println("    --> STARTING HOME");
    bool isHome = false;
    while (!isHome) {
        bool allAxisHome = true;
        for (int i = 0; i < 3; i++) {
            allAxisHome = this->axis[i].home(0) && allAxisHome;
        }
        isHome = allAxisHome;
    }

    for (int i = 0; i < DOF; i++) {
        this->axis[i].setRPM(this->homingRPM[i] * 0.5, false);
        this->axis[i].setDirection(false);
    }

    uint32_t stopTime = micros() + SECONDS_TO_MICROSECONDS / 2;
    while (stopTime > micros()) {
        for (int i = 0; i < 3; i++) {
            this->axis[i].home(1);
        }
    }

    for (int i = 0; i < DOF; i++) {
        this->axis[i].setRPM(this->homingRPM[i] * 0.1, false);
        this->axis[i].setDirection(true);
    }
    isHome = false;
    while (!isHome) {
        bool allAxisHome = true;
        for (int i = 0; i < 3; i++) {
            allAxisHome = this->axis[i].home(0) && allAxisHome;
        }
        isHome = allAxisHome;
    }

    for (int i = 0; i < 500; i++) {
        for (int i = 0; i < DOF; i++) {
            this->axis[i].setDirection(false);
            this->axis[i].pulseStepper();
            this->axis[i].setCurrentPosition(0);
            this->axis[i].setTargetPosition(0, false, false, false);
            delayMicroseconds(750);
        }
    }
    Serial.println("  --> Homing All Axis: COMPLETE");
}

void Controller::update()
{
    for (int i = 0; i < DOF; i++) {
        this->axis[i].update();
    }
}

bool Controller::isActive() {
    for (int i = 0; i < DOF; i++) {
        if (this->axis[i].isActive())
            return true;
    }
    return false;
}