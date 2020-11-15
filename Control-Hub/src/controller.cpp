#include "../include/controller.h"

Controller::Controller(Stepper axis[DOF], double initialSpeed, double finalSpeed, double acceleration, double deceleration)
{
    for (int i = 0; i < DOF; i++) {
        this->axis[i] = axis[i];
    }

    this->initialSpeed = initialSpeed;
    this->finalSpeed = finalSpeed;
    this->acceleration = acceleration;
    this->deceleration = deceleration;
}

void Controller::homeAllAxis()
{
}

void Controller::update()
{
    for (int i = 0; i < DOF; i++) {
        this->axis[i].update();
    }
}