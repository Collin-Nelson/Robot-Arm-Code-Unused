/**
 * This file is responsible for homing and calibrating the arm
 * through the use of the controller object
 * @author Thomas Batchelder
 * @file calibration.cpp
 * @date 1/24/2021 - file created
 */

#include "../include/Calibration.h"

/**
 * This function is used to modify the calibrationMovement field to only move one axis
 * a certain amount of degrees.
 * @param controller is the controller of the arm
 * @param axis the axis being moved
 * @param targetPosition is the position the axis moving to 
 */
void singleAxisMovement(Controller* controller, uint8_t axis, double targetPosition)
{
    double calibrationMovement[DOF];
    for (int i = 0; i < DOF; i++) {
        calibrationMovement[i] = controller->getSteppers()[i].getCurrentPositionDegrees();
    }
    calibrationMovement[axis] = targetPosition;
    controller->traverseStraightLine(calibrationMovement, HOMING_VELOCITY * 5, HOMING_ACCELERATION * 4, 0, 0, false, true);
}

bool home(Controller* controller)
{
    bool axisHomed[DOF] = { false };
    int axisHomedCount = 0;
    Serial.println("  --- Homing Calibration: STARTING ---");
    Serial.println("  --> Starting First Home...");
    controller->getEventQueue()->addHomingEvent(HOMING_VELOCITY * 3 / 4, HOMING_ACCELERATION);
    while (controller->getEventQueue()->getQueueSize() != 0) {
        controller->update();
        for (int i = 0; i < DOF_ACTIVE; i++) {
            if (!axisHomed[i] && controller->getSteppers()[i].readLimitSwitch()) {
                Serial.print("     --> Axis [ ");
                Serial.print(i + 1);
                Serial.println(" ] Homed!");
                axisHomed[i] = true;
                controller->getSteppers()[i].setStatus(true);
                axisHomedCount++;
            }
        }
        if (axisHomedCount == DOF_ACTIVE) {
            while (controller->isActive()) {
                controller->getEventQueue()->eventCompleted();
            }
        }
    }
    if (axisHomedCount != DOF_ACTIVE) {
        Serial.println("  --> Unable to home: First Attemp");
        Serial.println("  --- Homing Calibration: FAILED ---");
        return false;
    }
    Serial.println("  --> First Home Completed!");
    for (int i = 0; i < DOF_ACTIVE; i++) {
        controller->getSteppers()[i].resetEncoderPosition();
        if (!controller->getSteppers()[i].setCurrentPosition(0)) {
            Serial.println("  --> Unable to home: Failed updating Motor Position");
            Serial.println("  --- Homing Calibration: FAILED ---");
            return false;
        }
        controller->getSteppers()[i].setStatus(false);
    }
    Serial.println("  --> Moving off of Limit Switches");
    double homingPosition[DOF] = { 10, 10, 10, 10, 10, 10 };
    controller->traverseStraightLine(homingPosition, HOMING_VELOCITY, HOMING_ACCELERATION, 0, 0, false, true);
    while (controller->getEventQueue()->getQueueSize() != 0) {
        controller->update();
    }
    Serial.println("  --> Starting Second Home...");
    bool axisHomed2[DOF] = { false };
    axisHomedCount = 0;
    controller->getEventQueue()->addHomingEvent(HOMING_VELOCITY / 5, HOMING_ACCELERATION / 5);
    while (controller->getEventQueue()->getQueueSize() != 0) {
        controller->update();
        for (int i = 0; i < DOF_ACTIVE; i++) {
            if (!axisHomed2[i] && controller->getSteppers()[i].readLimitSwitch()) {
                Serial.print("     --> Axis [ ");
                Serial.print(i + 1);
                Serial.println(" ] Homed!");
                axisHomed2[i] = true;
                controller->getSteppers()[i].setStatus(true);
                axisHomedCount++;
            }
        }
        if (axisHomedCount == DOF_ACTIVE) {
            while (controller->isActive()) {
                controller->getEventQueue()->eventCompleted();
            }
        }
    }
    if (axisHomedCount != DOF_ACTIVE) {
        Serial.println("  --> Unable to home: Second Attemp");
        Serial.println("  --- Homing Calibration: FAILED ---");
        return false;
    }
    double homingPosition3[DOF] = { 8, 8, 8, 8, 8, 8 };
    Serial.println("  --> Second Home Completed!");
    for (int i = 0; i < DOF_ACTIVE; i++) {
        controller->getSteppers()[i].resetEncoderPosition();
        controller->getSteppers()[i].setCurrentPosition(0);
        controller->getSteppers()[i].setStatus(false);
    }
    controller->traverseStraightLine(homingPosition3, HOMING_VELOCITY / 2, HOMING_ACCELERATION / 2, 0, 0, false, true);
    for (int i = 0; i < DOF_ACTIVE; i++) {
        controller->getSteppers()[i].resetEncoderPosition();
        controller->getSteppers()[i].setCurrentPosition(0);
        controller->getSteppers()[i].setStatus(false);
    }
    Serial.println("  --- Homing Calibration: COMPLETE ---");
    return true;
}

bool limitSwitchCalibration(Controller* controller)
{
    Serial.println("  --- Limit Switch Calibration: STARTING ---");
    Serial.println("  --> Reading Current State of each Switch");
    Serial.println("    --> Axis:  1 2 3 4 5 6");
    Serial.print("    --> State: ");
    for (int i = 0; i < DOF_ACTIVE; i++) {
        if (controller->getSteppers()[i].readLimitSwitch()) {
            singleAxisMovement(controller, i, 30.0);
            Serial.print("1 ");
        } else {
            Serial.print("0 ");
        }
    }
    Serial.println("\n  --> First Reading Completed");
    Serial.println("  --> Performing Secondary Reading");
    Serial.println("    --> Axis:  1 2 3 4 5 6");
    Serial.print("    --> State: ");
    for (int i = 0; i < DOF_ACTIVE; i++) {
        if (controller->getSteppers()[i].readLimitSwitch()) {
            Serial.println("\n\n  --- ERROR Encountered ---");
            Serial.print("  --> Axis [ ");
            Serial.print(i + 1);
            Serial.println(" ] failed Calibration");
            Serial.println("  --> Please Check Connections");
            Serial.println("  --- Limit Switch Calibration: FAILED ---");
            return false;
        } else {
            Serial.print("0 ");
        }
    }
    Serial.println("\n  --> All limit switches appear to be functional");
    Serial.println("  --- Limit Switch Calibration: COMPLETE ---");
    return true;
}

bool encoderCalibration(Controller* controller, uint8_t axis, int iterations)
{
    Serial.println("  --- Encoder Calibration: STARTING ---");
    double axisMovements[DOF] = { 270, 90, 180, 180, 90, 330 };
    double axisMovements2[DOF] = { 0 };
    for (int i = 0; i < DOF_ACTIVE; i++) {
        if (!controller->getSteppers()[i].isCrashDetectionEnabled()){
            Serial.println("  --> Axis Crash Detection is Disabled");
            continue;
        }
        if (axis == 0 || axis == i) {
            axisMovements2[i] = controller->getSteppers()[i].getDegreeChangePerStep() * controller->getSteppers()[i].getMicrosteping();
            Serial.print("\n  --> Calibrating Axis [ ");
            Serial.print(i + 1);
            Serial.println(" ]");
            int32_t averageDrift[iterations] = { 0 };
            for (int j = 0; j < iterations; j++) {
                Serial.print("Iteration [ ");
                Serial.print(j + 1);
                Serial.println(" ]:");
                int32_t delta[2] = { controller->getSteppers()[i].readEncoderPosition() };
                Serial.println(controller->getSteppers()[i].toString());
                singleAxisMovement(controller, i, axisMovements2[i]);
                Serial.println(controller->getSteppers()[i].toString());
                singleAxisMovement(controller, i, 0);
                delta[1] = controller->getSteppers()[i].readEncoderPosition();
                Serial.print("Calculated Drift: [ ");
                Serial.print(delta[1] - delta[0]);
                Serial.println(" ]\n");
                averageDrift[j] = delta[1] - delta[0];
            }
            double sum = 0;
            for (int j = 0; j < iterations; j++) {
                sum += averageDrift[j];
            }
            Serial.print("Calculated Average Drift: [ ");
            Serial.print(String(sum / iterations, 6));
            Serial.println(" ]\n");

            if (sum / iterations > 2.5) {
                Serial.println("ERROR: Axis needs Adjustment");
                return false;
            }

            Serial.println(controller->getSteppers()[i].toString());
            singleAxisMovement(controller, i, axisMovements[i]);
            Serial.println(controller->getSteppers()[i].toString());
            singleAxisMovement(controller, i, 0);
            Serial.println(controller->getSteppers()[i].toString());
            Serial.println("Axis Calibrated!");
        }
    }
    Serial.println("  --- Encoder Calibration: COMPLETE ---");
    return true;
}

bool performFullCalibration(Controller* controller)
{
    Serial.println("\n ===================================");
    Serial.println(" === Performing Full Calibration ===");
    Serial.println(" ===================================\n");
    if (!limitSwitchCalibration(controller)) {
        Serial.println("  --> Attemping Limit Switch Calibration Again");
        if (!limitSwitchCalibration(controller)) {
            Serial.println(" === Full Calibration FAILED ===");
            return false;
        }
    }
    Serial.println();
    if (!home(controller)) {
        Serial.println("  --> Attemping Homing Calibration Again");
        if (!home(controller)) {
            Serial.println(" === Full Calibration FAILED ===");
            return false;
        }
    }
    Serial.println();
    if (!encoderCalibration(controller, 0, 3)) {
        return false;
    }
    Serial.println();

    double homingPosition2[DOF] = { 190, 45, 140, 167, 75, 170 };
    controller->traverseStraightLine(homingPosition2, HOMING_VELOCITY * 10, HOMING_ACCELERATION * 7.5, 0, 0, false, true);
    Serial.println("\n ==================================");
    Serial.println(" === Completed Full Calibration ===");
    Serial.println(" ==================================\n");
    return true;
}