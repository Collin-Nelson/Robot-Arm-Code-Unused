/**
 * The purpose of file/class is to manage and schedule all robot event, primarily movements.
 * All Events are stored in a queue, where the head of the queue contains the event that
 * is currently being performed. This class is designed to be used by the Controller class
 * 
 * @author Thomas Batchelder
 * @file EventQueue.cpp
 * @date 1/19/2021 - File created
 */

#include "../include/EventQueue.h"

// +---------------------------------------------------+ //
// |  --- Event Construction and helper functions ---  | //
// +---------------------------------------------------+ //

EventQueue::EventQueue(Stepper* motors)
{
    this->head = NULL;
    this->tail = NULL;
    this->queueSize = 0;
    this->motors = motors;
}

bool EventQueue::isArmActive()
{
    return this->isRobotActive;
}

bool EventQueue::isArmMoving()
{
    return this->isRobotMoving;
}

uint32_t EventQueue::getQueueSize()
{
    return this->queueSize;
}
uint32_t EventQueue::getErrorCodeAndReset()
{
    uint32_t temp = this->errorCode;
    this->errorCode = 0;
    return temp;
}

// +---------------------------------------------------+ //
// |              --- Event Processing ---             | //
// +---------------------------------------------------+ //

void EventQueue::update()
{
    if (this->head == NULL)
        return;
    switch (this->head->eventCode) {
    case MOVEMENT_EVENT:
        processMovementEvent();
        break;
    case SLEEP_EVENT:
        processSleepEvent();
        break;
    case HOMING_EVENT:
        processHomingEvent();
        break;
    }
}

void EventQueue::eventCompleted()
{
    if (this->head != NULL) {
        EventNode* temp = this->head->nextEvent;
        free(this->head);
        this->head = temp;
        this->queueSize--;
        this->isRobotMoving = false;
        this->isRobotActive = false;
    }
}

void EventQueue::addEvent(EventNode* newEvent)
{
    if (this->head == NULL) {
        this->head = newEvent;
        this->tail = this->head;
    } else {
        this->tail->nextEvent = newEvent;
        this->tail = newEvent;
    }
    newEvent->nextEvent = NULL;
    this->queueSize++;
}

// +---------------------------------------------------+ //
// |               --- Movement Event ---              | //
// +---------------------------------------------------+ //

bool EventQueue::addMovementEvent(double* data)
{
    double finalPosition[DOF] = { data[0], data[1], data[2], data[3], data[4], data[5] };
    double velocity = data[6];
    double acceleration = data[7];
    double initVelocity = data[8];
    double finalVelocity = data[9];
    bool useEncoderPosition = (bool)((int)data[10]);
    return addMovementEvent(finalPosition, velocity, acceleration, initVelocity, finalVelocity, useEncoderPosition);
}

bool EventQueue::addMovementEvent(
    double* finalPosition,
    double velocity,
    double acceleration,
    double initVelocity,
    double finalVelocity,
    bool useEncoderPosition)
{
    // Error Checking
    for (int i = 0; i < DOF; i++) {
        if (this->motors[i].getMaximumPosition() != -1 && (this->motors[i].getMaximumPosition() * this->motors[i].getDegreeChangePerStep() < finalPosition[i] || finalPosition[i] < 0)) {
            this->errorCode = OUTSIDE_OF_MOTOR_BOUNDS;
            return false;
        } else if (velocity > MAX_VELOCITY) {
            this->errorCode = VELOCITY_TOO_HIGH;
            return false;
        }
    }
    if (this->printEventInfo) {
        Serial.println("  --- Straight Line Movement: STARTING ---");
        Serial.println("Axis Angles:       \tAxis 1\t Axis 2\t Axis 3\t Axis 4\t Axis 5\t Axis 6");
        Serial.print("Target Trajectory:\t");
        for (int i = 0; i < DOF; i++) {
            Serial.print(String(finalPosition[i], 2));
            Serial.print("\t ");
        }
        Serial.print("\nVelocity:         \t");
        Serial.println(String(velocity, 12));
        Serial.print("Acceleration:     \t");
        Serial.println(String(acceleration, 12));
        Serial.print("Initial Velocity: \t");
        Serial.println(String(initVelocity, 12));
        Serial.print("Final Velocity:   \t");
        Serial.println(String(finalVelocity, 12));
        Serial.print("Use Encoder Position:\t");
        Serial.println(useEncoderPosition);
        Serial.println();
    }
    // Creating a new EventNode
    EventNode* newEvent = (EventNode*)malloc(sizeof(EventNode));
    newEvent->eventCode = MOVEMENT_EVENT;
    newEvent->timeVariable = micros();
    newEvent->kinematicInfo[0] = velocity;
    newEvent->kinematicInfo[1] = acceleration;
    newEvent->kinematicInfo[2] = initVelocity;
    newEvent->kinematicInfo[3] = finalVelocity;
    newEvent->useEncoderPosition = useEncoderPosition;
    for (int i = 0; i < DOF; i++) {
        newEvent->targetPosition[i] = finalPosition[i];
    }
    addEvent(newEvent);
    return true;
}

void EventQueue::processMovementEvent()
{
    if (!this->isRobotActive) {
        this->isRobotActive = true;
        this->isRobotMoving = true;
        calculateMovementEvent();
    }

    uint32_t currentTime = micros();
    if (currentTime - this->eventStartTime <= this->tfin) {
        timeDelta = currentTime - this->eventStartTime;
        //acceleration phase
        if (timeDelta <= this->tap) {
            this->scaler = this->initVelocity * this->timeDelta + this->acceleration * this->timeDelta * this->timeDelta / 2.0;
        }
        //contant maximum speed phase
        if (timeDelta > this->tap && timeDelta <= this->tcsp) {
            this->scaler = this->lap + this->velocity * (timeDelta - this->tap);
        }
        //deceleration phase
        if (timeDelta > tcsp) {
            this->scaler = this->lcsp + this->velocity * (timeDelta - this->tcsp) - this->acceleration * (timeDelta - this->tcsp) * (timeDelta - this->tcsp) / 2.0;
        }
        //trajectory x and y as a function of scalar
        double updatedTrajectory[6];
        for (int i = 0; i < DOF; i++) {
            updatedTrajectory[i] = this->initialPosition[i] + (this->targetPosition[i] - this->initialPosition[i]) / this->largestDegreeChange * this->scaler;
        }
        performTrajectory(updatedTrajectory);
        for (int i = 0; i < DOF_ACTIVE; i++) {
            if (!this->motors[i].comparePositionToEncoder()) {
                Serial.println("Crash Detected! Recalculating Movement...");
                Serial.print("Axis: ");
                Serial.println(i + 1);
                Serial.print("Motor Pos:\t");
                Serial.println(this->motors[i].getCurrentPositionSteps());
                Serial.print("Encoder Pos:\t");
                Serial.println(this->motors[i].readEncoderPosition());
                if (this->head->kinematicInfo[0] / 2 > 0.1e-4)
                    this->head->kinematicInfo[0] = this->head->kinematicInfo[0] / 2;
                this->head->useEncoderPosition = true;
                delay(1000);
                calculateMovementEvent();
                break;
            }
        }
    } else {
        if (this->printEventInfo) {
            Serial.print("Final Trajectory:\t");
            for (int i = 0; i < DOF; i++) {
                Serial.print(String(this->motors[i].getCurrentPositionDegrees(), 2));
                Serial.print("\t ");
            }
            Serial.println("\n  --- Straight Line Movement: COMPLETE ---");
        }
        eventCompleted();
    }
}

void EventQueue::calculateMovementEvent()
{
    for (int i = 0; i < DOF; i++) {
        if (!this->head->useEncoderPosition) {
            this->initialPosition[i] = motors[i].getCurrentPositionDegrees();
        } else {
            this->motors[i].setCurrentPosition(this->motors[i].readEncoderPosition());
            this->initialPosition[i] = motors[i].getCurrentPositionDegrees();
        }
        this->targetPosition[i] = this->head->targetPosition[i];
    }
    this->velocity = this->head->kinematicInfo[0];
    this->acceleration = this->head->kinematicInfo[1];
    this->initVelocity = this->head->kinematicInfo[2];
    this->finalVelocity = this->head->kinematicInfo[3];
    if (printEventInfo) {
        Serial.print("Velocity:\t\t");
        Serial.println(String(this->velocity, 10));
        Serial.print("Acceleration:\t\t");
        Serial.println(String(this->acceleration, 10));
        Serial.print("Initial Velocity:\t");
        Serial.println(String(this->initVelocity, 10));
        Serial.print("Final Velocity:\t\t");
        Serial.println(String(this->finalVelocity, 10));
    }

    this->largestDegreeChange = 0;
    for (int i = 0; i < DOF; i++) {
        this->largestDegreeChange = max(this->largestDegreeChange, abs(this->targetPosition[i] - this->initialPosition[i]));
    }

    for (int i = 0; i < DOF; i++) {
        if (this->targetPosition[i] - initialPosition[i] < 0.0) {
            this->motors[i].setDirection(COUNTERCLOCKWISE);
        } else {
            this->motors[i].setDirection(CLOCKWISE);
        }
    }

    this->eventStartTime = micros();
    this->scaler = 0.0;
    this->velocity = min(this->velocity, sqrt(this->largestDegreeChange * this->acceleration + 0.5 * sq(this->initVelocity) + 0.5 * sq(this->finalVelocity)));

    this->tap = (this->velocity / this->acceleration) - (this->initVelocity / this->acceleration);
    this->lap = (this->initVelocity * this->tap) + (this->acceleration * sq(this->tap)) / 2.0;
    this->lcsp = this->largestDegreeChange - (sq(this->velocity) / 2.0 / this->acceleration - sq(this->finalVelocity) / 2.0 / this->acceleration);
    this->tcsp = (this->lcsp - this->lap) / this->velocity + this->tap;
    this->tfin = (this->velocity / this->acceleration) - (this->finalVelocity / this->acceleration) + this->tcsp;

    if (this->printEventInfo) {
        Serial.print("Initial Trajectory:\t");
        for (int i = 0; i < DOF; i++) {
            Serial.print(String(this->initialPosition[i], 2));
            Serial.print("\t ");
        }
        Serial.println();
        Serial.print("Total Movement Time:\t");
        Serial.println(this->tfin);
        Serial.print("Largest Degree Change:\t");
        Serial.println(this->largestDegreeChange);
    }
}

void EventQueue::performTrajectory(double* updatedTrajectory)
{
    for (int i = 0; i < DOF_ACTIVE; i++) {
        if (this->motors[i].getDirection() == COUNTERCLOCKWISE) {
            while (-updatedTrajectory[i] + this->motors[i].getCurrentPositionDegrees() > this->motors[i].getDegreeChangePerStep() && !this->motors[i].isDisabled()) {
                this->motors[i].pulse();
            }
        } else {
            while (updatedTrajectory[i] - this->motors[i].getCurrentPositionDegrees() > this->motors[i].getDegreeChangePerStep() && !this->motors[i].isDisabled()) {
                this->motors[i].pulse();
            }
        }
    }
}

// +---------------------------------------------------+ //
// |                --- Sleep Event ---                | //
// +---------------------------------------------------+ //

void EventQueue::addSleepEvent(uint32_t sleepTime)
{
    if (this->printEventInfo) {
        Serial.print("Sleep Event Added: ");
        Serial.print(sleepTime / SECONDS_TO_MICROSECONDS);
        Serial.println(" Seconds");
    }
    EventNode* newEvent = (EventNode*)malloc(sizeof(EventNode));
    newEvent->eventCode = SLEEP_EVENT;
    newEvent->timeVariable = sleepTime;
    addEvent(newEvent);
}

void EventQueue::processSleepEvent()
{
    if (!this->isRobotActive) {
        this->eventStartTime = micros();
        this->isRobotActive = true;
        if (printEventInfo) {
            Serial.print("Sleep Event Started: ");
            Serial.print(this->head->timeVariable / SECONDS_TO_MICROSECONDS);
            Serial.println(" Seconds!");
        }
    }
    if (this->eventStartTime + this->head->timeVariable < micros()) {
        if (printEventInfo) {
            Serial.print("Slept for ");
            Serial.print(this->head->timeVariable / SECONDS_TO_MICROSECONDS);
            Serial.println(" Seconds!");
        }
        eventCompleted();
    }
}

// +---------------------------------------------------+ //
// |                --- Homing Event ---               | //
// +---------------------------------------------------+ //

void EventQueue::addHomingEvent(double velocity, double acceleration)
{
    double homingMovement[DOF] = { -345.0, -200.0, -280.0, -280.0, -180.0, -360.0 };
    this->addMovementEvent(homingMovement, velocity, acceleration, 0, 0, false);
    this->tail->eventCode = HOMING_EVENT;
}

void EventQueue::processHomingEvent()
{
    if (!this->isRobotActive) {
        this->isRobotActive = true;
        this->isRobotMoving = true;
        calculateMovementEvent();
    }

    uint32_t currentTime = micros();
    if (currentTime - this->eventStartTime <= this->tfin) {
        timeDelta = currentTime - this->eventStartTime;
        //acceleration phase
        if (timeDelta <= this->tap) {
            this->scaler = this->initVelocity * this->timeDelta + this->acceleration * this->timeDelta * this->timeDelta / 2.0;
        }
        //contant maximum speed phase
        if (timeDelta > this->tap && timeDelta <= this->tcsp) {
            this->scaler = this->lap + this->velocity * (timeDelta - this->tap);
        }
        //deceleration phase
        if (timeDelta > tcsp) {
            this->scaler = this->lcsp + this->velocity * (timeDelta - this->tcsp) - this->acceleration * (timeDelta - this->tcsp) * (timeDelta - this->tcsp) / 2.0;
        }
        //trajectory x and y as a function of scalar
        double updatedTrajectory[6];
        for (int i = 0; i < DOF; i++) {
            updatedTrajectory[i] = this->initialPosition[i] + (this->targetPosition[i] - this->initialPosition[i]) / this->largestDegreeChange * this->scaler;
        }
        performTrajectory(updatedTrajectory);
    } else {
        eventCompleted();
    }
}