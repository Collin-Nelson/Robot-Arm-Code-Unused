/**
 * The purpose of file/class is to manage and schedule all robot event, primarily movements.
 * All Events are stored in a queue, where the head of the queue contains the event that
 * is currently being performed. This class is designed to be used by the Controller class
 * 
 * @author Thomas Batchelder
 * @file EventQueue.h
 * @date 1/19/2021 - File created
 */

#include "Configuration.h"
#include "Stepper.h"
#include <Arduino.h>

/** All event codes */
#define MOVEMENT_EVENT 1
#define SLEEP_EVENT 2
#define HOMING_EVENT 3

/** All Error codes */
#define OUTSIDE_OF_MOTOR_BOUNDS 1
#define VELOCITY_TOO_HIGH 2

struct EventNode {
    /** This is the next node is the queue */
    EventNode* nextEvent;
    /** This is the code that is associated with this event */
    uint8_t eventCode;
    /** Variable to store the point in time when the event was created */
    uint32_t timeVariable;
    /** This is the degrees that each axis needs to travel to complete the event */
    double targetPosition[DOF];
    /** This contains the velocity, acceleration, initial velocity, and final velocity of the event */
    double kinematicInfo[4];
    /** Determines if the movement should encoder position or motor position */
    bool useEncoderPosition;
};

class EventQueue {
protected:
    /** This is the head of the queue */
    EventNode* head;
    /** This is the tail of the queue */
    EventNode* tail;
    /** This is the size of the queue */
    uint32_t queueSize;

    /** Motors of the robot */
    Stepper* motors;

    /** Variables used for straight line movements */
    double tap = 0, // Point in time when the movement stops accelerating
        lap = 0, // Variable used in movement calculations for constant velocity
        lcsp = 0, // Variable used in movement calculations for deceleration
        tcsp = 0, // Point in time when the movement start decelerating
        tfin = 0, // Point in time when the movement has finished
        scaler = 0, // scaler used when calculating the degree change for each motor
        velocity = 0, // peak velocity of the movement
        acceleration = 0, // acceleration/deceleration of the movement
        initVelocity = 0, // initial velocity of the movement
        finalVelocity = 0, // final velocity of the movement
        largestDegreeChange = 0; // largest degree change of all of the active axises
    /** Variables that will be used during time calculations and events */
    uint32_t eventStartTime = 0, timeDelta = 0;
    /** array of degrees used to calculate and keep track of the arms current position */
    double targetPosition[DOF], initialPosition[DOF];

    bool isRobotActive = false, // Used to determine if the controller is processing an event
        isRobotMoving = false; // Used to determine if the arm is physically moving

    bool printEventInfo = false; // Used to determine if information should be printed to Serial monitor

    /** This used to determine why a function may have failed */
    uint32_t errorCode = 0;

    /**
     * This function is used to add a new event to the end of the queue
     * @param newEvent is the event being added
     */
    void addEvent(EventNode* newEvent);

    /** This function is used to process a sleep event */
    void processSleepEvent();

    /** This function is used to process a movement event */
    void processMovementEvent();

    /** This function is used to process a homing event */
    void processHomingEvent();

    /** This function is used to perform all of the calculations for a movement event */
    void calculateMovementEvent();

    /**
     * This function is used to perform a trajectory
     * @param updatedTrajectory is the new Trajectory
     */
    void performTrajectory(double* updatedTrajectory);

public:
    /** This function is used to remove the current event and replace it with the next event in the queue */
    void eventCompleted();

    /** Used to construct a new Event Queue and initialize fields */
    EventQueue(Stepper* motors);

    /** Default Constructor */
    EventQueue() { }

    /** This function is used update queue */
    void update();

    /**
     * Used to determine if the arm is currently processing an event
     * @return is true if the arm is processing an event, otherwise false is returned
     */
    bool isArmActive();

    /**
     * Used to determine if the arm is currently moving
     * @return is true if the arm is moving, otherwise false is returned
     */
    bool isArmMoving();

    /**
     * Used to get the size of the queue
     * @return is the size of the event queue
     */
    uint32_t getQueueSize();

    /**
     * This function is used to get the latest error code
     * @return is the latest error code
     */
    uint32_t getErrorCodeAndReset();

    /**
     * This function is used to add a sleep event to the queue
     * @param sleepTime is the amount of time the event will be sleeping in microseconds
     */
    void addSleepEvent(uint32_t sleepTime);

    /**
     * This function is used to add a movement to the event queue. If an error occurs
     * the correct error code will be placed in to the error field
     * @param finalPosition is an array of angles that the motors will move
     * @param velocity is the velocity the motors will travel at
     * @param acceleration is the acceleration/deceleration of the movement
     * @param initVelocity is the initial velocity of the movement
     * @param finalVelocity is the final velocity of the movement0
     * @param useEncoderPosition is used to determine if the arm should use encoder position
     * @return is false if the movement cannot be added, otherwise true is returned
     */
    bool addMovementEvent(
        double* finalPosition,
        double velocity,
        double acceleration,
        double initVelocity,
        double finalVelocity,
        bool useEncoderPosition);

    /**
     * This function is used to add a movement to the event queue. If an error occurs
     * the correct error code will be placed in to the error field
     * @param data contains all of the information needed for a movement
     */
    bool addMovementEvent(double* data);

    /**
     * This function is used to add a homing event to the queue
     * @param velocity is the velocity of the homing event
     * @param acceleration is the acceleration of the homing event
     */
    void addHomingEvent(double velocity, double acceleration);
};
