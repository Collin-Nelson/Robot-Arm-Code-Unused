/**
 * This file is used to control of the robots functions and talk to all of the sensors
 * 
 * @author Thomas Batchelder
 * @file Controller.h
 * @date 1/19/2021 - File created
 */

#include "../include/Communication.h"
#include "../include/Configuration.h"
#include "../include/Stepper.h"
#include "../include/Util.h"
#include <Arduino.h>
#include <Encoder.h>

/**
 * This class is responsible for controlling everything in the robot
 */
class Controller {
protected:
    /** This is an array of all the steppers in the robot */
    Stepper motors[DOF];
    /** Array containing pointers to all of the encoders */
    Encoder* encoders[DOF];
    /** Event queue used to manage events */
    EventQueue eventQueue = EventQueue();
    /** Object used for communicating over Serial */
    Communication communication = Communication();
    /** Motor Encoder for axis 1 */
    Encoder motorEncoder1 = Encoder(ENCODER_1_PINS);
    /** Motor Encoder for axis 2 */
    Encoder motorEncoder2 = Encoder(ENCODER_2_PINS);
    /** Motor Encoder for axis 3 */
    Encoder motorEncoder3 = Encoder(ENCODER_3_PINS);
    /** Motor Encoder for axis 4 */
    Encoder motorEncoder4 = Encoder(ENCODER_4_PINS);
    /** Motor Encoder for axis 5 */
    Encoder motorEncoder5 = Encoder(ENCODER_5_PINS);
    /** Motor Encoder for axis 6 */
    Encoder motorEncoder6 = Encoder(ENCODER_6_PINS);

public:
    /** Function for constructing the Controller. Takes an array of steppers as the parameter */
    Controller();

    /**
     * This function designed to be run in constant loop in main.cpp.
     * It is used to update all the motors and function when it is needed.
     */
    void update();

    /**
     * This function is used to add a straight line movement to the eventQueue
     * @param targetPosition is target position for the arm
     * @param velocity is the max velocity of the movement
     * @param acceleration is the acceleration/deceleration of the movement
     * @param initVelocity is the starting velocity of the movement
     * @param useEncoderPosition is used to determine if the movement will use encoder positioning
     * or motor positioning
     * @param blocking is used to determine if the movement will be performed by blocking other
     * functions or running in the super loop
     */
    void traverseStraightLine(
        double* targetPosition,
        double velocity,
        double acceleration,
        double initVelocity,
        double finalVelocity,
        bool useEncoderPosition,
        bool blocking);

    /**
     * This function is used to save a position reading the encoder values
     */
    void savePosition();

    /**
     * This function is used to get the array of steppers to be used by other classes
     * @return is a pointer to an array of steppers
     */
    Stepper* getSteppers();

    /**
     * This function is used to get a pointer to all the encoders in the arm
     * @return is a pointer to all of the encoders
     */
    Encoder* getEncoders();

    /**
     * This function is used to get the eventQueue
     * @return is the event queue
     */
    EventQueue* getEventQueue();

    /** 
     * Used to determine if the robot is currently moving.
     * @return is true of the robot is moving, otherwise false is returned
     */
    bool isActive();
};