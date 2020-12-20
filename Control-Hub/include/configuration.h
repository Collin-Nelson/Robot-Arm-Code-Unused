/**
 * This file contains all of the data used to configure the robot
 * 
 * @author Thomas Batchelder
 * @date    11/1/2020 - Created configuration file
 */

#pragma once
#include <Arduino.h>

#define BAUDRATE 250000 // Serial monitor baud rate

#define HOME_RPM_FAST 8 // RPM of motors while homing
#define HOME_RPM_SLOW 2 // RPM of motors while homing the second time

#define DEFAULT_START_SPEED 0.5 // default starting
#define DEFAULT_FINAL_SPEED 0.5 // default starting
#define DEFAULT_ACCELERATION 0.1 // Starting acceleration for motors
#define DEFAULT_DECELERATION 0.1 // Starting deceleration for motors

#define UNLIMITED_ROTATIONS -1 // Used to indicate that a motor is not limited

#define REVERSE false // Used to set the direction of the motor to reverse
#define FORWARD true // Used to set the direction of the motor to forward

// Constants
#define DOF 6 // Degrees of freedom in the arm
#define SECONDS_TO_MICROSECONDS 1000000.0f // Converstion from seconds to microseconds
#define DEGREES_PER_ROTATION 360.0f // Number of degrees in one rotation
#define START_HOME_MOVEMENT -1 // This is used to setermine if a stepper motor needs to home
#define MINUTES_TO_SECONDS 60.0 // Number of seconds in a minute

// Equations
#define RADIANS(d) ((d) * float(M_PI) / 180.0f) // Equation used to convert degrees to radians
#define DEGREES(r) ((r)*180.0f / float(M_PI)) //Equation used to convert radians to degrees
#define DEGREES_TO_STEPS(d, m) ((long)(((d) / DEGREES_PER_ROTATION) * m))

// Axis Pins:               1   2   3   4   5   6
#define STEP_PINS         { 3,  9,  1,  6, 24, 32}
#define DIR_PINS          { 5, 11,  0,  8, 26, 31}
#define ENA_PINS          { 7, 25,  2, 10, 28, 29}
#define LIMIT_SWITCH_PINS {38, 37, 36, 35, 34, 33}
#define MICROSTEPPING     {3200, 1600, 1600, 1600, 1600, 1600}
#define ABS_MAX_RPM       {5000, 5000, 5000, 5000, 5000, 5000}
#define GEAR_REDUCTION    {(10.0 * (50.0 / 15.0)), 50.0, 99.05, 1.0, 1.0, 1.0}
#define MAX_POSITION      {2 * 47000, -1, -1, -1, -1, -1}
#define HOMING_RPM        {9, 9, 9, 10, 10, 10}
#define INVERT_DIR        {1, 0, 0, 0, 0, 0}