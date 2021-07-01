/**
 * This file contains all of the data used to configure the robot
 * 
 * @author Thomas Batchelder
 * @date    1/19/2021 - Created configuration file
 */

#pragma once
#include <Arduino.h>

#define BAUDRATE 250000 // Serial monitor baud rate
#define STARTUP_DELAY 7500 // Delay at beginning of program to let Serial monitor initialize

#define UNLIMITED_ROTATIONS -1 // Used to indicate that a motor is not limited
#define CLOCKWISE false // Used to set the direction of the motor to reverse
#define COUNTERCLOCKWISE true // Used to set the direction of the motor to forward
#define MAX_VELOCITY 1e-2 // Maximum velocity the arm can travel at

#define MAX_STEPPER_ENCODER_DIFFERENCE 500 // number of steps that the encoder and stepper can differ
#define ENCODER_CPR 4000.0 // Number of counts per revolution of the encoder

#define DOF 6 // Degrees of freedom in the arm
#define DOF_ACTIVE 6 // Degrees of freedom in the arm that are active

#define MINUTES_TO_SECONDS 60.0 // Number of seconds in a minute
#define SECONDS_TO_MICROSECONDS 1000000.0f // Converstion from seconds to microseconds
#define DEGREES_PER_ROTATION 360.0 // Number of degrees in one rotation

// Homing configuration
#define HOMING_VELOCITY 0.04e-3
#define HOMING_ACCELERATION 0.03e-9

// Axis Pins:               1   2   3   4   5   6
#define STEP_PINS         { 3,  9,  1,  6, 24, 32}
#define DIR_PINS          { 5, 11,  0,  8, 26, 31}
#define LIMIT_SWITCH_PINS {38, 37, 36, 35, 34, 33}

#define ENCODER_THRESHOLD {100, 100, 100, 100, 100, 100}
#define MICROSTEPING      {1000, 1000, 1000, 1600, 1000, 1000}
#define GEAR_REDUCTION    {40.0, 50.0, 50.0, 14.0 * (28.0 / 10), 10.0, 19.0}
#define MAX_POSITION      {-1, -1, -1, -1, -1, -1}
#define INVERT_DIR        {1, 1, 1, 1, 1, 1}
#define CRASH_DETECTION   {1, 1, 1, 1, 1, 1}

#define ENCODER_1_PINS    39, 7
#define ENCODER_2_PINS    40, 25
#define ENCODER_3_PINS     2, 41
#define ENCODER_4_PINS    21, 10
#define ENCODER_5_PINS    23, 28
#define ENCODER_6_PINS    22, 29