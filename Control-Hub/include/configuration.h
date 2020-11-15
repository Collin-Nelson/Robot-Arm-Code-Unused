/**
 * This file contains all of the data used to configure the robot
 * 
 * @author Thomas Batchelder
 * @date    11/1/2020 - Created configuration file
 */

#include "stepper.h"

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
#define DOF 2 // Degrees of freedom in the arm
#define SECONDS_TO_MICROSECONDS 1000000.0f // Converstion from seconds to microseconds
#define DEGREES_PER_ROTATION 360.0f // Number of degrees in one rotation
#define START_HOME_MOVEMENT -1 // This is used to setermine if a stepper motor needs to home
#define MINUTES_TO_SECONDS 60.0 // Number of seconds in a minute

// Equations
#define RADIANS(d) ((d) * float(M_PI) / 180.0f) // Equation used to convert degrees to radians
#define DEGREES(r) ((r)*180.0f / float(M_PI)) //Equation used to convert radians to degrees
#define DEGREES_TO_STEPS(d, m, g) ((long)(((d) / DEGREES_PER_ROTATION) * m * g))
#define ROTATION_DIVISOR(m) ((DEGREES_PER_ROTATION / (m)) / 6.0) // Used when calculating the step time for the motor

// Configuration used for axis 1
#define SIZE_AXIS_1 23
#define AMP_AXIS_1 2.0
#define STEP_PIN_AXIS_1 47
#define DIR_PIN_AXIS_1 45
#define ENA_PIN_AXIS_1 43
#define MICROSTEPPING_AXIS_1 1600
#define ABS_MAX_RPM_AXIS_1 1000
#define GEAR_REDUCTION_AXIS_1 99.05
#define MAX_POS_AXIS_1 -1
#define END_STOP_PIN_1 41
#define INVERT_DIR_1 false

#define AXIS_1_INIT(name) Stepper name(SIZE_AXIS_1, AMP_AXIS_1, STEP_PIN_AXIS_1, DIR_PIN_AXIS_1, ENA_PIN_AXIS_1, END_STOP_PIN_1, MICROSTEPPING_AXIS_1, ABS_MAX_RPM_AXIS_1, GEAR_REDUCTION_AXIS_1, MAX_POS_AXIS_1, INVERT_DIR_1);

// Configuration used for axis 2
#define SIZE_AXIS_2 23
#define AMP_AXIS_2 2.0
#define STEP_PIN_AXIS_2 52
#define DIR_PIN_AXIS_2 51
#define ENA_PIN_AXIS_2 49
#define MICROSTEPPING_AXIS_2 1600
#define ABS_MAX_RPM_AXIS_2 1000
#define GEAR_REDUCTION_AXIS_2 1
#define MAX_POS_AXIS_2 -1
#define END_STOP_PIN_2 41
#define INVERT_DIR_2 true

#define AXIS_2_INIT(name) Stepper name(SIZE_AXIS_2, AMP_AXIS_2, STEP_PIN_AXIS_2, DIR_PIN_AXIS_2, ENA_PIN_AXIS_2, END_STOP_PIN_2, MICROSTEPPING_AXIS_2, ABS_MAX_RPM_AXIS_2, GEAR_REDUCTION_AXIS_2, MAX_POS_AXIS_2, INVERT_DIR_2);

// Controller configuration
#define CONTROLLER_INIT(name, axis1, axis2) \
    Stepper axis[DOF] = { axis1, axis2 };   \
    Controller name(axis, DEFAULT_START_SPEED, DEFAULT_FINAL_SPEED, DEFAULT_ACCELERATION, DEFAULT_DECELERATION);
