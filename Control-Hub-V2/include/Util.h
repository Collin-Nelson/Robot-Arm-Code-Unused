/**
 * This file contains random functions that may be useful to the user
 * 
 * @author Thomas Batchelder
 * @file Util.h
 * @date 1/19/2021 - created file
 */

#include "Configuration.h"
#include <Encoder.h>
#include <Arduino.h>

/** 
 * This function is useful for testing the limit switches
 * The current position of the limit switch is printed to the serial monitor as either 0 or 1
 * @param iterations is the number of iterations the test will go through
 * @param delayms is the delay between each limit switch reading
 * @param pins is an array of pins that are connencted to the limit switches
 */
void testLimitSwitchs(int iterations, int delayms, int* pins);

/**
 * This function is used to filter the input received from a limit switch
 * Interference can occur when the motors are turned on and this function is used to negate that
 * @param limPin is the pin the read from
 * @param iterations is the number of readings in that pin
 * @param threshold is the percentage of readings that have to pass in order to return true (0 to 1)
 * @return is true if the readings fall within the threshold, otherwise false is returned
 */
bool limitSwitchFilter(int limPin, int iterations, double threshold);

/**
 * This function is used to print out the position of the each of the encoders
 * @param is the motors for each axis
 */
void testEncoderPosition(Encoder* encoders);