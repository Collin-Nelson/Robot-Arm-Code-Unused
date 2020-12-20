#include "configuration.h"
#include <Arduino.h>

/**
 * This function is used to test the limit switches in the arm.
 * 500 ms delay between each iteration
 * 
 * @param iterations is the number of iterations to cycle through
 * @param is the delay between each iteration
 * @param pins is a pointer to the limit switch pins
 */
void testLimitSwitchs(int iterations, int delayms, int* pins);

/**
 * This method is used to determine if a value from the limit switch is correct
 * 
 * @param limPin is the pin to the limit switch
 * @param is the number of reads that will occur on the pin
 * @param threshold is the minimum precentage to pass the filter
 */
bool limitSwitchFilter(int limPin, int iterations, double threshold);