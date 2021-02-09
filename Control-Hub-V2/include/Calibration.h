/**
 * This file is responsible for homing and calibrating the arm
 * through the use of the controller object
 * @author Thomas Batchelder
 * @file calibration.h
 * @date 1/24/2021 - file created
 */

#include "Controller.h"
#include <Arduino.h>

/**
 * This function is used to home all of the active axis
 * @param controller in the controller for the arm
 * @return is true if the arm homes successfully, otherwise false is returned
 */
bool home(Controller* controller);

/**
 * This function is used to calibrate the limit switch and determine that
 * they are all working correctly
 * @param controller is the main controller of the arm
 * @return is true if the limit switches are working correclty, otherwise false is returned
 */
bool limitSwitchCalibration(Controller* controller);

/**
 * This function is used to determine that all axis' encoders are functional
 * and returning the correct values. If the encoders are not tracking properly
 * A message will display how much they are off.
 * @param controller is the controller for the arm
 * @param axis is the axis that is being calibrated. If axis equal 0, then
 * all axis will be calibrated starting at axis 1
 * @param iterations is the number of times the axis gets tested
 * @return is true if the axis specified is successfully calibrated, otherwise false is returned
 */
bool encoderCalibration(Controller* controller, uint8_t axis, int iterations);

/**
 * This function is used to perfrom a full calibration. All calibration functions will be
 * run and the arm will be homed when completed.
 * @param controller is the controller for the arm
 * @return is true if the arm passes all calibration tests, otherwise false is returned
 */
bool performFullCalibration(Controller* controller);