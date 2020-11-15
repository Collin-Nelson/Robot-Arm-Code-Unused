#include "../include/gcode.h"

/**
 * This method is used to process a GCODE provided by main. This function will call the
 * appropriate GCODE function for the code. If the code doesn't exist, it is ignored.
 * 
 * @param code is GCODE being checked
 */
void processCode(String code);

/**
 * This function is used to home to arm to it's original position.
 */
void G28();