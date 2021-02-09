/**
 * This class is meant to be used to communicate Event information between a computer and
 * the microcontroller
 * @author Thomas Batchelder
 * @file Communication.h
 * @date 1/30/2021 - file created
 */

#include "EventQueue.h"
#include <Configuration.h>

#define INIT_STATE 0
#define MOVEMENT_INPUT 1
#define END_TRANSMISSION -1

/**
 * This class is used to communcate between a computer and the Teensy microcontroller
 */
class Communication {
private:
    /** Array for storing input data */
    double data[20];
    /** Counter used for counting input data */
    long counter;
    /** The currect state of communication */
    int state;
    /** Pointer to the EventQueue being used by the controller */
    EventQueue* eventQueue;

public:
    Communication() {}
    Communication(EventQueue* eventQueue);
    void update();
};
