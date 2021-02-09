/**
 * This class is meant to be used to communicate Event information between a computer and
 * the microcontroller
 * @author Thomas Batchelder
 * @file Communication.cpp
 * @date 1/30/2021 - file created
 */

#include "../include/Communication.h"

Communication::Communication(EventQueue* eventQueue)
{
    for (int i = 0; i < DOF; i++) {
        this->data[i] = 0;
    }
    this->counter = 0;
    this->state = INIT_STATE;
    this->eventQueue = eventQueue;
}

void Communication::update()
{
    double input;
    if (Serial.available()) {
        Serial.readBytes((char*)&input, sizeof(input));
        Serial.println(String(input, 12));

        if (this->state == INIT_STATE) {
            if ((int)input == MOVEMENT_EVENT) { 
                Serial.println("Starting Event Transmission");
                this->state = MOVEMENT_INPUT;
            }
        } else if (this->state == MOVEMENT_INPUT) {
            if ((int)input != END_TRANSMISSION) {
                Serial.print("Adding to data [ ");
                Serial.print(this->counter);
                Serial.println(" ]");
                this->data[this->counter++] = input;
            } else {
                Serial.println("Adding Movement Event");
                eventQueue->addMovementEvent(data);
                this->state = INIT_STATE;
                this->counter = 0;
            }
        }
    }
}
