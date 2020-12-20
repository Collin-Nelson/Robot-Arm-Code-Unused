#include "../include/limitSwitch.h"

void testLimitSwitchs(int iterations, int delayms, int* pins)
{
    Serial.println("Axis: 1 2 3 4 5 6");
    for (int i = 0; i < iterations; i++) {
        Serial.print("  --> ");
        for (int j = 0; j < DOF; j++) {
            Serial.print(digitalRead(pins[j]));
            Serial.print(" ");
        }
        Serial.println();
        delay(delayms);
    }
    Serial.println("--- Finished Testing ---");
}

bool limitSwitchFilter(int limPin, int iterations, double threshold)
{
    int count = 0;
    for (int i = 0; i < iterations; i++) {
        count += digitalReadFast(limPin);
    }
    return (count / iterations) > threshold;
}