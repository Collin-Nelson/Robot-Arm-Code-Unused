#include "../include/controller.h"

Controller::Controller()
{
    // Getting pins for controlling the stepper motors
    int stepPins[DOF] = STEP_PINS;
    int dirPins[DOF] = DIR_PINS;
    int enaPins[DOF] = ENA_PINS;
    int limPins[DOF] = LIMIT_SWITCH_PINS;

    // Getting motor information
    long mircostepping[DOF] = MICROSTEPPING;
    long absMaxRPM[DOF] = ABS_MAX_RPM;
    double gearReduction[DOF] = GEAR_REDUCTION;
    long maxPosition[DOF] = MAX_POSITION;
    bool invertDir[DOF] = INVERT_DIR;
    double homingRPM[DOF] = HOMING_RPM;

    for (size_t i = 0; i < DOF; i++) {
        this->axis[i] = Stepper(
            stepPins[i],
            dirPins[i],
            enaPins[i],
            limPins[i],
            mircostepping[i],
            absMaxRPM[i],
            gearReduction[i],
            maxPosition[i],
            invertDir[i],
            homingRPM[i]);
    }
}

void Controller::home()
{
    Serial.println("  --> Homing All Axis: STARTED");
    for (int i = 0; i < DOF; i++) {
        this->axis[i].setDirection(COUNTERCLOCKWISE);
    }
    Serial.println("    --> Homing information written to steppers");
    Serial.println("    --> First Home: STARTED");
    bool isHome = false;
    while (!isHome) {
        bool allAxisHome = true;
        for (int i = 0; i < 3; i++) {
            allAxisHome = this->axis[i].home(0, 1) && allAxisHome;
        }
        isHome = allAxisHome;
    }

    for (int i = 0; i < DOF; i++) {
        this->axis[i].setDirection(CLOCKWISE);
    }
    Serial.println("    --> First Home: COMPLETED");
    Serial.println("    --> Reseting Motors: STARTED");
    uint32_t stopTime = micros() + SECONDS_TO_MICROSECONDS / 2;
    while (stopTime > micros()) {
        for (int i = 0; i < 3; i++) {
            this->axis[i].home(1, 0.5);
        }
    }
    Serial.println("    --> Reseting Motors: COMPLETED");
    Serial.println("    --> Second Home: STARTED");
    for (int i = 0; i < DOF; i++) {
        this->axis[i].setDirection(true);
    }
    isHome = false;
    while (!isHome) {
        bool allAxisHome = true;
        for (int i = 0; i < 3; i++) {
            allAxisHome = this->axis[i].home(0, 0.1) && allAxisHome;
        }
        isHome = allAxisHome;
    }
    Serial.println("    --> Second Home: COMPLETED");
    for (int i = 0; i < 500; i++) {
        for (int i = 0; i < DOF; i++) {
            this->axis[i].setDirection(false);
            this->axis[i].pulseStepper();
            this->axis[i].setCurrentPosition(0);
            delayMicroseconds(500);
        }
    }
    Serial.println("    --> Updated Homing information written to motors");
    Serial.println("  --> Homing All Axis: COMPLETE");
}

void printDouble(double val, int precision)
{
    // prints val with number of decimal places determine by precision
    // precision is a number from 0 to 6 indicating the desired decimial places
    // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

    Serial.print(int(val)); //prints the int part
    if (precision > 0) {
        Serial.print("."); // print the decimal point
        unsigned long frac;
        unsigned long mult = 1;
        int padding = precision - 1;
        while (precision--)
            mult *= 10;

        if (val >= 0)
            frac = (val - int(val)) * mult;
        else
            frac = (int(val) - val) * mult;
        unsigned long frac1 = frac;
        while (frac1 /= 10)
            padding--;
        while (padding--)
            Serial.print("0");
        Serial.print(frac, DEC);
    }
}

void Controller::traverseStraightLine(
    double* initAngles, // The current angles of all of the motors
    double* finalAngles, // The target angle for all of the motors
    double velocity, // The peak target velocity of the motors
    double acceleration, // The acceleration the the motors will be using
    double initVelocity, // The initial velocity the motors will start with
    double finalVelocity) // The final velocity the motors will end with
{
    Serial.println("--- Straight Line Movement: STARTED");
    Serial.println("Axis Angles:       \tAxis 1  \tAxis 2  \tAxis 3  \tAxis 4  \tAxis 5  \tAxis 6  \t");
    Serial.print("Current Trajectory:\t");
    for (int i = 0; i < DOF; i++) {
        printDouble(initAngles[i], 6);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("Target Trajectory:\t");
    for (int i = 0; i < DOF; i++) {
        printDouble(finalAngles[i], 6);
        Serial.print("\t");
    }
    Serial.println();

    // --- Calculating the greatest changes in degrees of each axis --- //
    double largestDegreeChange = 0;
    for (int i = 0; i < DOF; i++) {
        largestDegreeChange = max(largestDegreeChange, abs(finalAngles[i] - initAngles[i]));
    }
    Serial.println(largestDegreeChange);

    // Set the direction for each of the axis
    for (int i = 0; i < DOF; i++) {
        if (finalAngles[i] - initAngles[i] < 0.0) {
            this->axis[i].setDirection(COUNTERCLOCKWISE);
        } else {
            this->axis[i].setDirection(CLOCKWISE);
        }
    }

    uint32_t startTime = micros(); // Getting the starting time for the movement
    double scaler = 0.0; // Variable used to calculate when steps are processed
    // Calculate if velocity needs to be updated
    velocity = min(velocity, sqrt(largestDegreeChange * acceleration + 0.5 * sq(initVelocity) + 0.5 * sq(finalVelocity)));
    uint32_t currentTime = micros(); // Variable for storing the current time
    uint32_t timeDelta = 0; // Difference in time between startTime and current time

    double tap = (velocity / acceleration) - (initVelocity / acceleration);
    double lap = (initVelocity * tap) + (acceleration * sq(tap)) / 2.0;
    double lcsp = largestDegreeChange - (sq(velocity) / 2.0 / acceleration - sq(finalVelocity) / 2.0 / acceleration);
    double tcsp = (lcsp - lap) / velocity + tap;
    double tfin = (velocity / acceleration) - (finalVelocity / acceleration) + tcsp;

    while (currentTime - startTime <= tfin) {
        timeDelta = currentTime - startTime;
        //acceleration phase
        if (timeDelta <= tap) {
            scaler = initVelocity * timeDelta + acceleration * timeDelta * timeDelta / 2.0;
        }
        //contant maximum speed phase
        if (timeDelta > tap && timeDelta <= tcsp) {
            scaler = lap + velocity * (timeDelta - tap);
        }
        //deceleration phase
        if (timeDelta > tcsp) {
            scaler = lcsp + velocity * (timeDelta - tcsp) - acceleration * (timeDelta - tcsp) * (timeDelta - tcsp) / 2.0;
        }

        //trajectory x and y as a function of scalar
        double updatedTrajectory[6];
        for (int i = 0; i < DOF; i++) {
            updatedTrajectory[i] = initAngles[i] + (finalAngles[i] - initAngles[i]) / largestDegreeChange * scaler;
        }
        performTrajectory(updatedTrajectory);
        currentTime = micros();
    }
    Serial.print("Updated Trajectory:\t");
    for (int i = 0; i < DOF; i++) {
        printDouble(this->axis[i].getCurrentAngle(), 6);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("Actual Stepper POS:\t");
    for (int i = 0; i < DOF; i++) {
        printDouble(this->axis[i].getCurrentPosition(), 0);
        Serial.print("\t\t");
    }
    Serial.println();
    Serial.print("Largest Degree Change:\t");
    Serial.println(largestDegreeChange);
    Serial.print("Total Movement TIme:\t");
    Serial.println(tfin);
    Serial.println("--- Straight Line Movement: COMPLETED\n");
}

void Controller::performTrajectory(double* updatedTrajectory)
{
    for (int i = 0; i < DOF; i++) {
        if (this->axis[i].getDirection() == COUNTERCLOCKWISE) {
            while (-updatedTrajectory[i] + this->axis[i].getCurrentAngle() > this->axis[i].getDegreeChangePerStep()) {
                if (this->axis[i].pulseStepper()) {
                    this->axis[i].setCurrentAngle(this->axis[i].getCurrentAngle() - this->axis[i].getDegreeChangePerStep());
                }
            }
        } else {
            while (updatedTrajectory[i] - this->axis[i].getCurrentAngle() > this->axis[i].getDegreeChangePerStep()) {
                if (this->axis[i].pulseStepper()) {
                    this->axis[i].setCurrentAngle(this->axis[i].getCurrentAngle() + this->axis[i].getDegreeChangePerStep());
                }
            }
        }
    }
}

void Controller::update()
{
}