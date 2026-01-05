#include "stepper.h"

StepperMotor motor1;
const int pins[] = {19, 18, 5, 17};

void app_main() {
    
    initStepperTimer();
    
    // Set up motor movement
    setUpMovement(&motor1, pins, 500, 20000, false);
    
    // Main loop
    while (1) {
        if (isTimerTickReady()) {
            stepMotor(&motor1);
        }
    }
}