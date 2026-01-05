#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Motor state structure - holds all variables needed for stepMotor
typedef struct {
    int pins[4];           // GPIO pins for the 4 phases
    int stepsPerSecond;    // Speed: number of steps per second
    int totalSteps;        // Target: total steps to move
    int currentStep;       // Progress: current step count
    int phase;             // Current phase (0-3)
    long timer;            // Timer accumulator for step timing
    bool direction;        // true = forward, false = backward
    bool isActive;         // Whether this motor movement is active
} StepperMotor;

/**
 * Initialize the stepper motor timer system.
 * Must be called once before using any motors.
 * Sets up the hardware timer, semaphore, and ISR callback.
 */
void initStepperTimer(void);

/**
 * Stop and cleanup the stepper motor timer system.
 */
void deinitStepperTimer(void);

/**
 * Set up a motor movement with all necessary configuration.
 * This initializes the motor struct with the given parameters and configures GPIO pins.
 * 
 * @param motor         Pointer to the StepperMotor struct to initialize
 * @param pins          Array of 4 GPIO pin numbers for motor phases
 * @param stepsPerSecond Movement speed in steps per second
 * @param totalSteps    Total number of steps to complete
 * @param direction     true for forward, false for backward
 */
void setUpMovement(StepperMotor *motor, const int pins[4], int stepsPerSecond, int totalSteps, bool direction);

/**
 * Execute one step iteration for the motor.
 * Call this when the timer semaphore is available.
 * 
 * @param motor Pointer to the StepperMotor struct
 * @return true if motor is still moving, false if movement complete
 */
bool stepMotor(StepperMotor *motor);

/**
 * Stop the motor and turn off all phases.
 * 
 * @param motor Pointer to the StepperMotor struct
 */
void stopMotor(StepperMotor *motor);

/**
 * Check if motor movement is complete.
 * 
 * @param motor Pointer to the StepperMotor struct
 * @return true if movement complete, false if still moving
 */
bool isMovementComplete(StepperMotor *motor);

/**
 * Check if timer tick is ready (semaphore available).
 * Use this in your main loop to know when to call stepMotor.
 * 
 * @return true if a timer tick occurred and motors should be stepped
 */
bool isTimerTickReady(void);

#endif // STEPPER_MOTOR_H

