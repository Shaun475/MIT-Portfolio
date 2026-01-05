#include "stepper.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// Static timer resources (internal to library)
static gptimer_handle_t stepperTimer = NULL;
static SemaphoreHandle_t timerSemaphore = NULL;
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t isrCounter = 0;
static volatile uint32_t lastIsrAt = 0;

// Timer ISR callback
static bool IRAM_ATTR onStepperTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    
    portENTER_CRITICAL_ISR(&timerMux);
    isrCounter++;
    lastIsrAt = esp_log_timestamp();
    portEXIT_CRITICAL_ISR(&timerMux);
    
    xSemaphoreGiveFromISR(timerSemaphore, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

// Internal function to configure GPIO pins for a motor
static void configureMotorPins(const int pins[4]) {
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Enable pull-down for clean low state
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_drive_capability(pins[i], GPIO_DRIVE_CAP_3);  // Max drive strength
        gpio_set_level(pins[i], 0);
    }
}

void initStepperTimer(void) {
    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();

    // Create and configure timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &stepperTimer));

    // Register timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onStepperTimer,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(stepperTimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(stepperTimer));

    // Configure alarm to fire every 1ms (1000us)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(stepperTimer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_start(stepperTimer));
}

void deinitStepperTimer(void) {
    if (stepperTimer) {
        ESP_ERROR_CHECK(gptimer_stop(stepperTimer));
        ESP_ERROR_CHECK(gptimer_disable(stepperTimer));
        ESP_ERROR_CHECK(gptimer_del_timer(stepperTimer));
        stepperTimer = NULL;
    }
    if (timerSemaphore) {
        vSemaphoreDelete(timerSemaphore);
        timerSemaphore = NULL;
    }
}

void setUpMovement(StepperMotor *motor, const int pins[4], int stepsPerSecond, int totalSteps, bool direction) {
    // Copy pin configuration
    for (int i = 0; i < 4; i++) {
        motor->pins[i] = pins[i];
    }
    
    // Set movement parameters
    motor->stepsPerSecond = stepsPerSecond;
    motor->totalSteps = totalSteps;
    motor->direction = direction;
    
    // Initialize state variables
    motor->currentStep = 0;
    motor->phase = 0;
    motor->timer = 0;
    motor->isActive = true;
    
    // Configure the GPIO pins for output
    configureMotorPins(motor->pins);
}

bool stepMotor(StepperMotor *motor) {
    if (!motor->isActive || motor->currentStep >= motor->totalSteps) {
        motor->isActive = false;
        return false;
    }
    
    motor->timer += motor->stepsPerSecond;
    
    if (motor->timer / 1000 > motor->currentStep) {
        // Turn off current phase
        gpio_set_level(motor->pins[motor->phase], 0);
        
        // Update phase based on direction
        if (motor->direction) {
            motor->phase = (motor->phase + 1) % 4;
        } else {
            motor->phase = (motor->phase - 1 + 4) % 4;
        }
        
        // Turn on new phase
        gpio_set_level(motor->pins[motor->phase], 1);
        motor->currentStep++;
    }
    
    return motor->currentStep < motor->totalSteps;
}

void stopMotor(StepperMotor *motor) {
    // Turn off all phases
    for (int i = 0; i < 4; i++) {
        gpio_set_level(motor->pins[i], 0);
    }
    motor->isActive = false;
}

bool isMovementComplete(StepperMotor *motor) {
    return motor->currentStep >= motor->totalSteps;
}

bool isTimerTickReady(void) {
    return xSemaphoreTake(timerSemaphore, 0) == pdTRUE;
}

