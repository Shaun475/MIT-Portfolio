#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "esp_err.h"

// PWM Configuration
#define PWM_FREQ_HZ      1500
#define PWM_RESOLUTION   LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY     255  // (2^8) - 1
#define FADE_TIME_MS     6000

// GPIO Pins
#define GPIO_PIN_19      19
#define GPIO_PIN_33      33

void app_main(void)
{
    printf("Hello world!\n");
    printf("Setting up PWM fade on pins 19 and 33...\n");

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        printf("LEDC timer config failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Configure channel for pin 19
    ledc_channel_config_t ledc_channel_19 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = GPIO_PIN_19,
        .duty       = 0,
        .hpoint     = 0
    };
    err = ledc_channel_config(&ledc_channel_19);
    if (err != ESP_OK) {
        printf("LEDC channel 19 config failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Configure channel for pin 33
    ledc_channel_config_t ledc_channel_33 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = GPIO_PIN_33,
        .duty       = 0,
        .hpoint     = 0
    };
    err = ledc_channel_config(&ledc_channel_33);
    if (err != ESP_OK) {
        printf("LEDC channel 33 config failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Initialize fade service
    err = ledc_fade_func_install(0);
    if (err != ESP_OK) {
        printf("LEDC fade install failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("PWM setup complete on pins 19 and 33.\n");

    vTaskDelay(3000 / portTICK_PERIOD_MS);
int counter = 0;
    // Fade up and down loop
    while (counter < 3) {
        // Fade up to max duty on both pins
        printf("Fading up...\n");
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX_DUTY, FADE_TIME_MS);
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX_DUTY, FADE_TIME_MS);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_WAIT_DONE);

        // Fade down to 0 on both pins
        printf("Fading down...\n");
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, FADE_TIME_MS);
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, FADE_TIME_MS);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_WAIT_DONE);
        printf("Counter: %d\n", counter);
        counter++;
    }
}
