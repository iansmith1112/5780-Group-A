#include "main.h"
#include <stm32f0xx_hal.h>

// Defining Parameters
#define LED_BITS 24 // 8 bits for each color (red, blue, green)
#define LED_COUNT 10 // use 10 led modules for now
#define LED_BUFFER_SIZE  (LED_COUNT * LED_BITS)

uint8_t led_data[LED_COUNT][3]; // GRB format, each led has 3 colors (green, red, blue)

/*
This function sets the rgb values on a single led determined by the led index by changin
the values on the led_data variable. if the led index is greater than the LED_COUNT, this
function is ignored.
*/
void set_led_color(uint8_t ledIndex, uint8_t red, uint8_t green, uint8_t blue) {
    if (ledIndex < LED_COUNT) {
        led_data[ledIndex][0] = green;
        led_data[ledIndex][1] = red;
        led_data[ledIndex][2] = blue;
    }
}

/*
Main function
*/
void ledStripTestMain (void) {
    HAL_Init();
    SystemClock_Config();
    // Infinite While Loop
    while (1)
    {
        
    }
    
}