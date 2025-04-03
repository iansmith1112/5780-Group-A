#include <stm32f0xx_hal.h>
int main(void) {
       
    HAL_Init();  // Reset all peripherals, initialize Flash and Systick
    SystemClock_Config();  // Configure the system clock

    // #1 Enable the clocks for GPIOC and GPIOA (PC6, PC7 for LEDs, PA0 for the button)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;

    // #2 Configure PC6, PC7 as output mode
    GPIOC->MODER &= ~(3 << (6 * 2)); // Clear mode bits for PC6
    GPIOC->MODER &= ~(3 << (7 * 2)); // Clear mode bits for PC7
    GPIOC->MODER |= (1 << (6 * 2));  // Set PC6 as the output
    GPIOC->MODER |= (1 << (7 * 2));  // Set PC7 as the output

    // #3 Configure PA0 (button) as input mode
    GPIOA->MODER &= ~(3 << (0 * 2));  // Clear mode bits for PA0
    
    // #4 Enable the pull-up resistor for PA0 (prevents floating state)
    GPIOA->PUPDR &= ~(3 << (0 * 2));  // Clear PUPDR for PA0
    GPIOA->PUPDR |= (1 << (0 * 2));   // Set PA0 as pull-up mode

    // #5 Set initial LED states (Red LED ON, Blue LED OFF)
    GPIOC->BSRR = (1 << 6);        // turn on PC6 (Red LED)
    GPIOC->BSRR = (1 << (7 + 16)); // turn off PC7 (Blue LED)

    // Infinite While Loop
    while (1) {
        if (GPIOA->IDR & (1 << 0)) {  // Check PA0's (button) state
            // Button is not pressed
            GPIOC->BSRR = (1 << 6);        // Turn ON Red LED (PC6)
            GPIOC->BSRR = (1 << (7 + 16)); // Turn OFF Blue LED (PC7)
        }
        
        else {
            // Button pressed
            GPIOC->BSRR = (1 << 7);        // Turn ON Blue LED (PC7)
            GPIOC->BSRR = (1 << (6 + 16)); // Turn OFF Red LED (PC6)
        }
    }
}