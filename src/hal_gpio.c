#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <assert.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    for (uint8_t pin = 0; pin < 16; pin++) {
        if (GPIO_Init->Pin & (1 << pin))  {
            if (GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) {
                // "01" 
                GPIOx->MODER &= ~(3U << (2 * pin));
                GPIOx->MODER |=  (1U << (2 * pin)); 
                
                GPIOx->OTYPER &= ~(1U << pin);  // Push-pull => OTYPER = 0
            }
            else if (GPIO_Init->Mode == GPIO_MODE_INPUT) {
                GPIOx->MODER &= ~(3U << (2 * pin)); // "00" => input
            }

            // PUPDR
            GPIOx->PUPDR &= ~(3U << (2 * pin)); 
            GPIOx->PUPDR |=  (GPIO_Init->Pull << (2 * pin));

            // OSPEEDR
            GPIOx->OSPEEDR &= ~(3U << (2 * pin));
            GPIOx->OSPEEDR |=  (GPIO_Init->Speed << (2 * pin));
        }
    }
}

void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
    GPIOx->MODER &= ~(3U << (GPIO_Pin * 2));
    GPIOx->OTYPER &= ~(1U << GPIO_Pin);
    GPIOx->OSPEEDR &= ~(3U << (GPIO_Pin * 2));
    GPIOx->PUPDR &= ~(3U << (GPIO_Pin * 2));
}

GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return (GPIOx->IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET) {
        GPIOx->ODR |= GPIO_Pin;
    }
    else {
        GPIOx->ODR &= ~GPIO_Pin;
    }
}

void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin;
}

void Configure_EXTI_UserButton(void)
{
    GPIO_InitTypeDef btnInit;
    // Configure PA0 as input, low speed, with internal pull-down resistor.
    btnInit.Pin   = GPIO_PIN_0;
    btnInit.Mode  = GPIO_MODE_INPUT;
    btnInit.Pull  = GPIO_PULLDOWN;
    btnInit.Speed = GPIO_SPEED_FREQ_LOW;
    My_HAL_GPIO_Init(GPIOA, &btnInit);
    // Enable EXTI interrupt on line 0.
    // Unmask EXTI line 0 in the Interrupt Mask Register.
    EXTI->IMR |= (1 << 0);
    // Enable rising edge trigger for EXTI line 0 in the Rising Trigger Selection Register.
    EXTI->RTSR |= (1 << 0);
    // (Optionally) Clear any pending flag.
    EXTI->PR |= (1 << 0);
    // Enable clock for SYSCFG peripheral.
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    assert((SYSCFG->EXTICR[0] & 0xF) == 0);

    // Configure SYSCFG to route PA0 to EXTI line 0.
    // Clear the bits for EXTI0 (in EXTICR[0]) then set them to map port A (value 0).
    SYSCFG->EXTICR[0] &= ~(0xF);
    SYSCFG->EXTICR[0] |= 0x0; // Port A mapping; 0 is typically for PA.
    assert((SYSCFG->EXTICR[0] & 0xF) == 0x0);
}

// For lab 3.2
void TIM3_PWM(void) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure timer 3 for 800 Hz
    TIM3->PSC = 100; // Set PSC to 100 = 8 MHz / 100 = 80 kHz 
    TIM3->ARR = 99; // Set ARR to 100 = 80 kHz / 100 - 1 = 800 Hz

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M); // Clear bits

    // Configure Channel 1 (PWM Mode 2)
    TIM3->CCMR1 |= (7 << TIM_CCMR1_OC1M_Pos); // PWM Mode 2
    // Configure Channel 2 (PWM Mode 1)
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos); // PWM Mode 1

    // Enable preload
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    // Enable Output for CH1 & CH2
    TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output
    TIM3->CCER |= TIM_CCER_CC2E; // Enable CH2 output

    // Enable TIM3 counter
    TIM3->CR1 |= TIM_CR1_CEN;

    TIM3->CCR1 = 20; // Red LED duty cycle
    // duty cycle > 20, the red LED will be dimmer
    // duty cycle < 20, the red LED will be brighter

    TIM3->CCR2 = 20; // Blue LED (?) duty cycle
    // duty cycle > 20, the blue LED will be brighter
    // duty cycle < 20, the blue LED will be dimmer
}

// For Lab 3.2
void GPIO_Init_TIM3_PWM(void)
{
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Clear mode bits for PC6 and PC7
    GPIOC->MODER &= ~(0b11 << (6 * 2)); // Clear mode bits for PC6
    GPIOC->MODER &= ~(0b11 << (7 * 2)); // Clear mode bits for PC7

    // Set PC6 and PC7 to alternate Mode
    GPIOC->MODER |= (0b10 << (6 * 2)); // Set PC6 to alternate mode
    GPIOC->MODER |= (0b10 << (7 * 2)); // Set PC7 to alternate mode

    // Set PC6 and PC7 to AF0 (TIM3_CH1 and TIM3_CH2)
    GPIOC->AFR[0] &= ~(0b1111 << (6 * 4));  // AF bits for PC6
    GPIOC->AFR[0] &= ~(0b1111 << (7 * 4));  // AF bits for PC7
}


