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

/*
We need to initilize a tim3 clock
*/
void TIM3_PWM(void) {

}