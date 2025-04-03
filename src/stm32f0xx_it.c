/**
  ******************************************************************************
  * @file    Templates/Src/stm32f0xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include <stm32f0xx_hal.h>
#include <stm32f0xx_it.h>

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

// Modifying SysTick_Handler(void) 
volatile uint32_t tick_counter = 0;  // Counter to track SysTick interrupts

// commented out to get rid of flashing blue led on lab 4
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  // tick_counter = tick_counter + 1; // Increment by 1
  // if (tick_counter == 200) {
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);  // Toggle PC7 (Blue LED)
  //   tick_counter = 0;  // Reset counter
  // }
  // Original part of handler
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

// lab 2 second checkoff
void EXTI0_1_IRQHandler(void)
{
  // Check if EXTI line 0 is pending.
  if (EXTI->PR & (1 << 0)) {
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); //PC8 (orange LED)
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // PC9 (green LED)

    volatile uint32_t delaycount; // variable that can be changed at any time.
    // delay loop that is a delay of about 1.5 seconds.
    for (delaycount = 0; delaycount < 1500000; delaycount++)
    {
      // Empty loop
    }

    // Toggle the pins again to represent this step:
    // Add a second LED toggle so that the green and orange LEDs should exchange once before
    // and after the delay loop.
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); //PC8 (orange LED)
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // PC9 (green LED)

    // Clear pending flag for EXTI line 0.
    EXTI->PR |= (1 << 0);
  }
}

// Lab 3.1
void TIM2_IRQHandler(void) {
  if (TIM2->SR & TIM_SR_UIF) {  // Check update interrupt flag
      TIM2->SR &= ~TIM_SR_UIF;  // Clear the flag

      // Toggle PC8 and PC9 (both should be alternating)
      GPIOC->ODR ^= (1 << 8);  // Toggle Green LED (PC8)
      GPIOC->ODR ^= (1 << 9);  // Toggle Orange LED (PC9)
  }
}
