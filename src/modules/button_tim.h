#ifndef BUTTON_TIM_H
#define BUTTON_TIM_H

#include "stm32f446xx.h"
#include <stdint.h>

// Global variable to indicate button press event
extern volatile uint8_t button_pressed;

// Public API
void Button_LED_Init(void);   // Initializes PA5 (LED) and PC13 (button)
void GPIO_PA5_Toggle(void);   // Toggles PA5 LED
void TIM2_Init(void);         // Starts TIM2 periodic interrupt (5 ms)

#endif // BUTTON_TIM_H
