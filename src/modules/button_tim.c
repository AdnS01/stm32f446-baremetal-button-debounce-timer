#include "button_tim.h"

/*
Steps to initialize PA5 as output (LED):
    - Enable the GPIOA peripheral clock (RCC_AHB1ENR)
    - Configure PA5 as general-purpose output mode (GPIOA_MODER)
    - Configure PA5 output type as push-pull (GPIOA_OTYPER)
    - Configure PA5 output speed as low speed (GPIOA_OSPEEDR)
    - Disable pull-up and pull-down resistors on PA5 (GPIOA_PUPDR)

Steps to initialize PC13 as input with pull-up (button):
    - Enable the GPIOC peripheral clock (RCC_AHB1ENR)
    - Configure PC13 as input mode (GPIOC_MODER)
    - Enable internal pull-up resistor on PC13 (GPIOC_PUPDR)

Steps to initialize TIM2 for periodic interrupt generation (5 ms period):
    - Enable the TIM2 peripheral clock (RCC_APB1ENR)
    - Configure TIM2 prescaler to derive a 1 MHz timer counter clock
    - Configure auto-reload register for a 5 ms update period
    - Generate an update event to load prescaler and auto-reload values
    - Enable update interrupt generation (TIM_DIER_UIE)
    - Configure and enable TIM2 interrupt in the NVIC
    - Start the TIM2 counter (TIM_CR1_CEN)
*/

// Public button event
volatile uint8_t button_pressed = 0;

// GPIO Initialization: PA5 as output, PC13 as input 
void Button_LED_Init(void){
    // Enable clock to GPIO port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Enable clock to GPIO port C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Set PA5 as General purpose output mode "0b01"
    // PA5 default value is "0b00"
    GPIOA->MODER &= ~(3UL << (5U*2U));  // clear bits
    GPIOA->MODER |= (1UL << (5U*2U));    // "0b00" -> "0b01"
    // Set PC13 as General purpose input mode "0b00" (Default value)
    GPIOC->MODER &= ~(3UL << (13U*2U));    // clear bits

    // Set output type of PA5 as push-pull (Default value)
    GPIOA->OTYPER &= ~(1UL << 5U);   // clear mode bit

    // Set output speed of PA5 as low (Default value)
    GPIOA->OSPEEDR &= ~(3UL << (5U*2U));    // clear bits

    // Set PA5 as no pull-up, pull-down (Default value)
    GPIOA->PUPDR &= ~(3UL << (5U*2U));    // clear bits
    // Set PC13 as pull-up 
    GPIOC->PUPDR &= ~(3UL << (13U*2U));    // clear bits
    GPIOC->PUPDR |= (1UL << (13U*2U));    // "0b00" -> "0b01"
}

void GPIO_PA5_Toggle(void){
    // Toggle PA5 output state
    GPIOA->ODR ^= (1UL << 5U);  
}

// Set up timer 2 to generate an interrupt every 5 ms
void TIM2_Init(void){
    // Enable clock to Timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Stop timer while initializing
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Set prescaler to have 1 MHz timer clock
    // TIM2 input clock (TIM2CLK) is 16 MHz (HSI=16 MHz and APB1 prescaler = 1)
    // F_CK_PSC = F_CK / (PSC + 1)
    TIM2->PSC = (16U - 1U); 

    // Set auto-reload value for 5 ms interval
    // 1 MHz / 1000 = 1000 Hz -> 1 ms
    // 1 ms * 5 = 5 ms -> 5 * 1000 counts
    TIM2->ARR = (5000U - 1U); // 5 ms at 1 MHz

    // Load PSC/ARR immediately
    TIM2->EGR = TIM_EGR_UG;

    // Clear pending flags
    TIM2->SR = 0U;

    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // NVIC configuration
    NVIC_SetPriority(TIM2_IRQn, 5U);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable / Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

// Button Debounce and Press Detection
static void Button_DebounceAndDetect(void)
{
    static uint8_t counter = 0; 
    static uint8_t already_reported = 0;

    // Read PC13 input state
    // PC13 pull-up: pressed -> reads 0
    if ((GPIOC->IDR & (1UL << 13U)) == 0U) {
        // button is pressed
        if (!already_reported) { 
            counter++;
            if (counter >= 4U) {
                button_pressed = 1U;
                already_reported = 1U;      
            }
        }
    } else {
        // button is not pressed
        counter = 0U;
        already_reported = 0U;
    }
}

// Timer 2 Interrupt Handler
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF){ // Check update interrupt flag 
        TIM2->SR &= ~TIM_SR_UIF;    // Clear update flag
        Button_DebounceAndDetect();     // Call button debounce and detect function
    }
}