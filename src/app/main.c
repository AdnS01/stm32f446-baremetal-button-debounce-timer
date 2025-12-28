#include "button_tim.h"

int main(void)
{
    Button_LED_Init();   // Initialize button and LED
    TIM2_Init();   // start periodic interrupt (5 ms)

    while (1) {
        if (button_pressed) {   // Check if button press detected
            button_pressed = 0;
            GPIO_PA5_Toggle();   // LED toggle
        }
    }
}
