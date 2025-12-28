# STM32F446 Bare-Metal Button Debounce Using Timer Interrupts

This project demonstrates a **bare-metal STM32F446 application** implementing
**timer-based button debounce** using **TIM2 interrupts**, without STM32 HAL,
LL drivers, or CubeMX.  
All peripherals are configured directly via registers, using CMSIS Core
and Device headers only.

## Hardware

- **MCU**: STM32F446
- **Board**: STM32 Nucleo-F446RE
- **Button**: PC13 (internal pull-up, active-low)
- **LED**: PA5

## Architecture Overview

- TIM2 generates a periodic interrupt every **5 ms**
- Each interrupt samples the button state (PC13)
- A software debounce algorithm validates a stable press
- A debounced press sets a shared event flag (`button_pressed`)
- The main loop polls the flag and toggles the LED (PA5)

This design keeps the ISR lightweight and defers application logic to the main
context.

## Build Environment

- **Toolchain**: arm-none-eabi-gcc
- **Build system**: Makefile
- **Debug**: GDB + ST-Link

### Build
```bash
make
```

### Clean
```bash
make clean
```

## Debug

Debugging is performed using **GDB** and **ST-LINK**.

### Steps

```bash
# Terminal 1: start the ST-LINK GDB server
st-util

# Terminal 2: start GDB and connect to the target
make debug
```