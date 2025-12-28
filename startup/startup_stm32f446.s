// Arm Thumb assembly for Cortex-M4 microcontroller
.syntax unified
.cpu cortex-m4
.thumb

// Global memory locations.
.global _vector_table  
.global reset_handler
.global  Default_Handler

// External symbols defined in the linker script
.extern _estack        // End of stack (initial stack pointer)
.extern _sdata         // Start of .data section in RAM
.extern _edata         // End of .data section in RAM
.extern _sidata        // Start of .data section in FLASH
.extern _sbss          // Start of .bss section in RAM
.extern _ebss          // End of .bss section in RAM
.extern main          // main function

// Reset handler section -------------------------------------------------
.section .text.reset_handler, "ax", %progbits
.align 2    // 4-byte alignment
.thumb_func
.type reset_handler, %function
reset_handler:

    // Initialize the stack pointer
    ldr r0, =_estack
    bic r0, r0, #7      /* align down to 8 bytes */
    mov sp, r0

    // Copy .data from FLASH to RAM
    ldr r1, =_sdata
    ldr r2, =_edata
    ldr r3, =_sidata
    cmp r1, r2
    bcs data_is_empty   // branch if .data is empty
copy_data_loop:
    ldr  r4, [r3], #4      // load from source (Flash), then increment r3
    str  r4, [r1], #4      // store to destination (RAM), then increment r1
    cmp  r1, r2
    bcc  copy_data_loop   // branch while (r1 < r2)
data_is_empty:

    // Zero initialize .bss section
    ldr r1, =_sbss
    ldr r2, =_ebss
    mov r3, #0
    cmp r1, r2
    bcs bss_is_empty    // branch if .bss is empty
bss_loop:
    str  r3, [r1], #4     
    cmp  r1, r2
    bcc  bss_loop    // branch while (r1 < r2)
bss_is_empty:

    // Call main() function
    bl main    
    
    // If main returns, loop indefinitely
halt:
    b halt

.size reset_handler, .-reset_handler


// Default handler section -------------------------------------------------
// This handler is called for unexpected interrupts.
// It loops indefinitely to halt the system.
.section .text.Default_Handler,"ax",%progbits
.align 2
.thumb_func
.type Default_Handler, %function
Default_Handler:
Infinite_Loop:
    b  Infinite_Loop
.size Default_Handler, .-Default_Handler


// Vector table section -------------------------------------------------
.section .isr_vector, "a", %progbits
.align 2    // 4-byte alignment
.type _vector_table, %object
_vector_table:
    .word   _estack                /* Initial Stack Pointer */
    .word   reset_handler          /* Reset Handler */
    
    // Cortex-M4 Exception Handlers
    .word  NMI_Handler
    .word  HardFault_Handler
    .word  MemManage_Handler
    .word  BusFault_Handler
    .word  UsageFault_Handler
    .word  0
    .word  0
    .word  0
    .word  0
    .word  SVC_Handler
    .word  DebugMon_Handler
    .word  0
    .word  PendSV_Handler
    .word  SysTick_Handler

    /* External Interrupts */
    .word     WWDG_IRQHandler                   /* Window WatchDog              */                                        
    .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */                        
    .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */            
    .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
    .word     FLASH_IRQHandler                  /* FLASH                        */                                          
    .word     RCC_IRQHandler                    /* RCC                          */                                            
    .word     EXTI0_IRQHandler                  /* EXTI Line0                   */                        
    .word     EXTI1_IRQHandler                  /* EXTI Line1                   */                          
    .word     EXTI2_IRQHandler                  /* EXTI Line2                   */                          
    .word     EXTI3_IRQHandler                  /* EXTI Line3                   */                          
    .word     EXTI4_IRQHandler                  /* EXTI Line4                   */                          
    .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
    .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
    .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
    .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
    .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
    .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
    .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
    .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */                   
    .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */                         
    .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */                          
    .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */                          
    .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */                          
    .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
    .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */         
    .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */         
    .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
    .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
    .word     TIM2_IRQHandler                   /* TIM2                         */                   
    .word     TIM3_IRQHandler                   /* TIM3                         */                   
    .word     TIM4_IRQHandler                   /* TIM4                         */                   
    .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
    .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
    .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
    .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
    .word     SPI1_IRQHandler                   /* SPI1                         */                   
    .word     SPI2_IRQHandler                   /* SPI2                         */                   
    .word     USART1_IRQHandler                 /* USART1                       */                   
    .word     USART2_IRQHandler                 /* USART2                       */                   
    .word     USART3_IRQHandler                 /* USART3                       */                   
    .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
    .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
    .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */                       
    .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */         
    .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */         
    .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
    .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */                          
    .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
    .word     FMC_IRQHandler                    /* FMC                          */                   
    .word     SDIO_IRQHandler                   /* SDIO                         */                   
    .word     TIM5_IRQHandler                   /* TIM5                         */                   
    .word     SPI3_IRQHandler                   /* SPI3                         */                   
    .word     UART4_IRQHandler                  /* UART4                        */                   
    .word     UART5_IRQHandler                  /* UART5                        */                   
    .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */                   
    .word     TIM7_IRQHandler                   /* TIM7                         */
    .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
    .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
    .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
    .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
    .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                   
    .word     0                                 /* Reserved                     */                   
    .word     0                                 /* Reserved                     */                     
    .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */                          
    .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */                          
    .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */                          
    .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */                          
    .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */                   
    .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
    .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
    .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
    .word     USART6_IRQHandler                 /* USART6                       */                    
    .word     I2C3_EV_IRQHandler                /* I2C3 event                   */                          
    .word     I2C3_ER_IRQHandler                /* I2C3 error                   */                          
    .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */                   
    .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */                   
    .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */                         
    .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */                   
    .word     DCMI_IRQHandler                   /* DCMI                         */                   
    .word     0                                 /* Reserved                     */                   
    .word     0                                 /* Reserved                     */
    .word     FPU_IRQHandler                    /* FPU                          */
    .word     0                                 /* Reserved                     */
    .word     0                                 /* Reserved                     */
    .word     SPI4_IRQHandler                   /* SPI4                         */
    .word     0                                 /* Reserved                     */
    .word     0                                 /* Reserved                     */
    .word     SAI1_IRQHandler                   /* SAI1                         */
    .word     0                                 /* Reserved                     */
    .word     0                                 /* Reserved                     */
    .word     0                                 /* Reserved                     */
    .word     SAI2_IRQHandler                   /* SAI2                         */
    .word     QUADSPI_IRQHandler                /* QuadSPI                      */
    .word     CEC_IRQHandler                    /* CEC                          */
    .word     SPDIF_RX_IRQHandler               /* SPDIF RX                     */
    .word     FMPI2C1_EV_IRQHandler          /* FMPI2C 1 Event               */
    .word     FMPI2C1_ER_IRQHandler          /* FMPI2C 1 Error               */

.size _vector_table, .-_vector_table


/* Weak alias macro definition ---------------------------------------- */
.macro WEAK_DEFAULT name
    .weak \name
    .thumb_set \name, Default_Handler
.endm

/* Weak aliases -------------------------------------------------------- */
/* Cortex-M4 exception handlers */
WEAK_DEFAULT      NMI_Handler  
WEAK_DEFAULT      HardFault_Handler  
WEAK_DEFAULT      MemManage_Handler  
WEAK_DEFAULT      BusFault_Handler
WEAK_DEFAULT      UsageFault_Handler
WEAK_DEFAULT      SVC_Handler
WEAK_DEFAULT      DebugMon_Handler
WEAK_DEFAULT      PendSV_Handler
WEAK_DEFAULT      SysTick_Handler

/* STM32F4xx peripheral interrupt handlers / External interrupt handlers (IRQs) */  
WEAK_DEFAULT      WWDG_IRQHandler                                     
WEAK_DEFAULT      PVD_IRQHandler                     
WEAK_DEFAULT      TAMP_STAMP_IRQHandler                        
WEAK_DEFAULT      RTC_WKUP_IRQHandler                              
WEAK_DEFAULT      FLASH_IRQHandler                           
WEAK_DEFAULT      RCC_IRQHandler                        
WEAK_DEFAULT      EXTI0_IRQHandler                           
WEAK_DEFAULT      EXTI1_IRQHandler                              
WEAK_DEFAULT      EXTI2_IRQHandler                          
WEAK_DEFAULT      EXTI3_IRQHandler                                 
WEAK_DEFAULT      EXTI4_IRQHandler                           
WEAK_DEFAULT      DMA1_Stream0_IRQHandler                        
WEAK_DEFAULT      DMA1_Stream1_IRQHandler                                 
WEAK_DEFAULT      DMA1_Stream2_IRQHandler                                 
WEAK_DEFAULT      DMA1_Stream3_IRQHandler                                
WEAK_DEFAULT      DMA1_Stream4_IRQHandler                                
WEAK_DEFAULT      DMA1_Stream5_IRQHandler                                 
WEAK_DEFAULT      DMA1_Stream6_IRQHandler                                 
WEAK_DEFAULT      ADC_IRQHandler                     
WEAK_DEFAULT      CAN1_TX_IRQHandler               
WEAK_DEFAULT      CAN1_RX0_IRQHandler                                             
WEAK_DEFAULT      CAN1_RX1_IRQHandler                              
WEAK_DEFAULT      CAN1_SCE_IRQHandler                              
WEAK_DEFAULT      EXTI9_5_IRQHandler   
WEAK_DEFAULT      TIM1_BRK_TIM9_IRQHandler            
WEAK_DEFAULT      TIM1_UP_TIM10_IRQHandler            
WEAK_DEFAULT      TIM1_TRG_COM_TIM11_IRQHandler            
WEAK_DEFAULT      TIM1_CC_IRQHandler                     
WEAK_DEFAULT      TIM2_IRQHandler                              
WEAK_DEFAULT      TIM3_IRQHandler                              
WEAK_DEFAULT      TIM4_IRQHandler                              
WEAK_DEFAULT      I2C1_EV_IRQHandler                        
WEAK_DEFAULT      I2C1_ER_IRQHandler                        
WEAK_DEFAULT      I2C2_EV_IRQHandler                     
WEAK_DEFAULT      I2C2_ER_IRQHandler                              
WEAK_DEFAULT      SPI1_IRQHandler                                    
WEAK_DEFAULT      SPI2_IRQHandler                              
WEAK_DEFAULT      USART1_IRQHandler                           
WEAK_DEFAULT      USART2_IRQHandler                           
WEAK_DEFAULT      USART3_IRQHandler                        
WEAK_DEFAULT      EXTI15_10_IRQHandler                              
WEAK_DEFAULT      RTC_Alarm_IRQHandler                           
WEAK_DEFAULT      OTG_FS_WKUP_IRQHandler                     
WEAK_DEFAULT      TIM8_BRK_TIM12_IRQHandler                  
WEAK_DEFAULT      TIM8_UP_TIM13_IRQHandler                     
WEAK_DEFAULT      TIM8_TRG_COM_TIM14_IRQHandler            
WEAK_DEFAULT      TIM8_CC_IRQHandler                     
WEAK_DEFAULT      DMA1_Stream7_IRQHandler                                    
WEAK_DEFAULT      FMC_IRQHandler                                 
WEAK_DEFAULT      SDIO_IRQHandler                                 
WEAK_DEFAULT      TIM5_IRQHandler                                 
WEAK_DEFAULT      SPI3_IRQHandler                                 
WEAK_DEFAULT      UART4_IRQHandler                           
WEAK_DEFAULT      UART5_IRQHandler                           
WEAK_DEFAULT      TIM6_DAC_IRQHandler                                 
WEAK_DEFAULT      TIM7_IRQHandler                     
WEAK_DEFAULT      DMA2_Stream0_IRQHandler                              
WEAK_DEFAULT      DMA2_Stream1_IRQHandler                                 
WEAK_DEFAULT      DMA2_Stream2_IRQHandler                           
WEAK_DEFAULT      DMA2_Stream3_IRQHandler                           
WEAK_DEFAULT      DMA2_Stream4_IRQHandler               
WEAK_DEFAULT      CAN2_TX_IRQHandler                              
WEAK_DEFAULT      CAN2_RX0_IRQHandler                                             
WEAK_DEFAULT      CAN2_RX1_IRQHandler                                             
WEAK_DEFAULT      CAN2_SCE_IRQHandler                                             
WEAK_DEFAULT      OTG_FS_IRQHandler                           
WEAK_DEFAULT      DMA2_Stream5_IRQHandler                                 
WEAK_DEFAULT      DMA2_Stream6_IRQHandler                                 
WEAK_DEFAULT      DMA2_Stream7_IRQHandler                                 
WEAK_DEFAULT      USART6_IRQHandler                              
WEAK_DEFAULT      I2C3_EV_IRQHandler                           
WEAK_DEFAULT      I2C3_ER_IRQHandler                           
WEAK_DEFAULT      OTG_HS_EP1_OUT_IRQHandler                        
WEAK_DEFAULT      OTG_HS_EP1_IN_IRQHandler                           
WEAK_DEFAULT      OTG_HS_WKUP_IRQHandler                     
WEAK_DEFAULT      OTG_HS_IRQHandler                        
WEAK_DEFAULT      DCMI_IRQHandler            
WEAK_DEFAULT      FPU_IRQHandler                  
WEAK_DEFAULT      SPI4_IRQHandler            
WEAK_DEFAULT      SAI1_IRQHandler            
WEAK_DEFAULT      SAI2_IRQHandler               
WEAK_DEFAULT      QUADSPI_IRQHandler             
WEAK_DEFAULT      CEC_IRQHandler               
WEAK_DEFAULT      SPDIF_RX_IRQHandler             
WEAK_DEFAULT      FMPI2C1_EV_IRQHandler               
WEAK_DEFAULT      FMPI2C1_ER_IRQHandler            