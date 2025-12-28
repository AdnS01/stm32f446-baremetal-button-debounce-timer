/* Linker script for NUCLEO-F446RE Board */

/* Define the entry point */
ENTRY(reset_handler)

/* End of RAM / Start of stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Define heap and stack sizes */
__HeapSize  = 0x0800;  /* 2k */
__StackSize = 0x1000;  /* 4k */
__stack_limit__ = _estack - __StackSize;

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  /* Vector table at start of FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  /* (FLASH) .text section for program code */
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.text.reset_handler)) /* Reset handler */
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > FLASH

  /* (FLASH) .rodata section for read-only data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  /* (FLASH -> RAM) .data section for initialized data */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH
  /* Load address of .data section from FLASH */
  _sidata = LOADADDR(.data);

  /* (RAM) .bss section for zero-uninitialized data */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* (RAM)* Heap section */
  .dynamic_allocations (NOLOAD) :
  {
    . = ALIGN(8);
    __heap_start__ = .;
    . = . + __HeapSize;
    . = ALIGN(8);
    __heap_end__ = .;
  } > RAM

  /* Safety checks to prevent heap/stack overlap */
  /*
  IF (heap_end <= stack_limit)
      linking continues
  ELSE
      STOP THE BUILD
      print error message
      DO NOT generate firmware.elf
  */ 
  ASSERT(__heap_end__ <= __stack_limit__, "RAM overflow: heap overlaps stack") 
  ASSERT(__stack_limit__ >= ORIGIN(RAM), "RAM overflow: stack exceeds RAM")

  _end = .;
}
