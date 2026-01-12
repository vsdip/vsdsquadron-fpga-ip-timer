OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 6K
}

SECTIONS
{
  .text : {
    *(.text*)
    *(.rodata*)
  } > RAM

  .data : {
    *(.data*)
  } > RAM

  .bss : {
    *(.bss*)
    *(COMMON)
  } > RAM
}

