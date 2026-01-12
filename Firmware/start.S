.section .text
.globl _start

_start:
    li   sp, 0x1800      # stack top (6 KB RAM)
    call main

1:  j    1b              # infinite loop

