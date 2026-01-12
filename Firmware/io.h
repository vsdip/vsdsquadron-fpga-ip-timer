#ifndef IO_H
#define IO_H

/* Bare-metal fixed-width types */
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;

/* IO base (matches RTL: mem_addr[22] == 1) */
#define IO_BASE       0x00400000

#define IO_LEDS       4
#define IO_UART_DAT   8
#define IO_UART_CNTL  16

#define IO_IN(port)        (*(volatile uint32_t*)(IO_BASE + (port)))
#define IO_OUT(port,val)  (*(volatile uint32_t*)(IO_BASE + (port)) = (val))

#endif


