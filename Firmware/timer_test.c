#include "io.h"

#define TIMER_BASE   0x00400040

#define TIMER_CTRL   (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD   (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE  (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT   (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

int main(void)
{
    unsigned int v;

    /* 1. Stop timer */
    TIMER_CTRL = 0x0;

    /* 2. Load a LARGE value so CPU can catch it */
    TIMER_LOAD = 100000;

    /* 3. Enable timer */
    TIMER_CTRL = 0x1;

    /* 4. Read VALUE immediately */
    v = TIMER_VALUE;

    print_string("TIMER VALUE = ");
    print_hex(v);
    print_string("\n");

    /* 5. Wait until timeout */
    while ((TIMER_STAT & 0x1) == 0);

    print_string("TIMER TIMEOUT\n");

    asm volatile ("ecall");
    return 0;
}

