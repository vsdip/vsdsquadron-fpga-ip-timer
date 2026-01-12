#include "io.h"

#define TIMER_BASE   0x00400040

#define TIMER_CTRL   (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD   (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE  (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT   (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

int main(void)
{
    /* 1. Stop timer */
    TIMER_CTRL = 0x0;

    /* 2. Clear any stale timeout (W1C) */
    TIMER_STAT = 0x1;

    /* 3. Load small value */
    TIMER_LOAD = 10;

    /* 4. Enable timer (one-shot, no prescaler) */
    TIMER_CTRL = 0x1;

    /* 5. Wait until timeout is set */
    while ((TIMER_STAT & 0x1) == 0);

    /* 6. Clear timeout flag */
    TIMER_STAT = 0x1;

    /* 7. Confirm timeout is cleared */
    while ((TIMER_STAT & 0x1) != 0);

    /* End simulation */
    asm volatile ("ecall");
    return 0;
}

