#include "io.h"
#define TIMER_BASE 0x00400040
#define TIMER_CTRL (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_STAT (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

int main(void)
{
    TIMER_LOAD = 20;
    TIMER_CTRL = (1 << 0) | (1 << 1); // EN=1, MODE=1

    while (1) {
        while ((TIMER_STAT & 1) == 0);
        print_string("Periodic timeout\n");
        TIMER_STAT = 1;
    }
}

