# Timer IP – Example Usage & Validation

This document explains **how a user should use the Timer IP**, **what to observe**, and **how to validate behavior** using simulation waveforms, UART output, and FPGA LEDs.
It is written to be *plug‑and‑play* for users of the **VSDSquadron RISC‑V system**.

---

## How to Use the Timer IP (Software View)

All examples use the same memory‑mapped base address:

```c
#define TIMER_BASE   0x00400040
#define TIMER_CTRL   (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD   (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE  (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT   (*(volatile unsigned int *)(TIMER_BASE + 0x0C))
```

The basic usage sequence is always:

1. **Stop the timer** (`CTRL = 0`)
2. **Program LOAD** with a count value
3. **Enable the timer** using `CTRL`
4. **Observe VALUE / STATUS / timeout behavior**

---

## Example 1 – One‑Shot Timer (`timer_test.c`)

### Software Intent

This test demonstrates **one‑shot mode**:

* The timer counts down once
* `timeout` asserts only **one time**
* The timer does **not reload** automatically

### Code Used

<details>
<summary> one shot test (click to expand)</summary>

```c

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

```

</details>


---

## What the User Should Observe (Simulation Waveform)

When viewing the waveform for **one‑shot mode**, the following behavior should be visible:

### 1. Timer Load Phase

* `load_reg` is written with the programmed value
* `value_reg` is initially **0**, then loads `LOAD` when the timer is enabled

**What this means**
The timer correctly captures the software‑written LOAD value before counting starts.

**TIMER_LOAD**
<img width="1598" height="375" alt="loadtimer" src="https://github.com/user-attachments/assets/78d8df5d-2b0a-4125-9f55-293c46e6f164" />

---

### 2. Countdown Phase

* `value_reg` decreases by **1 on each timer tick**
* `timeout` remains **low (0)** during countdown
* CPU performs periodic reads of `VALUE` (`timer_rdata` shows changing values)

**What this means**
The timer core is decrementing correctly and is synchronized with the system clock.

**TIMER_DECREMENT**
<img width="1598" height="375" alt="timerfunctioning" src="https://github.com/user-attachments/assets/1214b75b-402c-43f3-98ae-88e49b1c123d" />

---

### 3. Timeout Event

* `value_reg` transitions from **1 → 0**
* `timeout` signal asserts **high**
* `timer_rdata[0]` reflects STATUS bit = 1

**What this means**
The timer has expired exactly once, which confirms correct one‑shot behavior.

**TIMEOUT_HIGH**
<img width="1317" height="114" alt="timeout_high_timer" src="https://github.com/user-attachments/assets/73b3097b-f7af-4423-b149-ebfbe626e3a7" />

---

### 4. Post‑Timeout State

* `value_reg` stays at **0**
* `timeout` remains **high** until software clears it
* No automatic reload occurs

## key confirmation
- The timer correctly stops after expiration and waits for software action.

---

## Example 2 – Periodic Timer (`timer_periodic.c`)

### code used
<details>
<summary> periodic test (click to expand)</summary>

```c

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

```
</details>

### What the User Should Observe

* Timer repeatedly expires
* `STATUS` bit sets periodically
* Software clears `STATUS` using W1C

**TIMER LOADED**
<img width="1597" height="191" alt="reload_timer_load" src="https://github.com/user-attachments/assets/f0952aa7-10a9-43aa-8009-a6a3cb5d3d14" />
**TIMER DECREMENT**
<img width="1597" height="191" alt="reload_timer_decrement" src="https://github.com/user-attachments/assets/14fa6f59-6fdd-480b-9702-0167ef03f908" />
**TIMER RELOADED**
<img width="1597" height="191" alt="reload_timer_reloaded" src="https://github.com/user-attachments/assets/ddca9c74-dece-4955-b13b-c2c0a487be0f" />
**TIMER READBACK**
<img width="1609" height="217" alt="reload_timer_readback" src="https://github.com/user-attachments/assets/35c47bf4-3b13-4bd1-b818-73b136d13d0d" />
**TIMEOUT_TOGGLE**
<img width="1057" height="194" alt="timeout_toggling" src="https://github.com/user-attachments/assets/9e63e4b8-f3d5-4f56-8832-c5a6f56d7581" />

## Key confirmation
- The timer reloads automatically after every timeout.

---

## Example 3 – Timeout Clear Test (`timer_clear_test.c`)

### code used
<details>
<summary> timer clear test (click to expand)</summary>

```c

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

```
</details>
### What the User Should Observe

1. Timer expires → `STATUS = 1`
2. Software writes `1` to `STATUS`
3. `STATUS` returns to `0`

**TIMER_LOADED**
<img width="1162" height="183" alt="timer_clear_png" src="https://github.com/user-attachments/assets/76d8a85d-ffe3-47fb-9c45-dcd4c1304fc8" />
**TIMER_DECREMENT & TIMEOUT_HIGH**
<img width="1612" height="198" alt="timer_cleaar_decrementing" src="https://github.com/user-attachments/assets/067047e9-a845-4ad1-a56c-c9075d1f5467" />
**TIMEOUT_RESET**
<img width="1612" height="198" alt="timeout_reset" src="https://github.com/user-attachments/assets/2de73b79-b9fa-487b-a403-18459c2394cc" />

## Key confirmation
- Write‑1‑to‑Clear semantics work correctly.

---

## Example 4 – FPGA Hardware Test (`timer_test2.c`)

### code used
<details>
<summary> hardware test (click to expand)</summary>

```c

#include "io.h"

#define TIMER_BASE   0x00400040
#define TIMER_CTRL   (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD   (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_STAT   (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

int main(void)
{
    /* Stop timer */
    TIMER_CTRL = 0;

    /* Load value (adjust if blink is too fast/slow) */
    TIMER_LOAD = 12000000;   // ~1 second at 12 MHz

    /* Enable timer, periodic mode */
    TIMER_CTRL = 0x3;        // bit0=EN, bit1=MODE(periodic)

    while (1) {
        if (TIMER_STAT & 0x1) {
            TIMER_STAT = 0x1;   // clear TIMEOUT (W1C)
        }
    }
}

```
</details>

### What the User Should Observe on Board

* No UART output required
* LED[0] toggles on **each timeout event**
* Blink rate depends on LOAD value

**Visible result**
A stable, periodic LED blink confirms:

* Timer logic is correct
* `timeout_o` is correctly connected
* Clocking and reset are valid in hardware

---

## Common Failure Symptoms & Meaning

| Symptom              | Likely Cause                       |
| -------------------- | ---------------------------------- |
| VALUE never changes  | Timer not enabled (CTRL.EN = 0)    |
| Immediate timeout    | LOAD too small                     |
| No timeout           | STATUS never cleared or EN not set |
| LED toggles randomly | Missing edge detection             |
| UART silent          | Baud mismatch or UART not enabled  |

---

## Summary

This example usage demonstrates:

* **One‑shot operation** (single timeout)
* **Periodic operation** (auto reload)
* **Correct timeout clearing**
* **Hardware‑level validation using LED output**

If the observed behavior matches the descriptions above, the Timer IP is correctly integrated and functioning as intended.


