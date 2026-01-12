# Hardware Timer IP 

- In commercial SoC and FPGA designs, a hardware timer is a foundational infrastructure IP , it is essential for deterministic behavior, functional safety, power management, and long-term maintainability of complex systems.
  
##  Purposes 

- **Deterministic Time Reference**  
   Provides a cycle-accurate and hardware-guaranteed notion of time, independent of software execution paths, interrupts, or pipeline effects.

- **Hardware-Enforced Time Boundaries**  
   Enables strict upper bounds on operation duration, preventing indefinite execution and ensuring bounded latency.

- **Precise Periodic Event Generation**  
   Generates stable, low-jitter periodic timing events required for scheduling and control systems.

- **Low-Power and Idle-State Timekeeping**  
   Maintains accurate time progression while processors are stalled, clock-gated, or in low-power states.

- **Offloading Timing Responsibility from Software**  
   Removes repetitive, timing-critical responsibilities from firmware, increasing overall system robustness.

---

##  Use Cases 

- **Peripheral Communication Timeouts**  
   Detects stalled UART, SPI, I²C, or memory-mapped peripheral transactions.

- **Operating System Scheduler Tick**  
   Serves as the fundamental timing source for RTOS or bare-metal schedulers.

- **Periodic Sampling and Control Systems**  
   Drives sensor sampling, control-loop execution, and monitoring tasks.

- **Safety and Supervision Logic**  
   Acts as a timing reference for watchdogs and fault-detection mechanisms.

- **Non-Blocking Delay and Pacing**  
   Implements precise delays without CPU busy-waiting, enabling efficient multitasking.

---

##  Why the Timer Is Used 

- **Determinism**  
   Hardware timers provide predictable timing unaffected by software variability.

- **System Reliability**  
   Prevents deadlocks and uncontrolled execution paths in failure scenarios.

- **Reduced Firmware Complexity**  
   Eliminates fragile software delay loops and ad-hoc timing constructs.

- **Improved Power Efficiency**  
   Enables sleep-based designs instead of continuous polling.

- **Scalability and Reusability**  
   Provides a reusable, standardized timing primitive across multiple designs and products.

---
## FEATURE SUMMARY

## Supported Modes:

**Timeout Signal Behavior:**  
The `timeout_o` signal is **level-asserted**, remaining high once the timer reaches zero, and is cleared explicitly by software via a write-one-to-clear (W1C) operation to the STATUS register.


The Timer IP supports multiple operating modes to accommodate a wide range of timing requirements in embedded and SoC designs.

### One-Shot Mode
In one-shot mode, the timer counts down from a programmed load value to zero and asserts a timeout event once. After the timeout event:
- The counter stops (or remains at zero).
- No further timeout events occur until the timer is explicitly reloaded and re-enabled.
- This mode is typically used for single-delay events or watchdog-style supervision.

### Periodic (Auto-Reload) Mode
In periodic mode, the timer automatically reloads the programmed load value upon reaching zero:
- Timeout events are generated repeatedly at a fixed interval.
- The interval remains deterministic and independent of CPU software latency.
- This mode is suitable for heartbeat signals, periodic interrupts, or time-sliced task scheduling.

### Prescaled Operation
Both one-shot and periodic modes may optionally operate with a programmable prescaler:
- The prescaler divides the input clock before it reaches the main counter.
- This allows long timing intervals to be achieved without increasing counter width.
- Prescaled operation reduces dynamic switching activity in low-frequency timing applications.

---

##  Bit Widths

The Timer IP uses explicitly defined bit widths to balance flexibility, synthesis efficiency, and portability.

### Counter Width
- The main counter (`VALUE`) is `32 bits` wide.
- This supports long-duration timing intervals even at relatively high clock frequencies.
- The width is suitable for both FPGA and ASIC implementations without modification.

### Prescaler Width
- The prescaler divider is `8 bits` wide.
- This allows clock division factors from 1 to 256.
- The prescaler counter itself is internally sized to ensure correct rollover behavior.

### Control and Status
- Control and status registers are `32 bits` wide.
- Only defined bits are used; unused bits are reserved and read as zero.
- This ensures forward compatibility and ease of software integration.

---

##  Clock Assumptions

**Reset Behavior:**  
The Timer IP uses an **active-low, synchronous reset (`resetn`)**, which must be asserted and deasserted synchronously to the input clock to guarantee deterministic internal state initialization.


The Timer IP is designed to operate synchronously with a single system clock domain.

### Clock Source
- The IP expects a free-running, stable clock input.
- The clock may originate from an on-chip oscillator, PLL, or external source.
- No specific frequency is assumed by the design.

### Synchronous Design
- All internal logic is fully synchronous to the input clock.
- No asynchronous clock crossings are present within the IP.
- Reset is assumed to be synchronous or properly synchronized externally.

### Deterministic Behavior
- Timer resolution is exactly one clock period (or one prescaled tick).
- Timeout events occur at deterministic clock boundaries.
- Behavior is independent of CPU instruction timing or bus access latency.

---

##  Limitations

While the Timer IP is intentionally general-purpose, certain limitations should be considered during system design.

### Single Timer Instance
- Each instance provides one timing channel.
- Multiple independent timers require multiple instances of the IP.

### No Interrupt Controller Integration
- The IP provides a timeout signal and status bit only.
- Interrupt routing, prioritization, or masking must be handled externally.

### No Capture or Compare
- The timer does not support input capture or compare-match outputs.
- It is optimized for countdown-based timing rather than waveform generation.

### Software-Driven Control
- Mode selection, reload, and timeout clearing are software-controlled.
- The IP does not autonomously sequence between different timing profiles.

---
## BLOCK DIAGRAM
<img width="646" height="436" alt="image" src="https://github.com/user-attachments/assets/c5cc46de-91a3-45fd-868b-b6f595c11de0" />


## Software Programming Model — Timer IP

This section describes how software interacts with the Timer IP at a **bare‑metal / low‑level driver** level, following conventions used in commercial SoC peripherals.

---
## HARDWARE USAGE

<img width="691" height="372" alt="hardware_usage" src="https://github.com/user-attachments/assets/55a637bb-39e1-4466-81ea-d600270f34ef" />

##  Software Control Model

The software-visible control mechanisms are:
- **CTRL register** – enables/disables the timer and selects operating mode<

- **LOAD register** – programs the initial countdown value
- **STATUS register** – reports timeout completion and supports W1C clearing
- **VALUE register** – provides visibility into the current countdown value

The timer operates autonomously once enabled; no continuous software intervention is required.

---

##  Typical Initialization Sequence

A recommended initialization sequence ensures deterministic behavior and avoids spurious timeouts.

### Recommended Sequence

- **Disable the timer**
   - Clear `CTRL[0]` (EN = 0)
   - Ensures the timer is not running during configuration

- **Clear pending timeout**
   - Write `1` to `STATUS[0]` (Write‑1‑to‑Clear)
   - Prevents stale timeout flags from previous runs

- **Program LOAD value**
   - Write the desired initial count to `LOAD`
   - This value is latched into the counter when the timer starts

- **Configure CTRL register**
   - Set:
     - `CTRL[1]` → Mode (one‑shot / periodic)
     - `CTRL[2]` → Prescaler enable (if required)
     - `CTRL[15:8]` → Prescaler divider (if enabled)

- **Enable the timer**
   - Set `CTRL[0]` (EN = 1)
   - Timer begins counting on the next clock edge

---

##  Polling vs Status Checking

The Timer IP is designed primarily for **polling-based software control**, making it suitable for:
- Bare‑metal firmware
- Simple RTOS environments
- Deterministic test firmware

### Polling Model

Software repeatedly reads the `STATUS` register until timeout is observed.

```c
while ((TIMER_STATUS & 0x1) == 0) {
    // wait for timeout
}
```

### Timeout Detection

- `STATUS[0] = 1` indicates the timer has expired
- The flag remains **asserted (level-sensitive)** until explicitly cleared by software

---

## Clearing Timeout (W1C Semantics)

The timeout flag uses **Write‑1‑to‑Clear (W1C)** behavior.

### Clearing Procedure

```c
TIMER_STATUS = 0x1;  // clears STATUS[0]
```

Important notes:
- Writing `0` has no effect
- Writing `1` clears the timeout flag
- Other bits are reserved and must be written as `0`

---

##  One‑Shot vs Periodic Software Behavior

### One‑Shot Mode (`CTRL[1] = 0`)
- Timer counts down once
- On expiry:
  - `STATUS[0]` is asserted
  - `VALUE` transitions to `0`
- Software must:
  - Clear `STATUS`
  - Reload and re‑enable timer if reuse is required

### Periodic Mode (`CTRL[1] = 1`)
- Timer automatically reloads `LOAD` on expiry
- `STATUS[0]` asserts on every period
- Software typically:
  - Polls `STATUS`
  - Clears flag after each period
  - Leaves timer continuously enabled

---

##  Recommended Software Practices

- Always clear `STATUS` before enabling the timer
- Avoid reading `VALUE` for control decisions (use `STATUS` instead)
- Do not modify `LOAD` while the timer is running
- Disable the timer before reprogramming configuration fields

---

##  Intended Software Usage Level

This programming model is suitable for:
- Firmware bring‑up
- Hardware validation
- Deterministic timing loops
- Educational and research SoCs

The design intentionally avoids complex interrupt or DMA coupling to maintain clarity and portability.

## Summary

The Timer IP provides a compact, deterministic, and configurable hardware timing block suitable for a wide range of embedded and SoC applications.  
Its design emphasizes clarity, predictability, and ease of reuse, aligning with industry best practices for reusable hardware IP.




