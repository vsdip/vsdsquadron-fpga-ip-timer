# Timer IP – Register Map

### This section describes the memory-mapped register interface of the **Timer IP**,all registers are 32-bit wide and aligned on 32-bit boundaries.

---

## Register Summary

| Offset | Register | Access | Description |
|------:|----------|:------:|-------------|
| 0x00 | CTRL   | R/W | Control register (enable, mode, prescaler control) |
| 0x04 | LOAD   | R/W | Load value (initial / reload count) |
| 0x08 | VALUE  | R   | Current counter value |
| 0x0C | STATUS | R/W1C | Timeout status (write-1-to-clear) |

---

## CTRL Register (Offset 0x00)

**Reset Value:** `0x0000_0000`  
**Access:** Read / Write

| Bit(s) | Name | Description |
|------:|------|-------------|
| [0] | EN | Timer enable. `1` = timer runs, `0` = timer stopped and preloaded |
| [1] | MODE | Operating mode: `0` = one-shot, `1` = periodic |
| [2] | PRESC_EN | Prescaler enable. `1` = prescaler active, `0` = bypass |
| [15:8] | PRESC_DIV | Prescaler divider value |
| [31:16] | RSVD | Reserved, read as 0 |

**Behavior:**
- When `EN=0`, the counter is preloaded with `LOAD`.
- When `EN=1`, the timer counts based on the prescaler configuration.
- MODE controls reload behavior after timeout.

---

## LOAD Register (Offset 0x04)

**Reset Value:** `0x0000_0000`  
**Access:** Read / Write

| Bit(s) | Name | Description |
|------:|------|-------------|
| [31:0] | LOAD | Load value used to initialize or reload the counter |

**Behavior:**
- Loaded into the counter when `EN` transitions from `0 → 1`.
- Reloaded automatically in periodic mode upon timeout.

---

## VALUE Register (Offset 0x08)

**Reset Value:** `0x0000_0000`  
**Access:** Read Only

| Bit(s) | Name | Description |
|------:|------|-------------|
| [31:0] | VALUE | Current counter value |

**Behavior:**
- Reflects the live counter value.
- Read-only; writes are ignored.

---

## STATUS Register (Offset 0x0C)

**Reset Value:** `0x0000_0000`  
**Access:** Read / Write-1-to-Clear (W1C)

| Bit(s) | Name | Description |
|------:|------|-------------|
| [0] | TIMEOUT | Timeout flag. Set when counter expires |
| [31:1] | RSVD | Reserved, read as 0 |

**Behavior:**
- `TIMEOUT` is asserted when the counter reaches expiry.
- Writing `1` to bit[0] clears the timeout flag.
- Writing `0` has no effect.

---

## Notes

- `TIMEOUT` is level-sensitive and remains asserted until cleared.
- All reserved bits should be written as zero.
- Software must clear `STATUS.TIMEOUT` explicitly before re-arming in one-shot mode.


