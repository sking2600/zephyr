# WCH Timer and Power Management Improvement Plan

## Executive Summary

The WCH LPTIM timer driver and power management code have several critical deficiencies that prevent proper operation with Zephyr's tickless kernel and power management features. This document outlines the identified issues and provides a roadmap for addressing them before pushing to the upstream fork.

---

## Critical Deficiencies Identified

### 1. Timer Driver - Missing Tickless Support (CRITICAL)

**File:** [`drivers/timer/wch_lptim_timer.c`](drivers/timer/wch_lptim_timer.c)

The driver claims to be `TICKLESS_CAPABLE` but is missing the essential [`sys_clock_set_timeout()`](drivers/timer/sys_clock_init.c:21) function required for tickless kernel operation.

```c
// Current state - uses weak noop default:
void __weak sys_clock_set_timeout(int32_t ticks, bool idle) { }
```

**Impact:** When `CONFIG_TICKLESS_KERNEL=y`, the kernel cannot schedule wakeups, leading to:
- System never entering low-power states
- Timer interrupts occurring at fixed intervals regardless of actual need
- Increased power consumption

**Reference Implementation:** See [`drivers/timer/stm32_lptim_timer.c`](drivers/timer/stm32_lptim_timer.c:305) lines 305-443 for a complete implementation.

---

### 2. Incomplete `sys_clock_elapsed()` Implementation

**Current Code:**
```c
uint32_t sys_clock_elapsed(void)
{
    if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
        return 0;
    }
    
    /* For now, just return 0 to indicate no extra elapsed time since last announce */
    /* We will implement full tickless support later */
    return 0;  // <-- Always returns 0!
}
```

**Problem:** Always returning 0 breaks the kernel's ability to calculate actual elapsed time between timer announcements, causing timing drift and scheduling issues.

**Required Fix:** Calculate elapsed cycles since last announcement using counter value + accumulated cycles.

---

### 3. Missing Device Tree Binding

The driver uses `DT_DRV_COMPAT wch_ch32_lptim` but no corresponding binding file exists in `dts/bindings/timer/`.

**Required:** Create `dts/bindings/timer/wch,ch32-lptim.yaml` with:
- Clock source selection properties
- Prescaler configuration
- Interrupt configuration
- Compatible with `wch,ch32-lptim`

---

### 4. Direct RCC Register Manipulation

**Current Code:**
```c
/* Enable LPTIM clock in RCC */
RCC->APB1PCENR |= RCC_PB1Periph_LPTIM;
```

**Problem:** Bypasses Zephyr's clock control API, breaking:
- Clock tree management
- Power state transitions
- Device runtime PM

**Required Fix:** Use `clock_control_on()` and `clock_control_get_rate()` like STM32 driver.

---

### 5. Power Management - Empty `pm_state_exit_post_ops()`

**File:** [`soc/wch/ch32v/qingke_v4c/soc.c`](soc/wch/ch32v/qingke_v4c/soc.c:56)

```c
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);
    // Empty! No clock restoration, no timer restart
}
```

**Problem:** After waking from sleep:
- Clocks may not be restored to pre-sleep state
- Timer may not be reconfigured correctly
- System may hang or drift

---

### 6. Improper Standby Mode Implementation

**Current Code:**
```c
case PM_STATE_STANDBY:
    /* Enter Standby mode */
    /* Set SLEEPDEEP bit in System Control Register (handled by arch if configured?)
     * For now, only WFI is safe without more complex context save/restore
     */
     __asm__ volatile("wfi");  // <-- Just WFI, no SLEEPDEEP!
    break;
```

**Problems:**
1. Comment admits confusion about SLEEPDEEP bit handling
2. WFI without SLEEPDEEP doesn't enter standby, just sleep
3. No context save/restore for standby mode
4. If standby isn't supported, it shouldn't be advertised

---

### 7. Race Condition in ARR Updates

**Current Code:**
```c
/* Wait for ARROK */
uint32_t timeout = 100000;
while (!(LPTIM->ISR & LPTIM_ISR_ARROK) && timeout--);
```

**Problems:**
1. No `LPTIM_GUARD_VALUE` to prevent setting ARR too close to counter
2. Timeout has no failure handling - continues even if ARR wasn't set
3. No spinlock protection during timeout calculation

**Reference:** STM32 driver uses 2-cycle guard value and proper locking.

---

### 8. Counter Driver Const-Correctness Violation

**File:** [`drivers/counter/counter_wch_lptim.c`](drivers/counter/counter_wch_lptim.c:225)

```c
/* Frequency */
((struct counter_lptim_config *)config)->info.freq = base_freq / config->prescaler;
```

**Problem:** Casts away const to modify config at runtime - undefined behavior.

---

### 9. Hardcoded Clock Frequency

**Current Code:**
```c
#if DT_NODE_EXISTS(DT_NODELABEL(clk_lsi))
#define LSI_FREQUENCY DT_PROP(DT_NODELABEL(clk_lsi), clock_frequency)
#else
#warning "LSI clock node not found in DTS, defaulting to 40kHz"
#define LSI_FREQUENCY 40000  // <-- Hardcoded!
#endif
```

**Problem:** Doesn't use `sys_clock_hw_cycles_per_sec()` or clock control API.

---

## Implementation Roadmap

### Phase 1: Critical Fixes (Required for Basic Functionality)

1. **Create Device Tree Binding**
   - Create `dts/bindings/timer/wch,ch32-lptim.yaml`
   - Define clock source, prescaler, and interrupt properties

2. **Implement `sys_clock_set_timeout()`**
   - Support tickless kernel scheduling
   - Handle ARR updates with proper synchronization
   - Add `LPTIM_GUARD_VALUE` (2 cycles minimum)

3. **Complete `sys_clock_elapsed()`**
   - Calculate actual elapsed cycles
   - Handle counter wraparound

4. **Fix Power Management**
   - Implement `pm_state_exit_post_ops()` with clock restoration
   - Either properly implement standby or remove support
   - Add SLEEPDEEP bit handling

### Phase 2: Code Quality Improvements

5. **Clock Control Integration**
   - Replace direct RCC access with `clock_control_on()`/`get_rate()`
   - Add runtime clock configuration support

6. **Counter Driver Fixes**
   - Fix const-correctness issue
   - Add spinlock protection
   - Add timeout failure handling

7. **Add Prescaler Support**
   - Allow flexible tick rates
   - Validate frequency vs tick rate compatibility

### Phase 3: Testing & Validation

8. **Test Matrix**
   - Tickless kernel enabled/disabled
   - Power management enabled/disabled  
   - Different clock sources (LSI, LSE, PCLK)
   - Various prescaler values

---

## Key Design Decisions

### LPTIM Clock Source Strategy

| Source | Frequency | Power | Accuracy | Use Case |
|--------|-----------|-------|----------|----------|
| LSI | ~40kHz | Low | Medium | Default, sleep wakeup |
| LSE | 32.768kHz | Very Low | High | RTC sync, precise timing |
| PCLK | System | High | High | High-res when not sleeping |
| HSI | 8MHz | Medium | Medium | Fast wakeup |

**Recommendation:** Default to LSI for tick source (runs in sleep), allow LSE for better accuracy.

### ARR Update Synchronization

The WCH LPTIM requires special handling when updating ARR:

```c
// Pattern from STM32 driver (recommended approach):
1. Check if ARR update is possible (not too close to counter)
2. Clear ARROK flag
3. Write new ARR value  
4. Wait for ARROK flag (hardware confirms sync)
5. If ARRM pending, handle it
```

### PM State Handling

For WCH CH32V series:
- **SUSPEND_TO_IDLE:** WFI is sufficient, LPTIM can wake
- **STANDBY:** Requires SLEEPDEEP + context save/restore
  - If full context save not implemented, don't advertise STANDBY support
  - Or use STM32 pattern with backup register storage

---

## Reference Implementation Comparison

### STM32 LPTIM (Good Example)
- ✅ Full tickless support with `sys_clock_set_timeout()`
- ✅ Clock control API integration
- ✅ Autonomous mode for sleep
- ✅ Standby timer for deep sleep
- ✅ Comprehensive ARR synchronization

### WCH LPTIM (Current State)
- ❌ No tickless support
- ❌ Direct register access
- ❌ Missing binding
- ❌ Incomplete PM integration
- ❌ Race conditions in ARR updates

---

## Files to Modify

| File | Changes |
|------|---------|
| `drivers/timer/wch_lptim_timer.c` | Add `sys_clock_set_timeout()`, fix `sys_clock_elapsed()`, add clock control API |
| `drivers/counter/counter_wch_lptim.c` | Fix const-correctness, add spinlocks |
| `soc/wch/ch32v/qingke_v4c/soc.c` | Complete PM implementation |
| `dts/bindings/timer/wch,ch32-lptim.yaml` | Create new binding |
| `drivers/timer/Kconfig.wch_lptim` | Add prescaler and clock source options |

---

## Acceptance Criteria

Before pushing to upstream fork:

1. [ ] `samples/basic/blinky` works with `CONFIG_TICKLESS_KERNEL=y`
2. [ ] `tests/kernel/tickless/tickless` passes
3. [ ] `tests/subsys/pm/power_mgmt` passes  
4. [ ] Timing accuracy verified with logic analyzer (<1% drift)
5. [ ] Sleep/wake cycles work correctly with timer wakeups
6. [ ] All builds pass with `-Werror -Wall`
