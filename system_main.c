/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * System entry point, exception handlers, scheduler primitives, and boot sequence.
 *
 * Target: ARM Cortex-M4 (Thumb-2), no OS, bare-metal.
 *
 * The chip operates as an HDMI 2.1 repeater/retimer between a source and a
 * sink.  On power-on the firmware:
 *   1. Seeds the PRNG and initialises global state (system_early_init).
 *   2. main() runs a one-time hardware bring-up sequence.
 *   3. Enters a permanent polling/event loop that services video, audio,
 *      eARC, SCDC, CEC and HDCP state machines.
 *
 * The scheduler is extremely lightweight — a single "current task" word and
 * a PendSV-triggered context switch driven by yield_execution().
 */

#include "include/defs.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Forward declarations — functions implemented in this file
 * ====================================================================== */

static void     prng_lfg_advance(void);        /* internal helper */

/* =========================================================================
 * Forward declarations — functions implemented in other modules
 * ====================================================================== */

/* hw_misc.c */
int  hw_misc_init_state_1(void);
int  hw_misc_set_state_4(void);
int  hw_misc_set_state_5(void);
int  hw_misc_set_state_6(void);
int  hw_misc_set_state_7(void);
int  hw_misc_set_state_9(void);
int  hw_misc_set_state_10(void);
int  hw_misc_set_state_21(void);
int  hw_misc_set_state_25(void);
int  hw_misc_set_state_26(void);
int  hw_misc_set_state_29(void);
int  hw_misc_set_state_31(void);
int  hw_misc_set_state_34(void);
int  hw_misc_set_state_35(void);
int  hw_misc_set_state_37(void);
int  hw_misc_set_state_50(void);
int  hw_misc_set_state_59(void);
int  hw_misc_set_state_64(void);
int  hw_misc_set_state_73(void);
int  hw_misc_set_state_1(void);
int  hw_misc_init_state_3(void);
int  hw_misc_init_state_6(void);
int  hw_misc_return_state_2(void);
int  hw_misc_process_state_3(void);
int  hw_misc_process_state_4(void);
int  hw_misc_process_state_6(void);
int  hw_misc_process_state_12(void);
int  hw_misc_process_state_24(void);
int  hw_misc_process_state_25(void);
int  hw_misc_process_state_33(uint16_t cap_byte);
int  hw_misc_process_state_34(void);
int  hw_misc_process_state_35(void);
int  hw_misc_process_state_38(int flags);
int  hw_misc_process_state_39(void);
int  hw_misc_process_state_43(void);
int  hw_misc_process_state_44(void);
int  hw_misc_process_state_55(int a1, int a2, int a3);
int  hw_misc_process_state_59(void);
int  hw_misc_process_state_86(void);
int  hw_misc_process_state_107(int cfg_reg);
int  hw_misc_process_state_109(void);
int  hw_misc_process_state_124(int flags, int hpd_state);
int  hw_misc_process_state_146(void);
int  hw_misc_process_state_147(void);
int  hw_misc_set_state_28(uint32_t cfg);
int  hw_misc_set_state_50(void);

/* flash_nvram.c */
int  flash_efface_firmware_update(int flags);

/* crypto_hdcp.c */
int  crypto_init_and_parse_im4m(int a1, int a2, int a3, int a4);

/* hdmi_frl_video.c */
int  cec_irq_handler(void);
int  frl_reset_state_6(void);
int  video_set_state_10(void);
int  video_check_state_20(void);
int  video_set_hw_state_21(int hpd_level);
int  video_set_hw_state_26(void);
int  video_get_hw_state_27(int channel);
int  video_get_hw_state_28(int channel);
int  video_get_hw_state_31(int channel);
int  video_check_state_38(void);
bool video_check_state_39(void);
int  video_process_event_queue(int a1, int a2, int a3);
int  video_init_hw_state_65(void);
int  video_process_state_84(void);
int  video_init_hw_state_90(void);
int  video_timer_update_95(int a1, int a2, int a3);
int  video_set_hw_state_100(void);
int  video_process_state_113(int code, int fatal);
int  video_set_state_123(void);
int  video_set_state_124(void);
int  video_get_state_125(int base, int offset);
int  video_state_machine_3(int a1, int a2, int a3);
int  video_process_state_164(int a1, int a2, int a3);
int  video_init_state_165(void);
int  video_set_hw_state_177(void);
int  video_state_machine_4(int a1, int a2, int a3);

/* audio_earc.c */
int  audio_timer_update(int a1, int a2, int a3);
int  audio_check_format_change(void);
int  audio_set_hw_state_1(int enable);
int  audio_process_state_33(void);
int  audio_init_state_97(void);
int  audio_state_machine_1(void);

/* i2c_scdc.c */
int  scdc_periodic_task(void);

/* string_utils.c */
char *check_null_string(int unused, uint32_t *str);

/* libc_printf.c */
int  custom_printf(const char *fmt, ...);

/* =========================================================================
 * PRNG — Lagged Fibonacci Generator (LFG/ALFG)
 *
 * Parameters: m=2^31, lags (j,k) = (24,55) — the classical Knuth sequence.
 * The ring buffer lives at PRNG_BUF_BASE (0x41D40), 55 uint32_t entries.
 * Two pointers (ptr_a, ptr_b) walk the ring; each call produces:
 *   out = buf[ptr_b] + buf[ptr_a];   buf[ptr_b] = out;
 *   advance both pointers (wrapping at ring end).
 * Returns the 31-bit value (MSB cleared for positive range).
 * ====================================================================== */

/*
 * prng_seed_lcg() — initialise the LFG ring buffer using an LCG.
 *
 * The LCG used is  x_{n+1} = 69069 * x_n + 1725307361  (mod 2^32),
 * seeding all 55 slots in reverse order.  The HIWORD XOR ensures the
 * upper 16 bits contribute to the seed spread even on a 32-bit result.
 *
 * After filling the ring, ptr_a is set to the last element
 * (buf[54] = 0x41D40 + 54*4 = 0x41E08 → sentinel 0x41E1C),
 * ptr_b is set to buf[0] (= 0x41D40 → sentinel 0x41D40 = 269632).
 */
uint32_t prng_seed_lcg(uint32_t seed)
{
    int i = 55;

    /* Reset ring-pointer sentinels */
    REG_PRNG_PTR_A = 269756u;   /* 0x41D40 + 54*4 + 4 = 0x41E1C — one-past last slot */
    REG_PRNG_PTR_B = 269632u;   /* 0x41D40 — first slot                               */

    while (i-- > 0) {
        /* Store seed-derived value into slot i */
        *(volatile uint32_t *)(4u * i + PRNG_BUF_BASE) = seed + (seed >> 16);
        /* Advance LCG */
        seed = 69069u * seed + 1725307361u;
    }
    return seed;
}

/*
 * prng_init_default() — seed the PRNG with the fixed default seed (1).
 */
uint32_t prng_init_default(void)
{
    return prng_seed_lcg(1u);
}

/*
 * prng_get_next() — pull one 31-bit pseudo-random value from the LFG ring.
 *
 * The two lag pointers walk a circular buffer of 55 uint32_t words:
 *   value = *ptr_b + *ptr_a;   *ptr_b = value;
 * Each pointer advances by one word, wrapping at the ring sentinel (0x41E1C).
 */
uint32_t prng_get_next(void)
{
    volatile uint32_t *ptr_b = (volatile uint32_t *)REG_PRNG_PTR_A; /* pointer to lag-b slot */
    volatile uint32_t *ptr_a = (volatile uint32_t *)REG_PRNG_PTR_B; /* pointer to lag-a slot */

    uint32_t val = *ptr_b + *ptr_a;
    *ptr_b = val;

    /* Advance ptr_b */
    uint32_t next_b = REG_PRNG_PTR_A + 4u;
    if (next_b >= PRNG_RING_END)
        next_b = PRNG_BUF_BASE;

    /* Advance ptr_a */
    uint32_t next_a = REG_PRNG_PTR_B + 4u;
    if (next_a >= PRNG_RING_END)
        next_a = PRNG_BUF_BASE;

    REG_PRNG_PTR_A = next_b;
    REG_PRNG_PTR_B = next_a;

    return val & 0x7FFFFFFFu;   /* mask to 31 bits — keep MSB clear */
}

/* =========================================================================
 * Utility — compile-time magic constant
 *
 * Returned as a 64-bit value. The two halves are:
 *   high = 0x00043E88 (clock/divider constant for 67.5 MHz system clock)
 *   low  = 0x00041E88 (PRNG ring-buffer base + sizeof ring)
 * Used by the audio subsystem for PLL ratio calculations.
 * ====================================================================== */
uint64_t get_magic_constant(void)
{
    return 0x00043E8800041E88ULL;
}

/* =========================================================================
 * Global state accessors
 *
 * The firmware keeps a small set of global structs in SRAM.
 * These functions return the base addresses; they are called through
 * function pointers in a few places to allow ROM-patch overrides.
 * ====================================================================== */

/*
 * get_global_struct_ptr() — return the address of the primary global state
 * struct (product model string, capability flags, etc.).
 */
uint32_t get_global_struct_ptr(void)
{
    return GLOBAL_STRUCT_BASE;  /* 0x41E84 = 269892 */
}

/*
 * get_global_state_ptr() — return the address of the secondary global state
 * block (task/power-state tracking).
 */
uint32_t get_global_state_ptr(void)
{
    return GLOBAL_STATE_BASE;   /* 0x41E64 = 269860 */
}

/* =========================================================================
 * Arithmetic utility
 *
 * fast_div_by_10() — branchless unsigned 64-bit division by 10 using the
 * multiply-high reciprocal method.  Used by the printf float formatter.
 * ====================================================================== */
uint64_t fast_div_by_10(uint64_t n)
{
    uint64_t q;

    /* Reciprocal multiply: q ≈ n * (2^35 / 10) >> 35  [error ≤ 1 for n < 2^32] */
    q  = n - (n >> 2);
    q  = q + (q >> 4);
    q  = q + (q >> 8);
    q += (q >> 16);
    q += (q >> 32);
    q >>= 3;

    /* Correct any off-by-one */
    if ((n - 10u * q) >= 10u)
        ++q;

    return q;
}

/* =========================================================================
 * Cortex-M scheduler primitives
 *
 * The scheduler is single-level: the "current task" identifier is stored in
 * REG_TCB_CURRENT_TASK.  Cooperative yield is achieved by:
 *   1. Recording whether a task is ready (REG_TCB_TASK_READY).
 *   2. Setting the PendSV pending bit so the exception fires after the
 *      current exception context returns.
 *   3. The PendSV handler (in ROM) updates REG_TCB_CURRENT_TASK.
 * ====================================================================== */

/*
 * trigger_pendsv() — set the PendSV pending bit in ICSR.
 * Returns the ICSR value after the write (used as an implicit fence).
 */
int trigger_pendsv(void)
{
    SCB_ICSR = SCB_ICSR_PENDSVSET;
    return (int)&SCB_ICSR;  /* 0xE000ED04 — caller treats as opaque handle */
}

/*
 * set_system_priority() — set PendSV / SysTick to the lowest priority (0xFF)
 * so that all hardware IRQs preempt the scheduler.
 */
int set_system_priority(void)
{
    SCB_SHPR3 = 0xFFu;          /* PendSV priority = lowest (255) */
    return (int)&SCB_SHPR3;     /* 0xE000ED22 */
}

/*
 * yield_execution() — cooperative yield point.
 *
 * Snapshots the current task ID, signals "ready" based on whether any task
 * is waiting, then triggers PendSV.  If the task ID changed after PendSV
 * returns (i.e. a context switch occurred and the stack canary was touched),
 * we halt — this indicates a stack corruption / task overrun.
 */
int yield_execution(void)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;

    REG_TCB_YIELD_FLAG = (REG_TCB_TASK_READY == 0) ? 1u : 0u;
    trigger_pendsv();

    if (REG_TCB_CURRENT_TASK != saved_task)
        system_halt_clear_flag();   /* stack smash / task corruption detected */

    return (int)saved_task;
}

/*
 * get_system_status() — return the current task/system state identifier.
 */
int get_system_status(void)
{
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * delay_loop() — busy-wait for approximately 'ms' milliseconds.
 *
 * The loop constant (27 iterations per unit) was calibrated against the
 * 67.5 MHz system clock with the Cortex-M4 pipeline in mind.
 * Returns the current task ID (used as a fence to prevent optimisation).
 */
int delay_loop(int ms)
{
    volatile uint32_t i = 0;
    uint32_t limit = 27u * (uint32_t)ms;
    do { ++i; } while (i < limit);
    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * Bit-count utility
 *
 * count_leading_zeros_in_array() — count the number of leading zero bits
 * across the first `len` bytes of the firmware capability buffer at
 * DATA_LEADING_ZEROS_BUF.  Scans byte-by-byte, bit-by-bit from MSB,
 * stopping at the first set bit.
 * Used by the HDCP/certificate parser to determine key length in bits.
 * ====================================================================== */
int count_leading_zeros_in_array(uint32_t len)
{
    uint32_t byte_idx  = 0;
    int      bit_count = 0;
    bool     found     = false;

    while (byte_idx < len && !found) {
        uint8_t bval = *(volatile uint8_t *)((uintptr_t)byte_idx + DATA_LEADING_ZEROS_BUF);
        uint8_t bit  = 0;
        while (((bval << bit) & 0x80u) == 0u) {
            bit_count = (uint8_t)(bit_count + 1);
            bit       = (uint8_t)(bit + 1);
            if (bit >= 8u)
                goto next_byte;
        }
        found = true;
next_byte:
        byte_idx = (uint8_t)(byte_idx + 1);
    }
    return bit_count;
}

/* =========================================================================
 * Exception / fault handlers
 * ====================================================================== */

/*
 * system_early_init() — called from the reset vector before main().
 *
 * Seeds the PRNG with the default seed and initialises the global struct's
 * model-string pointer to a safe default (so early printf calls don't crash
 * if no model string has been provisioned yet).
 */
void system_early_init(void)
{
    prng_init_default();

    uint32_t gs = get_global_struct_ptr();
    /* Slot +12: model string pointer — initialise to default empty string  */
    *(volatile uint32_t *)(gs + 12u) = (uint32_t)check_null_string(0, NULL);
}

/*
 * empty_stub() — intentional no-op placeholder.
 *
 * Called by system_halt_with_stub() to give a debugger a safe breakpoint
 * target before the chip is halted.  May be patched by ROM extensions.
 */
void empty_stub(void)
{
    /* Intentionally empty */
}

/*
 * system_breakpoint_halt() — trigger a BKPT and spin forever.
 *
 * BKPT 0xAB is the ARM semihosting breakpoint; when no debugger is attached
 * the processor generates a HardFault and we spin in the fault handler.
 * With a debugger attached this gives a clean halt with preserved registers.
 */
__attribute__((noreturn)) void system_breakpoint_halt(void)
{
    __breakpoint(0xAB);         /* BKPT #171 — ARM semihosting vector */
    for (;;) ;
}

/*
 * system_halt_with_stub() — call the empty stub then halt.
 *
 * The stub call is a deliberate hook: ROM patches or a debugger can redirect
 * empty_stub() to dump diagnostics before the chip stops.
 */
__attribute__((noreturn)) void system_halt_with_stub(int fault_type, int fault_info)
{
    (void)fault_type;
    (void)fault_info;
    empty_stub();
    system_breakpoint_halt();
}

/*
 * system_halt_wrapper() — thin wrapper so callers in ROM can reach
 * system_halt_with_stub() through a stable ABI.
 */
__attribute__((noreturn)) void system_halt_wrapper(int fault_type, int fault_info)
{
    system_halt_with_stub(fault_type, fault_info);
}

/*
 * system_halt_clear_flag() — disable the run-enable clocks then spin.
 *
 * Called when a stack canary mismatch is detected (yield_execution,
 * simple_memset, etc.) — the chip stops processing but does not reset,
 * allowing a debugger to inspect the state.
 * The two clock-enable bits (bit 23) are cleared so the video and audio
 * output paths go silent rather than driving undefined data.
 */
__attribute__((noreturn)) void system_halt_clear_flag(void)
{
    REG_CLK_EN_A &= ~REG_CLK_RUN_BIT;
    REG_CLK_EN_B &= ~REG_CLK_RUN_BIT;
    for (;;) ;
}

/*
 * system_fatal_halt() — assert a hard reset / watchdog via the system
 * control register, then spin.
 *
 * hw_misc_set_state_5() gates off the output buffers first so the HDMI
 * sink sees a clean HPD deassert before the chip stops.
 */
__attribute__((noreturn)) void system_fatal_halt(void)
{
    hw_misc_set_state_5();
    REG_SYS_CTRL |= REG_SYS_CTRL_HALT;
    for (;;) ;
}

/*
 * infinite_loop() — bare infinite spin, used as a fallback unreachable stub.
 */
__attribute__((noreturn)) void infinite_loop(void)
{
    for (;;) ;
}

/* -------------------------------------------------------------------------
 * Fault dump helpers
 *
 * system_crash_dump_and_halt() is the common tail for all fault handlers.
 * It writes a crash record to the dedicated SRAM crash region, sends a
 * "fatal error" display code to the video subsystem (so the HDMI sink may
 * show a diagnostic screen), flushes any pending hardware state, then calls
 * system_fatal_halt().
 *
 * The crash-region layout:
 *   0x44C00  : magic = 0x55AA55F0   (marks the record as valid across resets)
 *   0x44C04  : fault_type           (0=NMI, 1=MemManage, 2=BusFault,
 *                                    3=HardFault, 4=UsageFault)
 *   0x44C05  : CFSR / HFSR snapshot
 *   0x44C09  : BFAR / MMFAR snapshot
 *   0x44C0D  : sp_frame pointer (address of the exception stack frame)
 *   0x44C11  : saved PC  (sp_frame[5] = exception frame offset +20)
 *   0x44C15  : saved LR  (sp_frame[6] = exception frame offset +24)
 * ---------------------------------------------------------------------- */

__attribute__((noreturn)) void system_crash_dump_and_halt(int sp_frame)
{
    /* Mark crash record valid */
    CRASH_MAGIC    = CRASH_MAGIC_VALUE;
    CRASH_SP_FRAME = (uint32_t)sp_frame;

    /* Save the faulting PC and LR from the exception stack frame */
    CRASH_SAVED_PC = *(uint32_t *)(sp_frame + 20);  /* EXC frame: r0,r1,r2,r3,r12,lr,pc,xpsr */
    CRASH_SAVED_LR = *(uint32_t *)(sp_frame + 24);

    /* Tell the video subsystem to display an error overlay (code 16) */
    hw_misc_init_state_1();
    video_process_state_113(16, 1);

    /* Flush hardware state / power down gracefully */
    hw_misc_process_state_25();

    /* Cancel any pending firmware-exec override */
    REG_FW_JUMP_ADDR = 0u;

    system_fatal_halt();
}

/*
 * hard_fault_handler() — HardFault exception entry.
 *
 * Records fault type 3, captures BFAR (the faulting bus address) and CFSR
 * (combined fault status), then dumps and halts.
 */
__attribute__((noreturn)) void hard_fault_handler(int sp_frame)
{
    CRASH_FAULT_TYPE    = 3u;
    CRASH_CFSR_OR_FLAGS = (uint32_t)SCB_BFAR;      /* Bus Fault Address Register */
    CRASH_BFAR_OR_PC    = (uint32_t)SCB_CFSR;      /* Combined Fault Status      */
    system_crash_dump_and_halt(sp_frame);
}

/*
 * mem_manage_handler() — MemManage (MPU) exception entry.
 *
 * Records fault type 1 and the MemManage fault status register.
 */
__attribute__((noreturn)) void mem_manage_handler(int sp_frame)
{
    CRASH_FAULT_TYPE    = 1u;
    CRASH_CFSR_OR_FLAGS = (uint32_t)SCB_HFSR;      /* HardFault Status (encodes MemManage escalation) */
    system_crash_dump_and_halt(sp_frame);
}

/*
 * bus_fault_handler() — BusFault exception entry.
 *
 * Records fault type 2, BFAR, and CFSR.
 */
__attribute__((noreturn)) void bus_fault_handler(int sp_frame)
{
    CRASH_FAULT_TYPE    = 2u;
    CRASH_CFSR_OR_FLAGS = (uint32_t)SCB_MMFAR;     /* MemManage Fault Address */
    CRASH_BFAR_OR_PC    = (uint32_t)SCB_CFSR;
    system_crash_dump_and_halt(sp_frame);
}

/*
 * usage_fault_handler() — UsageFault exception entry.
 *
 * Records fault type 4 and CFSR.
 */
__attribute__((noreturn)) void usage_fault_handler(int sp_frame)
{
    CRASH_FAULT_TYPE    = 4u;
    CRASH_CFSR_OR_FLAGS = (uint32_t)SCB_CFSR;
    system_crash_dump_and_halt(sp_frame);
}

/*
 * nmi_or_generic_fault_handler() — NMI / unhandled exception entry.
 *
 * NMIs on this chip can be triggered by:
 *   - bit 0 (REG_MISC_IRQ_WDT_BIT): internal watchdog timeout.
 *   - bit 1 (REG_MISC_IRQ_OSC_BIT): oscillator failure / clock loss.
 * Both sources are acknowledged by writing a reload value to the watchdog
 * counter and re-asserting the flag bit, then we fall through to the crash
 * dump.  This gives the watchdog one more cycle to complete the reset if the
 * crash handler itself hangs.
 *
 * Uses a different set of crash slots (NMI_PC / NMI_LR) so that the normal
 * HardFault record at 0x44C11 is not overwritten.
 */
__attribute__((noreturn)) void nmi_or_generic_fault_handler(int sp_frame)
{
    /* Acknowledge oscillator-fail IRQ if asserted */
    if ((REG_MISC_IRQ_STATUS & REG_MISC_IRQ_OSC_BIT) != 0u) {
        REG_MISC_IRQ_RELOAD = REG_MISC_WDT_KICK_VAL;
        REG_MISC_IRQ_STATUS |= REG_MISC_IRQ_OSC_BIT;
    }

    /* Acknowledge watchdog-timeout IRQ if asserted */
    if ((REG_MISC_IRQ_STATUS & REG_MISC_IRQ_WDT_BIT) != 0u) {
        REG_MISC_IRQ_RELOAD = REG_MISC_WDT_KICK_VAL;
        REG_MISC_IRQ_STATUS |= REG_MISC_IRQ_WDT_BIT;
    }

    CRASH_FAULT_TYPE = 0u;                          /* 0 = NMI */
    CRASH_MAGIC      = CRASH_MAGIC_VALUE;
    CRASH_SP_FRAME   = (uint32_t)sp_frame;

    /* Saved LR is at frame offset +24, PC at +20 — note swapped slots vs normal */
    CRASH_NMI_PC = *(uint32_t *)(sp_frame + 24);   /* 0x44C13 */
    CRASH_NMI_LR = *(uint32_t *)(sp_frame + 20);   /* 0x44C17 */

    /* Display NMI error overlay (code 17 = NMI/clock-loss) */
    video_process_state_113(17, 1);
    hw_misc_init_state_1();
    hw_misc_process_state_25();

    REG_FW_JUMP_ADDR = 0u;
    system_fatal_halt();
}

/* =========================================================================
 * main() — hardware bring-up and main event loop
 *
 * Execution flow:
 *
 * Phase 1 — Essential hardware initialisation (before HPD is asserted)
 *   - Configure clocks, PHY, output drivers.
 *   - Read HPD input state so we know if a sink was already present
 *     at power-on (cold plug vs warm plug).
 *   - Initialise all subsystem state machines (video, audio, FRL, SCDC).
 *   - Load and verify the firmware image (signature check via IMG4/IM4M).
 *   - Authenticate HDCP keys.
 *
 * Phase 2 — HPD assertion
 *   - Assert HPD to the HDMI source to signal the retimer is ready.
 *   - Wait for the HDMI PHY to lock (REG_HDMI_PHY_STATUS == 0xAA).
 *   - If the source was already active (DVI mode), poll until FRL link is up.
 *
 * Phase 3 — Main event loop (runs forever)
 *   - Poll hardware IRQ summary registers.
 *   - Run all state machines in round-robin order.
 *   - Sleep with WFI between iterations to save power.
 *   - Log state changes for debugging via custom_printf().
 * ====================================================================== */
int main(void)
{
    bool hpd_changed_during_boot;
    int  hpd_state_at_boot;        /* non-zero if HPD was asserted at power-on */

    /* Unused multi-return values from functions that carry outputs in r1-r3 */
    int unused_r1, unused_r2, unused_r3;

    /* ---------------------------------------------------------------
     * Phase 1a: clock / PHY bring-up
     * ------------------------------------------------------------ */

    hpd_changed_during_boot = false;

    /*
     * Snapshot the live clock frequency into the working register and the
     * global struct PRNG constant.  From this point the firmware runs at
     * the frequency reported by the boot ROM.
     */
    REG_GLOBAL_STRUCT_CLK = REG_TCB_CLOCK_COPY;   /* MEMORY[0x41E88] = MEMORY[0x4019C] */
    REG_BOOT_FLAGS         = 0u;                   /* clear boot-phase flag              */

    /* Configure the main PHY / serialiser: 0xC00 = default lane config */
    hw_misc_set_state_28(0xC00u);

    /* Clear HDMI soft-reset bit */
    REG_HDMI_RESET &= ~0x40u;

    /* De-assert HPD output so the source sees us as disconnected during init */
    video_set_hw_state_21(0);

    /* Set HDMI control byte 1 to default mode 4 */
    REG_HDMI_CTRL_1 = 4u;

    /* Power up the retimer output stage */
    hw_misc_set_state_29();

    /*
     * Sample HPD input from the HDMI sink.
     * Returns non-zero if a sink is connected; save for later comparison.
     */
    hpd_state_at_boot = video_init_hw_state_65();

    /* Release the output driver and configure the I/O mux */
    hw_misc_process_state_39();

    /* Configure video path default parameters */
    video_set_state_123();
    video_set_hw_state_100();

    /* Enable the clock/power rails for video and audio PHY */
    hw_misc_set_state_35();
    hw_misc_set_state_34();

    /* Disable any spurious audio interrupts during init */
    hw_misc_process_state_44();

    /* Clear HPD debounce timer */
    REG_HPD_CTRL = 0u;

    /*
     * Verify that the HDMI block is ready to accept configuration.
     * If not (hardware fault), spin forever — nothing we can do.
     */
    if (!video_check_state_39()) {
        for (;;) ;
    }

    /* Enable PHY PLL and wait for it to lock */
    hw_misc_process_state_86();

    /* Clear PHY PLL control register */
    REG_PHY_PLL_CTRL = 0u;

    /* Enable FRL link detection interrupt */
    REG_HDMI_FRL_STATUS = 0x2000000u;

    /* Set up the 40-kHz reference clock for HDCP/CEC */
    hw_misc_process_state_38(0x40000);

    /* ---------------------------------------------------------------
     * Phase 1b: latch the actual operating clock
     * ------------------------------------------------------------ */
    /*
     * At this point the PLL is running; read the clock from the hardware
     * and propagate it to all consumers: the global struct, the TCB, and
     * the NVRAM model-ID register.
     */
    REG_TCB_CLOCK_COPY     = REG_TCB_CLOCK_HZ;    /* 0x4019C = 0x40194 */
    REG_GLOBAL_STRUCT_CLK  = REG_TCB_CLOCK_HZ;    /* 0x41E88 = 0x40194 */
    REG_NVRAM_MODEL_ID     = REG_TCB_CLOCK_HZ;    /* 0x406D4 = 0x40194 */

    /* Program the serialiser/equaliser with the actual clock */
    hw_misc_set_state_37();

    /* Latch clock into the task control block */
    REG_TCB_CURRENT_TASK = REG_TCB_CLOCK_SHADOW;  /* 0x401A0 = 0x40198 */

    /* ---------------------------------------------------------------
     * Phase 1c: subsystem state machine initialisation
     * ------------------------------------------------------------ */
    hw_misc_process_state_3();
    video_init_hw_state_90();
    video_init_state_165();
    hw_misc_return_state_2();
    audio_init_state_97();
    frl_reset_state_6();
    hw_misc_init_state_3();
    video_set_state_10();

    /* Configure FRL training parameters using the detected HPD state */
    hw_misc_process_state_124(0x40000, hpd_state_at_boot);

    /* ---------------------------------------------------------------
     * Phase 1d: interrupt and priority configuration
     * ------------------------------------------------------------ */
    hw_misc_process_state_35();
    hw_misc_set_state_10();
    hw_misc_set_state_9();
    hw_misc_set_state_59();
    video_set_hw_state_26();

    /* Enable the video-active flag */
    REG_VIDEO_FLAGS |= 0x8000000u;

    /* Read back HPD to detect changes since power-on */
    video_check_state_38();

    /* Enable the interrupt controller for video/audio events */
    hw_misc_set_state_21();

    /* Lower PendSV to lowest priority so HW IRQs always preempt the loop */
    set_system_priority();

    /* ---------------------------------------------------------------
     * Phase 1e: firmware integrity + HDCP key validation
     * ------------------------------------------------------------ */
    /*
     * flash_efface_firmware_update() checks whether a pending firmware
     * update exists in the secondary flash bank; if so it applies it and
     * returns the new image base address, otherwise it returns the primary
     * image address.
     */
    int fw_image_base = flash_efface_firmware_update(0x40000);

    /*
     * Parse the IMG4 manifest (IM4M) attached to the firmware image and
     * verify the RSA-PKCS1v15/SHA-384 signature.  On success, HDCP key
     * slots are loaded from the authenticated image.
     * r1/r2/r3 carry internal parser state — unused by caller.
     */
    crypto_init_and_parse_im4m(fw_image_base, unused_r1, unused_r2, unused_r3);

    /* Run the video state machine post-authentication step */
    video_process_state_84();

    /* ---------------------------------------------------------------
     * Phase 1f: CEC and HPD assertion
     * ------------------------------------------------------------ */
    if (cec_irq_handler()) {
        /* CEC detected a pending message during boot — if a sink was
         * present at boot time, treat HPD as "changed" so the source
         * gets re-trained. */
        hpd_changed_during_boot = (hpd_state_at_boot != 0);
    }

    /* Assert HPD-out to the HDMI source */
    REG_CLK_EN_B |= 4u;

    if ((REG_HDMI_CTRL_0 & 4u) != 0u) {
        /*
         * DVI (HDMI 1.x) source detected — the source drives TMDS, not FRL.
         * Must wait for PHY to lock before enabling the I2C/DDC slave,
         * otherwise the source will see malformed EDID responses.
         */
        video_check_state_38();
        hw_misc_set_state_1();

        /* Poll until both: FRL-status bit 25 is set AND PHY lock reads 0xAA */
        while ((REG_HDMI_FRL_STATUS & 0x2000000u) == 0u ||
               REG_HDMI_PHY_STATUS != 0xAAu) {
            hw_misc_set_state_50();
        }
    } else {
        /*
         * HDMI 2.1 FRL source — link training will be handled asynchronously
         * by the FRL state machine in the main loop.
         */
        hw_misc_set_state_1();
    }

    /* ---------------------------------------------------------------
     * Phase 1g: post-HPD initialisation
     * ------------------------------------------------------------ */
    hw_misc_process_state_109();
    video_set_hw_state_177();

    /*
     * If HPD changed during boot AND a sink was present at startup,
     * log a message so the host can detect the warm-plug scenario.
     */
    if (video_check_state_20() && hpd_state_at_boot) {
        custom_printf("HDMI_HPD changed during bootup\n");
        hpd_changed_during_boot = true;
    }

    /* ---------------------------------------------------------------
     * Phase 2: full peripheral initialisation
     * ------------------------------------------------------------ */
    hw_misc_process_state_4();
    hw_misc_process_state_6();
    hw_misc_set_state_7();
    hw_misc_set_state_6();
    hw_misc_set_state_25();
    hw_misc_init_state_6();
    hw_misc_set_state_26();

    /* Pass the high byte of the HDMI config register as HDCP capability flags */
    hw_misc_process_state_33((uint16_t)(REG_HDMI_CONFIG_REG >> 8));

    int hdcp_key_slot = hw_misc_set_state_73();
    hw_misc_process_state_55(hdcp_key_slot, unused_r1, unused_r2);

    /* Load the full HDMI capability register into the HDCP engine */
    hw_misc_process_state_107((int)REG_HDMI_CONFIG_REG);

    /* Enable HDCP authentication */
    hw_misc_set_state_4();

    /* ---------------------------------------------------------------
     * Phase 3: pre-loop setup  (only if not in "HPD changed" fast path)
     * ------------------------------------------------------------ */
    if (!hpd_state_at_boot || !hpd_changed_during_boot) {
        /*
         * Normal boot path: prepare the video pipeline for the first frame.
         */
        video_check_state_38();
        hw_misc_set_state_31();
        hw_misc_process_state_43();

        /* Signal normal operation mode */
        REG_BOOT_FLAGS = 1u;

        /* Arm the video event queue */
        video_set_state_124();
        video_get_state_125(0x40168, 0);   /* 0x40168 = 262568 decimal */

        /* Clear HPD debounce counter */
        REG_HPD_STATUS = 0u;

        /* Program the audio output drive level if the HPD polarity is inverted */
        if (REG_HPD_FLAGS & 1u)
            REG_MISC_AUDIO_CTRL = (uint8_t)-127;  /* 0x81 = mute */

        /* Enable I2S receiver */
        REG_MISC_I2S_FLAGS |= 8u;

        /* =============================================================
         * Main event loop — runs forever
         * ========================================================== */
        for (;;) {
            /*
             * --- Hardware interrupt summary ---
             * hw_misc_process_state_12() reads the top-level IRQ status
             * register and clears processed bits.  Its return value (in r0)
             * is passed to video_process_event_queue() which dispatches
             * queued video-path events.
             */
            int irq_flags = hw_misc_process_state_12();
            video_process_event_queue(irq_flags, unused_r1, unused_r2);

            /* Audio format detection (IEC 60958 / DSD / HBR) */
            audio_check_format_change();

            /* HDCP / DDC / CEC polling */
            hw_misc_process_state_24();

            /* Audio channel-status / info-frame state machine */
            audio_state_machine_1();

            /*
             * SCDC (Status and Control Data Channel) periodic task:
             * reads FRL status bits from the source and updates the FRL
             * link-training state machine.  Returns an event bitmask.
             */
            int scdc_events = scdc_periodic_task();
            video_process_state_164(scdc_events, unused_r1, unused_r2);

            /* eARC / audio state machine (handles H2.0 and H2.3 protocols) */
            int audio_events = audio_process_state_33();

            /* Video output pixel/timing state machine */
            int video_events = video_state_machine_4(audio_events, unused_r1, unused_r2);
            video_state_machine_3(video_events, unused_r1, unused_r2);

            /* Timer/housekeeping tick */
            int hw_time = hw_misc_process_state_59();
            audio_timer_update(hw_time, unused_r1, unused_r2);

            /* HDCP / repeater topology housekeeping */
            hw_misc_process_state_146();
            hw_misc_process_state_147();

            /* FRL link-rate adaptation */
            hw_misc_process_state_34();

            /* Increment main-loop iteration counter (used for timeouts) */
            ++REG_VIDEO_LOOP_CTR;

            /* Video timer / blanking period handling */
            int video_time = hw_misc_set_state_50();
            video_timer_update_95(video_time, unused_r1, unused_r2);

            /* ---- Deferred audio/video event handling ---- */

            if (REG_VIDEO_IRQ_FLAGS & REG_VIDEO_IRQ_AUDIO_ON) {
                /* Audio un-mute event from the source */
                audio_set_hw_state_1(1);
                REG_VIDEO_IRQ_FLAGS &= ~REG_VIDEO_IRQ_AUDIO_ON;
            } else if (REG_VIDEO_IRQ_FLAGS & REG_VIDEO_IRQ_AUDIO_OFF) {
                /* Audio mute event from the source */
                audio_set_hw_state_1(0);
                REG_VIDEO_IRQ_FLAGS &= ~REG_VIDEO_IRQ_AUDIO_OFF;
            } else if (REG_VIDEO_IRQ_FLAGS & REG_VIDEO_IRQ_CEC) {
                /* CEC topology-update event */
                hw_misc_set_state_64();
                REG_VIDEO_IRQ_FLAGS &= ~REG_VIDEO_IRQ_CEC;
            }

            /* Sleep until the next interrupt */
            __wfi();

            /* ---- Debug / diagnostic polling ----
             *
             * These comparisons catch state changes and log them via the
             * debug UART (write_to_hw_fifo_20001000 / custom_printf).
             * In production firmware this is compiled in and gated by
             * a debug-enable flag, but on this build it always runs.
             */

            /* audio_ready: tracks whether the audio PLL has locked */
            {
                int cur = video_get_hw_state_31(0);
                if (cur != (int)REG_AUDIO_READY_SHADOW) {
                    REG_AUDIO_READY_SHADOW = (uint8_t)cur;
                    custom_printf("audio_ready change to %d\n", cur);
                }
            }

            /* audio_afifo: audio FIFO overflow/underflow flag */
            {
                int cur = video_get_hw_state_28(0);
                if (cur != (int)REG_AUDIO_AFIFO_SHADOW) {
                    REG_AUDIO_AFIFO_SHADOW = (uint8_t)cur;
                    custom_printf("audio_afifo change to %d\n", cur);
                }
            }

            /* audio_abuffer: audio sample buffer depth */
            {
                int cur = video_get_hw_state_27(0);
                if (cur != (int)REG_AUDIO_ABUF_SHADOW) {
                    REG_AUDIO_ABUF_SHADOW = (uint8_t)cur;
                    custom_printf("audio_abuffer change to %d\n", cur);
                }
            }

            /* REG_EARC_OFF20: eARC capability register offset 0x20 */
            {
                if (REG_EARC_OFF20_SHADOW != REG_EARC_OFF20) {
                    REG_EARC_OFF20_SHADOW = REG_EARC_OFF20;
                    custom_printf("REG_EARC_OFF20 change to 0x%x\n",
                                  (unsigned)REG_EARC_OFF20);
                }
            }

            /* DPCD 0x182C: eARC capability advertisement via DP-AUX */
            {
                if (REG_DPCD_182C_SHADOW != REG_VIDEO_DPCD) {
                    REG_DPCD_182C_SHADOW = REG_VIDEO_DPCD;
                    custom_printf("DPCD182C change to 0x%02x\n",
                                  (unsigned)REG_VIDEO_DPCD);
                }
            }
        }
        /* NOTREACHED */
    }

    /* ---------------------------------------------------------------
     * "HPD changed during boot" fast path
     *
     * If the sink was present at boot and HPD toggled, we skip the full
     * event loop and fall out of main() to let the boot ROM re-run the
     * hot-plug sequence.  Before doing so:
     *   - Enable HDCP authentication config.
     *   - Check whether a valid HDCP key has been provisioned and record it.
     * ------------------------------------------------------------ */
    int status = (int)REG_TCB_CURRENT_TASK;

    REG_HDCP_CONFIG |= REG_HDCP_ENABLE_BIT;

    if (REG_HDCP_FLAGS) {
        if ((REG_HDCP_KEY_FLAG & 4u) != 0u)
            REG_HDCP_KEY_VALID = 1u;
    } else {
        REG_HDCP_FLAGS = 127u;  /* default: all capability bits set */
    }

    return status;
}
