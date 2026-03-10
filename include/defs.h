/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * ARM Cortex-M4 (Thumb-2) target
 *
 * Common type definitions, register maps, and platform abstractions.
 */

#ifndef PS190_DEFS_H
#define PS190_DEFS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Compiler / architecture portability
 * ---------------------------------------------------------------------- */

#ifndef __ARMCC_VERSION
#  define __breakpoint(x)     __asm__ volatile("bkpt " #x)
#  define __disable_irq()     __asm__ volatile("cpsid i" ::: "memory")
#  define __enable_irq()      __asm__ volatile("cpsie i" ::: "memory")
#  define __wfi()             __asm__ volatile("wfi")
#  define __clz(x)            __builtin_clz(x)
#  define bswap32(x)          __builtin_bswap32(x)
#endif

#define HIWORD(x)   ((uint32_t)(x) >> 16)
#define LOWORD(x)   ((uint32_t)(x) & 0xFFFF)
#define HIBYTE(x)   (((uint32_t)(x) >> 8) & 0xFF)
#define LOBYTE(x)   ((uint32_t)(x) & 0xFF)
#define HIDWORD(x)  ((uint32_t)(((uint64_t)(x)) >> 32))

/* IDA-style type aliases kept for direct translation compatibility */
typedef uint8_t   _BYTE;
typedef uint16_t  _WORD;
typedef uint32_t  _DWORD;
typedef uint64_t  _QWORD;
typedef int       BOOL;

#ifndef nullptr
#  define nullptr ((void *)0)
#endif

/* -------------------------------------------------------------------------
 * SRAM / data layout
 *
 * The PS190 SRAM is mapped at 0x00040000 (256 KB).
 * A secondary FIFO region exists at 0x20001000.
 * Code executes from internal ROM/Flash.
 * ---------------------------------------------------------------------- */

/*
 * Scheduler / task control block  (0x401A0 area)
 * Written by the PendSV handler; read by yield/task logic.
 */
#define REG_TCB_CURRENT_TASK    (*(volatile uint32_t *)0x401A0)   /* current task ID / stack pointer */
#define REG_TCB_TASK_READY      (*(volatile uint32_t *)0x401C0)   /* non-zero when a task is ready */
#define REG_TCB_YIELD_FLAG      (*(volatile uint32_t *)0x401C4)   /* set before PendSV to yield */
#define REG_TCB_CLOCK_HZ        (*(volatile uint32_t *)0x40194)   /* system clock (Hz) */
#define REG_TCB_CLOCK_SHADOW    (*(volatile uint32_t *)0x40198)   /* secondary clock value */
#define REG_TCB_CLOCK_COPY      (*(volatile uint32_t *)0x4019C)   /* working copy of clock */

/* PRNG state (Lagged Fibonacci generator, 55-tap)                         */
#define PRNG_BUF_BASE           0x41D40u                          /* uint32_t[55] ring buffer */
#define PRNG_RING_END           0x41E1Cu                          /* past-end sentinel        */
#define REG_PRNG_PTR_A          (*(volatile uint32_t *)0x41E1C)   /* lag-pointer A            */
#define REG_PRNG_PTR_B          (*(volatile uint32_t *)0x41E20)   /* lag-pointer B            */

/* Global system state struct base (returned by get_global_struct_ptr)     */
#define GLOBAL_STRUCT_BASE      0x41E84u
#define REG_GLOBAL_STRUCT_CLK   (*(volatile uint32_t *)(GLOBAL_STRUCT_BASE +  0)) /* +0  */
#define REG_GLOBAL_STRUCT_STR   (*(volatile uint32_t *)(GLOBAL_STRUCT_BASE + 12)) /* +12 model string ptr */

/* GLOBAL_STRUCT_BASE is 0x41E84; get_global_struct_ptr() returns 0x41E84 (=269892) */
/* get_global_state_ptr() returns 0x41E64 (=269860)                         */
#define GLOBAL_STATE_BASE       0x41E64u

/* -------------------------------------------------------------------------
 * Crash / fault dump region (0x44C00)
 * ---------------------------------------------------------------------- */
#define CRASH_MAGIC             (*(volatile uint32_t *)0x44C00)   /* 0x55AA55F0 = valid dump */
#define CRASH_FAULT_TYPE        (*(volatile uint8_t  *)0x44C04)   /* 0=NMI,1=MemManage,2=BusFault,3=HardFault,4=UsageFault */
#define CRASH_CFSR_OR_FLAGS     (*(volatile uint32_t *)0x44C05)   /* CFSR / auxiliary fault status */
#define CRASH_BFAR_OR_PC        (*(volatile uint32_t *)0x44C09)   /* BFAR or saved PC          */
#define CRASH_SP_FRAME          (*(volatile uint32_t *)0x44C0D)   /* stack pointer at fault    */
#define CRASH_SAVED_PC          (*(volatile uint32_t *)0x44C11)   /* exception frame PC (+20)  */
#define CRASH_SAVED_LR          (*(volatile uint32_t *)0x44C15)   /* exception frame LR (+24)  */
#define CRASH_NMI_PC            (*(volatile uint32_t *)0x44C13)
#define CRASH_NMI_LR            (*(volatile uint32_t *)0x44C17)
#define CRASH_MAGIC_VALUE       0x55AA55F0u

/* -------------------------------------------------------------------------
 * ARM Cortex-M System Control Block (SCB) registers
 * ---------------------------------------------------------------------- */
#define SCB_ICSR                (*(volatile uint32_t *)0xE000ED04)  /* Interrupt Control and State Register */
#define SCB_ICSR_PENDSVSET      0x10000000u                         /* set PendSV pending                   */
#define SCB_SHPR3               (*(volatile uint8_t  *)0xE000ED22)  /* SysTick / PendSV priority            */
#define SCB_CFSR                (*(volatile uint32_t *)0xE000ED28)  /* Configurable Fault Status Register   */
#define SCB_MMFSR               (*(volatile uint8_t  *)0xE000ED28)  /* MemManage Fault Status               */
#define SCB_BFSR                (*(volatile uint8_t  *)0xE000ED29)  /* BusFault Status                      */
#define SCB_UFSR                (*(volatile uint16_t *)0xE000ED2A)  /* UsageFault Status                    */
#define SCB_HFSR                (*(volatile uint32_t *)0xE000ED2C)  /* HardFault Status Register            */
#define SCB_DFSR                (*(volatile uint32_t *)0xE000ED30)  /* Debug Fault Status Register          */
#define SCB_MMFAR               (*(volatile uint32_t *)0xE000ED34)  /* MemManage Fault Address Register     */
#define SCB_BFAR                (*(volatile uint32_t *)0xE000ED38)  /* BusFault Address Register            */

/* -------------------------------------------------------------------------
 * PS190 peripheral register block at 0x40100000
 *
 * Registers are named by subsystem: VIDEO_, AUDIO_, HDCP_, FRL_, CEC_,
 * HPD_, EARC_, MISC_ etc.
 * ---------------------------------------------------------------------- */

/* --- Power / Reset / Watchdog --- */
#define REG_SYS_CTRL            (*(volatile uint32_t *)0x40100E14)  /* system control; bit15 = halt/reset request */
#define REG_SYS_CTRL_HALT       0x8000u
#define REG_CLK_EN_A            (*(volatile uint32_t *)0x40100E40)  /* clock / IRQ enable A; bit23 = run */
#define REG_CLK_EN_B            (*(volatile uint32_t *)0x40100E44)  /* clock / IRQ enable B; bit23 = run */
#define REG_CLK_RUN_BIT         0x800000u

/* --- HPD (Hot-Plug Detect) --- */
#define REG_HPD_CTRL            (*(volatile uint32_t *)0x40100EEC)  /* HPD control / debounce */
#define REG_HPD_STATUS          (*(volatile uint32_t *)0x40100ED0)  /* HPD pin status */
#define REG_HPD_FLAGS           (*(volatile uint32_t *)0x40100E80)  /* HPD additional flags; bit0 = polarity */

/* --- HDMI / FRL core control --- */
#define REG_HDMI_CTRL_0         (*(volatile uint32_t *)0x4010120C)  /* HDMI main control 0; bit2 = DVI mode */
#define REG_HDMI_CTRL_1         (*(volatile uint32_t *)0x40101213)  /* HDMI control 1 (byte) */
#define REG_HDMI_RESET          (*(volatile uint32_t *)0x4010121C)  /* HDMI reset; bit6 = soft reset */
#define REG_HDMI_PHY_STATUS     (*(volatile uint32_t *)0x40101224)  /* PHY lock status; 0xAA = locked */
#define REG_HDMI_FRL_STATUS     (*(volatile uint32_t *)0x40101278)  /* FRL status; bit25 = link ready */
#define REG_HDMI_MISC           (*(volatile uint32_t *)0x40101284)  /* HDMI miscellaneous; bit15 = phy reset */
#define REG_HDMI_CONFIG_REG     (*(volatile uint32_t *)0x40101694)  /* HDMI config / capability register */

/* --- Video path --- */
#define REG_VIDEO_FLAGS         (*(volatile uint32_t *)0x401004EC)  /* video flags; bit27 = active */
#define REG_VIDEO_IRQ_FLAGS     (*(volatile uint32_t *)0x40100414)  /* video IRQ / event flags */
#define REG_VIDEO_IRQ_AUDIO_ON  0x00010000u                         /* bit16 = audio mute off event */
#define REG_VIDEO_IRQ_AUDIO_OFF 0x00020000u                         /* bit17 = audio mute on event  */
#define REG_VIDEO_IRQ_CEC       0x00040000u                         /* bit18 = CEC event            */
#define REG_VIDEO_LOOP_CTR      (*(volatile uint32_t *)0x40100EE0)  /* main loop iteration counter  */
#define REG_VIDEO_DPCD          (*(volatile uint8_t  *)0x401004DC)  /* DPCD 0x182C (eARC cap)       */

/* --- Audio debug shadow registers (main loop polling) --- */
#define REG_AUDIO_READY_SHADOW  (*(volatile uint8_t  *)0x40378)
#define REG_AUDIO_AFIFO_SHADOW  (*(volatile uint8_t  *)0x40379)
#define REG_AUDIO_ABUF_SHADOW   (*(volatile uint8_t  *)0x4037A)
#define REG_EARC_OFF20_SHADOW   (*(volatile uint16_t *)0x4037C)
#define REG_DPCD_182C_SHADOW    (*(volatile uint8_t  *)0x40380)

/* --- eARC / EARC peripheral --- */
#define REG_EARC_OFF20          (*(volatile uint16_t *)0x40101620)  /* eARC register offset 0x20    */

/* --- Interrupt / misc peripheral at 0x40108000 --- */
#define REG_MISC_IRQ_STATUS     (*(volatile uint32_t *)0x40108064)  /* misc IRQ status; bit0=wdog, bit1=osc */
#define REG_MISC_IRQ_RELOAD     (*(volatile uint32_t *)0x40108074)  /* misc IRQ reload / watchdog counter   */
#define REG_MISC_WDT_KICK_VAL   6750000u                            /* watchdog reload value (~100ms @ 67.5MHz) */
#define REG_MISC_IRQ_OSC_BIT    0x00000002u
#define REG_MISC_IRQ_WDT_BIT    0x00000001u

/* --- PHY PLL status --- */
#define REG_PHY_PLL_CTRL        (*(volatile uint32_t *)0x40108510)  /* PHY PLL control / lock status */

/* --- HDCP / crypto control --- */
#define REG_HDCP_FLAGS          (*(volatile uint32_t *)0x40265)     /* HDCP feature flags (byte offset) */
#define REG_HDCP_KEY_VALID      (*(volatile uint8_t  *)0x40266)     /* 1 = valid HDCP key present */
#define REG_HDCP_CONFIG         (*(volatile uint32_t *)0x401002F0)  /* HDCP config; bit10 = enable auth */
#define REG_HDCP_ENABLE_BIT     0x400u
#define REG_HDCP_KEY_FLAG       (*(volatile uint32_t *)0x40100E16)  /* bit2 = key provisioned */

/* --- Boot / firmware mode flags --- */
#define REG_BOOT_FLAGS          (*(volatile uint32_t *)0x40190)     /* boot state flags; 0=boot,1=normal */
#define REG_FW_JUMP_ADDR        (*(volatile uint32_t *)0x401004D0)  /* firmware jump/exec address (0=no override) */

/* --- I2C / DDC register area --- */
#define REG_I2C_CTRL            (*(volatile uint32_t *)0x401002C2)  /* I2C control (byte); bits 6-7 = mode */
#define REG_DDC_FIFO_ADDR       0x20001000u                         /* DDC/I2C FIFO write port */

/* --- Misc NVRAM / model ID --- */
#define REG_NVRAM_MODEL_ID      (*(volatile uint32_t *)0x406D4)     /* NVRAM model/product ID */

/* --- Bit-bang / timer misc --- */
#define REG_MISC_CTL_1          (*(volatile uint32_t *)0x401002F0)  /* misc control 1 */
#define REG_MISC_AUDIO_CTRL     (*(volatile uint32_t *)0x4010052A)  /* audio mute/ctrl (byte) */
#define REG_MISC_I2S_FLAGS      (*(volatile uint32_t *)0x40101052)  /* I2S flags; bit3 = enable */

/* -------------------------------------------------------------------------
 * Flash / firmware image layout
 * ---------------------------------------------------------------------- */
#define FLASH_BASE_ADDR         0x45400u    /* base address of active firmware image in flash */
#define FLASH_IMAGE_SIZE        0xC00u      /* firmware image region size                     */
#define FLASH_IM4M_CERT_ADDR    0x283BCu    /* IMG4 manifest / IM4M certificate blob          */
#define FLASH_IM4M_CERT_SIZE    0x18u

/* -------------------------------------------------------------------------
 * Misc data region addresses
 * ---------------------------------------------------------------------- */
#define DATA_MODEL_STR_BASE     0x1E8A0u    /* product model string (used by check_null_string) */
#define DATA_MODEL_STR_DEFAULT  0x1E8A8u    /* default/fallback model string byte               */
#define DATA_LEADING_ZEROS_BUF  0x40EFCu    /* byte array for count_leading_zeros_in_array()    */

/* -------------------------------------------------------------------------
 * Convenience MEMORY macro (used in auto-generated code for raw accesses)
 * ---------------------------------------------------------------------- */
#define MEMORY(addr)  (*(volatile uint32_t *)(addr))

/* -------------------------------------------------------------------------
 * Forward declarations for cross-module linkage
 * (full prototypes in each module's own header)
 * ---------------------------------------------------------------------- */

/* system_main.c */
void system_early_init(void);
void __attribute__((noreturn)) system_fatal_halt(void);
void __attribute__((noreturn)) system_breakpoint_halt(void);
void __attribute__((noreturn)) system_halt_with_stub(int fault_type, int fault_info);
void __attribute__((noreturn)) system_halt_wrapper(int fault_type, int fault_info);
void __attribute__((noreturn)) system_halt_clear_flag(void);
void __attribute__((noreturn)) system_crash_dump_and_halt(int sp_frame);
void __attribute__((noreturn)) hard_fault_handler(int sp_frame);
void __attribute__((noreturn)) mem_manage_handler(int sp_frame);
void __attribute__((noreturn)) bus_fault_handler(int sp_frame);
void __attribute__((noreturn)) usage_fault_handler(int sp_frame);
void __attribute__((noreturn)) nmi_or_generic_fault_handler(int sp_frame);
void __attribute__((noreturn)) infinite_loop(void);

uint32_t prng_get_next(void);
uint32_t prng_seed_lcg(uint32_t seed);
uint32_t prng_init_default(void);

int      trigger_pendsv(void);
int      set_system_priority(void);
uint64_t get_magic_constant(void);

uint32_t  get_global_struct_ptr(void);
uint32_t  get_global_state_ptr(void);
uint64_t  fast_div_by_10(uint64_t value);
int       yield_execution(void);
int       get_system_status(void);
int       delay_loop(int ms);
int       count_leading_zeros_in_array(uint32_t len);

/* string_utils.c */
uint32_t *fast_memcpy(uint32_t *dst, const uint32_t *src, uint32_t size, int extra);
uint32_t *bzero_align(uint32_t *dst, uint32_t size);
uint32_t *bzero(uint32_t *dst, uint32_t size);
uint32_t *fast_memset(uint32_t *dst, uint32_t size, int val);
int       fast_strcmp(const uint32_t *a, const uint32_t *b);
int       memcmp_custom(const uint8_t *a, const uint8_t *b, uint32_t len);
int       simple_memset(int dst, char val, uint32_t len);
int       write_to_hw_fifo_20001000(char *buf, uint32_t len);
char     *check_null_string(int unused, uint32_t *str);

/* libc_printf.c */
int       custom_printf(const char *fmt, ...);

#endif /* PS190_DEFS_H */
