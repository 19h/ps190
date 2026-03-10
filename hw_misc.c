/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * hw_misc.c - misc hardware glue, IRQ/timer helpers, power/clock setup,
 * audio-side orchestration, and compatibility shims for unreconstructed ROM
 * entry points.
 *
 * This rewrite keeps the cross-module symbol surface stable for the already
 * reconstructed modules.  The functions that those modules actively use are
 * rewritten into structured C; several less-certain command-interface and
 * legacy helper paths are preserved as compatibility-oriented approximations.
 */

#include "include/defs.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================= */
/* Cross-module declarations                                                  */
/* ========================================================================= */

int custom_printf(const char *fmt, ...);
uint32_t prng_seed_lcg(uint32_t seed);
uint32_t prng_get_next(void);
uint32_t *bzero(uint32_t *dst, uint32_t size);
uint32_t *bzero_align(uint32_t *dst, uint32_t size);
int delay_loop(int ms);
int yield_execution(void);

int flash_wait_ready(uint8_t *status);
int flash_setup_magic_struct(uint32_t *dst, int cmd, int payload_addr);

int audio_get_format(void);
BOOL audio_protocol_check_active(void);
int audio_apply_port_config(unsigned int cfg);
int audio_process_info_frame(void);
int audio_protocol_dispatch_rd_cmd(void);
int audio_poll_mute_status(int event);
int audio_apply_mute_and_freq(int enabled);
int audio_protocol_update_timer(void);
int audio_lookup_mute_freq_a(char value);
int audio_lookup_mute_freq_b(unsigned int value);
int audio_get_highest_bit(unsigned int value);
int audio_get_fifo_index_val(void);

int scdc_load_nvram_feature_flags(void);
int scdc_apply_capability_regs(void);
BOOL scdc_check_frl_pll_lock(void);
int scdc_calc_frl_audio_clock_div(int rate);

int video_enqueue_event(int event, char arg0, char arg1);
int video_is_flag_bit27_set(void);
int video_is_active_and_not_muted(void);
int video_is_debug_flag_bit0_set(void);
int video_is_debug_flag_bit4_set(void);
int video_is_debug_flag_bit3_set(void);
int video_get_h23_work_pending(void);
int video_get_timing_lock_bit3(int channel);
int video_get_timing_lock_bit6(int channel);
int video_get_timing_lock_bit7(int channel);
int video_hpd_irq_handler(void);
int video_update_audio_infoframes(void);
int video_pulse_reg_e1d_wrapper(void);
int video_cmdif_stream_regs_if_debug(void);
int video_poll_vpll_warn_irq(void);
int video_update_reg_1607_bit4(void);
int video_enqueue_event_110_0(void);
int video_enqueue_event_110_ff(void);
int video_enqueue_event_103(void);
int video_enqueue_event_104(void);
int video_handle_fatal_error_overlay(int code, int fatal);
int video_cmdif_set_req_state(int addr);
int video_set_reg_e55(int enable);
int video_set_reg_31b(int enable);
int video_main_ctrl_set_bit4(int enable);
int video_clear_reg_1038_bit25(void);
int video_set_reg_1038_bit25_and_1068(void);
int video_set_earc_ctrl_0800(void);
int video_pulse_reg_e1d(uint32_t arg);
int video_set_path_ctrl_bit27(int enable);
int video_update_reg_335_353(void);
int video_main_ctrl_set_bit0(int enable);
int video_clear_mode_flags_bit8(void);
int video_clear_mode_flags_bit16(void);
int video_read_h23_readbuf(int offset);

int ddc_transfer_abort_callback(void);
int ddc_write_request(int dev_addr, int reg_offset, uint16_t *req);
int i2c_slave_clear_irq(void);

/* Local forward declarations for cross-referenced exported helpers. */
int hw_misc_kick_watchdog(void);
int hw_misc_clear_reg_492_and_irq_b(void);
int hw_misc_process_state_152(void);
int hw_misc_process_state_154(void);

/* ========================================================================= */
/* Raw register helpers                                                       */
/* ========================================================================= */

#define REG8(addr)   (*(volatile uint8_t  *)(uintptr_t)(addr))
#define REG16(addr)  (*(volatile uint16_t *)(uintptr_t)(addr))
#define REG32(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))

#define HW_MISC_TASK_GUARD() uint32_t saved_task = REG_TCB_CURRENT_TASK
#define HW_MISC_TASK_CHECK() do { if (REG_TCB_CURRENT_TASK != saved_task) system_halt_clear_flag(); } while (0)
#define HW_MISC_TASK_RETURN(value) do { HW_MISC_TASK_CHECK(); return (value); } while (0)

#define REG_IRQ_ENABLE_A              REG32(0x40100E40)
#define REG_IRQ_ENABLE_B              REG32(0x40100E44)
#define REG_MISC_IRQ_ROUTE            REG32(0x40100E74)
#define REG_TOP_IRQ_STATUS            REG32(0x40101230)
#define REG_TOP_IRQ_MASK              REG32(0x40108220)
#define REG_IRQ_PENDING_A             REG32(0x40101278)
#define REG_IRQ_ENABLE_MASK_A         REG32(0x401012B8)
#define REG_IRQ_ENABLE_MASK_B         REG32(0x401012B0)
#define REG_IRQ_CLEAR_A               REG32(0x4010123C)
#define REG_IRQ_CLEAR_B               REG32(0x4010127C)
#define REG_IRQ_CLEAR_C               REG32(0x401012BC)
#define REG_AUDIO_DDC_MODE            REG32(0x40103708)
#define REG_AUDIO_DDC_ENGINE          REG32(0x40103710)
#define REG_AUDIO_DDC_PENDING         REG8(0x40444)
#define REG_HDCP_AUTH_CTRL            REG32(0x401002F0)
#define REG_HDCP_PENDING_TIMER        REG8(0x40265)
#define REG_HDCP_KEY_VALID_LOCAL      REG8(0x40266)
#define REG_HDCP_BOOT_KEY_FLAG        REG8(0x40100E16)
#define REG_MISC_BOOT_COMMAND         REG8(0x401004D0)
#define REG_MISC_BOOT_FLAG            REG8(0x401004EF)
#define REG_MISC_HPD_SHADOW           REG8(0x401D5)
#define REG_HPD_EVENT_FLAGS           REG8(0x4010045E)
#define REG_MISC_GLOBAL_FLAGS         REG32(0x40100438)
#define REG_VIDEO_STATUS_FLAGS        REG32(0x40100414)
#define REG_VIDEO_STATUS_FLAGS_HI     REG8(0x40100415)

#define REG_SINK_COUNT_FLAG           REG8(0x41B98)
#define REG_LINK_DOWN_FLAG            REG8(0x401DC)
#define REG_HPD_STATE                 REG8(0x40269)
#define REG_HPD_PENDING               REG8(0x4026A)

#define REG_EVENT_TIME_SUBSEC         REG8(0x40418)
#define REG_EVENT_TIME_SEC            REG16(0x4041C)
#define REG_EVENT_TIME_TICKS          REG32(0x40420)
#define REG_EVENT_TICK_DIV5           REG8(0x40424)
#define REG_EVENT_FATAL_FLAG          REG8(0x40414)
#define REG_EVENT_TICK_LATCH          REG8(0x40415)

#define REG_AUDIO_STATUS_PAGE         REG8(0x4010073C)
#define REG_AUDIO_STATUS_DATA         REG8(0x4010073D)
#define REG_AUDIO_STATUS_DATA1        REG8(0x4010073E)
#define REG_AUDIO_PATH_CFG            REG8(0x40101698)
#define REG_AUDIO_ROUTE_FLAGS         REG8(0x4010168C)
#define REG_AUDIO_ROUTE_FLAGS2        REG8(0x4010169A)
#define REG_AUDIO_ROUTE_FLAGS3        REG8(0x4010169B)
#define REG_AUDIO_MEAS_FLAGS          REG8(0x401016A4)
#define REG_AUDIO_MISC_STATUS         REG8(0x401016CC)
#define REG_AUDIO_EARC_CTRL           REG32(0x40101620)
#define REG_AUDIO_MUTE_CTRL           REG8(0x4010161B)
#define REG_AUDIO_MUTE_SOURCE         REG8(0x401004DC)
#define REG_AUDIO_MUTE_STATUS         REG8(0x401004DE)
#define REG_AUDIO_MUTE_MASK           REG8(0x401004DF)
#define REG_AUDIO_MUTE_STATE          REG8(0x4039B)
#define REG_AUDIO_MUTE_DIRTY          REG8(0x4039C)
#define REG_AUDIO_ROUTE_POLARITY      REG8(0x4039D)
#define REG_AUDIO_ROUTE_INIT          REG8(0x4039E)
#define REG_AUDIO_NOTIFY_PENDING      REG8(0x4039F)
#define REG_AUDIO_STABLE_FLAG         REG8(0x403A0)
#define REG_AUDIO_ROUTE_CHANGED       REG8(0x403A1)
#define REG_AUDIO_NOTIFY_MASK         REG8(0x403A2)
#define REG_AUDIO_NOTIFY_STATE        REG8(0x403A3)
#define REG_AUDIO_NOTIFY_TIMER        REG32(0x403A4)
#define REG_AUDIO_NOTIFY_LATCH        REG16(0x403BC)
#define REG_AUDIO_CFG_SHADOW          REG32(0x403B4)
#define REG_AUDIO_CFG_CONSUMED        REG16(0x403B6)
#define REG_AUDIO_HPD_HYST            REG8(0x40389)
#define REG_AUDIO_NON_PCM             REG8(0x40397)
#define REG_AUDIO_SAMPLE_WIDTH        REG8(0x40396)
#define REG_AUDIO_MUTE_CODE           REG8(0x4038D)
#define REG_AUDIO_CS_BYTE0            REG8(0x4038E)
#define REG_AUDIO_CS_BYTE1            REG8(0x4038F)
#define REG_AUDIO_CS_BYTE2            REG8(0x40390)
#define REG_AUDIO_MUTE_ACTIVE         REG8(0x4038C)
#define REG_AUDIO_HW_CFG_CACHE        REG32(0x403C0)
#define REG_AUDIO_CAP_CACHE           REG16(0x403C2)
#define REG_AUDIO_CAP_FLAGS           REG8(0x403C4)

#define REG_FRL_SCDC_STATE            REG8(0x404BE)
#define REG_FRL_SCDC_TIMEOUT_INIT     REG32(0x404C4)
#define REG_FRL_SCDC_TIMEOUT          REG32(0x404C8)
#define REG_FRL_LANE_CFG_A            REG8(0x41BE8)
#define REG_FRL_LANE_CFG_B            REG8(0x41BE9)
#define REG_FRL_RATE_CODE             REG8(0x41BF5)
#define REG_FRL_TRAINED_MASK          REG32(0x41BF7)
#define REG_FRL_TRAIN_MODE            REG8(0x41BFA)
#define REG_EARC_STATE                REG8(0x40458)
#define REG_EARC_CHECK_FLAG           REG8(0x40459)
#define REG_EARC_STATUS_LATCH         REG8(0x4045B)
#define REG_FRL_MODE_INDEX            REG8(0x4045D)

#define REG_CMDIF_REQ_STATE           REG8(0x411CD)
#define REG_CMDIF_REQ_SHADOW          REG8(0x411C4)
#define REG_CMDIF_FLAGS               REG8(0x40332)

/* ========================================================================= */
/* Local helpers                                                              */
/* ========================================================================= */

typedef union {
    uint64_t u64;
    double d;
} DoubleBits;

static uint64_t double_to_bits(double value)
{
    DoubleBits bits;
    bits.d = value;
    return bits.u64;
}

static double bits_to_double(uint64_t value)
{
    DoubleBits bits;
    bits.u64 = value;
    return bits.d;
}

static uint64_t float64_trunc_to_integral_bits(uint64_t bits)
{
    uint32_t exp_field = (uint32_t)((bits >> 52u) & 0x7FFu);
    uint64_t sign = bits & UINT64_C(0x8000000000000000);

    if (exp_field == 0x7FFu)
        return bits;

    int exp = (int)exp_field - 1023;
    if (exp < 0)
        return sign;
    if (exp >= 52)
        return bits;

    uint64_t frac_mask = (UINT64_C(1) << (52 - exp)) - 1u;
    if ((bits & frac_mask) == 0u)
        return bits;

    return bits & ~frac_mask;
}

static void hw_misc_cmdif_stage_reset_a(void)
{
    REG8(0x40100394) &= (uint8_t)~7u;
}

static void hw_misc_cmdif_stage_reset_b(void)
{
    REG8(0x401D0) = 0;
    REG8(0x401D1) = 0;
    REG8(0x401E8) = 0;
    REG8(0x401D2) = 0;
}

static int hw_misc_hdcp_auth_init(void)
{
    __disable_irq();
    if (REG8(0x40264) != 0u) {
        REG8(0x40264) = 1u;
        __enable_irq();
        return (int)REG_TCB_CURRENT_TASK;
    }
    __enable_irq();

    REG_HDCP_AUTH_CTRL |= REG_HDCP_ENABLE_BIT;
    if (REG_HDCP_PENDING_TIMER != 0u) {
        if ((REG_HDCP_BOOT_KEY_FLAG & 4u) != 0u)
            REG_HDCP_KEY_VALID_LOCAL = 1u;
    } else {
        REG_HDCP_PENDING_TIMER = 127u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

/* ========================================================================= */
/* Frequently used boot / clock / interrupt helpers                           */
/* ========================================================================= */

int hw_misc_disable_clk_run_bits(void)
{
    HW_MISC_TASK_GUARD();
    REG_IRQ_ENABLE_A &= ~REG_CLK_RUN_BIT;
    REG_IRQ_ENABLE_B &= ~REG_CLK_RUN_BIT;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_copy_rom_defaults(void)
{
    HW_MISC_TASK_GUARD();

    for (uint16_t i = 0; i < 0x200u; ++i)
        REG8(0x44C1B + i + 0xFFFFBFFFu) = REG8(0x40103000u + i);
    for (uint16_t i = 0; i < 0x200u; ++i)
        REG8(0x44E1B + i + 0xFFFFBFFFu) = REG8(0x40103C00u + i);

    REG32(0x44C19) = REG32(0x40102244);
    REG32(0x44C1B) = REG32(0x40102250);
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_hdcp_auth_irq(void)  { HW_MISC_TASK_GUARD(); REG_MISC_IRQ_ROUTE |= 0x200u; HW_MISC_TASK_RETURN((int)saved_task); }
int hw_misc_disable_hdcp_auth_irq(void)  { HW_MISC_TASK_GUARD(); REG_MISC_IRQ_ROUTE &= ~0x200u; HW_MISC_TASK_RETURN((int)saved_task); }
int hw_misc_clear_reg_2c4_bit20(void)  { HW_MISC_TASK_GUARD(); REG32(0x401002C4) &= ~0x00100000u; HW_MISC_TASK_RETURN((int)saved_task); }

int hw_misc_set_reg_238_234(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0x40100238) |= 0x8000u;
    REG32(0x40100234) |= 0x00040000u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_update_reg_29c(void)
{
    HW_MISC_TASK_GUARD();
    if ((REG8(0x40100B3A) & 0x20u) != 0u)
        REG8(0x4029C) = (REG8(0x40100B42) >> 4) & 1u;
    else
        REG8(0x4029C) = 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_is_sink_count_1(void)
{
    return (REG_SINK_COUNT_FLAG & 1u) != 0u;
}

int hw_misc_handle_sink_count_change(void)
{
    HW_MISC_TASK_GUARD();

    if (hw_misc_is_sink_count_1() || (REG8(0x401E5) & 4u) != 0u) {
        custom_printf("SINK_COUNT = 1\n");
        hw_misc_cmdif_stage_reset_a();
        __disable_irq();
        REG32(0x40101264) = 4u;
        __asm__ volatile("dsb" ::: "memory");
        REG32(0x40103740) |= 1u;
        ddc_transfer_abort_callback();
        __enable_irq();
        REG8(0x40100330) &= 0x40u;
        REG8(0x40100330) |= 1u;
        REG32(0x4010035C) = 1u;
    } else {
        custom_printf("SINK_COUNT = 0\n");
        REG8(0x401EF) &= (uint8_t)~1u;
        REG8(0x40100330) &= 0x40u;
        REG32(0x4010035C) = 0u;
    }

    if (!hw_misc_is_sink_count_1())
        REG32(0x40494) = 0u;

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_check_crash_magic(void)
{
    HW_MISC_TASK_GUARD();
    if (CRASH_MAGIC == CRASH_MAGIC_VALUE) {
        REG8(0x401004EF) |= 2u;
        CRASH_MAGIC = 0u;
    }
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_top_irq_handler_and_yield(void)
{
    HW_MISC_TASK_GUARD();

    if (REG8(0x401B8) == 1u && REG8(0x401B9) == 1u) {
        REG8(0x401B8) = 0u;
        REG8(0x401B9) = 0u;
        REG_TCB_TASK_READY = 0u;
        REG_TCB_YIELD_FLAG = 1u;
    } else if (REG8(0x401B8) == 1u && REG8(0x401B9) == 0u) {
        yield_execution();
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_irq_enable_b_clear_bit0(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100E50) |= 1u;
    REG_IRQ_ENABLE_B &= ~1u;
    REG8(0x40100E66) &= (uint8_t)~1u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_all_irqs(void)
{
    HW_MISC_TASK_GUARD();
    REG_IRQ_CLEAR_A = 0xFFFFFFFFu;
    REG_IRQ_CLEAR_B = 0xFFFFFFFFu;
    REG_IRQ_CLEAR_C = 0x800u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_poll_hpd_event_flags(void)
{
    HW_MISC_TASK_GUARD();

    if ((((REG_MISC_HPD_SHADOW ^ REG_HPD_EVENT_FLAGS) & 0x80u) != 0u) &&
        ((REG_HPD_EVENT_FLAGS & 0x80u) != 0u)) {
        REG_HPD_EVENT_FLAGS |= 1u;
        __disable_irq();
        for (;;) {
            hw_misc_kick_watchdog();
        }
    }

    REG_MISC_HPD_SHADOW = REG_HPD_EVENT_FLAGS;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_scb_irqs(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0xE000ED24) |= 0x00070000u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_wait_for_boot_flag(void)
{
    HW_MISC_TASK_GUARD();

    uint32_t stable_count = 0u;
    bool boot_wait_enabled = false;
    if (REG8(0x4000D) == 1u && REG8(0x4000B) == 1u && REG8(0x401C8) == 1u)
        boot_wait_enabled = (REG8(0x401C9) == 1u);

    while (stable_count < 0x71E36Au) {
        if ((REG_MISC_BOOT_COMMAND & 0x8Fu) == 0x80u) {
            REG_MISC_BOOT_COMMAND = 0u;
            system_fatal_halt();
        }

        if ((REG_MISC_BOOT_FLAG & 2u) != 0u) {
            stable_count = 0u;
        } else if (boot_wait_enabled) {
            ++stable_count;
        } else {
            stable_count = 0u;
        }
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_hpd_plugin_handler(void)
{
    HW_MISC_TASK_GUARD();
    hw_misc_handle_sink_count_change();
    hw_misc_hdcp_auth_init();
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_cmdif_reset_all(void)
{
    HW_MISC_TASK_GUARD();
    hw_misc_cmdif_stage_reset_a();
    hw_misc_cmdif_stage_reset_b();
    REG8(0x401EF) &= (uint8_t)~1u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_video_audio_regs(void)
{
    HW_MISC_TASK_GUARD();

    REG32(0x40F34) = 0u;
    REG32(0x40F38) = 0u;
    REG32(0x40F3C) = 0u;
    REG8(0x401D9) = 0u;
    REG32(0x40100388) = 0u;
    REG32(0x4010038C) = 0u;
    REG32(0x4010034C) &= 0xFF00FFFFu;
    REG8(0x40100327) &= (uint8_t)~0x80u;
    REG8(0x40100084) &= (uint8_t)~4u;
    REG32(0x40100414) &= 0xFF00FFFFu;
    bzero((uint32_t *)0x41080, 84u);
    REG8(0x401E4) = 0u;
    REG8(0x401EC) = 0u;
    REG8(0x40200) = 0u;
    REG8(0x40204) = 0u;
    REG32(0x40F40) = 0u;
    REG32(0x40F44) = 0u;
    REG32(0x40F48) = 0u;
    bzero_align((uint32_t *)0x41BD8, 14u);
    hw_misc_cmdif_stage_reset_a();
    hw_misc_cmdif_stage_reset_b();
    video_clear_mode_flags_bit8();
    video_clear_mode_flags_bit16();

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_ddc_mode_init(void)
{
    HW_MISC_TASK_GUARD();
    REG_AUDIO_DDC_MODE &= 0xFFF8FFFFu;
    REG_AUDIO_DDC_MODE |= 0x80u;
    REG_AUDIO_DDC_ENGINE = 0u;
    REG_AUDIO_DDC_PENDING = 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_apply_hdmi_cap_byte(unsigned int cap_byte)
{
    HW_MISC_TASK_GUARD();

    REG8(0x40100701) = 0u;
    REG8(0x40100778) = 14u;
    REG8(0x40100717) = 3u;
    REG8(0x40100733) |= 8u;
    REG8(0x40100704) = (uint8_t)((REG8(0x40100704) & 0xFCu) | 1u);
    REG8(0x40100713) |= 8u;
    REG8(0x40100779) |= 3u;
    REG8(0x40100777) = 39u;
    REG8(0x4010077B) &= (uint8_t)~0x80u;

    audio_apply_port_config(cap_byte);

    REG8(0x4010073C) = 0xCDu;
    REG_AUDIO_MUTE_ACTIVE = (REG8(0x4010073D) & 0x10u) != 0u;
    if (REG_AUDIO_MUTE_ACTIVE != 0u) {
        uint8_t mute_code = REG8(0x4010073E);
        REG_AUDIO_MUTE_CODE = (uint8_t)(mute_code & 0x0Fu);
        REG_AUDIO_CS_BYTE0 = mute_code;
        REG_AUDIO_CS_BYTE1 = mute_code;
        REG8(0x4010073C) = 37u;
        REG_AUDIO_CS_BYTE2 = mute_code;
    }

    video_update_audio_infoframes();
    REG16(0x403C2) = (uint16_t)cap_byte;
    REG8(0x403C4) = REG8(0x4010169A) & 1u;
    REG8(0x40100700) |= 0x80u;

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_poll_audio_status_page9(void)
{
    HW_MISC_TASK_GUARD();
    __disable_irq();
    REG_AUDIO_STATUS_PAGE = 9u;
    REG8(0x40385) = REG_AUDIO_STATUS_DATA;
    __enable_irq();

    if (REG8(0x40385) != REG8(0x40384)) {
        REG8(0x40384) = REG8(0x40385);
        REG8(0x40100EE2) = REG8(0x40385);
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_irq_routing_init(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100E42) |= 0x10u;
    REG8(0x40100E46) &= (uint8_t)~0x10u;
    REG8(0x40100E50) &= (uint8_t)~0x10u;
    REG8(0x40100E50) |= 0x0Eu;
    REG8(0x40100E54) = (uint8_t)((REG8(0x40100E54) & 0xFCu) | 1u);
    REG8(0x40100E50) &= (uint8_t)~0x20u;
    REG8(0x40100E42) &= (uint8_t)~0x20u;
    REG8(0x40100E46) &= (uint8_t)~0x20u;
    REG8(0x40100E56) = 1u;
    REG8(0x40100E66) &= (uint8_t)~8u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_irq_mask_a_enable_lower(void)
{
    HW_MISC_TASK_GUARD();
    REG_IRQ_ENABLE_MASK_A |= 0x0Fu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_update_audio_notify_timer(void)
{
    HW_MISC_TASK_GUARD();

    if (REG32(0x403A4) != 0u) {
        __disable_irq();
        --REG32(0x403A4);
        __enable_irq();
    }
    if (video_get_timing_lock_bit7(0) && video_get_timing_lock_bit6(0))
        video_get_timing_lock_bit3(0);

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_phy_serialiser_init(unsigned int cfg)
{
    HW_MISC_TASK_GUARD();
    if (cfg < 8u)
        cfg = 8u;
    REG32(0x40298) = 0u;
    REG8(0x40102204) = 23u;
    REG8(0x40102200) = 152u;
    REG32(0x4010223C) = cfg - 8u;
    REG8(0x4010221C) = 143u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_power_up_output_stage(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100E18) |= 1u;
    REG8(0x40100E18) |= 2u;
    REG8(0x40108404) |= 0x80u;
    REG8(0x40108404) &= (uint8_t)~0x80u;
    REG32(0x40100E00) |= 0xC0000000u;
    REG32(0x40100E04) |= 0xC000007Fu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_seed_prng_and_clock(int seed_ptr)
{
    HW_MISC_TASK_GUARD();
    prng_seed_lcg(REG32((uintptr_t)seed_ptr + 68u));
    REG_TCB_CLOCK_HZ = prng_get_next();
    REG_TCB_CLOCK_SHADOW = prng_get_next();
    HW_MISC_TASK_RETURN(1);
}

int hw_misc_init_event_queue_slots(void)
{
    HW_MISC_TASK_GUARD();
    for (uint8_t i = 0; i < 32u; ++i) {
        REG8(0x40ED4u + 3u * i + 0u) = 1u;
        REG8(0x40ED4u + 3u * i + 1u) = 0u;
        REG8(0x40ED4u + 3u * i + 2u) = 0u;
    }
    REG8(0x40100EE1) = 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

/* ========================================================================= */
/* IRQ routing and boot-phase plumbing                                        */
/* ========================================================================= */

int hw_misc_enable_top_irq_masks(void)
{
    HW_MISC_TASK_GUARD();
    REG_TOP_IRQ_MASK |= 0x393FFu;
    REG32(0x40101280) |= 0x10FD2022u;
    REG32(0x40101284) |= 0x3E0u;
    REG32(0x40101288) |= 0x00E05000u;
    REG32(0x401012A4) |= 0x100u;
    REG32(0x4010129C) |= 0x800u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_disable_audio_irqs_during_init(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100E50) &= (uint8_t)~0x40u;
    REG8(0x40100E47) &= (uint8_t)~1u;
    REG8(0x40100E43) |= 1u;
    REG32(0x40108000) = 0u;
    REG32(0x40108058) = 0xFFFFFFFFu;
    REG32(0x40108050) = 0xFFFFFFFFu;
    REG32(0x40108010) = 10u * (uint32_t)audio_get_fifo_index_val();
    REG32(0x40108014) = 0u;
    REG32(0x40108030) = 10u * (uint32_t)audio_get_fifo_index_val();
    REG32(0x40108034) = 0u;
    REG32(0x40108054) |= 1u;
    REG32(0x40108000) = 3u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_power_rail_a(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0xE000E280) = 0xFFFFFFFFu;
    REG32(0xE000E284) = 0xFFFFFFFFu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_power_rail_b(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0xE000E180) = 0xFFFFFFFFu;
    REG32(0xE000E184) = 0xFFFFFFFFu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_set_serialiser_clock(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0xE000E100) = 234495u;
    REG32(0xE000E104) = 10u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_init_pll_and_fifo_regs(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x41069) = 0u;
    REG8(0x40100D80) = 0x81u;
    REG8(0x40100D81) = 0u;
    REG8(0x40100D82) = 2u;
    REG8(0x40100D83) = 68u;
    REG8(0x40100D87) = 0x80u;
    REG8(0x40100D8B) = 122u;
    REG8(0x40100D8E) = 0u;
    REG8(0x40100D8F) = 2u;
    REG8(0x40100D9D) = 32u;
    REG32(0x40100D84) = (REG32(0x40100D84) & 0xFF0000FFu) | 0x8000u;
    REG32(0x40100DA0) = (REG32(0x40100DA0) & 0x00FFFFFFu) | 0x01000000u;
    REG8(0x40100D8C) = (uint8_t)((REG8(0x40100D8C) & 0x0Fu) | 0xE0u);
    REG8(0x4010104F) |= 0x80u;
    REG8(0x40100089) |= 0x40u;
    REG8(0x4010008A) |= 8u;
    REG8(0x40101001) |= 1u;
    REG8(0x40100DA1) |= 2u;
    REG32(0x40101288) |= 0x00800000u;
    REG8(0x4106B) = 0u;
    REG8(0x4106C) = 0u;
    REG8(0x4106A) = 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

/* ========================================================================= */
/* Audio / mute / FRL integration                                             */
/* ========================================================================= */

int hw_misc_audio_route_earc_ctrl(void)
{
    HW_MISC_TASK_GUARD();
    int fmt = audio_get_format();
    if (fmt == 4 || fmt == 5) {
        REG_AUDIO_EARC_CTRL &= 0xF3u;
        REG8(0x40388) = 1u;
        REG32(0x403B0) = 100u * (uint32_t)REG8(0x40100EF1);
        REG_AUDIO_MUTE_CTRL = 0x81u;
        video_enqueue_event(134, (char)REG_AUDIO_ROUTE_FLAGS, (char)REG8(0x40386));
    }
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_hdcp_key_slot_init(int a1, int a2, int a3)
{
    (void)a1;
    (void)a2;
    (void)a3;
    HW_MISC_TASK_GUARD();

    uint8_t status = 0u;
    REG8(0x40290) = ((REG32(0x40107800) >> 2) & 3u) ? 2u : 3u;
    REG8(0x40291) = REG8(0x40290);
    flash_wait_ready(&status);
    flash_wait_ready(&status);
    REG16(0x40103C14) = (uint16_t)(((uint16_t)REG8(0x40290) << 8) | REG8(0x40291));
    REG16(0x40104080) = REG16(0x40103C14);
    REG32(0x40103C00) = 256u;

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_handle_event_45_46_10_113(uint8_t *event)
{
    HW_MISC_TASK_GUARD();

    switch (event[0]) {
    case 45:
        video_enqueue_event_103();
        break;
    case 46:
    case 47:
        break;
    case 10:
        hw_misc_clear_reg_492_and_irq_b();
        break;
    default:
        if (event[0] == 113 && event[1] == 0) {
            REG8(0x40496) = 0u;
        } else if (event[0] == 113 && event[1] == 1) {
            REG8(0x40496) = 0u;
        }
        break;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_poll_reg_492_49f(void)
{
    HW_MISC_TASK_GUARD();
    if (REG8(0x40492) != 0u && REG8(0x4049F) != 0u) {
        REG32(0x40101D98) |= 0x00080000u;
        if ((REG8(0x40101DF0) & 7u) != REG8(0x40493)) {
            __disable_irq();
            video_enqueue_event(46, 0, 0);
            __enable_irq();
        }
    }
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_get_current_task(void)
{
    return (int)REG_TCB_CURRENT_TASK;
}

int hw_misc_clear_reg_492_and_irq_b(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40492) = 0u;
    REG8(0x41069) = 0u;
    REG8(0x4106A) = 0u;
    REG8(0x4106B) = 0u;
    REG8(0x4106E) = 0u;
    REG32(0x40101270) = 0x10000u;
    REG_IRQ_ENABLE_MASK_B &= 0xFFD4FFFFu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_reg_106e_106f(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x4010106E) = 0u;
    REG8(0x4010106F) = 0u;
    REG8(0x41080) &= (uint8_t)~0x80u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_kick_watchdog(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0x40108074) = REG_MISC_WDT_KICK_VAL;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_lock_hpd_state(int *ctx)
{
    HW_MISC_TASK_GUARD();
    (void)ctx;
    __disable_irq();
    REG_HPD_STATE = 1u;
    *ctx = 0;
    __enable_irq();
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_unlock_hpd_state_and_check(int *ctx)
{
    HW_MISC_TASK_GUARD();
    __disable_irq();

    if (REG_HPD_PENDING != 0u) {
        REG8(0x41090) = 0u;
        REG8(0x41094) = 0u;
        hw_misc_clear_reg_492_and_irq_b();
        audio_protocol_dispatch_rd_cmd();
        REG8(0x41082) = 0u;
        REG32(0x41BD4) = 0u;
        REG8(0x41084) = 0u;
        REG8(0x41088) = 0u;
        REG8(0x41086) = 0u;
        REG8(0x40274) = 0u;
        REG8(0x4026D) = 0u;
        *ctx = 1;
        REG_HPD_PENDING = 0u;
    }

    REG_HPD_STATE = 0u;
    __enable_irq();
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_scdc_set_state3_timeout(void)
{
    HW_MISC_TASK_GUARD();
    REG_FRL_SCDC_STATE = 3u;
    REG_FRL_SCDC_TIMEOUT = REG_FRL_SCDC_TIMEOUT_INIT;
    REG32(0x4010129C) |= 0x18u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_scdc_apply_lane_config(char lane_a, char lane_b)
{
    HW_MISC_TASK_GUARD();
    REG32(0x4010125C) = 8248u;
    REG32(0x40101130) = (REG32(0x40101130) & 0xFFFF00FFu)
                        | ((uint32_t)(((uint8_t)lane_b & 7u) | ((uint8_t)lane_a << 4)) << 8);
    REG32(0x4010129C) |= 0x38u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_frl_pll_ctrl_bits(void)
{
    HW_MISC_TASK_GUARD();
    video_main_ctrl_set_bit0(0);
    REG32(0x40100E0C) &= ~0x00200000u;
    REG32(0x40100E0C) &= ~0x00800000u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_set_reg_254(int enable)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40254) = (enable != 0) ? 1u : 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_update_timers_6b_6e(void)
{
    HW_MISC_TASK_GUARD();

    if (REG8(0x4106B) != 0u) {
        --REG8(0x4106B);
        if (REG8(0x4106B) == 0u && REG8(0x41069) == 3u)
            video_is_debug_flag_bit0_set();
    }

    if (REG8(0x4106E) != 0u) {
        --REG8(0x4106E);
        if (REG8(0x4106E) == 0u && REG8(0x41069) == 6u) {
            if (scdc_check_frl_pll_lock()) {
                REG8(0x40101288) |= 0xA0u;
            } else {
                __disable_irq();
                REG_IRQ_ENABLE_MASK_B |= 0x002B0000u;
                __enable_irq();
                REG8(0x41070) = 2u;
            }
        }
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_scdc_phy_ctrl_init(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x4010125C) = 0x80u;
    REG8(0x40101160) &= (uint8_t)~0x20u;
    REG8(0x40101160) |= 0x12u;
    REG32(0x4010129C) |= 0x1080u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_init_regs_1300_131d(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40101300) = 0xA4u;
    REG8(0x40101302) = 17u;
    REG8(0x40101304) = 35u;
    REG8(0x40101305) = 40u;
    REG8(0x40101310) = 40u;
    REG8(0x40101308) = 79u;
    REG8(0x4010130A) = 95u;
    REG8(0x40101306) = 56u;
    REG8(0x4010131D) |= 0x20u;
    REG8(0x40101301) |= 0x80u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

uint64_t hw_misc_trunc_float64(uint64_t value)
{
    return float64_trunc_to_integral_bits(value);
}

int hw_misc_poll_audio_mute_and_csb(void)
{
    HW_MISC_TASK_GUARD();

    int changed = 0;
    uint8_t mute_code = REG8(0x4010162C) & 0x0Fu;
    if (REG_AUDIO_MUTE_CODE != mute_code) {
        REG_AUDIO_MUTE_CODE = mute_code;
        video_update_audio_infoframes();
        changed = 1;
    }

    if (REG_AUDIO_NOTIFY_TIMER == 0u && REG_AUDIO_ROUTE_CHANGED != (uint8_t)changed) {
        REG_AUDIO_ROUTE_CHANGED = (uint8_t)changed;
        video_enqueue_event(132, 0, 0);
        if (changed)
            REG_AUDIO_NOTIFY_TIMER = 1u;
    }

    REG_AUDIO_CS_BYTE2 = REG8(0x4010162D);
    REG_AUDIO_CS_BYTE1 = REG8(0x4010162B);
    REG_AUDIO_CS_BYTE0 = REG8(0x40101628);
    hw_misc_process_state_154();

    uint8_t cs_type = REG_AUDIO_CS_BYTE0 & 0x3Bu;
    bool stereo_family = (cs_type == 2u) || (cs_type == 10u);
    if (REG_AUDIO_ROUTE_INIT == 0u || REG_AUDIO_ROUTE_POLARITY != (uint8_t)stereo_family) {
        if (REG_AUDIO_ROUTE_INIT == 0u)
            REG_AUDIO_ROUTE_INIT = 1u;
        REG_AUDIO_MUTE_SOURCE |= stereo_family ? 0x10u : 0x20u;
        REG_AUDIO_ROUTE_POLARITY = (uint8_t)stereo_family;
        REG_AUDIO_NOTIFY_PENDING = 1u;
        video_update_audio_infoframes();
    }

    if ((int8_t)REG_AUDIO_MISC_STATUS < 0) {
        REG8(0x4038A) = 1u;
        if (REG_AUDIO_ROUTE_POLARITY != 0u) {
            REG_AUDIO_HW_CFG_CACHE |= 4u;
            REG32(0x40101694) |= 4u;
            REG8(0x40100797) |= 1u;
        } else {
            REG_AUDIO_HW_CFG_CACHE &= ~4u;
            REG32(0x40101694) &= ~4u;
            REG8(0x40100797) &= (uint8_t)~1u;
        }
    } else {
        REG8(0x4038A) = 0u;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_irq_enable_mask_b_clear(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0x40101270) = 0x10000u;
    REG_IRQ_ENABLE_MASK_B &= 0xFFD4FFFFu;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_handle_event_41_cmdif(uint8_t *event)
{
    HW_MISC_TASK_GUARD();

    if (event[0] == 41u) {
        uint8_t pending = REG8(0x401B8) | REG8(0x401B9);
        if (REG_CMDIF_REQ_STATE == 128u && REG_CMDIF_FLAGS == 0u) {
            REG_CMDIF_REQ_SHADOW = REG8(0x401004EB);
            if (pending == 0u) {
                flash_setup_magic_struct((uint32_t *)0x401B4, 136085, 265936);
                REG8(0x401B8) = 1u;
            }
        } else if (REG_CMDIF_REQ_STATE == 136u &&
                   REG_CMDIF_REQ_SHADOW == REG8(0x401004EB) &&
                   REG_CMDIF_FLAGS == 1u) {
            if (pending == 0u) {
                flash_setup_magic_struct((uint32_t *)0x401B4, 136085, 265936);
                REG8(0x401B8) = 1u;
            }
        } else if (REG_CMDIF_REQ_STATE == 128u &&
                   REG_CMDIF_REQ_SHADOW != REG8(0x401004EB) &&
                   REG_CMDIF_FLAGS == 1u) {
            custom_printf("cmdif special path cmdif_request_state = %x\n", 1);
            REG_CMDIF_REQ_SHADOW = REG8(0x401004EB);
            REG_CMDIF_FLAGS = 0u;
            if (pending == 0u) {
                flash_setup_magic_struct((uint32_t *)0x401B4, 136085, 265936);
                REG8(0x401B8) = 1u;
            }
        } else {
            video_cmdif_set_req_state(1074791654);
        }
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_phy_pll_and_wait(void)
{
    HW_MISC_TASK_GUARD();

    REG8(0x40338) = 0u;
    REG8(0x4035C) = 1u;
    REG32(0x401004E4) |= 0x00010000u;

    int feature = scdc_load_nvram_feature_flags();
    REG8(0x4035F) = (uint8_t)feature;
    if (feature == 0)
        REG32(0x4033C) = 0x2000u;
    else if (feature == 1)
        REG32(0x4033C) = 155648u;

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_set_reg_114_bits(int bit0, int bit1, int bit2)
{
    HW_MISC_TASK_GUARD();
    uint32_t v = 0u;
    if (bit0) v |= 0x100u;
    if (bit1) v |= 0x200u;
    if (bit2) v |= 0x400u;
    REG32(0x40100114) = (REG32(0x40100114) & 0xFFFFF8FFu) | v;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_frl_link_loss_recovery(void)
{
    HW_MISC_TASK_GUARD();
    video_main_ctrl_set_bit4(0);
    hw_misc_clear_frl_pll_ctrl_bits();
    REG8(0x4026C) = 0u;
    REG_EARC_STATUS_LATCH = 0u;
    REG_FRL_MODE_INDEX = 0u;
    video_is_debug_flag_bit4_set();
    REG8(0x41C04) = 0u;
    if (REG_EARC_CHECK_FLAG != 0u) {
        REG_EARC_STATE = 4u;
        REG8(0x41C04) = 1u;
        REG_EARC_CHECK_FLAG = 0u;
    } else {
        REG_EARC_STATE = 1u;
    }
    video_enqueue_event(100, 0, 0);
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_reg_1996_bit2(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40101996) &= (uint8_t)~4u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_clear_reg_38b(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x4038B) = 0u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_check_reg_9b8_and_audio(void)
{
    HW_MISC_TASK_GUARD();
    int enable_default = 1;
    if ((REG32(0x401009B8) & 2u) != 0u && audio_protocol_check_active()) {
        video_set_reg_1038_bit25_and_1068();
        enable_default = 0;
    } else {
        video_clear_reg_1038_bit25();
    }
    HW_MISC_TASK_RETURN(enable_default);
}

int hw_misc_process_state_101(uint8_t *event)
{
    HW_MISC_TASK_GUARD();

    if (event[0] == 51u) {
        if (event[1] == 1u) {
            video_is_debug_flag_bit3_set();
            REG8(0x402A1) = 0u;
        } else if (event[1] == 2u) {
            video_is_debug_flag_bit3_set();
            REG8(0x402A1) = 2u;
            i2c_slave_clear_irq();
            REG32(0x401002C2) |= 0xC0u;
            REG32(0x40101244) = 0x8000u;
            delay_loop(5);
            REG32(0x40101244) = 0x8000u;
            delay_loop(5);
            __disable_irq();
            REG32(0x40101284) |= 0x8000u;
            __enable_irq();
        }
    } else if (event[0] == 14u || event[0] == 13u) {
        REG8(0x402A1) = 0u;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_107(int cfg)
{
    HW_MISC_TASK_GUARD();

    int audio_fmt = audio_get_format();
    uint32_t delta = REG_AUDIO_HW_CFG_CACHE ^ (uint32_t)cfg;
    REG_AUDIO_HW_CFG_CACHE = (uint32_t)cfg;

    if ((delta & 1u) != 0u) {
        video_enqueue_event(123, (char)cfg, 0);
        if ((cfg & 1) != 0) {
            REG8(0x4010160E) &= 0xF3u;
            REG8(0x4010160D) |= 1u;
            if (audio_fmt != 5)
                REG8(0x40100E14) |= 6u;
            REG8(0x40101610) |= 1u;
        } else {
            REG8(0x4010160E) &= (uint8_t)~4u;
            REG8(0x4010160E) |= 8u;
            REG8(0x40100E14) &= (uint8_t)~4u;
            if (audio_fmt == 5)
                REG8(0x40100E14) |= 2u;
            else
                REG8(0x40100E14) &= (uint8_t)~2u;
            REG8(0x40101610) &= (uint8_t)~1u;
        }
    }

    if ((delta & 2u) != 0u)
        video_update_reg_1607_bit4();

    if (REG8(0x4038A) == 1u && (delta & 4u) != 0u) {
        if ((cfg & 4) != 0)
            REG_AUDIO_EARC_CTRL |= 0x01000000u;
        else
            REG_AUDIO_EARC_CTRL &= ~0x01000000u;
        REG_AUDIO_EARC_CTRL |= 0x02000000u;
        video_enqueue_event(120, (char)(cfg & 4), 0);
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_set_reg_398(char value)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100398) = (uint8_t)value;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_get_reg_39d(void)
{
    return (int)REG8(0x4010039D);
}

int hw_misc_set_reg_39d(char value)
{
    HW_MISC_TASK_GUARD();
    REG8(0x4010039D) = (uint8_t)value;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_109(void)
{
    HW_MISC_TASK_GUARD();

    if ((REG_HDMI_CTRL_0 & 4u) != 0u)
        REG_MISC_GLOBAL_FLAGS |= 0x04000000u;
    else
        REG_MISC_GLOBAL_FLAGS &= ~0x04000000u;

    uint32_t fifo = REG32(0x40107800);
    if ((fifo & 0x10u) != 0u) REG_MISC_GLOBAL_FLAGS |= 0x08000000u; else REG_MISC_GLOBAL_FLAGS &= ~0x08000000u;
    if ((fifo & 0x08u) != 0u) REG_MISC_GLOBAL_FLAGS |= 0x10000000u; else REG_MISC_GLOBAL_FLAGS &= ~0x10000000u;
    if ((fifo & 0x04u) != 0u) REG_MISC_GLOBAL_FLAGS |= 0x20000000u; else REG_MISC_GLOBAL_FLAGS &= ~0x20000000u;
    if ((fifo & 0x02u) != 0u) REG_MISC_GLOBAL_FLAGS |= 0x40000000u; else REG_MISC_GLOBAL_FLAGS &= ~0x40000000u;

    HW_MISC_TASK_RETURN((int)saved_task);
}

/* ========================================================================= */
/* Video / FRL setup and timer state                                          */
/* ========================================================================= */

int hw_misc_process_state_124(int flags, int hpd_state)
{
    HW_MISC_TASK_GUARD();

    REG8(0x401002C0) |= 4u;
    REG8(0x401002C0) &= (uint8_t)~8u;
    REG8(0x401DA) = 0u;
    REG8(0x401DB) = 0u;
    REG_LINK_DOWN_FLAG = 0u;
    video_update_reg_335_353();

    REG32(0x40F5C) = 2109454u;
    REG32(0x40F60) = 2109455u;
    REG32(0x40F64) = 2109441u;
    for (uint8_t i = 3; i < 0x20u; ++i)
        REG32(0x40F5C + 4u * i) = 0u;
    for (uint8_t i = 0; i < 0x20u; ++i)
        REG32(0x40FDC + 4u * i) = 0u;
    REG32(0x4010028C) = 3u;
    for (uint8_t i = 0; i < 0x20u; ++i)
        REG32(0x40100280) = REG32(0x40F5C + 4u * i);
    for (uint8_t i = 0; i < 0x20u; ++i)
        REG32(0x40100284) = REG32(0x40FDC + 4u * i);
    REG32(0x4010028C) = 1u;
    REG32(0x40101244) = 0x40000u;
    REG32(0x40101284) |= 0x40000u;
    REG8(0x401E2) = 0u;
    REG8(0x401E3) = 0x80u;

    REG8(0x401003C7) |= 1u;
    REG8(0x4010030C) |= 0x40u;
    REG8(0x401003CC) |= 0x40u;
    REG32(0x401003E8) = (REG32(0x401003E8) & 0xFFFF00FFu) | 0x500u;

    if (((REG32(0x40040) >> 22) & 3u) == 2u) {
        REG32(0x40100400) = (REG32(0x40100400) & 0x00FFFFFFu) | 0x63000000u;
        REG32(0x40100408) = (REG32(0x40100408) & 0xFFFFFF00u) | 0x63u;
    }
    REG32(0x40100408) |= 0x16030000u;

    if ((REG8(0x40100B2D) & 0x40u) != 0u) {
        REG32(0x40100450) = 0u;
    } else {
        uint8_t f15 = REG8((uintptr_t)flags + 15u);
        uint8_t f11 = REG8((uintptr_t)flags + 11u);
        if (f15 == 0u && f11 == 0u)
            REG32(0x40100450) = 1u;
        else if (f15 == 1u && f11 == 0u)
            REG32(0x40100450) = 2u;
        else if (f15 == 1u && f11 == 1u)
            REG32(0x40100450) = 3u;
        else
            REG32(0x40100450) = 0u;
    }

    REG32(0x40F34) = 0u;
    REG32(0x40F38) = 0u;
    REG32(0x40F3C) = 0u;
    hw_misc_cmdif_stage_reset_a();
    if (hpd_state != 0)
        REG8(0x4010035B) = REG8(0x402A2);

    REG8(0x401D9) = 0u;
    REG8(0x401D6) = 0u;
    REG8(0x401CC) = 20u;
    REG8(0x40100390) |= 1u;
    REG8(0x4010023C) = 33u;
    REG8(0x4010023D) = 91u;
    REG8(0x40100244) = 59u;
    REG8(0x40100247) = 96u;

    if (hpd_state == 0) {
        REG8(0x40100329) = 64u;
        REG8(0x40100440) = 84u;
    }

    REG8(0x401F8) = REG8(0x4010043F);
    REG8(0x401FA) = REG8(0x40100440) & 3u;
    REG16(0x401FC) = (uint16_t)((REG16(0x40100440) >> 2) & 0x03FFu);
    REG32(0x404D4) = (uint32_t)(uintptr_t)video_pulse_reg_e1d;

    REG8(0x401004B0) = 44u;
    REG8(0x401004B1) = 216u;
    REG8(0x401004B2) = 45u;
    REG8(0x401004B3) = 152u;
    REG8(0x401004B4) = 46u;
    REG8(0x401004B5) = 152u;
    REG8(0x401004B6) = 47u;
    REG8(0x401004B7) = 152u;
    REG8(0x401004B8) = 36u;
    REG8(0x401004B9) = 152u;

    video_set_reg_31b(1);
    REG8(0x401E5) &= (uint8_t)~1u;
    REG8(0x401E4) = REG8(0x4010038E);
    REG8(0x401E5) = (uint8_t)((REG8(0x401E5) & 0xFBu)
                            | ((((REG32(0x40100438) & 0x02000000u) != 0u) ? 1u : 0u) << 2));

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_update_timers_2a1_1cc_200_1d0(void)
{
    HW_MISC_TASK_GUARD();

    if (REG8(0x402A1) != 0u) {
        --REG8(0x402A1);
        if (REG8(0x402A1) == 0u)
            video_cmdif_stream_regs_if_debug();
    }

    if (REG8(0x401CC) != 0u)
        --REG8(0x401CC);

    if (REG8(0x40200) != 0u) {
        --REG8(0x40200);
        if (REG8(0x40200) == 0u) {
            REG8(0x40100459) |= 0x80u;
            REG8(0x40101323) |= 0x80u;
        }
    }

    if (REG8(0x401D0) != 0u)
        --REG8(0x401D0);
    if (REG8(0x401D1) != 0u)
        --REG8(0x401D1);

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_disable_irq_a_b_bit1(void)
{
    HW_MISC_TASK_GUARD();
    REG8(0x40100E51) &= 0xF3u;
    REG_IRQ_ENABLE_A &= ~2u;
    REG_IRQ_ENABLE_B &= ~2u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_enable_hdcp_irq_mask(void)
{
    HW_MISC_TASK_GUARD();
    REG32(0x40101294) |= 0x007C0735u;
    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_audio_earc_ctrl_init(void)
{
    HW_MISC_TASK_GUARD();

    REG8(0x40100EF0) = 4u;
    REG8(0x40100EF1) = 8u;
    REG8(0x40101600) = 0xF9u;
    REG8(0x40101610) = 1u;
    REG8(0x40101604) = 123u;
    REG8(0x40101605) = 215u;
    REG8(0x40101607) |= 0x40u;
    REG8(0x4010160D) = 107u;
    REG8(0x4010160F) = 121u;
    REG8(0x401016F3) = 0x80u;
    REG8(0x4010160E) &= (uint8_t)~2u;
    REG8(0x4010160E) &= (uint8_t)~0x80u;
    REG8(0x4010160E) &= (uint8_t)~0x20u;
    REG8(0x401016B8) = 1u;
    REG32(0x403B8) = 0u;

    REG32(0x40101688) |= 0x0F000000u;
    REG_AUDIO_EARC_CTRL &= 0xFF00FFFFu;
    REG_AUDIO_EARC_CTRL |= 0x1000u;
    REG_AUDIO_EARC_CTRL &= ~0x40000000u;
    REG_AUDIO_EARC_CTRL |= 0x80000000u;

    audio_apply_mute_and_freq(0);
    REG8(0x40101625) |= 0x80u;
    REG8(0x40100727) = 0xE0u;
    REG8(0x40100704) |= 0x40u;
    video_set_earc_ctrl_0800();

    HW_MISC_TASK_RETURN((int)saved_task);
}

/* ========================================================================= */
/* Periodic mute / route maintenance                                          */
/* ========================================================================= */

int hw_misc_process_state_146(void)
{
    HW_MISC_TASK_GUARD();

    uint8_t low = (uint8_t)REG_VIDEO_STATUS_FLAGS;
    uint8_t high = REG_VIDEO_STATUS_FLAGS_HI;
    uint8_t packed = (uint8_t)(((REG_VIDEO_STATUS_FLAGS >> 4) & 7u) | ((REG_VIDEO_STATUS_FLAGS_HI & 1u) << 3));

    audio_poll_mute_status(REG_AUDIO_PATH_CFG);

    if (low != 0u) {
        REG8(0x40100795) = low;
        REG_VIDEO_STATUS_FLAGS = 0u;
    }
    if (high != 0u) {
        REG8(0x40100796) = high;
        REG_VIDEO_STATUS_FLAGS_HI = 0u;
    }

    uint8_t new_bits = (uint8_t)(packed & REG_AUDIO_CFG_SHADOW) & (uint8_t)~REG_AUDIO_CFG_CONSUMED;
    if (new_bits != 0u) {
        REG_AUDIO_NOTIFY_PENDING = 1u;
        REG_AUDIO_CFG_CONSUMED |= new_bits;
    }

    if (REG_AUDIO_NOTIFY_PENDING != 0u) {
        custom_printf("SW CSB CHG\n");
        __disable_irq();
        hw_misc_process_state_152();
        __enable_irq();
        REG_AUDIO_NOTIFY_PENDING = 0u;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_147(void)
{
    HW_MISC_TASK_GUARD();

    if ((REG_AUDIO_PATH_CFG & 4u) != 0u && REG_AUDIO_MUTE_ACTIVE != 0u) {
        uint8_t cs0 = (uint8_t)(REG_AUDIO_CS_BYTE0 & 0x3Bu);
        if (cs0 == 32u || cs0 == 2u || cs0 == 10u || cs0 == 42u) {
            int scale = 1;
            switch ((uint8_t)(REG_AUDIO_CS_BYTE2 >> 4)) {
            case 3: scale = 5; break;
            case 7: scale = 3; break;
            case 11: scale = 4; break;
            default: break;
            }

            __disable_irq();
            REG_AUDIO_STATUS_PAGE = 35u;
            uint8_t meas = REG_AUDIO_STATUS_DATA1;
            __enable_irq();

            unsigned int ch_status_fs = (unsigned int)audio_lookup_mute_freq_a((char)meas);
            if (ch_status_fs != 0xFFFFu && (REG_AUDIO_MEAS_FLAGS & 0x80u) == 0u) {
                int measured_fs = audio_lookup_mute_freq_b((REG_AUDIO_MEAS_FLAGS >> 1) & 0x0Fu);
                if (measured_fs != 0xFFFF && (ch_status_fs >> (scale - 1)) == (unsigned int)measured_fs) {
                    REG8(0x40100795) = 2u;
                    custom_printf("Clr FS_MISMATCH Mute cond\n");
                }
            }
        }
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_150(int value)
{
    HW_MISC_TASK_GUARD();

    if (REG_AUDIO_MUTE_STATE == (uint8_t)value) {
        if (REG_AUDIO_MUTE_DIRTY != 0u) {
            REG_AUDIO_NOTIFY_PENDING = 1u;
            REG_AUDIO_MUTE_DIRTY = 0u;
        }
        if (REG_AUDIO_MUTE_STATE == 1u)
            REG8(0x401004E0) |= 4u;
        else
            REG8(0x401004E0) &= (uint8_t)~4u;
    } else {
        REG_AUDIO_MUTE_STATE = (uint8_t)value;
        REG_AUDIO_MUTE_DIRTY = 1u;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_152(void)
{
    HW_MISC_TASK_GUARD();

    uint8_t cfg = (uint8_t)(REG_AUDIO_MUTE_STATE
                  | ((REG_AUDIO_ROUTE_POLARITY != 0u) ? 2u : 0u)
                  | ((REG_AUDIO_ROUTE_POLARITY == 0u) ? 4u : 0u)
                  | ((REG_AUDIO_ROUTE_CHANGED != 0u) ? 8u : 0u));

    if (cfg != (uint8_t)REG_AUDIO_CFG_SHADOW) {
        REG_AUDIO_CFG_SHADOW = cfg;
        REG_AUDIO_CFG_CONSUMED &= cfg;
    }

    REG_AUDIO_MUTE_STATUS = (uint8_t)(cfg & (uint8_t)~REG_AUDIO_CFG_CONSUMED);

    bool notify_enabled = false;
    if (REG_AUDIO_STABLE_FLAG == 0u ||
        (((int16_t)REG_AUDIO_NOTIFY_LATCH < 0) && ((REG_AUDIO_CAP_FLAGS & 1u) != 0u)) ||
        (((uint8_t)(cfg & (uint8_t)~REG_AUDIO_CFG_CONSUMED) & REG_AUDIO_NOTIFY_MASK) != 0u &&
         ((REG_AUDIO_CAP_FLAGS & 1u) != 0u)) ||
        REG_AUDIO_HPD_HYST < 0x30u) {
        notify_enabled = true;
    }

    uint16_t packed = (uint16_t)((REG_AUDIO_NOTIFY_LATCH & REG_AUDIO_ROUTE_FLAGS2 & 0x1Eu)
                      | REG_AUDIO_STABLE_FLAG
                      | ((REG_AUDIO_MUTE_STATUS & REG_AUDIO_MUTE_MASK & 0x0Fu) << 5)
                      | (((REG_AUDIO_NOTIFY_LATCH & (REG_AUDIO_ROUTE_FLAGS2 | ((uint16_t)REG_AUDIO_ROUTE_FLAGS3 << 8)) & 0x300u) != 0u) ? 0x0002u : 0u)
                      | (notify_enabled ? 0x0800u : 0u)
                      | ((REG_AUDIO_HPD_HYST != 0u) ? 0x1000u : 0u));

    video_enqueue_event(135,
        (char)((REG_AUDIO_NOTIFY_LATCH & REG_AUDIO_ROUTE_FLAGS2 & 0x1Eu)
             | REG_AUDIO_STABLE_FLAG
             | ((REG_AUDIO_MUTE_STATUS & REG_AUDIO_MUTE_MASK & 0x0Fu) << 5)),
        (char)(packed >> 8));

    if ((uint8_t)notify_enabled != REG_AUDIO_NOTIFY_STATE) {
        if (notify_enabled) {
            REG8(0x40100E42) |= 0x10u;
            REG_AUDIO_MUTE_SOURCE |= 0x40u;
        } else {
            REG8(0x40100E42) &= (uint8_t)~0x10u;
            REG_AUDIO_MUTE_SOURCE |= 0x80u;
        }
        video_enqueue_event(133, (char)packed, (char)(packed >> 8));
        REG_AUDIO_NOTIFY_STATE = (uint8_t)notify_enabled;
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}

int hw_misc_process_state_154(void)
{
    HW_MISC_TASK_GUARD();
    (void)audio_get_format();

    if ((int8_t)REG_AUDIO_MISC_STATUS < 0) {
        uint8_t cs_type = (uint8_t)(REG_AUDIO_CS_BYTE0 & 0x3Bu);
        bool fs_ok = (cs_type == 32u) || (cs_type == 2u) || (cs_type == 10u) || (cs_type == 42u);
        int scale = 1;
        switch ((uint8_t)(REG_AUDIO_CS_BYTE2 >> 4)) {
        case 3: scale = 5; break;
        case 7: scale = 3; break;
        case 11: scale = 4; break;
        default: break;
        }
        if (REG_AUDIO_NON_PCM != 0u && (!fs_ok || scale == 1) && REG_AUDIO_SAMPLE_WIDTH == 3u)
            audio_apply_mute_and_freq(1);
        else
            audio_apply_mute_and_freq(0);
    }

    HW_MISC_TASK_RETURN((int)saved_task);
}
