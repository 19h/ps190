/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * hdmi_frl_video.c — FRL link control, video timing control, event queue,
 * hot-plug handling, video-side IRQ decoding, and light CEC glue.
 *
 * Rewritten from decompiler output into structured C with stable exported
 * symbol names so the remaining modules can continue to link against the
 * current interface while reconstruction is still in progress.
 */

#include "include/defs.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================= */
/* Cross-module declarations                                                  */
/* ========================================================================= */

int custom_printf(const char *fmt, ...);
uint32_t *bzero(uint32_t *dst, uint32_t size);
int delay_loop(int ms);

int ddc_read_data_buffer(void);
int ddc_transfer_complete_callback(void);
int ddc_clear_request(void);
int ddc_write_request(int dev_addr, int reg_offset, uint16_t *req);

int scdc_is_state_1(void);
int scdc_is_state_5(void);
int scdc_is_state_7(void);
int scdc_handle_link_down_cleanup(void);
int scdc_poll_ddc_idle(void);
int scdc_handle_earc_32ch_event(void);
int scdc_enable_frl_mode(void);
int scdc_read_config_1(int port);
int scdc_set_hpd_debounce_timer(void);
int scdc_enable_tmds_mode(void);
int scdc_fsm_active_to_ready(char rate, char lane_cfg);
int scdc_fsm_force_ready(char rate, char lane_cfg, int caller_tag);

int hw_misc_hpd_plugin_handler(void);
int hw_misc_cmdif_reset_all(void);
int hw_misc_clear_video_audio_regs(void);
int hw_misc_process_state_37(void);
int hw_misc_audio_route_earc_ctrl(void);
int hw_misc_process_state_60(void);
int hw_misc_clear_reg_492_and_irq_b(void);
int hw_misc_clear_frl_pll_ctrl_bits(void);
int hw_misc_poll_audio_mute_and_csb(void);
int hw_misc_set_reg_114_bits(int a1, int a2, int a3);
int hw_misc_process_state_150(int value);
int hw_misc_set_state_36(int a1, int a2);
int hw_misc_set_state_38(int a1, int a2);
int hw_misc_set_reg_254(int value);
int hw_misc_set_state_55(int value);
int hw_misc_disable_irq_a_b_bit1(void);
int hw_misc_update_audio_notify_timer(void);
int hw_misc_update_timers_6b_6e(void);
int hw_misc_update_timers_2a1_1cc_200_1d0(void);
int hw_misc_handle_event_45_46_10_113(uint8_t *event);
int hw_misc_lock_hpd_state(int *ctx);
int hw_misc_unlock_hpd_state_and_check(int *ctx);
int hw_misc_handle_event_41_cmdif(uint8_t *event);
int hw_misc_process_state_101(uint8_t *event);

int audio_get_format(void);
int audio_get_bit_depth(int mode);
int audio_get_h14_status2(void);
int audio_get_h14_status1(void);
int audio_get_h14_status0(void);
int audio_h23_get_rdval1_bit3(void);
int audio_h23_get_rdval1_bit2(void);
int audio_protocol_get_status_bit(void);
int audio_protocol_get_mode(void);
int audio_protocol_get_state_code(void);
int audio_h14_status_reg_b_check_bit14(void);
BOOL audio_calc_check_result_valid(void);
int audio_check_idle_timeout(void);
int audio_h14_update_timers(void);
int audio_h23_update_timers(void);
int audio_dispatch_pending_scdc_tasks(void);
int audio_h14_fsm_init(void);
int audio_h14_aphy_check_status(void);
int audio_h14_status_reg_set_c000(void);
int audio_h14_aphy_check_error(void);
int audio_h23_aphy_check_status(void);
int audio_h23_cfg_state_reset_alt(void);
int audio_h23_cfg_state_advance_to_2(void);
int audio_h23_cfg_state_advance_to_0(void);
int audio_h23_parse_discovery_pkt(void);
int audio_h23_substate_advance_to_6(void);
int audio_h23_substate_advance_to_5(void);
int audio_h23_substate_advance_to_2(void);
int audio_h23_substate_advance_to_3_or_4(void);
int audio_enqueue_video_event_11_4(void);
int audio_protocol_start_machine(void);
int audio_calc_flags_set(int a1, int a2);
int audio_init_spdif_i2s_regs(void);
int audio_irq_ack_a_bit4(void);
int audio_calc_set_rx_flags(int a1, int a2);
int audio_calc_set_result(int a1);
int audio_h23_fsm_init(void);
int audio_h23_cfg_state_reset(void);
int audio_h14_rx_cmd_handler(uint8_t *event);
int audio_h14_tx_cmd_handler(uint8_t *event);
int audio_h23_rx_cmd_handler(uint8_t *event);
int audio_h23_tx_cmd_handler(uint8_t *event);
int audio_earc_irq_handler(uint8_t *event);

int i2c_miic0_translate_error(uint8_t *event);

/* ========================================================================= */
/* Local register / SRAM aliases                                              */
/* ========================================================================= */

#define REG8(addr)   (*(volatile uint8_t  *)(uintptr_t)(addr))
#define REG16(addr)  (*(volatile uint16_t *)(uintptr_t)(addr))
#define REG32(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))

#define VIDEO_TASK_GUARD() uint32_t saved_task = REG_TCB_CURRENT_TASK
#define VIDEO_TASK_CHECK() do { if (REG_TCB_CURRENT_TASK != saved_task) system_halt_clear_flag(); } while (0)

#define REG_EVENT_Q_HEAD            REG8(0x401BA)
#define REG_EVENT_Q_TAIL            REG8(0x401BB)
#define REG_EVENT_Q_OVERFLOW        REG8(0x401BC)
#define VIDEO_EVENT_BASE            0x40ED4u
#define VIDEO_EVENT_COUNT           32u

#define REG_EVENT_TIME_SUBSEC       REG8(0x40418)
#define REG_EVENT_TIME_SEC          REG16(0x4041C)
#define REG_EVENT_TIME_TICKS        REG32(0x40420)

#define REG_VIDEO_FATAL_FLAG        REG8(0x40414)
#define REG_VIDEO_HPD_STATE         REG8(0x40269)
#define REG_VIDEO_HPD_PENDING       REG8(0x4026A)
#define REG_VIDEO_LINK_DOWN         REG8(0x401DC)

#define REG_FRL_MUTEX               REG8(0x40456)
#define REG_FRL_DEFERRED_EVENT      REG8(0x40457)
#define REG_EARC_STATE              REG8(0x40458)
#define REG_FRL_STATE               REG_EARC_STATE
#define REG_FRL_RETRY_TIMER         REG8(0x40464)
#define REG_FRL_HPD_IRQ_LATCH       REG8(0x40443)

#define REG_FRL_MODE_INDEX          REG8(0x4045D)
#define REG_DSC_MODE_INDEX          REG8(0x4045E)
#define REG_SCDC_FRL_MODE           REG_DSC_MODE_INDEX
#define REG_SCDC_CURRENT_RATE       REG8(0x41BF5)
#define REG_SCDC_TRAINED_MASK       REG32(0x41BF7)
#define REG_SCDC_LANE_CFG_A         REG8(0x41BE8)
#define REG_SCDC_LANE_CFG_B         REG8(0x41BE9)
#define REG_SCDC_DSC_ENABLE         REG8(0x41BFB)

#define REG_VIDEO_DEBUG_FLAGS       REG8(0x40100ECC)
#define REG_VIDEO_IRQ_ACK_A         REG32(0x40101250)
#define REG_VIDEO_IRQ_MASK_A        REG32(0x40101290)
#define REG_VIDEO_I2C_IRQ_STATUS    REG32(0x40101264)
#define REG_VIDEO_I2C_IRQ_ENABLE    REG32(0x401012A4)
#define REG_VIDEO_I2C_SLAVE_IRQ     REG32(0x40101260)
#define REG_VIDEO_I2C_SLAVE_ENABLE  REG32(0x401012A0)

#define REG_VIDEO_FRL_PLL_CTRL      REG32(0x40100E0C)
#define REG_VIDEO_FRL_CTRL          REG32(0x40100E70)
#define REG_VIDEO_MAIN_CTRL         REG32(0x40100004)
#define REG_VIDEO_PATH_CTRL         REG32(0x4010035C)
#define REG_VIDEO_HPD_CFG           REG32(0x40100E80)
#define REG_VIDEO_IRQ_ROUTE         REG32(0x40101298)
#define REG_VIDEO_IRQ_ROUTE_B       REG32(0x401012B8)
#define REG_VIDEO_MISC_IRQ_EN       REG32(0x40100E74)
#define REG_VIDEO_SCDC_IRQ_MASK     REG32(0x4010129C)

#define REG_VIDEO_BIST_STATUS       REG32(0x40101A80)
#define REG_VIDEO_FIFO_STATUS       REG32(0x40107800)

#define REG_VIDEO_TIMING_LOCK0      REG8(0x4010073C)
#define REG_VIDEO_TIMING_LOCK1      REG8(0x4010073D)

#define REG_VIDEO_AUDIO_ROUTE       REG8(0x40100723)
#define REG_VIDEO_EARC_CTRL         REG16(0x40101620)
#define REG_VIDEO_AUDIO_FMT         REG8(0x40100702)
#define REG_VIDEO_AUDIO_BITS        REG8(0x4010075F)
#define REG_VIDEO_AUDIO_LAYOUT      REG8(0x40100778)
#define REG_VIDEO_AUDIO_CH_STATUS   REG8(0x4010079D)
#define REG_VIDEO_AUD_FMT           REG_VIDEO_AUDIO_FMT
#define REG_VIDEO_AUD_BITS          REG_VIDEO_AUDIO_BITS
#define REG_VIDEO_AUD_LAYOUT        REG_VIDEO_AUDIO_LAYOUT
#define REG_VIDEO_AUD_CH_STATUS     REG_VIDEO_AUDIO_CH_STATUS

#define REG_VIDEO_TMDS_LOW          REG16(0x4109C)
#define REG_VIDEO_TMDS_HIGH         REG16(0x4109E)
#define REG_VIDEO_PIXEL_CLOCK       REG16(0x410A0)
#define REG_VIDEO_TIMING0           REG16(0x410A2)
#define REG_VIDEO_TIMING1           REG16(0x410A4)
#define REG_VIDEO_TIMING2           REG16(0x410A6)
#define REG_VIDEO_TIMING3           REG16(0x410A8)
#define REG_VIDEO_TIMING4           REG16(0x410AA)

#define REG_VIDEO_VPLL_STATE        REG8(0x41090)
#define REG_VIDEO_SM_STATE          REG8(0x41094)

#define REG_VIDEO_AUDIO_EN_PENDING  REG8(0x40398)
#define REG_VIDEO_AUDIO_HYST        REG8(0x40389)
#define REG_VIDEO_AUDIO_MUTE_ACTIVE REG8(0x4038C)
#define REG_VIDEO_AUDIO_MUTE_CODE   REG8(0x4038D)
#define REG_VIDEO_AUDIO_MODE        REG8(0x40391)
#define REG_VIDEO_AUDIO_FORCE       REG8(0x4039D)
#define REG_VIDEO_AUDIO_LAYOUT3D    REG8(0x403A9)

#define REG_VIDEO_TIMER_A           REG32(0x403AE)
#define REG_VIDEO_TIMER_B           REG32(0x40387)
#define REG_VIDEO_TIMER_C           REG32(0x403B0)
#define REG_VIDEO_TIMER_C_STEPS     REG8(0x40386)
#define REG_VIDEO_TIMER_C_FLAG      REG8(0x403A7)
#define REG_VIDEO_TIMER_C_VALID     REG8(0x403A8)
#define REG_VIDEO_START_TIMER       REG32(0x40274)
#define REG_VIDEO_START_TIMESTAMP   ((uint8_t *)0x40284)
#define REG_VIDEO_PHASE_GOOD        REG8(0x41082)
#define REG_VIDEO_PHASE_POLL_TIMER  REG8(0x41084)
#define REG_VIDEO_PHASE_RETRY_TIMER REG8(0x41086)
#define REG_VIDEO_PHASE_ARM_TIMER   REG8(0x41088)
#define REG_VIDEO_PHASE_MIN         REG32(0x410C0)
#define REG_VIDEO_PHASE_MAX         REG32(0x410BC)
#define REG_VIDEO_PHASE_DISABLE     REG8(0x4026D)
#define REG_VIDEO_PHASE_BYPASS      REG8(0x4026E)
#define REG_VIDEO_MISC_CFG_A        REG8(0x4010002B)
#define REG_VIDEO_MISC_CFG_B        REG8(0x4010002C)
#define REG_VIDEO_MISC_CACHE_A      REG8(0x41099)
#define REG_VIDEO_MISC_CACHE_B      REG8(0x4109A)
#define REG_VIDEO_CLOCK_CLASS       REG8(0x410CD)
#define REG_VIDEO_CLOCK_FLAG_A      REG8(0x410CE)
#define REG_VIDEO_CLOCK_FLAG_B      REG8(0x410CF)
#define REG_VIDEO_CLOCK_FLAG_C      REG8(0x410D0)
#define REG_VIDEO_CLOCK_FLAG_D      REG8(0x410D1)
#define REG_VIDEO_TIMING_ACTIVE     REG8(0x410DD)
#define REG_VIDEO_AUDIO_VALID       REG8(0x410C5)
#define REG_VIDEO_CMDIF_PTR         REG16(0x402A4)
#define REG_VIDEO_CMDIF_A           REG8(0x402A2)
#define REG_VIDEO_CMDIF_B           REG8(0x402A3)
#define REG_VIDEO_AUDIO_IRQ_STATUS  REG8(0x40101268)
#define REG_VIDEO_AUDIO_IRQ_MASK    REG8(0x401012A8)
#define REG_VIDEO_AUDIO_IRQ2_STATUS REG16(0x4010126C)
#define REG_VIDEO_AUDIO_IRQ2_MASK   REG16(0x401012AC)

#define REG_VIDEO_STATUS_IRQ        REG32(0x40101210)

enum {
    VIDEO_EVENT_TICK = 5,
    VIDEO_EVENT_HPD_CHANGE = 10,
    VIDEO_EVENT_CEC_GPIO = 111,
    VIDEO_EVENT_CEC_HW = 112,
    VIDEO_EVENT_VPLL_WARN = 138,
    VIDEO_EVENT_AUDIO_ROUTE = 139,
    VIDEO_EVENT_SCDC_STATUS = 140,
    VIDEO_EVENT_PATH_RESET = 142,
    VIDEO_EVENT_IRQ_125 = 125,
    VIDEO_EVENT_IRQ_126 = 126,
};

typedef struct {
    uint8_t code;
    uint8_t arg0;
    uint8_t arg1;
} VideoEvent;

/* Internal forward declarations used before definition order. */
int video_set_fatal_flag(void);
int video_clear_fatal_flag(void);
int video_update_audio_infoframes(void);
int video_cmdif_stream_regs(void);
static int video_start_timer_init(void);
static int video_check_timing_changed(int a1, int a2, int a3);
int video_link_down_cleanup(void);
int video_update_phase_timers(int a1, int a2, int a3);
int video_debug_print_status(void);
int video_enqueue_event(int code, char arg0, char arg1);
int video_main_ctrl_set_bit4(int enabled);
int video_frl_rate_regs_apply(int rate, int enable);
int video_main_ctrl_set_0a_and_clear(void);
unsigned int video_calc_elapsed_ticks(uint8_t *timestamp, int a2, int a3);
int video_snapshot_event_time(int base, int atomic);
int video_update_start_timer(int a1, int a2, int a3);
int frl_decrement_retry_timer(void);

/* ========================================================================= */
/* Event queue helpers                                                        */
/* ========================================================================= */

static inline volatile uint8_t *video_event_slot_ptr(uint8_t index)
{
    return (volatile uint8_t *)(uintptr_t)(VIDEO_EVENT_BASE + (uint32_t)index * 3u);
}

static inline VideoEvent video_event_read(uint8_t index)
{
    volatile uint8_t *slot = video_event_slot_ptr(index);
    VideoEvent event;

    event.code = slot[0];
    event.arg0 = slot[1];
    event.arg1 = slot[2];
    return event;
}

static inline void video_event_write(uint8_t index, VideoEvent event)
{
    volatile uint8_t *slot = video_event_slot_ptr(index);

    slot[0] = event.code;
    slot[1] = event.arg0;
    slot[2] = event.arg1;
}

static inline void video_event_mark_empty(uint8_t index)
{
    video_event_slot_ptr(index)[0] = 1u;
}

static inline uint8_t video_event_next(uint8_t index)
{
    ++index;
    return (index == VIDEO_EVENT_COUNT) ? 0u : index;
}

static uint8_t video_queue_depth(uint8_t head, uint8_t tail)
{
    return (tail >= head) ? (uint8_t)(tail - head) : (uint8_t)(VIDEO_EVENT_COUNT + tail - head);
}

static void video_cmdif_write_byte(uint16_t addr, uint8_t value)
{
    REG8(0x401002BE) = value;
    REG8(0x401002BC) = (uint8_t)addr;
    REG8(0x401002BD) = (uint8_t)(((addr >> 8) & 7u) | 0xE0u);
}

static void video_cmdif_stream(uint16_t *addr, uintptr_t src_addr, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i) {
        video_cmdif_write_byte(*addr, REG8(src_addr + i));
        ++(*addr);
    }
}

static int video_read_phase_sample(void)
{
    VIDEO_TASK_GUARD();

    REG8(0x40100115) |= 0x40u;
    delay_loop(1);

    REG8(0x401000BD) = 80u;
    while (REG8(0x401000BD) != 80u) {}
    REG8(0x401000BD) = 81u;
    while (REG8(0x401000BD) != 81u) {}
    REG8(0x401000BD) = 82u;
    while (REG8(0x401000BD) != 82u) {}
    REG8(0x401000BD) = 83u;
    while (REG8(0x401000BD) != 83u) {}

    int sample = (int)REG8(0x401000BE)
               | ((int)REG8(0x401000BE) << 8)
               | ((int)REG8(0x401000BE) << 16)
               | ((int)REG8(0x401000BE) << 24);

    REG8(0x40100115) &= (uint8_t)~0x40u;
    if ((REG8(0x40100ED4) & 4u) != 0u)
        custom_printf("f-PHASE: %08x\n", sample);

    VIDEO_TASK_CHECK();
    return sample;
}

static bool video_dequeue_event(VideoEvent *event)
{
    uint8_t head;

    __disable_irq();
    head = REG_EVENT_Q_HEAD;

    while (video_event_read(head).code == 1u && head != REG_EVENT_Q_TAIL) {
        head = video_event_next(head);
        REG_EVENT_Q_HEAD = head;
    }

    if (video_event_read(head).code == 1u) {
        __enable_irq();
        return false;
    }

    *event = video_event_read(head);
    video_event_mark_empty(head);
    REG_EVENT_Q_HEAD = video_event_next(head);

    if (REG_EVENT_Q_OVERFLOW != 0u) {
        if (video_queue_depth(REG_EVENT_Q_HEAD, REG_EVENT_Q_TAIL) <= 24u) {
            video_clear_fatal_flag();
            hw_misc_set_state_38(0, 234495);
            REG_EVENT_Q_OVERFLOW = 0u;
        }
    }

    __enable_irq();
    return true;
}

/* ========================================================================= */
/* Low-level helpers                                                          */
/* ========================================================================= */

static int video_dispatch_event(VideoEvent event)
{
    switch (event.code) {
    case VIDEO_EVENT_TICK:
        audio_check_idle_timeout();
        video_update_phase_timers(hw_misc_update_timers_2a1_1cc_200_1d0(), 0, 0);
        hw_misc_update_timers_6b_6e();
        audio_h14_update_timers();
        audio_h23_update_timers();
        frl_decrement_retry_timer();
        hw_misc_update_audio_notify_timer();
        video_debug_print_status();
        break;

    case 13:
        custom_printf("ESYS_HDMIPORT_PLUGOUT\n");
        REG8(0x40264) = 0;
        hw_misc_hpd_plugin_handler();
        break;

    case 14:
        custom_printf("ESYS_HDMIPORT_PLUGIN\n");
        REG8(0x40264) = 0;
        hw_misc_hpd_plugin_handler();
        break;

    case 33:
        hw_misc_cmdif_reset_all();
        break;

    case 34:
        hw_misc_clear_video_audio_regs();
        break;

    case VIDEO_EVENT_CEC_GPIO:
        custom_printf("CEC IRQ rise, GPIO\n");
        break;

    case VIDEO_EVENT_CEC_HW:
        custom_printf(event.arg0 ? "CEC HW IRQ, NO HPD toggle\n" : "CEC HW IRQ, HPD toggle\n");
        break;

    default:
        hw_misc_handle_event_41_cmdif((uint8_t *)&event);
        hw_misc_handle_event_45_46_10_113((uint8_t *)&event);
        hw_misc_process_state_101((uint8_t *)&event);
        audio_h14_rx_cmd_handler((uint8_t *)&event);
        audio_h23_rx_cmd_handler((uint8_t *)&event);
        audio_earc_irq_handler((uint8_t *)&event);
        i2c_miic0_translate_error((uint8_t *)&event);
        break;
    }

    return (int)REG_TCB_CURRENT_TASK;
}

static void video_apply_hpd_level(int asserted)
{
    REG_VIDEO_HPD_STATE = asserted ? 1u : 0u;
    REG_VIDEO_HPD_PENDING = 0u;

    if (asserted) {
        scdc_handle_link_down_cleanup();
        REG_VIDEO_LINK_DOWN = 1u;
    } else {
        video_link_down_cleanup();
    }
}

/* ========================================================================= */
/* CEC / FRL                                                                  */
/* ========================================================================= */

int cec_irq_handler(void)
{
    VIDEO_TASK_GUARD();

    video_enqueue_event(VIDEO_EVENT_CEC_GPIO, 0, 0);

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int frl_txpll_enable_lol(void)
{
    VIDEO_TASK_GUARD();

    REG_VIDEO_FRL_PLL_CTRL |= 0x00800000u;
    delay_loop(1);
    REG_VIDEO_FRL_PLL_CTRL &= ~0x00200000u;
    REG_FRL_STATE = 7u;

    custom_printf("[hdmi21_frl_txpll_enable_lol]\n");

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int frl_rate_set_scdc_state1(void)
{
    VIDEO_TASK_GUARD();
    uint8_t lane_cfg = (REG_SCDC_DSC_ENABLE != 0u) ? 3u : 0u;

    __disable_irq();
    REG_FRL_MUTEX = 1u;
    __enable_irq();

    REG_FRL_RETRY_TIMER = 0u;
    if (scdc_is_state_1()) {
        REG_SCDC_LANE_CFG_A = lane_cfg;
        REG_SCDC_LANE_CFG_B = REG_SCDC_CURRENT_RATE;
        REG_FRL_RETRY_TIMER = 5u;
        custom_printf("#0: FRL Rate set: %d\n", REG_SCDC_CURRENT_RATE);
    }

    REG_FRL_STATE = 9u;

    __disable_irq();
    REG_FRL_MUTEX = 0u;
    if (REG_FRL_DEFERRED_EVENT != 0u) {
        scdc_poll_ddc_idle();
        REG_FRL_DEFERRED_EVENT = 0u;
    }
    __enable_irq();

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int frl_rate_set_scdc_state5_7(void)
{
    VIDEO_TASK_GUARD();
    uint8_t lane_cfg = (REG_SCDC_DSC_ENABLE != 0u) ? 3u : 0u;

    __disable_irq();
    REG_FRL_MUTEX = 1u;
    __enable_irq();

    REG_FRL_RETRY_TIMER = 0u;
    if (scdc_is_state_5()) {
        scdc_fsm_active_to_ready((char)REG_SCDC_CURRENT_RATE, (char)lane_cfg);
        custom_printf("#2: FRL Rate set: %d\n", REG_SCDC_CURRENT_RATE);
        REG_FRL_RETRY_TIMER = 5u;
    } else if (scdc_is_state_7()) {
        scdc_fsm_force_ready((char)REG_SCDC_CURRENT_RATE, (char)lane_cfg, 105637);
        custom_printf("#1: FRL Rate set: %d\n", REG_SCDC_CURRENT_RATE);
        REG_FRL_RETRY_TIMER = 5u;
    }

    REG_FRL_STATE = 3u;

    __disable_irq();
    REG_FRL_MUTEX = 0u;
    if (REG_FRL_DEFERRED_EVENT != 0u) {
        scdc_poll_ddc_idle();
        REG_FRL_DEFERRED_EVENT = 0u;
    }
    __enable_irq();

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int frl_check_pll_lock(void) { return (int)(REG32(0x40100FF4) & 1u); }
int frl_check_pattern_lock(void) { return (int)(REG8(0x40100F93) >> 7); }

int frl_fsm_advance_to_state4(void)
{
    VIDEO_TASK_GUARD();
    int progressed = 0;

    if (REG_FRL_RETRY_TIMER == 0u) {
        video_main_ctrl_set_bit4(0);
        hw_misc_clear_frl_pll_ctrl_bits();
        REG_FRL_STATE = 4u;
        progressed = 1;
    }

    VIDEO_TASK_CHECK();
    return progressed;
}

int frl_fsm_advance_to_state10(void)
{
    VIDEO_TASK_GUARD();
    int progressed = 0;

    if (REG_FRL_RETRY_TIMER == 0u) {
        video_main_ctrl_set_bit4(1);
        REG_FRL_STATE = 10u;
        progressed = 1;
    }

    VIDEO_TASK_CHECK();
    return progressed;
}

int frl_decrement_retry_timer(void)
{
    if (REG_FRL_RETRY_TIMER != 0u)
        --REG_FRL_RETRY_TIMER;
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_get_rate_params(uint8_t *lane_cfg, uint8_t *rate_code)
{
    VIDEO_TASK_GUARD();

    switch (REG_SCDC_FRL_MODE) {
    case 1: *lane_cfg = 3;  *rate_code = 3; break;
    case 2: *lane_cfg = 6;  *rate_code = 3; break;
    case 3: *lane_cfg = 6;  *rate_code = 4; break;
    case 4: *lane_cfg = 8;  *rate_code = 4; break;
    case 5: *lane_cfg = 10; *rate_code = 4; break;
    case 6: *lane_cfg = 12; *rate_code = 4; break;
    default: *lane_cfg = 0; *rate_code = 0; break;
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int frl_fsm_reset_to_state6(void)
{
    REG_FRL_STATE = 6u;
    REG_FRL_MUTEX = 0u;
    REG_FRL_DEFERRED_EVENT = 0u;
    REG_FRL_RETRY_TIMER = 0u;
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_clear_tx_force_bit(void)
{
    REG32(0x40100F38) &= ~0x00008000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_is_state_8_or_higher(void)
{
    return REG_FRL_STATE >= 8u;
}

int frl_fsm_advance_to_state9(void)
{
    if ((REG_HDMI_FRL_STATUS & 0x02000000u) == 0u)
        return 0;
    REG_FRL_STATE = 9u;
    return 1;
}

int frl_apply_rate_config(int enable)
{
    video_frl_rate_regs_apply(REG_SCDC_CURRENT_RATE, enable);
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_set_current_rate(int rate)
{
    REG_SCDC_CURRENT_RATE = (uint8_t)rate;
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_set_tx_force_bit(void)
{
    REG32(0x40100F38) |= 0x00008000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int frl_trigger_link_down_cleanup(void)
{
    scdc_handle_link_down_cleanup();
    return (int)REG_TCB_CURRENT_TASK;
}

/* ========================================================================= */
/* Video timing / audio glue                                                  */
/* ========================================================================= */

int video_is_flag_bit27_set(void)
{
    return (REG_VIDEO_FLAGS & 0x08000000u) != 0u;
}

int video_set_reg_e55(int value)
{
    REG8(0x40100E55) = (uint8_t)value;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_set_reg_31b(int value)
{
    REG8(0x4010131B) = (uint8_t)value;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_main_ctrl_set_bit4(int enabled)
{
    if (enabled)
        REG_VIDEO_MAIN_CTRL |= 0x10u;
    else
        REG_VIDEO_MAIN_CTRL &= ~0x10u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_main_ctrl_set_0a(void)
{
    REG_VIDEO_MAIN_CTRL |= 0x0Au;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_event_queue_init(void)
{
    REG_EVENT_Q_HEAD = 0u;
    REG_EVENT_Q_TAIL = 0u;
    REG_EVENT_Q_OVERFLOW = 0u;
    for (uint8_t i = 0; i < VIDEO_EVENT_COUNT; ++i)
        video_event_mark_empty(i);
    return (int)REG_TCB_CURRENT_TASK;
}

int video_clear_reg_1038_bit25(void)
{
    REG32(0x40101038) &= ~0x02000000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_set_reg_1038_bit25_and_1068(void)
{
    uint8_t v0 = REG8(0x40100412) & 0x1Fu;
    uint8_t v1 = (uint8_t)((((uint32_t)REG8(0x40100413) << 3) & 0x1Fu) + (REG8(0x40100412) >> 5));
    uint8_t v2 = (REG8(0x40100413) >> 2) & 0x1Fu;

    REG32(0x40101038) |= 0x02000000u;
    if ((v0 | v1 | v2) != 0u) {
        REG32(0x4010106C) &= 0xFFFFFF00u;
        REG32(0x40101068) = (REG32(0x40101068) & 0x0000FFFFu) | ((uint32_t)v1 << 24) | ((uint32_t)v2 << 16);
        REG32(0x4010106C) |= v0;
    } else {
        REG32(0x4010106C) |= 0xFFu;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int video_pulse_reg_e1d(void)
{
    VIDEO_TASK_GUARD();

    REG8(0x40100E1D) &= (uint8_t)~0x02u;
    delay_loop(20);
    REG8(0x40100E1D) |= 0x08u;
    delay_loop(20);
    REG8(0x40100E1D) &= (uint8_t)~0x10u;
    delay_loop(20);
    REG8(0x40100E1D) |= 0x04u;

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

static int video_frl_pll_ctrl_update(int enabled)
{
    if (enabled)
        REG_VIDEO_FRL_CTRL &= ~1u;
    else
        REG_VIDEO_FRL_CTRL |= 1u;

    REG_VIDEO_FRL_PLL_CTRL &= 0xFFFCFFFFu;
    REG_VIDEO_FRL_PLL_CTRL |= (uint32_t)(REG8(0x40100C80) & 3u) << 16;
    REG_VIDEO_FRL_PLL_CTRL &= ~0x10000000u;
    REG_VIDEO_FRL_PLL_CTRL |= (uint32_t)(REG8(0x40100C81) >> 7) << 28;
    return (int)REG_TCB_CURRENT_TASK;
}

static int video_frl_pll_regs_init(void)
{
    REG8(0x40100F52) = 64;
    REG8(0x40100C09) = 0xF0u;
    REG8(0x40100C0C) = 0x9Bu;
    REG8(0x40100F19) = 16;
    REG8(0x40100C21) = 0xF6u;
    REG8(0x40100C19) = 51;
    REG8(0x40100C02) = 0xF0u;
    REG8(0x40100C14) = 66;
    REG8(0x40100C34) = 66;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_set_reg_c81_lower(char value)
{
    REG8(0x40100C81) = (uint8_t)((REG8(0x40100C81) & 0xF0u) | ((uint8_t)value & 0x0Fu));
    return (int)REG_TCB_CURRENT_TASK;
}

int video_frl_rate_regs_apply(int rate, int enable)
{
    REG8(0x40100C9E) = 1;
    REG8(0x40100C9D) = 0xC0u;

    if (enable) {
        switch (rate) {
        case 1:
        case 2:
        case 3:
        case 6:
            REG8(0x40100C96) = 16;
            REG8(0x40100C95) = 64;
            REG8(0x40100CBA) = 2;
            REG8(0x40100CB9) = 5;
            REG8(0x40100C97) = 0xB0u;
            REG8(0x40100C91) = 2;
            REG8(0x40100C90) = 5;
            REG8(0x40100C9A) = 32;
            REG8(0x40100CB4) = 5;
            REG8(0x40100C92) = 7;
            REG8(0x40100CA6) = 5;
            break;
        default:
            break;
        }
    }

    return (int)REG_TCB_CURRENT_TASK;
}

int video_is_active_and_not_muted(void)
{
    VIDEO_TASK_GUARD();
    bool active = ((REG32(0x40101000) & 2u) != 0u) && (REG8(0x40451) == 0u);

    if (active && audio_get_format() == 4 && ((REG16(0x40101614) >> 8) & 0x81u) == 0x80u)
        active = false;

    if ((REG8(0x41B98) & 0x80u) != 0u && !active)
        REG8(0x41B98) &= (uint8_t)~1u;

    VIDEO_TASK_CHECK();
    return active ? 1 : 0;
}

int video_set_hpd_pin_level(int hpd_level)
{
    VIDEO_TASK_GUARD();
    __disable_irq();
    REG_VIDEO_HPD_PENDING = 1u;
    __enable_irq();
    video_apply_hpd_level(hpd_level != 0);
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_irq_routing_init(void)
{
    REG_VIDEO_IRQ_ROUTE |= 0x1FFFu;
    REG_VIDEO_IRQ_ROUTE_B |= 0x00030000u;
    REG_VIDEO_MISC_IRQ_EN |= 0x2100u;
    REG8(0x4010131B) = 1;
    REG32(0x40101044) &= 0xFFFFE0FFu;
    REG_VIDEO_FRL_CTRL |= 0x08000000u;
    REG32(0x401002F0) |= 0x303u;
    REG32(0x401002F4) |= 0x1C000000u;
    REG8(0x40100E55) = 69;
    REG8(0x40100E73) |= 0x30u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_hpd_irq_handler(void)
{
    return video_enqueue_event(VIDEO_EVENT_HPD_CHANGE, REG_VIDEO_HPD_STATE, REG_VIDEO_HPD_PENDING);
}

int video_i2c_master_irq_handler(void)
{
    VIDEO_TASK_GUARD();
    uint32_t status;

    REG_VIDEO_I2C_IRQ_STATUS &= REG_VIDEO_I2C_IRQ_ENABLE;
    status = REG_VIDEO_I2C_IRQ_STATUS;

    if ((status & 0x100u) != 0u) {
        REG_FRL_HPD_IRQ_LATCH = 1u;
        REG32(0x40100394) |= 3u;
    }

    if ((status & 0x80000000u) != 0u) {
        if (REG8(0x40441) == 1u)
            ddc_read_data_buffer();
        else if (REG8(0x40441) == 2u)
            ddc_clear_request();

        REG8(0x40440) = 0u;
        REG32(0x40103710) = 0u;
        audio_dispatch_pending_scdc_tasks();
    }

    if ((status & 4u) != 0u)
        ddc_transfer_complete_callback();

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_i2c_slave_irq_handler(void)
{
    REG_VIDEO_I2C_SLAVE_IRQ &= REG_VIDEO_I2C_SLAVE_ENABLE;
    if ((REG_VIDEO_I2C_SLAVE_IRQ & 0x80000000u) != 0u) {
        REG8(0x402A1) = 0u;
        REG_VIDEO_I2C_SLAVE_ENABLE &= ~1u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int video_get_timing_lock_bit3(int atomic)
{
    if (!atomic)
        __disable_irq();
    REG_VIDEO_TIMING_LOCK0 = 93;
    if (!atomic)
        __enable_irq();
    return (REG_VIDEO_TIMING_LOCK1 >> 3) & 1;
}

int video_get_timing_lock_bit6(int atomic)
{
    if (!atomic)
        __disable_irq();
    REG_VIDEO_TIMING_LOCK0 = 0x98u;
    if (!atomic)
        __enable_irq();
    return (REG_VIDEO_TIMING_LOCK1 >> 6) & 1;
}

int video_update_audio_route_earc_ctrl(void)
{
    if (REG_VIDEO_AUDIO_EN_PENDING != 0u || REG_VIDEO_AUDIO_HYST < 0x30u) {
        REG_VIDEO_AUDIO_ROUTE |= 1u;
        REG_VIDEO_EARC_CTRL &= (uint16_t)~0x0400u;
        REG_VIDEO_EARC_CTRL |= 0x0800u;
    } else {
        REG_VIDEO_AUDIO_ROUTE &= (uint8_t)~1u;
        REG_VIDEO_EARC_CTRL &= (uint16_t)~0x0800u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int video_set_earc_ctrl_0800(void)
{
    REG_VIDEO_EARC_CTRL &= (uint16_t)~0x0400u;
    REG_VIDEO_EARC_CTRL |= 0x0800u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_get_timing_lock_bit7(int atomic)
{
    if (!atomic)
        __disable_irq();
    REG_VIDEO_TIMING_LOCK0 = 8;
    if (!atomic)
        __enable_irq();
    return REG_VIDEO_TIMING_LOCK1 >> 7;
}

int video_poll_audio_mute_irq(void)
{
    VIDEO_TASK_GUARD();
    bool mute_irq = ((REG8(0x4010073D) >> 4) & 1u) != 0u;

    REG_VIDEO_TIMING_LOCK0 = 0xCDu;
    if (mute_irq != (REG_VIDEO_AUDIO_MUTE_ACTIVE != 0u)) {
        video_enqueue_event(mute_irq ? 107 : 108, REG_VIDEO_AUDIO_MUTE_CODE, 0);
        REG_VIDEO_AUDIO_MUTE_ACTIVE = mute_irq ? 1u : 0u;
    }

    video_update_audio_infoframes();

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_audio_infoframes(void)
{
    VIDEO_TASK_GUARD();
    int bit_depth = audio_get_bit_depth(REG_VIDEO_AUDIO_MODE);
    uint8_t mute_code = REG_VIDEO_AUDIO_MUTE_CODE & 0x0Fu;

    if (bit_depth == 24)
        REG_VIDEO_AUD_BITS = (uint8_t)((REG_VIDEO_AUD_BITS & 0xF8u) + 4u);
    else if (bit_depth == 16)
        REG_VIDEO_AUD_BITS = (uint8_t)((REG_VIDEO_AUD_BITS & 0xF8u) + 1u);
    else
        REG_VIDEO_AUD_BITS &= 0xF8u;

    if (REG_VIDEO_AUDIO_LAYOUT3D != 0u || REG_VIDEO_AUDIO_MUTE_ACTIVE == 0u || mute_code < 2u) {
        if (REG_VIDEO_AUDIO_MUTE_ACTIVE != 0u && mute_code >= 2u)
            REG_VIDEO_AUD_CH_STATUS = (uint8_t)((REG_VIDEO_AUD_CH_STATUS & 0xF0u) | mute_code);
        else if (REG_VIDEO_AUDIO_FORCE != 0u || bit_depth == 16)
            REG_VIDEO_AUD_CH_STATUS = (uint8_t)((REG_VIDEO_AUD_CH_STATUS & 0xF0u) | 2u);
        else
            REG_VIDEO_AUD_CH_STATUS = (uint8_t)((REG_VIDEO_AUD_CH_STATUS & 0xF0u) | 0x0Bu);

        REG_VIDEO_AUD_LAYOUT |= 0x20u;
    } else {
        REG_VIDEO_AUD_LAYOUT &= (uint8_t)~0x20u;
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_set_audio_fmt_reg(int muted, char idx)
{
    VIDEO_TASK_GUARD();
    uint8_t base = REG_VIDEO_AUD_FMT & 3u;

    if (muted) {
        REG_VIDEO_AUD_FMT = (uint8_t)(base | 4u);
    } else if (audio_get_format() == 5) {
        REG_VIDEO_AUD_FMT = (uint8_t)(base | 0x0Cu | (((uint8_t)idx & 0x0Fu) << 4));
    } else {
        REG_VIDEO_AUD_FMT = (uint8_t)(base | 0x04u | (((uint8_t)idx & 0x0Fu) << 4));
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_clear_h23_rdval_regs(void)
{
    REG8(0x4054F) = 0u;
    REG8(0x40550) &= 0xF0u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_cmdif_set_req_state(int req)
{
    VIDEO_TASK_GUARD();
    REG8((uintptr_t)req + 3u) = 1u;
    REG8(0x40332) = 0u;
    REG8((uintptr_t)req + 7u) = 32u;
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_is_debug_flag_bit0_set(void) { return REG_VIDEO_DEBUG_FLAGS & 1u; }
int video_is_debug_flag_bit5_set(void) { return (REG_VIDEO_DEBUG_FLAGS >> 5) & 1u; }
BOOL video_is_fifo_status_ok(void) { return (REG_VIDEO_FIFO_STATUS & 0x80000000u) == 0u; }

BOOL video_check_bist_status(int value)
{
    return (value != 1 || (REG_VIDEO_BIST_STATUS & 0x04000000u) != 0u)
        && (REG_VIDEO_BIST_STATUS & 0x10000000u) != 0u
        && (REG_VIDEO_BIST_STATUS & 0x00010000u) != 0u;
}

int video_is_debug_flag_bit4_set(void) { return (REG_VIDEO_DEBUG_FLAGS >> 4) & 1u; }
int video_is_debug_flag_bit3_set(void) { return (REG_VIDEO_DEBUG_FLAGS >> 3) & 1u; }
static int video_is_debug_flag_bit2_set(void) { return (REG_VIDEO_DEBUG_FLAGS >> 2) & 1u; }

int video_h23_rd_mode_fsm_tick(void)
{
    VIDEO_TASK_GUARD();

    if (REG8(0x4054A) == 5u) {
        if (REG8(0x40101974) == 46u) {
            REG8(0x4054B) = 0u;
            REG8(0x40101940) |= 8u;
            delay_loop(10);
            REG8(0x40101940) &= (uint8_t)~8u;
            REG8(0x4054A) = 4u;
            REG32(0x401012AC) |= 2u;
            REG8(0x4054C) = 0u;
        } else {
            ++REG8(0x4054C);
        }
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_process_event_queue(int a1, int a2, int a3)
{
    (void)a1;
    (void)a2;
    (void)a3;

    VIDEO_TASK_GUARD();
    VideoEvent event;

    if (video_dequeue_event(&event))
        video_dispatch_event(event);

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_enqueue_event(int code, char arg0, char arg1)
{
    VIDEO_TASK_GUARD();
    uint8_t tail;
    VideoEvent event;

    if (code == 1) {
        VIDEO_TASK_CHECK();
        return 1;
    }

    __disable_irq();
    tail = REG_EVENT_Q_TAIL;
    if (video_event_read(tail).code == 1u) {
        event.code = (uint8_t)code;
        event.arg0 = (uint8_t)arg0;
        event.arg1 = (uint8_t)arg1;
        video_event_write(tail, event);
        REG_EVENT_Q_TAIL = video_event_next(tail);

        if (REG_EVENT_Q_OVERFLOW == 0u && video_queue_depth(REG_EVENT_Q_HEAD, REG_EVENT_Q_TAIL) >= 26u) {
            video_set_fatal_flag();
            hw_misc_set_state_36(0, 234495);
            REG_EVENT_Q_OVERFLOW = 1u;
        }

        __enable_irq();
        VIDEO_TASK_CHECK();
        return 0;
    }

    __enable_irq();
    VIDEO_TASK_CHECK();
    return 1;
}

int video_debug_print_status(void)
{
    static uint32_t loop_print_divider;

    if (++loop_print_divider >= 400u) {
        custom_printf("Cobra run... %08x,%02x,%02x,%02x,%02x,%02x,%02x\n",
                      REG32(0x40100E94),
                      REG_FRL_STATE,
                      REG8(0x404BE),
                      REG_VIDEO_VPLL_STATE,
                      REG8(0x40440),
                      REG8(0x40385),
                      REG8(0x401CE));
        loop_print_divider = 0u;
    }

    return (int)REG_TCB_CURRENT_TASK;
}

int video_calculate_checksum(int src)
{
    uint8_t sum = 0u;
    uint8_t len = REG8((uintptr_t)src);

    for (uint8_t i = 0; i < len; ++i)
        sum = (uint8_t)(sum + REG8((uintptr_t)src + 1u + i));
    return sum;
}

int video_get_timing3(void) { return REG_VIDEO_TIMING3; }
int video_get_tmds_high(void) { return REG_VIDEO_TMDS_HIGH; }
int video_get_tmds_low(void) { return REG_VIDEO_TMDS_LOW; }
int video_get_pixel_clock(void) { return REG_VIDEO_PIXEL_CLOCK; }
int video_get_timing2(void) { return REG_VIDEO_TIMING2; }
int video_get_timing4(void) { return REG_VIDEO_TIMING4; }
int video_get_timing1(void) { return REG_VIDEO_TIMING1; }
int video_get_timing0(void) { return REG_VIDEO_TIMING0; }

int video_pulse_reg_e1d_wrapper(void)
{
    return video_pulse_reg_e1d();
}

int video_cmdif_stream_regs_if_debug(void)
{
    if (video_is_debug_flag_bit3_set())
        video_cmdif_stream_regs();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_poll_vpll_warn_irq(void)
{
    VIDEO_TASK_GUARD();
    REG8(0x40101321) = 0u;
    if (REG8(0x401DD) != REG8(0x40101322)) {
        REG8(0x401DD) = REG8(0x40101322);
        video_enqueue_event(VIDEO_EVENT_VPLL_WARN, REG8(0x40101322), 0);
    }
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_timers_abc(void)
{
    VIDEO_TASK_GUARD();

    if (REG_VIDEO_TIMER_A != 0u && --REG_VIDEO_TIMER_A == 0u)
        hw_misc_audio_route_earc_ctrl();

    if (REG_VIDEO_TIMER_B != 0u && --REG_VIDEO_TIMER_B == 0u) {
        hw_misc_process_state_150(0);
        REG_VIDEO_TIMER_A = 100u * REG32(0x40100EF0);
    }

    if (REG_VIDEO_TIMER_C != 0u && --REG_VIDEO_TIMER_C == 0u) {
        if (REG_VIDEO_TIMER_C_STEPS < 8u && ++REG_VIDEO_TIMER_C_STEPS == 8u) {
            int route = 0;
            if (audio_get_format() == 5) {
                REG8(0x4010161B) = 0x81u;
                route = REG8(0x4010168C) & 0x1Cu;
            } else if (audio_get_format() == 4) {
                REG8(0x4010161B) = 0x80u;
                route = REG8(0x4010168C) & 0x1Cu;
            }

            if (REG_VIDEO_TIMER_C_VALID == 0u || ((route != 0) != (REG_VIDEO_TIMER_C_FLAG != 0u))) {
                if (route != 0) {
                    REG_VIDEO_AUDIO_HYST = 48u;
                    video_update_audio_route_earc_ctrl();
                    hw_misc_poll_audio_mute_and_csb();
                    video_enqueue_event(VIDEO_EVENT_AUDIO_ROUTE, (char)route, 0);
                } else {
                    REG_VIDEO_AUDIO_HYST = 0u;
                    video_update_audio_route_earc_ctrl();
                }
                REG_VIDEO_TIMER_C_FLAG = (route != 0) ? 1u : 0u;
                REG_VIDEO_TIMER_C_VALID = 1u;
            }

            REG_VIDEO_TIMER_C_STEPS = 0u;
        }
        hw_misc_audio_route_earc_ctrl();
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_h23_param0(void)
{
    VIDEO_TASK_GUARD();

    REG8(0x40101C03) &= 0xFCu;
    if (audio_protocol_get_status_bit()) {
        uint8_t v = (uint8_t)audio_h23_get_rdval1_bit3() | (uint8_t)(2 * audio_h23_get_rdval1_bit2());
        REG8(0x40101C03) |= v;
    }
    if (audio_protocol_get_mode() == 1)
        REG8(0x40101C03) |= 1u;

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_math_pow2(int value)
{
    int result = 1;
    for (int i = 0; i < value; ++i)
        result <<= 1;
    return result;
}

int video_update_reg_1607_bit4(void)
{
    VIDEO_TASK_GUARD();
    if ((REG8(0x403C0) & 2u) != 0u && REG8(0x40392) != 0u)
        REG8(0x40101607) |= 0x10u;
    else
        REG8(0x40101607) &= (uint8_t)~0x10u;
    video_enqueue_event(117, (char)REG8(0x40101607), 0);
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_print_cs_regs(void)
{
    VIDEO_TASK_GUARD();
    custom_printf("CS0 %02x\n", REG8(0x40101628));
    custom_printf("CS3 %02x\n", REG8(0x4010162B));
    custom_printf("CS5 %02x\n", REG8(0x4010162D));
    custom_printf("CS18 %02x\n", REG8(0x4010163A));
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_print_csb_status(void)
{
    VIDEO_TASK_GUARD();
    if ((REG_VIDEO_AUDIO_MUTE_CODE & 0x0Fu) <= 1u)
        custom_printf("CSB sa w==0/1\n");
    else
        custom_printf("CSB sa w!=0\n");
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_copy_to_h23_fifo(int src, unsigned int len)
{
    for (unsigned int i = 0; i < len; ++i)
        REG8(0x40101986 + i) = REG8((uintptr_t)src + i);
    return (int)REG_TCB_CURRENT_TASK;
}

int video_get_h23_work_pending(void)
{
    return REG8(0x4054B);
}

int video_read_h23_readbuf(int offset)
{
    REG8(0x4010198A) = (uint8_t)offset;
    return REG8(0x4010197E);
}

int video_enqueue_event_110_0(void)
{
    return video_enqueue_event(110, 0, 0);
}

int video_enqueue_event_110_ff(void)
{
    return video_enqueue_event(110, (char)0xFF, 0);
}

int video_poll_audio_mute_irq_wrapper(void)
{
    return video_poll_audio_mute_irq();
}

int video_check_reg_bf_bit4(void)
{
    REG8(0x401000BD) = 4u;
    return (REG8(0x401000BF) >> 4) & 1;
}

static int video_init_regs_1200_0232(void)
{
    REG8(0x40101200) = 17u;
    REG8(0x40101201) = 17u;
    REG8(0x40100232) = 1u;
    REG8(0x40100238) |= 4u;
    REG8(0x401002D8) |= 0x70u;
    REG8(0x401002C4) |= 0x40u;
    return (int)REG_TCB_CURRENT_TASK;
}

static int video_init_regs_434_43c(void)
{
    REG8(0x40434) = 0u;
    REG8(0x40439) = 0u;
    REG16(0x4043C) = 0x8000u;
    REG8(0x40438) = 0u;
    return (int)REG_TCB_CURRENT_TASK;
}

static int video_init_regs_2d8_214(void)
{
    audio_init_spdif_i2s_regs();
    REG32(0x401002D8) |= 0x3000u;
    REG32(0x40100214) |= 0x10000000u;
    REG32(0x40100218) = (REG32(0x40100218) & 0xFFFFFFCFu) | 0x20u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_hw_init_sequence(void)
{
    VIDEO_TASK_GUARD();
    video_init_regs_434_43c();
    video_init_regs_1200_0232();
    video_init_regs_2d8_214();
    REG_VIDEO_STATUS_IRQ &= ~0x20u;
    REG_VIDEO_STATUS_IRQ &= ~0x10u;
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_timer_ed0(int a1, int a2, int a3)
{
    (void)a1;
    VIDEO_TASK_GUARD();
    uint32_t elapsed = video_calc_elapsed_ticks((uint8_t *)(uintptr_t)0x401A8, a2, a3);
    if (REG32(0x40100ED0) < elapsed)
        REG32(0x40100ED0) = elapsed;
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_set_misc_audio_ctrl_81(void)
{
    REG32(0x4010052A) = 0xFFFFFF81u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_set_path_ctrl_bit27(int enable)
{
    if (enable)
        REG_VIDEO_PATH_CTRL |= 0x08000000u;
    else
        REG_VIDEO_PATH_CTRL &= ~0x08000000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_update_h23_rdval_from_197c(void)
{
    REG8(0x4054F) = (uint8_t)((((REG16(0x4010197C) >> 4) & 0x1Fu) << 3) | ((REG8(0x4010197D) >> 1) & 7u));
    REG8(0x40550) = (uint8_t)((REG8(0x40550) & 0xF0u)
                            | ((REG8(0x4010197C) & 8u) != 0u)
                            | ((((REG8(0x4010197C) & 4u) != 0u) ? 1u : 0u) << 1)
                            | ((((REG8(0x4010197C) & 2u) != 0u) ? 1u : 0u) << 2)
                            | ((REG8(0x4010197C) & 1u) << 3));
    return (int)REG_TCB_CURRENT_TASK;
}

int video_enqueue_event_103(void)
{
    return video_enqueue_event(103, 0, 0);
}

int video_enqueue_event_104(void)
{
    return video_enqueue_event(104, 0, 0);
}

int video_handle_fatal_error_overlay(int code, int fatal)
{
    VIDEO_TASK_GUARD();
    uint8_t bit_a = 0u;
    uint8_t bit_b = 0u;

    switch (code) {
    case 0: bit_a = 1u;   break;
    case 1: bit_a = 2u;   break;
    case 2: bit_a = 4u;   break;
    case 3: bit_a = 8u;   break;
    case 4: bit_a = 16u;  break;
    case 5: bit_a = 32u;  break;
    case 6: bit_a = 64u;  break;
    case 7: bit_a = 128u; break;
    case 16: bit_b = 64u; break;
    case 17: bit_b = 128u; break;
    default: break;
    }

    if ((REG8(0x4010040E) & bit_a) != 0u) {
        REG8(0x4010040C) |= bit_a;
        if ((REG8(0x40100410) & bit_a) != 0u) {
            if (!fatal)
                __disable_irq();
            REG8(0x40100405) = 1u;
            video_enqueue_event(VIDEO_EVENT_IRQ_125, (char)REG8(0x40100410), (char)bit_a);
            if (!fatal)
                __enable_irq();
        }
    }

    if ((REG8(0x4010040F) & bit_b) != 0u) {
        REG8(0x4010040D) |= bit_b;
        if ((REG8(0x40100411) & bit_b) != 0u) {
            if (!fatal)
                __disable_irq();
            REG8(0x40100405) = 1u;
            video_enqueue_event(VIDEO_EVENT_IRQ_126, (char)REG8(0x40100411), (char)bit_b);
            if (!fatal)
                __enable_irq();
        }
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_reg_335_353(void)
{
    if (REG8(0x401DA) == 1u) {
        if (REG8(0x401DB) == 1u) {
            if (REG_VIDEO_LINK_DOWN == 1u) {
                REG8(0x40100335) = 7u;
                REG8(0x40100353) = 7u;
            } else {
                REG8(0x40100335) = 3u;
                REG8(0x40100353) = 3u;
            }
        } else if (REG_VIDEO_LINK_DOWN == 1u) {
            REG8(0x40100335) = 5u;
            REG8(0x40100353) = 5u;
        } else {
            REG8(0x40100335) = 1u;
            REG8(0x40100353) = 1u;
        }
    } else {
        REG8(0x40100335) = 0u;
        REG8(0x40100353) = 0u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int video_update_reg_335_353_wrapper(void)
{
    return video_update_reg_335_353();
}

unsigned int video_calc_elapsed_ticks(uint8_t *timestamp, int a2, int a3)
{
    (void)a2;
    VIDEO_TASK_GUARD();
    uint8_t now_subsec;
    uint16_t now_sec;
    uint32_t now_ticks;
    unsigned int elapsed = 0u;

    video_snapshot_event_time((int)&now_subsec, 0);
    now_sec = *(uint16_t *)((uintptr_t)&now_subsec + 2u);
    now_ticks = a3;

    if (timestamp != NULL) {
        uint8_t old_subsec = timestamp[0];
        uint16_t old_sec = *(volatile uint16_t *)(timestamp + 2);

        if (old_subsec <= now_subsec) {
            elapsed = (unsigned int)(now_subsec - old_subsec);
        } else {
            elapsed = (unsigned int)(now_subsec + 100u - old_subsec);
            now_sec = (now_sec == 0u) ? 999u : (uint16_t)(now_sec - 1u);
        }

        if (old_sec <= now_sec)
            elapsed += 100u * (unsigned int)(now_sec - old_sec);
        else
            elapsed += 100u * (unsigned int)(now_sec + 1000u - old_sec);

        *(volatile uint32_t *)(timestamp + 0) = *(uint32_t *)(uintptr_t)&now_subsec;
        *(volatile uint32_t *)(timestamp + 4) = now_ticks;
    }

    VIDEO_TASK_CHECK();
    return elapsed;
}

int video_set_fatal_flag(void)
{
    REG_VIDEO_FATAL_FLAG = 1u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_clear_fatal_flag(void)
{
    REG_VIDEO_FATAL_FLAG = 0u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_snapshot_event_time(int base, int atomic)
{
    VIDEO_TASK_GUARD();
    if (base != 0) {
        if (!atomic)
            __disable_irq();
        REG8((uintptr_t)base + 0u) = REG_EVENT_TIME_SUBSEC;
        REG16((uintptr_t)base + 2u) = REG_EVENT_TIME_SEC;
        REG32((uintptr_t)base + 4u) = REG_EVENT_TIME_TICKS;
        if (!atomic)
            __enable_irq();
    }
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_check_mode_flags_bit10(void) { return (REG32(0x40100384) >> 10) & 1u; }
int video_check_mode_flags_bit11(void) { return (REG32(0x40100384) >> 11) & 1u; }

int video_process_reg_1e0_commands(unsigned int count)
{
    VIDEO_TASK_GUARD();

    if (count > 16u)
        count = 16u;

    for (unsigned int i = 0; i < count; ++i) {
        uint8_t tag = REG8(0x401E0 + i) & 0x1Fu;
        uint8_t op = REG8(0x401E0 + i) & 0x60u;
        if (op == 0x20u) {
            REG8(0x40202 + tag) = REG8(0x40220 + i);
            if (tag == 0u) {
                REG8(0x40439) = 1u;
                REG16(0x4043C) = (uint16_t)((REG16(0x4043C) & 0xFF00u) | REG8(0x401E2));
            } else if (tag == 1u) {
                REG8(0x40439) = 1u;
                REG16(0x4043C) = (uint16_t)((REG16(0x4043C) & 0x00FFu) | ((uint16_t)REG8(0x401E3) << 8));
            } else if (tag == 2u) {
                REG8(0x4043A) = 1u;
                REG8(0x4043B) = REG8(0x401E4);
            }
        } else if (op == 0x60u) {
            REG8(0x40220 + i) = 0u;
        }
    }

    REG8(0x401002B5) |= 1u;
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_irq_handler_a_high(void)
{
    VIDEO_TASK_GUARD();
    uint32_t pending = REG_VIDEO_IRQ_ACK_A & REG_VIDEO_IRQ_MASK_A & 0x07000000u;
    REG_VIDEO_IRQ_ACK_A = pending;

    if ((pending & 0x04000000u) != 0u) {
        audio_h14_fsm_init();
        video_enqueue_event(62, 0, 0);
    }
    if ((pending & 0x01000000u) != 0u) {
        audio_h14_aphy_check_status();
        video_enqueue_event(63, 0, 0);
    }
    if ((pending & 0x02000000u) != 0u) {
        REG16(0x4052D) |= 0x20u;
        REG32(0x40100450) |= ((uint32_t)REG16(0x40536) << 16) | ((uint32_t)REG16(0x4052D) << 8);
        audio_h14_aphy_check_error();
        video_enqueue_event(64, 0, 0);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_irq_handler_a_low(void)
{
    VIDEO_TASK_GUARD();
    uint32_t pending = REG_VIDEO_IRQ_ACK_A & REG_VIDEO_IRQ_MASK_A & 0x0007FFFFu;
    REG_VIDEO_IRQ_ACK_A = pending;

    if ((pending & 0x20u) != 0u) {
        audio_h23_fsm_init();
        video_enqueue_event(55, 0, 0);
    }
    if ((pending & 0x200u) != 0u) {
        audio_h23_cfg_state_advance_to_2();
        video_enqueue_event(56, 0, 0);
    }
    if ((pending & 0x10u) != 0u) {
        audio_h23_aphy_check_status();
        video_enqueue_event(58, 0, 0);
    }
    if ((pending & 0x20000u) != 0u) {
        audio_h23_cfg_state_advance_to_0();
        video_enqueue_event(57, 0, 0);
    }
    if ((pending & 0x2000u) != 0u) {
        audio_h23_cfg_state_reset_alt();
        video_enqueue_event(59, 0, 0);
        REG8(0x4052C) = 0u;
        video_enqueue_event(VIDEO_EVENT_PATH_RESET, 0, 0);
    }
    if ((pending & 0x1000u) != 0u) {
        REG16(0x4052D) |= 0x20u;
        REG32(0x40100450) |= ((uint32_t)REG16(0x40536) << 16) | ((uint32_t)REG16(0x4052D) << 8);
        audio_h23_cfg_state_reset();
        video_enqueue_event(60, 0, 0);
        REG8(0x4052C) = 0u;
        video_enqueue_event(VIDEO_EVENT_PATH_RESET, 0, 0);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_read_reg_8b_banked(unsigned int addr)
{
    REG8(0x40100063) &= 0x0Fu;
    if (addr >= 0x10u) {
        REG8(0x40100063) |= 0xA0u;
        addr -= 0x10u;
    } else {
        REG8(0x40100063) |= 0x90u;
    }
    return REG8(0x4010008B + addr);
}

int video_init_hpd_and_frl_state(void)
{
    video_event_queue_init();
    REG_VIDEO_HPD_STATE = 0u;
    REG_VIDEO_HPD_PENDING = 0u;
    REG_VIDEO_LINK_DOWN = 0u;
    REG_FRL_STATE = 0u;
    REG_FRL_RETRY_TIMER = 0u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_fsm_poll_audio_and_h23(void)
{
    video_poll_audio_mute_irq();
    video_h23_rd_mode_fsm_tick();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_toggle_trained_mask_msb(void)
{
    if (REG_SCDC_TRAINED_MASK != 0u)
        REG_SCDC_TRAINED_MASK ^= 1u << (31u - __clz(REG_SCDC_TRAINED_MASK));
    return (int)REG_TCB_CURRENT_TASK;
}

int video_main_ctrl_set_bit0(int enable)
{
    if (enable)
        REG_VIDEO_MAIN_CTRL |= 1u;
    else
        REG_VIDEO_MAIN_CTRL &= ~1u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_reg_f38_set_bit15(int enable)
{
    if (enable)
        REG32(0x40100F38) |= 0x00008000u;
    else
        REG32(0x40100F38) &= ~0x00008000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_main_ctrl_set_bit4_cond(int enable)
{
    if (enable) {
        if ((REG_VIDEO_HPD_CFG & 0x80000000u) != 0u)
            REG_VIDEO_MAIN_CTRL &= ~0x10u;
        else
            REG_VIDEO_MAIN_CTRL |= 0x10u;
    } else {
        REG_VIDEO_MAIN_CTRL &= ~0x10u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int video_link_down_cleanup(void)
{
    REG_VIDEO_VPLL_STATE = 0u;
    REG_VIDEO_LINK_DOWN = 0u;
    video_main_ctrl_set_bit4_cond(0);
    video_main_ctrl_set_bit0(0);
    video_reg_f38_set_bit15(0);
    REG_VIDEO_FRL_PLL_CTRL &= ~0x00400000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_path_ctrl_enable(void)
{
    video_set_path_ctrl_bit27(1);
    video_update_reg_335_353();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_path_ctrl_disable(void)
{
    video_set_path_ctrl_bit27(0);
    video_update_reg_335_353();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_update_timer_119_wrapper(int a1, int a2, int a3)
{
    (void)a1;
    VIDEO_TASK_GUARD();
    video_update_start_timer(262560, a2, a3);
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_init_vpll_and_phase_state(void)
{
    VIDEO_TASK_GUARD();
    bzero((uint32_t *)0x41080, 84);
    REG32(0x40100000) |= 0x07000000u;
    REG32(0x40101284) &= 0xFCFFFFFFu;
    REG_VIDEO_VPLL_STATE = 0u;
    REG_VIDEO_SM_STATE = 0u;
    REG8(0x41084) = 0u;
    REG8(0x41088) = 0u;
    hw_misc_disable_irq_a_b_bit1();
    REG8(0x40274) = 0u;
    REG8(0x40101838) |= 0x40u;
    REG8(0x40268) = 0u;
    REG8(0x4026B) = 0u;
    REG16(0x40272) = 0u;
    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

static int video_init_regs_10c_108(void)
{
    video_main_ctrl_set_0a_and_clear();
    REG32(0x4010010C) &= 0xFF000000u;
    REG32(0x40100108) |= 0xBB610000u;
    REG32(0x4010010C) |= 0x04048002u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_trigger_scdc_event_84_cond1(void)
{
    if (REG_VIDEO_VPLL_STATE >= 2u && REG_VIDEO_VPLL_STATE != 10u)
        scdc_handle_link_down_cleanup();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_trigger_scdc_event_84_cond2(void)
{
    if (REG_VIDEO_VPLL_STATE >= 2u)
        scdc_handle_link_down_cleanup();
    return (int)REG_TCB_CURRENT_TASK;
}

int video_main_ctrl_set_0a_and_clear(void)
{
    REG_VIDEO_MAIN_CTRL |= 0x0Au;
    video_main_ctrl_set_bit4_cond(0);
    video_main_ctrl_set_bit0(0);
    video_reg_f38_set_bit15(0);
    REG32(0x40100048) = (REG32(0x40100048) & 0xFF00FFFFu) | 0x00A70000u;
    hw_misc_set_reg_114_bits(0, 0, 0);
    REG_VIDEO_FRL_PLL_CTRL &= ~0x00400000u;
    REG_VIDEO_FRL_PLL_CTRL &= 0x30FFFFFFu;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_sm_state_set_1(void)
{
    video_init_regs_10c_108();
    REG_VIDEO_SM_STATE = 1u;
    return (int)REG_TCB_CURRENT_TASK;
}

int video_fsm_vpll_sm_state(int a1, int a2, int a3)
{
    (void)a1;
    (void)a2;
    (void)a3;

    VIDEO_TASK_GUARD();
    int ctx = 0;

    while (REG_VIDEO_SM_STATE != 0u) {
        bool again = false;

        hw_misc_lock_hpd_state(&ctx);
        switch (REG_VIDEO_SM_STATE) {
        case 1:
            video_sm_state_set_1();
            break;
        case 2:
            again = (video_trigger_scdc_event_84_cond2() != 0);
            break;
        case 3:
            hw_misc_set_reg_114_bits(3, 0, 0);
            break;
        default:
            break;
        }

        __disable_irq();
        if (REG_VIDEO_HPD_PENDING != 0u)
            again = false;
        __enable_irq();

        hw_misc_unlock_hpd_state_and_check(&ctx);
        if (!again)
            break;
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_fsm_drain_events_and_hpd(int a1, int a2, int a3)
{
    (void)a1;
    (void)a2;
    (void)a3;

    VIDEO_TASK_GUARD();
    int events = 0;

    video_fsm_poll_audio_and_h23();
    video_process_event_queue(0, 0, 0);
    if (REG_VIDEO_HPD_PENDING != 0u) {
        video_apply_hpd_level(REG_VIDEO_HPD_STATE != 0u);
        ++events;
    }
    if (frl_is_state_8_or_higher())
        ++events;

    VIDEO_TASK_CHECK();
    return events;
}

int video_check_mode_flags_bit8(void) { return (REG32(0x40100384) >> 8) & 1u; }
int video_clear_mode_flags_bit8(void) { REG32(0x40100384) &= ~0x00000100u; return (int)REG_TCB_CURRENT_TASK; }
int video_check_mode_flags_bit16(void) { return (REG32(0x40100384) >> 16) & 1u; }
int video_clear_mode_flags_bit16(void) { REG32(0x40100384) &= ~0x00010000u; return (int)REG_TCB_CURRENT_TASK; }

/* ========================================================================= */
/* Small compatibility shims for not-yet-rewritten late video logic           */
/* ========================================================================= */

int video_cmdif_stream_regs(void)
{
    VIDEO_TASK_GUARD();
    uint16_t dst = REG_VIDEO_CMDIF_PTR;

    REG_VIDEO_CMDIF_A = REG8(0x4010035B);
    REG_VIDEO_CMDIF_B = REG8(0x40100459);

    video_cmdif_stream(&dst, 0x402A8u, 4u);
    video_cmdif_stream(&dst, 0x41BD9u, 1u);
    video_cmdif_stream(&dst, 0x41BDCu, 2u);
    video_cmdif_stream(&dst, 0x41BDAu, 1u);
    video_cmdif_stream(&dst, 0x41BDBu, 1u);
    video_cmdif_stream(&dst, 0x41BD8u, 1u);
    video_cmdif_stream(&dst, 0x41B99u, 1u);
    video_cmdif_stream(&dst, 0x41B9Au, 0x32u);
    video_cmdif_stream(&dst, 0x40443u, 1u);
    video_cmdif_stream(&dst, 0x402A2u, 1u);
    video_cmdif_stream(&dst, 0x402A3u, 1u);
    video_cmdif_stream(&dst, 0x41B98u, 1u);
    video_cmdif_stream(&dst, 0x40494u, 1u);

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

static int video_start_timer_init(void)
{
    VIDEO_TASK_GUARD();

    bool flag0 = false;
    bool flag1 = false;
    uint32_t calc = REG32(0x41BD4);

    if (REG_VIDEO_TIMING_ACTIVE == 0u) {
        audio_calc_set_result((int)calc);
        flag0 = audio_calc_check_result_valid() != 0;
        flag1 = calc > 1135214592u;
    }

    audio_calc_set_rx_flags(flag0 ? 1 : 0, flag1 ? 1 : 0);
    audio_calc_flags_set(flag0 ? 1 : 0, flag1 ? 1 : 0);
    REG_VIDEO_START_TIMER = 10000u;
    video_snapshot_event_time((int)REG_VIDEO_START_TIMESTAMP, 0);

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_enqueue_event_110_0_alt(void) { return video_enqueue_event(110, 0, 0); }
int video_enqueue_event_110_ff_alt(void) { return video_enqueue_event(110, (char)0xFF, 0); }

int video_audio_irq_handler(void)
{
    VIDEO_TASK_GUARD();

    REG_VIDEO_AUDIO_IRQ_STATUS &= REG_VIDEO_AUDIO_IRQ_MASK;
    uint8_t status = REG_VIDEO_AUDIO_IRQ_STATUS;

    if ((status & 0x10u) != 0u) {
        if (audio_h14_status_reg_b_check_bit14()) {
            audio_irq_ack_a_bit4();
            __disable_irq();
            video_enqueue_event(11, (char)audio_protocol_get_state_code(), 64);
            __enable_irq();
            __disable_irq();
            video_enqueue_event(72, 0, 0);
            __enable_irq();
        }
    }

    if ((status & 0x20u) != 0u) {
        scdc_handle_earc_32ch_event();
        video_enqueue_event(73, 0, 0);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_audio_irq2_handler(void)
{
    VIDEO_TASK_GUARD();

    REG_VIDEO_AUDIO_IRQ2_STATUS &= REG_VIDEO_AUDIO_IRQ2_MASK;
    uint16_t status = REG_VIDEO_AUDIO_IRQ2_STATUS;

    if ((status & 0x0010u) != 0u) {
        audio_h23_substate_advance_to_2();
        video_enqueue_event(66, 0, 0);
    }
    if ((status & 0x0008u) != 0u) {
        audio_h23_substate_advance_to_3_or_4();
        video_enqueue_event(67, 0, 0);
    }
    if ((status & 0x0400u) != 0u) {
        audio_h23_substate_advance_to_5();
        video_enqueue_event(68, 0, 0);
    }
    if ((status & 0x0002u) != 0u) {
        audio_h23_substate_advance_to_6();
        video_enqueue_event(69, 0, 0);
    }
    if ((status & 0x0004u) != 0u) {
        audio_h23_parse_discovery_pkt();
        video_enqueue_event(70, 0, 0);
    }
    if ((status & 0x0200u) != 0u) {
        audio_enqueue_video_event_11_4();
        video_enqueue_event(71, 0, 0);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

int video_update_start_timer(int a1, int a2, int a3)
{
    (void)a1;
    VIDEO_TASK_GUARD();
    int lock_ctx = 0;

    if (REG_VIDEO_START_TIMER != 0u) {
        hw_misc_lock_hpd_state(&lock_ctx);
        if (REG_VIDEO_START_TIMER != 0u) {
            unsigned int elapsed = video_calc_elapsed_ticks(REG_VIDEO_START_TIMESTAMP, a2, a3);
            if (REG_VIDEO_START_TIMER <= elapsed)
                REG_VIDEO_START_TIMER = 0u;
            else
                REG_VIDEO_START_TIMER -= elapsed;

            if (REG_VIDEO_START_TIMER == 0u) {
                __disable_irq();
                if (REG_VIDEO_HPD_PENDING == 0u) {
                    video_main_ctrl_set_bit4_cond(1);
                    video_main_ctrl_set_bit4(1);
                    if (REG_VIDEO_TIMING_ACTIVE != 0u && REG_VIDEO_AUDIO_VALID != 0u && REG_VIDEO_PHASE_BYPASS == 0u) {
                        REG_VIDEO_LINK_DOWN = 0u;
                        video_update_reg_335_353();
                        video_handle_fatal_error_overlay(4, 0);
                    } else {
                        REG_VIDEO_LINK_DOWN = 1u;
                        video_update_reg_335_353();
                    }
                }
                __enable_irq();

                hw_misc_clear_reg_492_and_irq_b();
                hw_misc_process_state_60();
                REG8(0x401000A0) &= 0xEBu;
                REG_VIDEO_PHASE_POLL_TIMER = 7u;
                if (REG_VIDEO_TIMING_ACTIVE != 0u)
                    scdc_set_hpd_debounce_timer();
                video_is_debug_flag_bit2_set();
                custom_printf("Start Video\n");
                REG_VIDEO_PHASE_GOOD = 1u;
                if (REG8(0x4051C) != 0u)
                    audio_protocol_start_machine();
            }
        }
        hw_misc_unlock_hpd_state_and_check(&lock_ctx);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}

static int video_check_timing_changed(int a1, int a2, int a3)
{
    (void)a1;
    (void)a2;
    (void)a3;
    VIDEO_TASK_GUARD();

    bool changed = false;
    if (audio_get_h14_status0() != 0) {
        if (REG_VIDEO_TMDS_LOW != (uint16_t)audio_get_h14_status0()
            || REG_VIDEO_TMDS_HIGH != (uint16_t)audio_get_h14_status1()
            || REG_VIDEO_TIMING3 != ((uint16_t)audio_get_h14_status2() & 0x7FFFu)) {
            changed = true;
        }
    } else {
        if (REG_VIDEO_TMDS_LOW != (uint16_t)video_get_tmds_low()
            || REG_VIDEO_TMDS_HIGH != (uint16_t)video_get_tmds_high()
            || REG_VIDEO_TIMING3 != ((uint16_t)video_get_timing3() & 0x7FFFu)) {
            changed = true;
        }
    }

    if (REG_VIDEO_PIXEL_CLOCK != (uint16_t)video_get_pixel_clock()) {
        changed = true;
        custom_printf("phase test: hwidth changed!!!\n");
    }

    if (changed) {
        int lock_ctx = 0;
        REG8(0x40268) = 0u;
        hw_misc_unlock_hpd_state_and_check(&lock_ctx);
        __disable_irq();
        scdc_handle_link_down_cleanup();
        REG_VIDEO_HPD_STATE = 1u;
        REG_VIDEO_HPD_PENDING = 0u;
        __enable_irq();
        hw_misc_lock_hpd_state(&lock_ctx);
        video_link_down_cleanup();
    }

    VIDEO_TASK_CHECK();
    return changed ? 1 : 0;
}

int video_update_phase_timers(int a1, int a2, int a3)
{
    (void)a1;
    VIDEO_TASK_GUARD();
    int lock_ctx = 0;

    video_update_start_timer(262560, a2, a3);

    if (REG8(0x4026B) != 0u)
        --REG8(0x4026B);

    if (REG_VIDEO_PHASE_POLL_TIMER != 0u || REG_VIDEO_PHASE_RETRY_TIMER != 0u
        || REG_VIDEO_PHASE_ARM_TIMER != 0u || REG_VIDEO_PHASE_DISABLE != 0u) {
        hw_misc_lock_hpd_state(&lock_ctx);

        if (REG_VIDEO_PHASE_RETRY_TIMER != 0u && --REG_VIDEO_PHASE_RETRY_TIMER == 0u)
            video_start_timer_init();

        if (REG_VIDEO_PHASE_DISABLE != 0u && --REG_VIDEO_PHASE_DISABLE == 0u) {
            REG32(0x4010125C) = 0x40000u;
            __disable_irq();
            REG_VIDEO_SCDC_IRQ_MASK |= 0x40000u;
            scdc_enable_tmds_mode();
            __enable_irq();
        }

        if (REG_VIDEO_PHASE_POLL_TIMER != 0u && --REG_VIDEO_PHASE_POLL_TIMER == 0u) {
            int phase = video_read_phase_sample();
            if (phase == 0 || phase == -1) {
                REG_VIDEO_PHASE_POLL_TIMER = 7u;
            } else if (REG_VIDEO_PHASE_MAX > (uint32_t)phase && REG_VIDEO_PHASE_MIN < (uint32_t)phase) {
                if (REG_VIDEO_PHASE_GOOD == 0u) {
                    custom_printf("good f-phase: %08x\n", phase);
                    REG_VIDEO_PHASE_GOOD = 1u;
                    if (REG8(0x4051C) != 0u)
                        audio_protocol_start_machine();
                }
                REG_VIDEO_PHASE_POLL_TIMER = 7u;
            } else {
                custom_printf("F-PHASE unlock\n, De-ass vid\n");
                scdc_enable_frl_mode();
                __disable_irq();
                REG_VIDEO_SCDC_IRQ_MASK &= ~0x40000u;
                __enable_irq();
                REG32(0x4010125C) = 0x40000u;
                video_main_ctrl_set_bit4_cond(0);

                if (video_check_timing_changed(hw_misc_set_reg_254(0), 0, 0) != 0)
                    REG_VIDEO_PHASE_ARM_TIMER = 0u;
                else
                    REG_VIDEO_PHASE_ARM_TIMER = 81u;

                if (REG_VIDEO_PHASE_GOOD != 0u) {
                    custom_printf("bad f-phase: %08x\n", phase);
                    REG_VIDEO_PHASE_GOOD = 0u;
                }
            }
        }

        if (REG_VIDEO_PHASE_ARM_TIMER != 0u && --REG_VIDEO_PHASE_ARM_TIMER == 0u) {
            custom_printf("Ass vid\n");
            video_main_ctrl_set_bit4_cond(1);
            REG_VIDEO_PHASE_DISABLE = 20u;
            REG_VIDEO_PHASE_POLL_TIMER = 7u;
        }

        hw_misc_unlock_hpd_state_and_check(&lock_ctx);
    }

    VIDEO_TASK_CHECK();
    return (int)saved_task;
}
