/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * i2c_scdc.c — I2C master (MIIC0), DDC transfer engine, SCDC state machine
 *
 * This module owns:
 *   - MIIC0 error translation (NACK / abort → errno)
 *   - I2C slave IRQ clear sequence
 *   - DDC (Display Data Channel) read/write DMA transfer engine
 *   - SCDC (Status and Control Data Channel, HDMI 2.1 spec §10) state machine
 *     States: 1=RESET, 2=INIT, 3=READY, 4=WAIT_PS, 5=ACTIVE,
 *             6=FRL_TRAINED, 7=STABLE, 8=ERROR
 *   - SCDC config reads/writes (DDC address 0xA8 = sink SCDC, 0x74 = source)
 *   - SCDC periodic timer countdowns (events 82, 83)
 *   - FRL link-rate clock calculation lookup table
 *
 * ARM Cortex-M4, bare-metal, no OS.
 */

#include "include/defs.h"
#include <stdarg.h>

/* =========================================================================
 * SRAM register addresses — DDC / SCDC state
 *
 * 0x40440  ddc_busy          — 1 while a DDC transfer is in flight
 * 0x40441  ddc_direction     — 1=write pending, 2=read pending
 * 0x40442  ddc_read_ok       — set to 1 if read data is valid
 * 0x40448  ddc_req_ptr       — pointer to current DDC request struct
 * 0x4044C  ddc_last_req_ptr  — shadow of last request pointer
 * 0x404B4  scdc_state        — (alias for 0x404BE)
 * 0x404BC  scdc_pending_cfg_read  — deferred scdc_read_config_1()
 * 0x404BD  scdc_pending_flags     — deferred scdc_read_status_flags()
 * 0x404BE  scdc_state        — current SCDC FSM state (1..8)
 * 0x404BF  scdc_pending_cfg_write — deferred scdc_write_config_1()
 * 0x404C2  scdc_last_ps      — cached PS value (for change detection)
 * 0x404C4  scdc_timeout_init — initial countdown for SCDC timer_c8
 * 0x404C8  scdc_timer_c8     — SCDC link-loss countdown
 * 0x404CC  scdc_timer_cc     — SCDC FRL stable countdown
 * 0x404D0  scdc_timer_83     — SCDC status-read interval countdown
 * 0x404D4  scdc_frl_rate_cb  — callback: FRL lane-rate changed
 * 0x404E0  scdc_ts_83        — timestamp word for timer_83
 * 0x404D8  scdc_ts_cc        — timestamp word for timer_cc
 * 0x404E8  scdc_status_req   — DDC request struct for status-flag read
 * 0x404F8  scdc_cfg1_req     — DDC request struct for config-1 write
 * 0x40508  scdc_cfg1_rd_req  — DDC request struct for config-1 read
 * 0x40520  scdc_earc_timer   — eARC debounce timer
 * 0x40522  scdc_earc_flag    — eARC pending flag
 * 0x40533  scdc_cfg2_pending — deferred scdc_write_config_2()
 * 0x40538  scdc_cfg2_req     — DDC request struct for config-2 write
 * 0x4045A  scdc_active       — 1 = SCDC link active
 * 0x4045C  scdc_earc_enabled — eARC capability / enable flag
 * 0x4045D  scdc_frl_mode     — FRL mode index (for clock LUT)
 * 0x4045E  scdc_dsc_mode     — DSC mode index (for scrambler bits)
 * 0x4045F  scdc_cfg3_pending — deferred scdc_write_config_3()
 * 0x40460  scdc_cfg4_pending — deferred scdc_write_config_4()
 * 0x40458  scdc_earc_state   — eARC connection state
 * 0x40459  scdc_earc_chk     — eARC check flag
 * 0x4046C  scdc_link_active  — secondary link active flag
 * 0x40470  scdc_cfg3_req     — DDC request struct for config-3 write
 * 0x40480  scdc_cfg4_req     — DDC request struct for config-4 write
 * 0x41BEC  scdc_timeout_cb   — callback: link-loss / timeout handler
 * 0x41BE8  scdc_lane_cfg_a   — lane config byte A (for hw_misc_scdc_apply_lane_config)
 * 0x41BE9  scdc_lane_cfg_b   — lane config byte B
 * 0x41BF0  scdc_hpd_cb       — callback: HPD / link-down notification
 * 0x41BF5  scdc_frl_rate     — current FRL bit-rate code (3/6/8/10/12)
 * 0x41BF7  scdc_trained_mask — bitmask of trained FRL rates
 * 0x41BFA  scdc_train_mode   — 1 = FRL training mode active
 *
 * DDC peripheral registers (0x40103xxx):
 * 0x40103004  DDC_XFER_CFG   — transfer config: byte count, direction, mode
 * 0x40103008  DDC_ADDR       — DDC device address and register offset
 * 0x4010300C  DDC_TRIGGER    — write 1 to start; poll until bit31 clears
 * 0x40103710  DDC_ENGINE_CMD — 0=idle, 3=arm/reset
 * 0x40103718  DDC_ENGINE_RDY — 1 = engine ready
 * 0x40103778  DDC_FLAGS      — misc flags; bit3 = enable, bit5 = eARC
 * 0x4010375C  DDC_STATUS     — FIFO / error status; bits[12:10] = error
 * 0x40101260  I2C_SLAVE_IRQ  — I2C slave IRQ status / clear register
 * 0x401012A0  I2C_SLAVE_CTRL — I2C slave control; bit0 = enable
 * 0x401012A4  I2C_SLAVE_IEN  — I2C slave interrupt enable mask
 * 0x40101264  I2C_INT_CTRL   — I2C interrupt control (write 6 = arm)
 * 0x4010129C  SCDC_IRQ_MASK  — SCDC IRQ enable bits
 * 0x40101088  SCDC_MODE_CTRL — FRL mode enable bits (bits 8,9)
 * 0x40101160  SCDC_PHY_CTRL  — PHY / scrambler control
 * 0x40101164  SCDC_TX_CTRL   — TX control; bit24 = force FRL
 * 0x4010117C  SCDC_RX_CTRL   — RX control
 * 0x4010107E  SCDC_PS_ADDR   — port-select address register (write=select)
 * 0x4010107F  SCDC_PS_DATA   — port-select data register
 * 0x40101110  SCDC_STATUS_0  — SCDC status byte 0; bit3=HotPlug, bit5=ARC
 * 0x40101114  SCDC_STATUS_1  — SCDC status byte 1 (FRL lane status)
 * 0x40101A80  SCDC_BIST_CTRL — BIST control; bit2 = pass
 * 0x40101A8C  SCDC_BIST_ERR0 — BIST error count lane 0
 * 0x40101A90  SCDC_BIST_ERR1 — BIST error count lane 1
 * 0x401010D8  SCDC_CLK_DIV_A — clock divider A (FRL audio)
 * 0x401010DC  SCDC_CLK_DIV_B — clock divider B (FRL audio)
 * 0x40100E0C  SCDC_CTRL_EXT  — extended SCDC/video control word
 * 0x40100E70  SCDC_FRL_CTRL  — FRL scrambler / rate control
 * 0x40101625  SCDC_AUX_CTRL  — auxiliary scrambler control (byte)
 * 0x40100797  SCDC_AUX2      — auxiliary control 2 (byte)
 * 0x40100C80  SCDC_CAP_0     — capability byte 0
 * 0x40100C81  SCDC_CAP_1     — capability byte 1
 * 0x40100D44  SCDC_PLL_CTRL  — PLL control; bit0 = reset
 * 0x40100D7E  SCDC_PLL_ADDR  — PLL register address (write=select)
 * 0x40100D7F  SCDC_PLL_DATA  — PLL register data
 * 0x40100D9E  SCDC_PLL_SEL   — PLL mux select
 * 0x40100D9F  SCDC_PLL_STATUS— PLL lock status; bit0 = locked
 * DDC RX FIFO at 0x40103800 (1074803200 = 0x40103800)
 * ========================================================================= */

/* -------------------------------------------------------------------------
 * Convenience SRAM accessors for SCDC / DDC state fields
 * ---------------------------------------------------------------------- */
#define DDC_BUSY            (*(volatile uint8_t  *)0x40440)
#define DDC_DIRECTION       (*(volatile uint8_t  *)0x40441)
#define DDC_READ_OK         (*(volatile uint8_t  *)0x40442)
#define DDC_REQ_PTR         (*(volatile uint32_t *)0x40448)
#define DDC_LAST_REQ_PTR    (*(volatile uint32_t *)0x4044C)

#define SCDC_PENDING_CFG_READ  (*(volatile uint8_t  *)0x404BC)
#define SCDC_PENDING_FLAGS     (*(volatile uint8_t  *)0x404BD)
#define SCDC_STATE             (*(volatile uint8_t  *)0x404BE)
#define SCDC_PENDING_CFG_WRITE (*(volatile uint8_t  *)0x404BF)
#define SCDC_LAST_PS           (*(volatile uint16_t *)0x404C2)
#define SCDC_TIMEOUT_INIT      (*(volatile uint32_t *)0x404C4)
#define SCDC_TIMER_C8          (*(volatile uint32_t *)0x404C8)
#define SCDC_TIMER_CC          (*(volatile uint32_t *)0x404CC)
#define SCDC_TIMER_83          (*(volatile uint32_t *)0x404D0)
#define SCDC_FRL_RATE_CB       (*(volatile uint32_t *)0x404D4)  /* fn ptr */

#define SCDC_ACTIVE            (*(volatile uint8_t  *)0x4045A)
#define SCDC_EARC_ENABLED      (*(volatile uint8_t  *)0x4045C)
#define SCDC_FRL_MODE          (*(volatile uint8_t  *)0x4045D)
#define SCDC_DSC_MODE          (*(volatile uint8_t  *)0x4045E)
#define SCDC_CFG3_PENDING      (*(volatile uint8_t  *)0x4045F)
#define SCDC_CFG4_PENDING      (*(volatile uint8_t  *)0x40460)
#define SCDC_EARC_STATE        (*(volatile uint8_t  *)0x40458)
#define SCDC_EARC_CHK          (*(volatile uint8_t  *)0x40459)
#define SCDC_LINK_ACTIVE       (*(volatile uint32_t *)0x4046C)
#define SCDC_CFG2_PENDING      (*(volatile uint8_t  *)0x40533)

#define SCDC_TIMEOUT_CB        (*(volatile uint32_t *)0x41BEC)  /* fn ptr */
#define SCDC_LANE_CFG_A        (*(volatile uint8_t  *)0x41BE8)
#define SCDC_LANE_CFG_B        (*(volatile uint8_t  *)0x41BE9)
#define SCDC_HPD_CB            (*(volatile uint32_t *)0x41BF0)  /* fn ptr */
#define SCDC_FRL_RATE          (*(volatile uint8_t  *)0x41BF5)
#define SCDC_TRAINED_MASK      (*(volatile uint32_t *)0x41BF7)
#define SCDC_TRAIN_MODE        (*(volatile uint8_t  *)0x41BFA)

/* DDC peripheral registers */
#define DDC_XFER_CFG           (*(volatile uint32_t *)0x40103004)
#define DDC_ADDR_REG           (*(volatile uint32_t *)0x40103008)
#define DDC_TRIGGER            (*(volatile uint32_t *)0x4010300C)
#define DDC_ENGINE_CMD         (*(volatile uint32_t *)0x40103710)
#define DDC_ENGINE_RDY         (*(volatile uint32_t *)0x40103718)
#define DDC_FLAGS              (*(volatile uint32_t *)0x40103778)
#define DDC_STATUS             (*(volatile uint32_t *)0x4010375C)
#define DDC_RX_FIFO_BASE       0x40103800u

/* I2C slave registers */
#define I2C_SLAVE_IRQ          (*(volatile uint32_t *)0x40101260)
#define I2C_SLAVE_CTRL         (*(volatile uint32_t *)0x401012A0)
#define I2C_SLAVE_IEN          (*(volatile uint32_t *)0x401012A4)
#define I2C_INT_CTRL           (*(volatile uint32_t *)0x40101264)

/* SCDC control / status registers */
#define SCDC_IRQ_MASK          (*(volatile uint32_t *)0x4010129C)
#define SCDC_MODE_CTRL         (*(volatile uint32_t *)0x40101088)
#define SCDC_PHY_CTRL          (*(volatile uint32_t *)0x40101160)
#define SCDC_TX_CTRL           (*(volatile uint32_t *)0x40101164)
#define SCDC_RX_CTRL           (*(volatile uint32_t *)0x4010117C)
#define SCDC_PS_ADDR           (*(volatile uint8_t  *)0x4010107E)
#define SCDC_PS_DATA           (*(volatile uint8_t  *)0x4010107F)
#define SCDC_STATUS_0          (*(volatile uint8_t  *)0x40101110)
#define SCDC_STATUS_1          (*(volatile uint32_t *)0x40101114)
#define SCDC_BIST_CTRL         (*(volatile uint32_t *)0x40101A80)
#define SCDC_BIST_ERR0         (*(volatile uint32_t *)0x40101A8C)
#define SCDC_BIST_ERR1         (*(volatile uint32_t *)0x40101A90)
#define SCDC_CLK_DIV_A         (*(volatile uint32_t *)0x401010D8)
#define SCDC_CLK_DIV_B         (*(volatile uint32_t *)0x401010DC)
#define SCDC_CTRL_EXT          (*(volatile uint32_t *)0x40100E0C)
#define SCDC_FRL_CTRL          (*(volatile uint32_t *)0x40100E70)
#define SCDC_AUX_CTRL          (*(volatile uint8_t  *)0x40101625)
#define SCDC_AUX2              (*(volatile uint8_t  *)0x40100797)
#define SCDC_CAP_0             (*(volatile uint8_t  *)0x40100C80)
#define SCDC_CAP_1             (*(volatile uint8_t  *)0x40100C81)
#define SCDC_PLL_CTRL          (*(volatile uint32_t *)0x40100D44)
#define SCDC_PLL_ADDR          (*(volatile uint8_t  *)0x40100D7E)
#define SCDC_PLL_DATA          (*(volatile uint8_t  *)0x40100D7F)
#define SCDC_PLL_SEL           (*(volatile uint8_t  *)0x40100D9E)
#define SCDC_PLL_STATUS        (*(volatile uint8_t  *)0x40100D9F)

/* Misc SRAM flags used across modules */
#define REG_LINK_DOWN_FLAG     (*(volatile uint8_t  *)0x401DC)
#define REG_HDCP_FLAGS_265     (*(volatile uint8_t  *)0x40265)
#define REG_HDCP_KEY_VALID_266 (*(volatile uint8_t  *)0x40266)
#define REG_HPD_DEBOUNCE_26D   (*(volatile uint8_t  *)0x4026D)
#define REG_SCDC_FRL_LOCK      (*(volatile uint8_t  *)0x4026C)
#define REG_VIDEO_HPD_STATE    (*(volatile uint8_t  *)0x40269)
#define REG_VIDEO_HPD_PENDING  (*(volatile uint8_t  *)0x4026A)
#define REG_EARC_DEBOUNCE      (*(volatile uint8_t  *)0x40520)
#define REG_EARC_PENDING       (*(volatile uint8_t  *)0x40522)
#define REG_IRQ_FLAGS_1E5      (*(volatile uint8_t  *)0x401E5)
#define REG_IRQ_FLAGS_1E4      (*(volatile uint8_t  *)0x401E4)
#define REG_VIDEO_HDCP_WAIT    (*(volatile uint8_t  *)0x401D7)
#define REG_HDCP_AUTH_CTRL     (*(volatile uint32_t *)0x401002F0)
#define REG_VIDEO_FLAGS_120    (*(volatile uint32_t *)0x40100120)
#define REG_FRL_CTRL_EXT       (*(volatile uint32_t *)0x4010038C)

/* Flash / NVRAM config comparison sentinels */
#define REG_CFG_SENTINEL_A     (*(volatile uint32_t *)0x40058)
#define REG_CFG_SENTINEL_B     (*(volatile uint32_t *)0x400A0)

/* Feature flags NVRAM layout */
#define REG_FEAT_MODE          (*(volatile uint32_t *)0x40008)
#define REG_FEAT_CAP_48        (*(volatile uint32_t *)0x40048)
#define REG_FEAT_VAL_4C        (*(volatile uint32_t *)0x4004C)
#define REG_FEAT_VAL_50        (*(volatile uint32_t *)0x40050)
#define REG_FEAT_OUT_340       (*(volatile uint32_t *)0x40340)
#define REG_FEAT_OUT_334       (*(volatile uint32_t *)0x40334)
#define REG_FEAT_OUT_331       (*(volatile uint8_t  *)0x40331)
#define REG_FEAT_OUT_35F       (*(volatile uint8_t  *)0x4035F)

/* eARC / audio SRAM */
#define REG_EARC_HDCP_082      (*(volatile uint8_t  *)0x41082)
#define REG_EARC_HDCP_084      (*(volatile uint8_t  *)0x41084)
#define REG_EARC_HDCP_086      (*(volatile uint8_t  *)0x41086)
#define REG_EARC_HDCP_088      (*(volatile uint8_t  *)0x41088)
#define REG_EARC_HDCP_090      (*(volatile uint32_t *)0x41090)
#define REG_EARC_HDCP_094      (*(volatile uint32_t *)0x41094)
#define REG_EARC_FLAG_274      (*(volatile uint8_t  *)0x40274)
#define REG_EARC_FLAG_BD4      (*(volatile uint32_t *)0x41BD4)
#define REG_EARC_FLAG_4E4      (*(volatile uint8_t  *)0x410DD)  /* eARC cap flag */
#define REG_EARC_FLAG_4E4B     (*(volatile uint8_t  *)0x410E4)  /* eARC enable flag */

/*
 * DDC request structure layout (at various SRAM addresses):
 *   +0  uint16_t  byte_count    — number of bytes to transfer
 *   +2  uint8_t   data[N]       — inline data buffer (for write requests)
 *       OR
 *   +2  uint8_t  *data_ptr      — pointer to external read buffer (for read)
 *   +12 void (*callback)(void)  — completion callback (optional)
 */

/* -------------------------------------------------------------------------
 * Internal forward declarations (functions defined later in this file)
 * ---------------------------------------------------------------------- */
int scdc_update_status_poll_timer(int a1, int a2, int a3);
int scdc_update_frl_stable_timer(int a1, int a2, int a3);
int scdc_fsm_wait_ps_to_active(void);

/* -------------------------------------------------------------------------
 * Cross-module forward declarations
 * ---------------------------------------------------------------------- */
/* hw_misc.c */
int  hw_misc_clear_reg_492_and_irq_b(void);
void hw_misc_scdc_set_state3_timeout(void);
int  hw_misc_scdc_apply_lane_config(uint8_t lane_a, uint8_t lane_b);
int  hw_misc_set_reg_254(int enable);
void hw_misc_scdc_phy_ctrl_init(void);
void hw_misc_irq_enable_mask_b_clear(void);
int  hw_misc_set_reg_398(uint8_t val);
int  hw_misc_set_reg_39d(uint8_t val);
int  hw_misc_get_reg_39d(void);
int  hw_misc_frl_link_loss_recovery(void);

/* audio_earc.c */
int  audio_handle_frl_training_result(int a1, int a2, int a3);
int  audio_dispatch_pending_scdc_tasks(void);
int  audio_h14_status_reg_set_c000(void);
int  audio_protocol_dispatch_rd_cmd(void);
int  audio_protocol_get_state_code(void);

/* hdmi_frl_video.c */
int  frl_is_state_8_or_higher(void);
int  frl_fsm_advance_to_state9(void);
int  frl_set_tx_force_bit(void);
int  video_main_ctrl_set_bit4(int enable);
int  video_is_debug_flag_bit4_set(void);
int  video_enqueue_event(int event, int arg0, int arg1);
int  video_frl_rate_regs_apply(int rate, int enable);
int  video_set_path_ctrl_bit27(int enable);
int  video_handle_fatal_error_overlay(int mode, int enable);
int  video_update_reg_335_353(void);
int  video_update_reg_335_353_wrapper(void);
unsigned int video_calc_elapsed_ticks(uint8_t *ts, int a2, int a3);
int  video_snapshot_event_time(int addr, int enable);
int  video_main_ctrl_set_bit0(int enable);
int  video_reg_f38_set_bit15(int enable);
int  video_main_ctrl_set_bit4_cond(int enable);
int  video_main_ctrl_set_0a_and_clear(void);

/* flash_nvram.c */
int  flash_read_data(int addr, unsigned int len, void *buf, int flags);
int  flash_read_firmware_image(unsigned int a1, unsigned int a2, int a3, int a4);
int  flash_read_hw_status(uint8_t *status_out, uint32_t *detail_out);

/* =========================================================================
 * I2C MIIC0 error translation
 * ========================================================================= */

/*
 * i2c_miic0_translate_error — translate MIIC0 hardware error code to string
 *
 * Called from the MIIC0 IRQ handler when a transfer fails.
 * Error code 104 = NACK received from slave.
 * Error code 105 = arbitration lost / abort.
 */
int i2c_miic0_translate_error(uint8_t *err_code)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    switch (*err_code) {
    case 104:
        custom_printf("MIIC0 translate nack\n");
        break;
    case 105:
        custom_printf("MIIC0 translate abort\n");
        break;
    default:
        break;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * I2C slave IRQ clear sequence
 * ========================================================================= */

/*
 * i2c_slave_clear_irq — clear pending I2C slave interrupt
 *
 * The I2C slave controller requires writing 0xF to the IRQ status register
 * multiple times with short delays between writes to fully de-assert all
 * pending interrupt lines.  After clearing, the slave is re-enabled.
 */
int i2c_slave_clear_irq(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", I2C_SLAVE_IRQ);
    I2C_SLAVE_IRQ = 0xF;   /* clear all pending IRQ bits */
    delay_loop(1);
    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", I2C_SLAVE_IRQ);
    delay_loop(5);
    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", I2C_SLAVE_IRQ);
    delay_loop(1);
    I2C_SLAVE_IRQ = 0xF;
    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", 15);
    delay_loop(5);
    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", I2C_SLAVE_IRQ);
    I2C_SLAVE_IRQ = 0xF;
    delay_loop(1);
    custom_printf("I2C_SLAVE_IRQ_STATUS %04x\n", I2C_SLAVE_IRQ);
    I2C_SLAVE_CTRL |= 1u;  /* re-enable I2C slave */

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * DDC low-level transfer engine
 * ========================================================================= */

/*
 * ddc_set_address — program DDC target device address and register offset
 *
 * @dev_addr:  I2C device address (7-bit, e.g. 0xA8 >> 1 = 0x54 for SCDC sink)
 * @reg_offset: register offset within the DDC device
 *
 * Packed into DDC_ADDR_REG as: (dev_addr << 16) | reg_offset
 */
static int ddc_set_address(int dev_addr, int reg_offset)
{
    DDC_ADDR_REG = (uint32_t)(reg_offset | (dev_addr << 16));
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * ddc_set_transfer_config — program DDC transfer length, mode, direction
 *
 * @byte_count_minus1: (byte_count - 1), packed into bits [29:13]
 * @reg_sub_offset:    sub-register offset packed into bits [12:3]
 * @mode:              transfer mode bits [2:1]
 * @direction:         0 = write to device (host→slave), 1 = read from device
 *
 * Register layout:
 *   bits [29:13] = (byte_count - 1) << 13
 *   bits [12: 3] = reg_sub_offset << 3 (masked to 10 bits)
 *   bits [ 2: 1] = mode << 1
 *   bit  [ 0]    = direction
 */
static int ddc_set_transfer_config(int byte_count_minus1, int16_t reg_sub_offset,
                                   int8_t mode, int8_t direction)
{
    DDC_XFER_CFG = ((uint32_t)(byte_count_minus1 << 13) & 0x7FFFFF)
                 | (uint32_t)((8 * (reg_sub_offset & 0x3FF)))
                 | (uint32_t)((2 * (mode & 3)))
                 | (uint32_t)(direction & 1);
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * ddc_wait_and_setup_transfer — arm DDC engine and configure for write
 *
 * Waits up to 80 polls for the DDC engine to become ready, then programs
 * the device address and transfer length.
 *
 * @dev_addr:   DDC device address
 * @reg_offset: DDC register offset
 * @byte_count: number of bytes to transfer (must be ≤ 0x400)
 *
 * Returns 0 on success, 255 on timeout or invalid byte_count.
 */
static int ddc_wait_and_setup_transfer(int dev_addr, int reg_offset,
                                       unsigned int byte_count)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    unsigned int polls = 0;
    int result;

    if (byte_count > 0x400u) {
        result = 255;
        goto done;
    }

    /* Arm the DDC engine: set enable bit, command=3, wait for ready */
    DDC_FLAGS     |= 8u;
    DDC_ENGINE_CMD = 3;
    do {
        polls++;
    } while (DDC_ENGINE_RDY != 1 && polls < 0x50u);

    if (polls == 80u) {
        /* Timeout — reset engine and fail */
        DDC_ENGINE_CMD = 0;
        result = 255;
    } else {
        ddc_set_address(dev_addr, reg_offset);
        /* Configure for write: (count-1) bytes, mode=0, direction=write */
        ddc_set_transfer_config((int16_t)((uint16_t)byte_count - 1), 0, 1, 1);
        result = 0;
    }

done:
    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * ddc_setup_read_transfer — arm DDC engine and fill TX FIFO for read request
 *
 * For DDC reads, the host must first write the register address to the device,
 * then clock in the data.  This function loads the address bytes into the
 * DDC TX FIFO at DDC_RX_FIFO_BASE and configures the transfer.
 *
 * @dev_addr:    DDC device address
 * @reg_offset:  DDC register offset
 * @byte_count:  number of bytes to read (must be ≤ 0x40)
 * @tx_buf:      buffer containing the address/command bytes to send first
 *
 * Returns 0 on success, 255 on timeout or invalid byte_count.
 */
static int ddc_setup_read_transfer(int dev_addr, int reg_offset,
                                   unsigned int byte_count, uint8_t *tx_buf)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    unsigned int polls = 0;
    int result;

    if (byte_count > 0x40u) {
        result = 255;
        goto done;
    }

    /* Arm the DDC engine */
    DDC_FLAGS     |= 8u;
    DDC_ENGINE_CMD = 3;
    do {
        polls++;
    } while (DDC_ENGINE_RDY != 1 && polls < 0x50u);

    if (polls == 80u) {
        DDC_ENGINE_CMD = 0;
        result = 255;
    } else {
        ddc_set_address(dev_addr, reg_offset);
        /* Configure for read: 0 write bytes, (count-1) read bytes, direction=read */
        ddc_set_transfer_config(0, (int16_t)((uint16_t)byte_count - 1), 1, 0);

        /* Copy address/command bytes into the DDC TX FIFO */
        volatile uint8_t *fifo = (volatile uint8_t *)DDC_RX_FIFO_BASE;
        for (unsigned int i = 0; i < byte_count + 1u; i++)
            *fifo++ = *tx_buf++;

        result = 0;
    }

done:
    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * ddc_write_request — initiate a DDC write transfer (async)
 *
 * If a transfer is already in progress (DDC_BUSY), returns 255 immediately.
 * Otherwise programs the DDC engine and arms the interrupt to deliver the
 * data from the request struct pointed to by @req.
 *
 * @dev_addr:   DDC device address (e.g. 0xA8 for SCDC sink)
 * @reg_offset: DDC register offset within the device
 * @req:        pointer to DDC request struct (first field = uint16_t byte_count)
 *
 * Returns 0 if transfer started, 255 on error.
 */
int ddc_write_request(int dev_addr, int reg_offset, uint16_t *req)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int result;

    if (DDC_BUSY) {
        result = 255;
    } else if (req == NULL) {
        result = 255;
    } else if (ddc_wait_and_setup_transfer(dev_addr, reg_offset, *req) == 255) {
        result = 255;
    } else {
        DDC_BUSY        = 1;
        DDC_REQ_PTR     = (uint32_t)req;
        I2C_INT_CTRL    = 6;          /* arm I2C interrupt controller */
        __asm__ volatile("dsb" ::: "memory");
        DDC_TRIGGER     = 1;          /* start transfer */
        while (DDC_TRIGGER & 0x80000000u) /* wait for engine to latch */ ;
        __asm__ volatile("dsb" ::: "memory");
        I2C_SLAVE_IEN  |= 5u;         /* enable TX-done and error IRQs */
        DDC_DIRECTION   = 1;          /* mark as write-in-progress */
        result = 0;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * ddc_read_request — initiate a DDC read transfer (async)
 *
 * Similar to ddc_write_request but initiates a read.  The request struct
 * must contain the byte count at offset 0 and a data pointer at offset 4.
 *
 * @dev_addr:   DDC device address
 * @reg_offset: DDC register offset
 * @req:        pointer to DDC request struct (offset 0 = byte_count, offset 4 = buf ptr)
 *
 * Returns 0 if transfer started, 255 on error.
 */
int ddc_read_request(int dev_addr, int reg_offset, uint8_t **req)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int result;

    if (DDC_BUSY) {
        result = 255;
    } else if (req == NULL) {
        result = 255;
    } else if (ddc_setup_read_transfer(dev_addr, reg_offset,
                                       *(uint16_t *)req, (uint8_t *)req[1]) == 255) {
        result = 255;
    } else {
        DDC_BUSY        = 1;
        DDC_REQ_PTR     = (uint32_t)req;
        I2C_INT_CTRL    = 6;
        __asm__ volatile("dsb" ::: "memory");
        DDC_TRIGGER     = 1;
        while (DDC_TRIGGER & 0x80000000u) ;
        __asm__ volatile("dsb" ::: "memory");
        I2C_SLAVE_IEN  |= 5u;
        DDC_DIRECTION   = 2;          /* mark as read-in-progress */
        result = 0;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * ddc_read_data_buffer — copy received DDC RX FIFO data into the request buffer
 *
 * Called from the DDC RX-complete IRQ.  Disables the RX IRQ enable bit,
 * then copies bytes from the DDC RX FIFO into the buffer pointer stored in
 * the current DDC request struct.  If any error bits are set in DDC_STATUS,
 * marks the read as failed.
 *
 * DDC error bits [14:12] of DDC_STATUS (0x1C00): non-zero = error condition.
 */
int ddc_read_data_buffer(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    DDC_READ_OK      = 1;
    I2C_SLAVE_IEN   &= ~1u;  /* disable RX-done IRQ */

    uint16_t *req = (uint16_t *)DDC_REQ_PTR;
    DDC_LAST_REQ_PTR = DDC_REQ_PTR;

    if (req != NULL) {
        DDC_REQ_PTR = 0;
        uint32_t byte_count = *req;
        uint8_t *dst        = (uint8_t *)(req + 1);

        if ((DDC_STATUS & 0x1C00u) != 0) {
            /* Transfer error — mark read data as invalid */
            DDC_READ_OK = 0;
        } else {
            /* Copy bytes from the DDC RX FIFO */
            volatile uint8_t *fifo = (volatile uint8_t *)DDC_RX_FIFO_BASE;
            for (uint32_t i = 0; i < byte_count; i++)
                *dst++ = *fifo++;
        }
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * ddc_transfer_complete_callback — DDC write/read transfer completion handler
 *
 * Called from the I2C interrupt when a DDC transfer finishes successfully.
 * Clears the TX-done interrupt enable bit, resets the DDC engine, and calls
 * the per-request completion callback if provided.  If no new transfer was
 * enqueued by the callback, signals the audio and video subsystems.
 */
int ddc_transfer_complete_callback(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    I2C_SLAVE_IEN &= ~4u;  /* clear TX-done IRQ enable */

    uint32_t req_addr    = DDC_REQ_PTR;
    DDC_LAST_REQ_PTR     = DDC_REQ_PTR;

    if (req_addr != 0) {
        DDC_REQ_PTR     = 0;
        DDC_BUSY        = 0;
        DDC_ENGINE_CMD  = 0;  /* reset DDC engine to idle */

        /* Call per-request completion callback if provided */
        void (*cb)(void) = *(void (**)(void))(req_addr + 12);
        if (cb)
            cb();

        /* If callback did not enqueue a new transfer, notify subsystems */
        if (!DDC_REQ_PTR) {
            DDC_DIRECTION = 0;
            audio_dispatch_pending_scdc_tasks();
        }
        /* Notify video subsystem that DDC channel is free */
        video_enqueue_event(105, 0, 0);
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * ddc_transfer_abort_callback — DDC transfer abort / error handler
 *
 * Called from the I2C interrupt when a DDC transfer is aborted (e.g.
 * arbitration lost, NACK from slave, timeout).  Resets state and notifies
 * subsystems, but does NOT enqueue a video event (unlike the success path).
 */
int ddc_transfer_abort_callback(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint32_t req_addr = DDC_REQ_PTR;
    DDC_LAST_REQ_PTR  = DDC_REQ_PTR;

    if (req_addr != 0) {
        DDC_REQ_PTR    = 0;
        DDC_BUSY       = 0;
        DDC_ENGINE_CMD = 0;

        void (*cb)(void) = *(void (**)(void))(req_addr + 12);
        if (cb)
            cb();

        if (!DDC_REQ_PTR) {
            DDC_DIRECTION = 0;
            audio_dispatch_pending_scdc_tasks();
        }
        /* Note: no video event on abort — caller must handle error */
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * ddc_clear_request — cancel any pending DDC read without completing it
 *
 * Called when a DDC read times out or is superseded.  Marks the read as
 * valid only if the DDC RX FIFO has no error bits set.
 */
int ddc_clear_request(void)
{
    DDC_READ_OK      = 1;
    I2C_SLAVE_IEN   &= ~1u;
    DDC_LAST_REQ_PTR = DDC_REQ_PTR;

    if (DDC_REQ_PTR) {
        DDC_REQ_PTR  = 0;
        /* Mark read OK only if no FIFO error bits */
        DDC_READ_OK  = (uint8_t)((DDC_STATUS & 0x1C00u) == 0);
    }

    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * SCDC status / config I/O helpers
 * ========================================================================= */

/*
 * scdc_read_status_flags — initiate a DDC read of the SCDC status flags
 *
 * Reads 49 bytes from DDC address 0xA8 (SCDC sink register 0x31 onwards).
 * The result is stored in the request struct at 0x404E8.
 *
 * @irq_ctx: 0 = called from main loop (disables IRQs around the request),
 *            1 = called from IRQ context (IRQs already disabled)
 */
int scdc_read_status_flags(int irq_ctx)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (!irq_ctx)
        __disable_irq();

    if (DDC_BUSY)
        SCDC_PENDING_FLAGS = 1;  /* defer until DDC is free */
    else
        ddc_read_request(0xA8, 49, (uint8_t **)0x404E8);

    if (!irq_ctx)
        __enable_irq();

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_read_config_1 — initiate a DDC read of SCDC config register 1
 *
 * Reads 4 bytes from DDC address 0xA8, register 0x10.
 * Result stored in request struct at 0x40508.
 *
 * @irq_ctx: same semantics as scdc_read_status_flags
 */
int scdc_read_config_1(int irq_ctx)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (!irq_ctx)
        __disable_irq();

    if (DDC_BUSY)
        SCDC_PENDING_CFG_READ = 1;
    else
        ddc_read_request(0xA8, 16, (uint8_t **)0x40508);

    if (!irq_ctx)
        __enable_irq();

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_write_config_1 — initiate a DDC write of SCDC config register 1
 *
 * Writes 53 bytes to DDC address 0xA8 (SCDC sink scrambler config).
 * Request struct at 0x404F8.
 */
int scdc_write_config_1(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (DDC_BUSY)
        SCDC_PENDING_CFG_WRITE = 1;
    else
        ddc_write_request(0xA8, 53, (uint16_t *)0x404F8);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_write_config_2 — DDC write: eARC capability register to 0x74 (source)
 *
 * Writes 80 bytes to DDC address 0x74.  Request struct at 0x40538.
 */
int scdc_write_config_2(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (DDC_BUSY)
        SCDC_CFG2_PENDING = 1;
    else
        ddc_write_request(0x74, 80, (uint16_t *)0x40538);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_write_config_3 — DDC write: FRL lane count config to sink (0xA8)
 *
 * Writes 64 bytes to DDC address 0xA8.  Request struct at 0x40470.
 */
int scdc_write_config_3(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (DDC_BUSY)
        SCDC_CFG3_PENDING = 1;
    else
        ddc_write_request(0xA8, 64, (uint16_t *)0x40470);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_write_config_4 — DDC write: scrambler enable to sink (0xA8 reg 0x01)
 *
 * Writes 1 byte to DDC address 0xA8, register 0x01.  Request struct at 0x40480.
 */
int scdc_write_config_4(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (DDC_BUSY)
        SCDC_CFG4_PENDING = 1;
    else
        ddc_write_request(0xA8, 1, (uint16_t *)0x40480);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_set_hpd_debounce_timer — set the HPD debounce timer to 20 ticks
 *
 * This timer governs how long the firmware waits after an HPD event before
 * initiating SCDC / FRL link training.
 */
int scdc_set_hpd_debounce_timer(void)
{
    REG_HPD_DEBOUNCE_26D = 20;
    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * SCDC port-select (PS) register helpers
 * ========================================================================= */

/*
 * scdc_read_ps — read the SCDC port-select (PS) register
 *
 * Selects register 0x94 (=-108 signed byte) via SCDC_PS_ADDR then reads
 * SCDC_PS_DATA.  The PS register reports the current link state:
 *   bits [9:8] = port state (1 = FRL active)
 *   bit  7     = scrambler active
 *
 * Returns a 16-bit value with the port state in bits [9:8] also replicated
 * in bits [1:0] for compatibility with callers.
 */
int scdc_read_ps(void)
{
    SCDC_PS_ADDR = (uint8_t)(-108);  /* 0x94 = SCDC PS register address */
    uint8_t ps   = SCDC_PS_DATA;
    return (int)ps | (int)((ps & 3) << 8);
}

/*
 * scdc_print_ps — print PS register if it changed since last call
 *
 * Caches the last PS value to avoid spamming the debug log.
 */
int scdc_print_ps(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    int ps = scdc_read_ps();
    if (SCDC_LAST_PS != (uint16_t)ps) {
        custom_printf("PS=%04x\n", ps);
        SCDC_LAST_PS = (uint16_t)ps;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * SCDC state machine — initialization
 * ========================================================================= */

/*
 * scdc_hw_init — initialize SCDC hardware peripheral registers
 *
 * Enables SCDC interrupt sources and configures the PHY for SCDC operation.
 * Called once at firmware startup before the main event loop.
 *
 * Register bits configured:
 *   0x40101088 bit10 = enable SCDC RX interrupt
 *   0x40101164 bit13 = enable SCDC TX
 *   0x40101160 bit13 = enable SCDC RX
 *   0x4010117C bit10 = enable SCDC RX control
 *   0x40101160 bit10 = enable SCDC RX lane
 *   0x40101160 bit11 = enable SCDC RX lane B
 *   0x40101160 bit1  = (clear) disable legacy I2C mode
 *   0x40101160 bit5  = (clear) disable alternate scrambler
 */
int scdc_hw_init(void)
{
    SCDC_MODE_CTRL |=  0x400u;   /* enable SCDC RX interrupt */
    SCDC_TX_CTRL   |=  0x2000u;  /* enable SCDC TX path */
    SCDC_PHY_CTRL  |=  0x2000u;  /* enable SCDC RX path */
    SCDC_RX_CTRL   |=  0x400u;   /* enable SCDC RX control */
    SCDC_PHY_CTRL  |=  0x400u;   /* enable SCDC RX lane A */
    SCDC_PHY_CTRL  |=  0x800u;   /* enable SCDC RX lane B */
    SCDC_PHY_CTRL  &= ~2u;       /* disable legacy I2C mode */
    SCDC_PHY_CTRL  &= ~0x20u;    /* disable alternate scrambler */
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * scdc_reset_state — reset SCDC FSM to initial state
 *
 * Sets state = RESET (1), clears link-loss countdown and timeout timer.
 */
int scdc_reset_state(void)
{
    SCDC_STATE     = 1;   /* SCDC_STATE_RESET */
    SCDC_TIMER_CC  = 0;
    SCDC_TIMER_C8  = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * SCDC state accessors
 * ========================================================================= */

/* scdc_is_active — return true if the SCDC link is currently active */
int scdc_is_active(void)
{
    return SCDC_ACTIVE != 0;
}

/* scdc_clear_active — clear SCDC active flag and secondary link flag */
int scdc_clear_active(void)
{
    SCDC_ACTIVE      = 0;
    SCDC_LINK_ACTIVE = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

/* scdc_is_state_1 — true when SCDC FSM is in RESET state */
int scdc_is_state_1(void)
{
    return SCDC_STATE == 1;
}

/* scdc_is_state_5 — true when SCDC FSM is in ACTIVE state */
int scdc_is_state_5(void)
{
    return SCDC_STATE == 5;
}

/* scdc_is_state_7 — true when SCDC FSM is in STABLE (FRL stable) state */
int scdc_is_state_7(void)
{
    return SCDC_STATE == 7;
}

/* scdc_is_state_8 — true when SCDC FSM is in ERROR state */
int scdc_is_state_8(void)
{
    return SCDC_STATE == 8;
}

/* =========================================================================
 * SCDC FRL mode enable / disable
 * ========================================================================= */

/*
 * scdc_enable_frl_mode — switch SCDC to FRL mode (disable legacy TMDS)
 *
 * Clears the TMDS-enable bit and sets the FRL-enable bit in SCDC_MODE_CTRL.
 * bit9 = TMDS mode, bit8 = FRL mode.
 */
int scdc_enable_frl_mode(void)
{
    SCDC_MODE_CTRL &= ~0x200u;  /* clear TMDS mode */
    SCDC_MODE_CTRL |=  0x100u;  /* set FRL mode */
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * scdc_enable_tmds_mode — switch SCDC to TMDS mode (disable FRL)
 */
int scdc_enable_tmds_mode(void)
{
    SCDC_MODE_CTRL &= ~0x100u;  /* clear FRL mode */
    SCDC_MODE_CTRL |=  0x200u;  /* set TMDS mode */
    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * SCDC periodic task and timer countdowns
 * ========================================================================= */

/*
 * scdc_periodic_task — called from the main event loop each tick
 *
 * Prints any PS change, ticks the two SCDC countdowns, and checks whether
 * state 4 (WAIT_PS) can advance to state 5 (ACTIVE).
 */
int scdc_periodic_task(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    scdc_print_ps();
    scdc_update_status_poll_timer(0, 0, 0);
    scdc_update_frl_stable_timer(0, 0, 0);
    scdc_fsm_wait_ps_to_active();

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_update_status_poll_timer — decrement SCDC status-read interval timer
 *
 * SCDC_TIMER_83 counts down in units of elapsed time returned by
 * video_calc_elapsed_ticks (uses a timestamp at 0x404E0).  When it reaches
 * zero, event 83 (SCDC_STATUS_POLL) is enqueued to the video event queue.
 *
 * Event 83 → triggers scdc_read_status_flags() to poll link status.
 */
int scdc_update_status_poll_timer(int a1, int a2, int a3)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_TIMER_83) {
        uint32_t elapsed = video_calc_elapsed_ticks((uint8_t *)0x404E0, a2, a3);
        if (SCDC_TIMER_83 <= elapsed)
            SCDC_TIMER_83 = 0;
        else
            SCDC_TIMER_83 -= elapsed;

        if (!SCDC_TIMER_83) {
            __disable_irq();
            video_enqueue_event(83, 0, 0);
            __enable_irq();
        }
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_update_frl_stable_timer — decrement FRL-stable countdown (event 82)
 *
 * When SCDC_TIMER_CC reaches zero, enqueues event 82 (FRL_TIMEOUT) and
 * invokes the link-loss callback (SCDC_TIMEOUT_CB) with arg 0 to signal
 * that the FRL link has become unstable.
 *
 * Timestamp word at 0x404D8.
 */
int scdc_update_frl_stable_timer(int a1, int a2, int a3)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_TIMER_CC) {
        uint32_t elapsed = video_calc_elapsed_ticks((uint8_t *)0x404D8, a2, a3);
        if (SCDC_TIMER_CC <= elapsed)
            SCDC_TIMER_CC = 0;
        else
            SCDC_TIMER_CC -= elapsed;

        if (!SCDC_TIMER_CC) {
            __disable_irq();
            video_enqueue_event(82, 0, 0);
            __enable_irq();
            if (SCDC_TIMEOUT_CB)
                ((void (*)(int))SCDC_TIMEOUT_CB)(0);
        }
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_update_link_loss_timer — decrement link-loss countdown (event via FRL)
 *
 * SCDC_TIMER_C8 counts link-loss intervals.  When it reaches zero while
 * not in state 7 (STABLE), kicks off frl_fsm_advance_to_state9() to attempt
 * FRL re-training.
 */
int scdc_update_link_loss_timer(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_TIMER_C8) {
        __disable_irq();
        if (--SCDC_TIMER_C8 == 0 && SCDC_STATE != 7)
            frl_fsm_advance_to_state9();
        __enable_irq();
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_dispatch_pending_ddc_reqs — drain deferred DDC requests
 *
 * Called from the main loop after each DDC completion event.  Only one
 * deferred request is dispatched per call (priority order: flags > cfg_write
 * > cfg_read).
 *
 * Returns 1 if a deferred request was dispatched, 0 if none pending.
 */
int scdc_dispatch_pending_ddc_reqs(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int dispatched;

    if (SCDC_PENDING_FLAGS) {
        SCDC_PENDING_FLAGS = 0;
        scdc_read_status_flags(1);
        dispatched = 1;
    } else if (SCDC_PENDING_CFG_WRITE) {
        SCDC_PENDING_CFG_WRITE = 0;
        scdc_write_config_1();
        dispatched = 1;
    } else if (SCDC_PENDING_CFG_READ) {
        SCDC_PENDING_CFG_READ = 0;
        scdc_read_config_1(1);
        dispatched = 1;
    } else {
        dispatched = 0;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return dispatched;
}

/* =========================================================================
 * SCDC state machine — state transition handlers (event handlers)
 * ========================================================================= */

/*
 * scdc_fsm_init_to_ready — INIT → READY transition (event: FRL training req)
 *
 * When in state 2 (INIT), copies the negotiated FRL lane config and
 * advances to state 3 (READY).  Also clears the FRL lock IRQ enable bit.
 */
int scdc_fsm_init_to_ready(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 2) {  /* SCDC_STATE_INIT */
        hw_misc_scdc_apply_lane_config(SCDC_LANE_CFG_A, SCDC_LANE_CFG_B);
        SCDC_TIMER_C8 = SCDC_TIMEOUT_INIT;
        SCDC_STATE    = 3;  /* SCDC_STATE_READY */
    }
    SCDC_IRQ_MASK &= ~0x80u;  /* clear FRL lock IRQ enable */

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_frl_hw_ready — handle SCDC event 76 (FRL link ready from HW)
 *
 * In state 2 (INIT), signals FRL set_hw_state_13 and enqueues event 76
 * (FRL_HW_READY) to the video event queue.
 * Clears the SCDC FRL-lock IRQ mask bit 0x1000.
 */
int scdc_handle_frl_hw_ready(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 2) {
        frl_set_tx_force_bit();
        video_enqueue_event(76, 0, 0);
    }
    SCDC_IRQ_MASK &= ~0x1000u;

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_frl_pattern_lock — handle SCDC event 77 (FRL pattern lock)
 *
 * In state 3 (READY), clears the scrambler enable bit, advances FRL state
 * machine, and enqueues event 77 (FRL_PATTERN_LOCK) to the video queue.
 */
int scdc_handle_frl_pattern_lock(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 3) {  /* SCDC_STATE_READY */
        SCDC_IRQ_MASK &= ~8u;   /* disable scrambler IRQ */
        frl_fsm_advance_to_state9();
        video_enqueue_event(77, 0, 0);
    }
    SCDC_IRQ_MASK &= ~0x20u;  /* clear pattern-lock IRQ enable */
    SCDC_IRQ_MASK &= ~0x10u;  /* clear pattern-lock alt IRQ enable */

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_frl_train_step — handle SCDC event 78 (FRL training step complete)
 *
 * In state 3 (READY), enqueues event 78 (FRL_TRAIN_STEP) and advances to
 * state 4 (WAIT_PS) with the link-loss timer reset.
 */
int scdc_handle_frl_train_step(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 3) {
        video_enqueue_event(78, 0, 0);
        SCDC_IRQ_MASK &= ~0x20u;
        SCDC_TIMER_C8  = 0;
        SCDC_STATE     = 4;  /* SCDC_STATE_WAIT_PS */
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_wait_ps_to_active — WAIT_PS → ACTIVE (poll PS == 0x80)
 *
 * Polled from the periodic task.  In state 4, reads the PS register
 * and if it equals 0x80 (FRL active, no errors) advances to state 5
 * and notifies the HPD callback with arg 1 (link up).
 */
int scdc_fsm_wait_ps_to_active(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 4) {  /* SCDC_STATE_WAIT_PS */
        __disable_irq();
        if (scdc_read_ps() == 128) {  /* 0x80 = FRL PS active */
            SCDC_STATE = 5;           /* SCDC_STATE_ACTIVE */
            if (SCDC_HPD_CB)
                ((void (*)(int))SCDC_HPD_CB)(1);
        }
        __enable_irq();
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_ready_to_frl_trained — FRL_TRAINED → handle FRL training complete
 *
 * In state 3 (READY, which is also used as "training complete" from event 81),
 * disables the FRL IRQ bit and enables FRL mode, then advances to state 6
 * (FRL_TRAINED) and enqueues event 81 (FRL_TRAINING_DONE).
 * Also clears the SCDC scrambler-enable IRQ (bit3).
 */
int scdc_fsm_ready_to_frl_trained(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 3) {
        SCDC_IRQ_MASK &= ~0x10u;  /* disable FRL lock IRQ */
        SCDC_IRQ_MASK |=  0x2000u; /* enable FRL trained IRQ */
        scdc_enable_frl_mode();   /* switch to FRL mode */
        SCDC_STATE = 6;            /* SCDC_STATE_FRL_TRAINED */
        video_enqueue_event(81, 0, 0);
    }
    SCDC_IRQ_MASK &= ~8u;  /* clear scrambler IRQ enable */

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_frl_trained_to_stable — FRL_TRAINED → STABLE (FRL link up)
 *
 * In state 6 (FRL_TRAINED), enqueues event 79 (FRL_LINK_UP), resets the
 * link-loss timer, advances to state 7 (STABLE), sets the FRL stable
 * countdown, and probes the link state.
 */
int scdc_fsm_frl_trained_to_stable(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 6) {  /* SCDC_STATE_FRL_TRAINED */
        video_enqueue_event(79, 0, 0);
        SCDC_TIMER_C8  = 0;
        SCDC_STATE     = 7;   /* SCDC_STATE_STABLE */
        SCDC_TIMER_CC  = 100; /* 100-tick FRL stable monitoring window */
        video_snapshot_event_time(0x40458, 1);
    }
    SCDC_IRQ_MASK &= ~0x2000u;  /* disable FRL trained IRQ */

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_active_to_ready — ACTIVE → READY (re-training requested)
 *
 * In state 5 (ACTIVE), restores saved lane config, returns to state 3
 * (READY), and pulses the PHY reset line briefly via SCDC_PHY_CTRL bit2.
 *
 * @lane_b, @lane_a: lane configuration bytes for hw_misc_scdc_apply_lane_config
 *   (note: parameter order is reversed at the call site)
 */
int scdc_fsm_active_to_ready(uint8_t lane_b, uint8_t lane_a)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (SCDC_STATE == 5) {
        SCDC_TIMER_C8  = SCDC_TIMEOUT_INIT;
        SCDC_STATE     = 3;  /* SCDC_STATE_READY */
        hw_misc_scdc_apply_lane_config(lane_a, lane_b);
        SCDC_PHY_CTRL |= 4u;   /* assert PHY reset */
        delay_loop(1);
        SCDC_PHY_CTRL &= ~4u;  /* deassert PHY reset */
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_force_ready — force FSM to READY state from any state
 *
 * Used when the host controller sends a force-retrain command.  Sets the
 * TX force-FRL bit, resets the link-loss timer, advances to state 3, stores
 * the provided timeout callback, and reconfigures the lane count.
 *
 * @lane_b, @lane_a:  lane configuration
 * @timeout_cb:       callback invoked if FRL training times out
 */
int scdc_fsm_force_ready(uint8_t lane_b, uint8_t lane_a, uint32_t timeout_cb)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    SCDC_TX_CTRL   |= 0x1000000u;  /* set force-FRL TX bit */
    SCDC_TIMER_C8   = SCDC_TIMEOUT_INIT;
    SCDC_STATE      = 3;
    SCDC_TIMEOUT_CB = timeout_cb;
    hw_misc_scdc_apply_lane_config(lane_a, lane_b);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_fsm_enter_active_state — enter ACTIVE state: configure FRL TX scrambler
 *
 * This is the main FRL activation path.  Called when the FRL link has been
 * fully trained and is ready to carry video data.
 *
 * Sequence:
 *   1. Set FRL TX rate in SCDC_CTRL_EXT (bits [30:28] = FRL rate)
 *   2. Enable the FRL transmitter (bit31 of SCDC_CTRL_EXT)
 *   3. Clear the FRL scrambler error bit (bit23)
 *   4. Clear the FRL scrambler enable bit (bit22) → re-enable below
 *   5. Set the FRL scrambler bit count (bits [10:8] of SCDC_FRL_CTRL)
 *   6. Clear the FRL aux scrambler disable (bit11)
 *   7. Enable video RX lane (video_main_ctrl_set_bit0)
 *   8. Clear FRL error accumulator bit (bit22)
 *   9. Set FRL data-path enable bit (bit21)
 *  10. Set SCDC FRL lock flag
 *  11. Advance SCDC_EARC_STATE to 5
 *  12. Trigger video_is_debug_flag_bit4_set() to validate video timing
 */
int scdc_fsm_enter_active_state(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    /* Configure FRL rate in the extended control word */
    video_frl_rate_regs_apply(SCDC_FRL_RATE, 1);

    /* Enable FRL TX and configure scrambler */
    SCDC_CTRL_EXT |=  0x80000000u;  /* enable FRL TX */
    SCDC_FRL_CTRL &= ~0xFFFFF8FFu;  /* clear FRL rate field */
    SCDC_FRL_CTRL |=  (uint32_t)((SCDC_FRL_RATE & 7u) << 8);
    SCDC_FRL_CTRL &= ~0x800u;       /* enable FRL scrambler */

    SCDC_CTRL_EXT &= ~0x800000u;    /* clear FRL scrambler error flag */
    video_main_ctrl_set_bit0(1);      /* enable video RX lane */
    SCDC_CTRL_EXT &= ~0x400000u;    /* clear FRL error accumulator */
    SCDC_CTRL_EXT |=  0x200000u;    /* enable FRL data path */

    REG_SCDC_FRL_LOCK   = 1;
    SCDC_EARC_STATE     = 5;
    video_is_debug_flag_bit4_set();

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_link_loss_timeout — reset FSM and invoke link-loss handler
 *
 * Resets SCDC to state 1 (RESET), then invokes the registered timeout
 * callback (if any) with arg 255 (= link completely lost).  If no callback
 * is registered, falls through to the default hw_misc recovery path.
 */
int scdc_handle_link_loss_timeout(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    SCDC_STATE = 1;  /* SCDC_STATE_RESET */

    void (*cb)(int) = NULL;
    if (SCDC_TIMEOUT_CB) {
        cb              = (void (*)(int))SCDC_TIMEOUT_CB;
        SCDC_TIMEOUT_CB = 0;
    }

    if (cb)
        cb(255);
    else
        hw_misc_frl_link_loss_recovery();

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * SCDC event handlers — video / HDCP notification
 * ========================================================================= */

/*
 * scdc_handle_frl_lane_error_update — FRL lane status changed (IRQ from SCDC_STATUS_1)
 *
 * Reads the lane error counts from SCDC_STATUS_1, fires the registered
 * FRL-rate-change callback, and enqueues event 87 (FRL_LANE_STATUS) with
 * lane error counts packed into the two argument bytes.
 *
 * SCDC_STATUS_1 bit layout (HDMI 2.1 spec §10.4.1.3):
 *   bits [11: 8]  = lane 0 bit error count (v1)
 *   bits [15:12]  = lane 1 bit error count (v2)
 *   bits [19:16]  = lane 2 bit error count (v3)
 *   bits [23:20]  = lane 3 bit error count (v4)
 */
int scdc_handle_frl_lane_error_update(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint32_t status = SCDC_STATUS_1;
    uint8_t lane0   = (uint8_t)((status >>  8) & 0xF);
    int16_t lane1   = (int16_t)(status >> 12);
    uint8_t lane2   = (uint8_t)((status >> 16) & 0xF);
    uint8_t lane3   = (uint8_t)((status >> 20) & 0xF);

    if (SCDC_FRL_RATE_CB)
        ((void (*)(int, int, int, int))SCDC_FRL_RATE_CB)(lane0, lane1, lane2, lane3);

    video_enqueue_event(87,
        (int)((uint8_t)(lane0 | (lane1 << 4))),
        (int)((uint8_t)(lane2 | (lane3 << 4))));

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_frl_link_status_change — FRL link-status changed (scrambler / EARC update)
 *
 * Reads the FRL scrambling status (SCDC_FRL_CTRL_EXT bit30 = DSC active),
 * the eARC enable flag, and the FRL state to compose the SCDC link-status
 * byte written to hw_misc.  Then:
 *   - If FRL is active: configures HDCP authentication
 *   - Otherwise: enqueues event 93 (FRL_STATUS_CHANGE) with FRL state
 */
int scdc_handle_frl_link_status_change(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    int dsc_active  = (int)((REG_FRL_CTRL_EXT >> 30) & 1u);
    uint8_t misc12  = (uint8_t)hw_misc_get_reg_39d();
    uint8_t scramble_byte = 0;

    /* Compose scramble control byte for hw_misc */
    if (dsc_active) {
        scramble_byte = (uint8_t)(2u * SCDC_FRL_MODE);
    } else if (SCDC_DSC_MODE) {
        scramble_byte = (uint8_t)(1u << SCDC_DSC_MODE);
    }

    int frl_active = frl_is_state_8_or_higher();
    uint8_t misc12_new;

    if (frl_active) {
        scramble_byte |= 1u;
        misc12_new = misc12 | 1u;
    } else {
        misc12_new = misc12 & 0xFEu;
    }

    hw_misc_set_reg_398(scramble_byte);
    hw_misc_set_reg_39d(misc12_new);
    REG_IRQ_FLAGS_1E5 |= 2u;  /* signal IRQ flags updated */

    if (frl_is_state_8_or_higher()) {
        /* FRL active — arm HDCP authentication */
        video_set_path_ctrl_bit27(1);
        REG_HDCP_AUTH_CTRL |= 0x400u;
        if (REG_HDCP_FLAGS_265) {
            if ((*(volatile uint8_t *)0x40100E16 & 4u) != 0)
                REG_HDCP_KEY_VALID_266 = 1;
        } else {
            REG_HDCP_FLAGS_265 = 127;
        }
    } else {
        int frl_state = frl_is_state_8_or_higher();
        video_enqueue_event(93, frl_state, 0);
        if (REG_TCB_CURRENT_TASK != saved)
            system_halt_clear_flag();
    }

    return (int)saved;
}

/*
 * scdc_handle_earc_state_change — eARC connection state changed
 *
 * If eARC is enabled (SCDC_EARC_ENABLED):
 *   - Updates misc12 bit0 = FRL active
 *   - If FRL TX supports 400MHz (SCDC_FRL_CTRL_EXT bit22): arms HDCP
 *   - Otherwise: sets the HDCP wait flag and enqueues event 94
 * If eARC is disabled:
 *   - Clears misc12 bit1 (eARC bit) and enqueues event 94 with arg0=0
 */
int scdc_handle_earc_state_change(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint8_t misc12    = (uint8_t)hw_misc_get_reg_39d();
    uint8_t misc12_new;

    if (SCDC_EARC_ENABLED) {
        misc12_new = misc12 | 2u;
    } else {
        misc12_new = misc12 & 0xFDu;
    }
    hw_misc_set_reg_39d(misc12_new);

    if (SCDC_EARC_ENABLED) {
        if ((REG_FRL_CTRL_EXT & 0x400000u) != 0) {
            /* FRL TX supports 400MHz — arm HDCP */
            video_set_path_ctrl_bit27(1);
            REG_HDCP_AUTH_CTRL |= 0x400u;
            if (REG_HDCP_FLAGS_265) {
                if ((*(volatile uint8_t *)0x40100E16 & 4u) != 0)
                    REG_HDCP_KEY_VALID_266 = 1;
            } else {
                REG_HDCP_FLAGS_265 = 127;
            }
            goto done;
        }
        REG_VIDEO_HDCP_WAIT = 1;
    }

    video_enqueue_event(94, SCDC_EARC_ENABLED, 0);

done:
    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_poll_ddc_idle — eARC / DDC idle poll
 *
 * Clears the eARC check flag, runs the video idle state poll, and if SCDC
 * is in RESET (state 1) and eARC state is not 1, enqueues event 101 to
 * re-trigger the DDC idle scan.
 */
int scdc_poll_ddc_idle(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    SCDC_EARC_CHK = 0;
    video_update_reg_335_353_wrapper();
    if (scdc_is_state_1() && SCDC_EARC_STATE != 1)
        video_enqueue_event(101, 0, 0);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_hdcp_reconfig_req — HDCP / link reconfiguration request
 *
 * Enables hw_misc state 54, clears the link-down flag, resets the video
 * mode register, re-triggers video configuration, and re-enqueues event 142.
 */
int scdc_handle_hdcp_reconfig_req(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    hw_misc_set_reg_254(1);
    REG_LINK_DOWN_FLAG = 0;
    video_update_reg_335_353();
    video_handle_fatal_error_overlay(4, 0);
    video_enqueue_event(142, 0, 0);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_hpd_or_link_loss — HPD / link loss detection
 *
 * Fired when SCDC_STATUS_0 changes indicate a link event.
 * - If bit3 (HotPlug) is set: clear the scrambler enable and re-write config
 * - If in FRL_TRAINED (6) or STABLE (7): check for ARC disconnect or PS != 1
 *   → if so, notify HPD callback with arg 2 (link down) and re-enqueue event 146
 */
int scdc_handle_hpd_or_link_loss(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint8_t status0 = SCDC_STATUS_0;

    if ((status0 & 8u) != 0) {
        /* HotPlug change: clear scrambler enable and resend config */
        SCDC_IRQ_MASK &= ~1u;
        scdc_write_config_1();
    }

    if (SCDC_STATE == 7 || SCDC_STATE == 6) {
        if ((status0 & 0x20u) != 0) {
            /* ARC bit set → link lost */
            hw_misc_scdc_set_state3_timeout();
            if (SCDC_HPD_CB)
                ((void (*)(int))SCDC_HPD_CB)(2);
            video_enqueue_event(146, 0, 0);
        } else if (((uint32_t)scdc_read_ps() >> 8 & 3u) != 1u) {
            /* PS state is not 1 (FRL active) → link lost */
            hw_misc_scdc_set_state3_timeout();
            if (SCDC_HPD_CB)
                ((void (*)(int))SCDC_HPD_CB)(2);
            video_enqueue_event(146, 1, 0);
        }
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * SCDC audio / eARC event handlers
 * ========================================================================= */

/*
 * scdc_handle_earc_32ch_event — audio format change (eARC 32-channel event)
 *
 * Dispatches an audio state refresh and enqueues video event 11 with the
 * current audio state code and subtype 32.  Also checks whether the DDC
 * engine has the eARC FIFO flag (bit5 of DDC_FLAGS = 0x20) and if so
 * enables the eARC FIFO drain bit in the SCDC register.
 */
int scdc_handle_earc_32ch_event(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    audio_h14_status_reg_set_c000();

    __disable_irq();
    uint8_t audio_state = (uint8_t)audio_protocol_get_state_code();
    video_enqueue_event(11, audio_state, 32);
    __enable_irq();

    /* If DDC eARC FIFO is enabled, set the eARC drain bit */
    if ((DDC_FLAGS & 0x20u) != 0) {
        *(volatile uint32_t *)0x40103764 |= 0x800u;
        *(volatile uint8_t  *)0x4052D    |= 0x10u;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_earc_16ch_event — audio format change (eARC 16-channel event)
 *
 * Sets the eARC debounce timer to 20 ticks, enqueues video event 11 with
 * the audio state and subtype 16, then clears the eARC pending flag.
 */
int scdc_handle_earc_16ch_event(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    REG_EARC_DEBOUNCE = 20;

    __disable_irq();
    uint8_t audio_state = (uint8_t)audio_protocol_get_state_code();
    video_enqueue_event(11, audio_state, 16);
    __enable_irq();

    REG_EARC_PENDING = 0;

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_handle_link_down_cleanup — HDMI link-down: reset video and audio paths
 *
 * Complete link-down cleanup:
 *   - Disable eARC or FRL video (depending on DSC mode flag)
 *   - Disable scrambler bits
 *   - Clear video flags
 *   - Reset audio subsystem
 *   - Clear HDCP state
 *   - Enqueue event 84 (LINK_DOWN) if HPD is not being re-asserted
 */
int scdc_handle_link_down_cleanup(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    if (REG_EARC_FLAG_4E4) {
        /* DSC/eARC mode — disable eARC RX */
        video_reg_f38_set_bit15(0);
    } else {
        /* Standard FRL mode */
        video_main_ctrl_set_bit0(0);
        SCDC_CTRL_EXT &= ~0x800000u;  /* clear scrambler error */
        SCDC_CTRL_EXT &= ~0x400000u;  /* clear FRL error accumulator */
        REG_SCDC_FRL_LOCK = 0;
        video_main_ctrl_set_bit4(0);
    }

    video_main_ctrl_set_bit4_cond(0);
    REG_VIDEO_FLAGS_120    &= ~0x8000u;
    video_main_ctrl_set_0a_and_clear();
    REG_LINK_DOWN_FLAG      = 0;
    video_update_reg_335_353();

    if (REG_VIDEO_HPD_STATE) {
        /* HPD is being re-asserted — defer cleanup */
        REG_VIDEO_HPD_PENDING = 1;
    } else {
        hw_misc_clear_reg_492_and_irq_b();
        audio_protocol_dispatch_rd_cmd();

        /* Clear HDCP / eARC session state */
        REG_EARC_HDCP_090 = 0;
        REG_EARC_HDCP_094 = 0;
        REG_EARC_HDCP_082 = 0;
        REG_EARC_HDCP_084 = 0;
        REG_EARC_HDCP_088 = 0;
        REG_EARC_HDCP_086 = 0;
        REG_EARC_FLAG_274  = 0;
        REG_HPD_DEBOUNCE_26D = 0;
        REG_EARC_FLAG_BD4  = 0;

        video_enqueue_event(84, 0, 0);
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/* =========================================================================
 * SCDC config verification and hw config updates
 * ========================================================================= */

/*
 * scdc_update_frl_trained_rate — update FRL rate training state
 *
 * If FRL training mode is active (SCDC_TRAIN_MODE == 1), records the newly
 * trained FRL rate in the trained-rates bitmask and updates the active rate.
 * Then clears the FRL training flags and notifies the audio subsystem of
 * the new rate.
 *
 * @rate: FRL bit-rate code (3 = 3Gbps, 6, 8, 10, 12 = 12Gbps)
 */
int scdc_update_frl_trained_rate(uint8_t rate)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint32_t trained_mask = SCDC_TRAINED_MASK;
    if (SCDC_TRAIN_MODE == 1) {
        trained_mask       = SCDC_TRAINED_MASK | (1u << (rate - 1));
        SCDC_TRAINED_MASK  = trained_mask;
        SCDC_FRL_RATE      = rate;
    }

    /* Reset FRL training flags */
    *(volatile uint8_t *)0x40454 = 0;
    *(volatile uint8_t *)0x40455 = 0;

    /* Notify audio: new FRL rate (arg0=255=all, arg1=0x404C4=config base) */
    audio_handle_frl_training_result(255, 0x404C4, (int)trained_mask);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_update_hdcp_video_flags — update HDCP / video path flags on eARC state change
 *
 * @mode: 1 = eARC disconnected, 2 = eARC connected
 *
 * In both cases:
 *   - Clears misc12 bit0 (FRL active)
 *   - Re-enables HDCP video path
 *   - Reconfigures video mode (6 = eARC disconnect, 7 = eARC connect)
 */
int scdc_update_hdcp_video_flags(int mode)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;

    uint8_t misc12;
    int video_mode;

    if (mode == 1) {
        video_mode = 6;
    } else if (mode == 2) {
        video_mode = 7;
    } else {
        goto done;
    }

    misc12 = (uint8_t)hw_misc_get_reg_39d();
    hw_misc_set_reg_39d((uint8_t)(misc12 & 0xFEu));
    video_set_path_ctrl_bit27(1);
    video_handle_fatal_error_overlay(video_mode, 1);

done:
    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return (int)saved;
}

/*
 * scdc_verify_nvram_sentinels — verify that flash config area matches sentinels
 *
 * Reads five 4-byte words from flash starting at @base_addr and compares
 * them against known sentinel values stored in SRAM at 0x40058 and 0x400A0.
 * Returns true (non-zero) if any value is uninitialized (0xFFFFFFFF) or
 * if the sentinel comparison fails.
 *
 * Used during boot to detect whether NVRAM needs to be re-initialized.
 */
int scdc_verify_nvram_sentinels(int base_addr)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int v0, v1, v2, v3, v4;
    int result;

    int err  = flash_read_data(base_addr,      4, &v0, 1);
    err     |= flash_read_data(base_addr +  4,  4, &v1, 1);
    err     |= flash_read_data(base_addr +  8,  4, &v2, 1);
    err     |= flash_read_data(base_addr + 16,  4, &v3, 0);
    err     |= flash_read_data(base_addr + 12,  4, &v4, 1);

    result = (err == 0xFF)
           || (v0 != -1)
           || (v1 != -1)
           || (REG_CFG_SENTINEL_A != (uint32_t)v2)
           || (v3 != -1)
           || (REG_CFG_SENTINEL_B != (uint32_t)v4);

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * scdc_load_nvram_feature_flags — load feature flags from NVRAM / OTP
 *
 * Reads the product mode from NVRAM and populates the feature enable flags:
 *   - Mode 1: copy flags from NVRAM region 0x40334 / 0x40331
 *   - Mode 0x2000: eARC-enabled device
 *   - Mode 0x26000 (155648): non-eARC device
 *
 * Returns the final value of REG_FEAT_OUT_35F (eARC feature flag).
 */
int scdc_load_nvram_feature_flags(void)
{
    if (REG_FEAT_MODE == 1) {
        REG_FEAT_OUT_340 = REG_FEAT_OUT_334;
        REG_FEAT_OUT_35F = REG_FEAT_OUT_331;
    } else if (REG_FEAT_CAP_48 == 0x2000u) {
        REG_FEAT_OUT_35F = 1;
        REG_FEAT_OUT_340 = REG_FEAT_VAL_4C;
    } else if (REG_FEAT_CAP_48 == 155648u) {
        REG_FEAT_OUT_35F = 0;
        REG_FEAT_OUT_340 = REG_FEAT_VAL_50;
    }
    return (int)REG_FEAT_OUT_35F;
}

/*
 * is_digit — return true if character is ASCII decimal digit ('0'..'9')
 */
int is_digit(int c)
{
    return (unsigned int)(c - '0') < 10u;
}

/*
 * scdc_verify_fw_config_block — verify a firmware config block against hardware status
 *
 * Reads the firmware image at @fw_addr, then reads the hardware status byte
 * and compares it against @expected_status.  On mismatch, writes error code
 * 7 into the config block.
 *
 * @fw_addr:         flash address of firmware config
 * @expected_status: expected hardware status byte
 * @cfg_block:       pointer to config block (byte +3 = error code out)
 *
 * Returns 0 on match, 255 on read error or mismatch.
 */
int scdc_verify_fw_config_block(unsigned int fw_addr, int expected_status, int cfg_block)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int result;
    uint8_t hw_status[4];
    uint32_t detail = (uint32_t)cfg_block;

    hw_misc_scdc_phy_ctrl_init();

    if (flash_read_firmware_image(fw_addr, fw_addr, cfg_block, 1) == 255) {
        result = 255;
    } else {
        flash_read_hw_status(hw_status, &detail);
        if (expected_status == (int)hw_status[0]) {
            result = 0;
        } else {
            *(uint8_t *)(cfg_block + 3) = 7;  /* write error code */
            result = 255;
        }
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * scdc_apply_capability_regs — update extended SCDC control word from capability regs
 *
 * Reads the SCDC capability bytes (0x40100C80–C81) and writes them into
 * SCDC_CTRL_EXT bits [17:16] (lane count) and bit 28 (scrambler polarity).
 */
int scdc_apply_capability_regs(void)
{
    SCDC_CTRL_EXT = (SCDC_CTRL_EXT & 0xEFFCFFFFu)
                  | (uint32_t)((SCDC_CAP_0 & 3u) << 16)
                  | (uint32_t)((SCDC_CAP_1 >> 7) << 28);
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * scdc_configure_aux_scrambler — configure auxiliary scrambler path
 *
 * @enable: 1 = enable auxiliary scrambler (eARC mode), 0 = disable
 *
 * When enabled, sets bits [4:3] = 0b10 in SCDC_AUX_CTRL and clears
 * SCDC_AUX2 bit0 (internal scrambler bypass).
 */
int scdc_configure_aux_scrambler(int enable)
{
    if (enable) {
        SCDC_AUX_CTRL = (SCDC_AUX_CTRL & 0xE7u) | 0x10u;
        SCDC_AUX2    &= ~1u;
    } else {
        SCDC_AUX_CTRL &= 0xE7u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

/*
 * scdc_check_frl_pll_lock — check FRL PLL lock status
 *
 * Pulses the PLL reset, waits, then reads the PLL lock bit.
 * If not locked, enables the PLL in hw_misc and re-reads.
 *
 * Returns true (1) if PLL was already locked, false (0) if recovery was
 * attempted (result of recovery stored in SRAM at 0x4025A/0x4025B).
 */
int scdc_check_frl_pll_lock(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int locked;

    /* Pulse PLL reset */
    SCDC_PLL_CTRL |= 1u;
    SCDC_PLL_CTRL &= ~1u;
    delay_loop(20);

    /* Select PLL status register (0x0B = lock status) */
    SCDC_PLL_ADDR = 11;
    if (SCDC_PLL_DATA & 0x80u) {
        /* Already locked */
        locked = 1;
    } else {
        __disable_irq();
        hw_misc_irq_enable_mask_b_clear();
        __enable_irq();
        delay_loop(20);

        /* Read lock status after recovery attempt */
        *(volatile uint8_t *)0x4025B = SCDC_PLL_STATUS & 1u;
        SCDC_PLL_SEL                  = 2;
        *(volatile uint8_t *)0x4025A  = SCDC_PLL_STATUS & 1u;
        locked = (SCDC_PLL_STATUS & 1u) != 0;
    }

    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return locked;
}

/*
 * scdc_verify_bist_lane_errors — verify BIST lane error counts (20 high / 20 low)
 *
 * Counts set bits in SCDC_BIST_ERR1 (8 bits) and SCDC_BIST_ERR0 (32 bits).
 * Valid BIST pass requires exactly 20 bits set and 20 bits clear across both
 * registers.  On pass, sets bit2 of SCDC_BIST_CTRL.
 *
 * Returns 0 on pass, 255 on fail.
 */
int scdc_verify_bist_lane_errors(void)
{
    uint32_t err1 = SCDC_BIST_ERR1;  /* 8-bit lane 1 error count */
    uint32_t err0 = SCDC_BIST_ERR0;  /* 32-bit lane 0 error count */
    int set_bits = 0, clear_bits = 0;

    /* Count bits in BIST_ERR1 (8 bits) */
    for (uint8_t i = 0; i < 8; i++) {
        if (err1 & 1u)
            set_bits   = (uint8_t)(set_bits + 1);
        else
            clear_bits = (uint8_t)(clear_bits + 1);
        err1 >>= 1;
    }

    /* Count bits in BIST_ERR0 (32 bits) */
    for (uint8_t i = 0; i < 32; i++) {
        if (err0 & 1u)
            set_bits   = (uint8_t)(set_bits + 1);
        else
            clear_bits = (uint8_t)(clear_bits + 1);
        err0 >>= 1;
    }

    if (set_bits != 20 || clear_bits != 20)
        return 255;

    SCDC_BIST_CTRL |= 4u;  /* mark BIST pass */
    return 0;
}

/*
 * scdc_check_earc_capability — check whether eARC feature is supported
 *
 * Reads the IRQ flags to determine eARC capability:
 *   bit3 of REG_IRQ_FLAGS_1E4 = eARC capable
 *   bit7 of REG_IRQ_FLAGS_1E4 = eARC enabled in OTP
 *   bit5 of REG_IRQ_FLAGS_1E4 = eARC FRL capable
 *
 * Returns 1 if eARC is supported and operational, 0 otherwise.
 */
int scdc_check_earc_capability(void)
{
    uint32_t saved = REG_TCB_CURRENT_TASK;
    int result;

    if (((uint8_t)REG_IRQ_FLAGS_1E4 >> 3) & 1u) {
        if ((uint8_t)REG_IRQ_FLAGS_1E4 >> 7) {
            if (((uint8_t)REG_IRQ_FLAGS_1E4 >> 5) & 1u) {
                if (!frl_is_state_8_or_higher()) {
                    result = 0;
                    goto done;
                }
                REG_EARC_FLAG_4E4  = 1;  /* eARC cap = DSC mode */
            } else {
                REG_EARC_FLAG_4E4  = 0;
            }
            REG_EARC_FLAG_4E4B = 1;  /* eARC enabled */
            result = 1;
        } else {
            result = 0;
        }
    } else {
        /* Not eARC capable */
        REG_EARC_FLAG_4E4B = 0;
        REG_EARC_FLAG_4E4  = 0;
        result = 1;
    }

done:
    if (REG_TCB_CURRENT_TASK != saved)
        system_halt_clear_flag();
    return result;
}

/*
 * scdc_handle_event_84_aux — audio/video teardown for link-down (event 84)
 *
 * Secondary cleanup called by scdc_handle_link_down_cleanup when HPD is stable:
 * resets eARC mute state and performs the hw_misc + audio cleanup.
 * (This is the scdc_handle_link_loss_timeout → hw_misc_frl_link_loss_recovery
 * path, invoked via function pointer at SCDC_TIMEOUT_CB.)
 */

/* =========================================================================
 * SCDC clock calculation lookup table
 * ========================================================================= */

/*
 * scdc_calc_frl_audio_clock_div — compute FRL audio clock divider from
 *                                  pixel clock and FRL bit-rate
 *
 * The HDMI 2.1 FRL audio clock is derived from the pixel clock and the
 * FRL link rate.  This function implements a two-dimensional lookup:
 *   - Outer dimension: pixel clock range (encoded as SRAM byte at 0x40260)
 *   - Inner dimension: FRL bit-rate code (@a1 = 3/6/8/10/12)
 *
 * The pixel clock ranges are:
 *   ≤0x22  : ~34 MHz (SD)
 *   ≤0x2D  : ~45 MHz
 *   ≤0x31  : ~49 MHz
 *   ≤0x41  : ~65 MHz
 *   ≤0x59  : ~89 MHz
 *   ≤0x62  : ~98 MHz
 *   ≤0x81  : ~129 MHz
 *   ≤0xB2  : ~178 MHz
 *   ≤0xC1  : ~193 MHz
 *   <0x102 : ~258 MHz
 *   <0x162 : ~354 MHz
 *   <0x182 : ~386 MHz
 *   ≤0x201 : ~513 MHz
 *   ≤0x2C2 : ~706 MHz
 *   >0x2C2 : >706 MHz (4K120Hz range)
 *
 * FRL bit-rate codes: 3=3Gbps, 6=6Gbps, 8=8Gbps, 10=10Gbps, 12=12Gbps
 *
 * The computed divider value is written into both SCDC_CLK_DIV_A and
 * SCDC_CLK_DIV_B (lower 20 bits only — upper 12 bits preserved).
 *
 * @a1: FRL bit-rate code
 */
int scdc_calc_frl_audio_clock_div(int a1)
{
    int div = 5760;  /* default clock divider */
    uint32_t pclk = *(volatile uint8_t *)0x40260;

    /*
     * Two-level if/else tree that maps (pclk_range, frl_rate) → clock divider.
     * The divider values are derived from:
     *   div = (FRL_BW_Mbps * 1000) / (pixel_clock_MHz * audio_ratio)
     * rounded to the nearest supported divider step.
     */
    if (pclk > 0x22u) {
        if (pclk > 0x2Du) {
            if (pclk > 0x31u) {
                if (pclk > 0x41u) {
                    if (pclk > 0x59u) {
                        if (pclk > 0x62u) {
                            if (pclk > 0x81u) {
                                if (pclk > 0xB2u) {
                                    if (pclk > 0xC1u) {
                                        if (pclk >= 0x102u) {
                                            if (pclk >= 0x162u) {
                                                if (pclk >= 0x182u) {
                                                    if (pclk > 0x201u) {
                                                        if (pclk > 0x2C2u) {
                                                            /* >706 MHz: 4K120Hz */
                                                            if      (a1 ==  3) div = 23040;
                                                            else if (a1 ==  6) div = 24192;
                                                            else if (a1 ==  8) div = 24192;
                                                            else if (a1 == 10) div = 20736;
                                                            else if (a1 == 12) div = 19008;
                                                        } else {
                                                            /* ≤706 MHz */
                                                            if      (a1 ==  3) div = 21168;
                                                            else if (a1 ==  6) div = 21168;
                                                            else if (a1 ==  8) div = 15876;
                                                            else if (a1 == 10) div = 15876;
                                                            else if (a1 == 12) div = 15876;
                                                        }
                                                    } else {
                                                        /* ≤513 MHz */
                                                        if      (a1 ==  3) div = 16896;
                                                        else if (a1 ==  6) div = 16128;
                                                        else if (a1 ==  8) div = 16128;
                                                        else if (a1 == 10) div = 13824;
                                                        else if (a1 == 12) div = 12288;
                                                    }
                                                } else {
                                                    /* <386 MHz */
                                                    if      (a1 ==  3) div = 11520;
                                                    else if (a1 ==  6) div = 12096;
                                                    else if (a1 ==  8) div = 12096;
                                                    else if (a1 == 10) div = 10368;
                                                    else if (a1 == 12) div =  9504;
                                                }
                                            } else {
                                                /* <354 MHz */
                                                if      (a1 ==  3) div = 10584;
                                                else if (a1 ==  6) div = 10584;
                                                else if (a1 ==  8) div =  7938;
                                                else if (a1 == 10) div =  7938;
                                                else if (a1 == 12) div =  7938;
                                            }
                                        } else {
                                            /* <258 MHz */
                                            if      (a1 ==  3) div =  8448;
                                            else if (a1 ==  6) div =  8064;
                                            else if (a1 ==  8) div =  8064;
                                            else if (a1 == 10) div =  6912;
                                            else if (a1 == 12) div =  6144;
                                        }
                                    } else {
                                        /* ≤193 MHz */
                                        if      (a1 ==  3) div = 23040;
                                        else if (a1 ==  6) div = 24192;
                                        else if (a1 ==  8) div = 24192;
                                        else if (a1 == 10) div = 20736;
                                        else if (a1 == 12) div = 19008;
                                    }
                                } else {
                                    /* ≤178 MHz */
                                    if      (a1 ==  3) div = 21168;
                                    else if (a1 ==  6) div = 21168;
                                    else if (a1 ==  8) div = 15876;
                                    else if (a1 == 10) div = 15876;
                                    else if (a1 == 12) div = 15876;
                                }
                            } else {
                                /* ≤129 MHz */
                                if      (a1 ==  3) div = 16896;
                                else if (a1 ==  6) div = 16128;
                                else if (a1 ==  8) div = 16128;
                                else if (a1 == 10) div = 13824;
                                else if (a1 == 12) div = 12288;
                            }
                        } else {
                            /* ≤98 MHz */
                            if      (a1 ==  3) div = 11520;
                            else if (a1 ==  6) div = 12096;
                            else if (a1 ==  8) div = 12096;
                            else if (a1 == 10) div = 10368;
                            else if (a1 == 12) div =  9504;
                        }
                    } else {
                        /* ≤89 MHz */
                        if      (a1 ==  3) div = 10584;
                        else if (a1 ==  6) div = 10584;
                        else if (a1 ==  8) div =  7938;
                        else if (a1 == 10) div =  7938;
                        else if (a1 == 12) div =  7938;
                    }
                } else {
                    /* ≤65 MHz */
                    if      (a1 ==  3) div =  8448;
                    else if (a1 ==  6) div =  8064;
                    else if (a1 ==  8) div =  8064;
                    else if (a1 == 10) div =  6912;
                    else if (a1 == 12) div =  6144;
                }
            } else {
                /* ≤49 MHz */
                if      (a1 ==  6) div = 6048;
                else if (a1 ==  8) div = 6048;
                else if (a1 == 10) div = 5184;
                else if (a1 == 12) div = 4752;
            }
        } else {
            /* ≤45 MHz */
            if      (a1 ==  3) div = 5292;
            else if (a1 ==  6) div = 5292;
            else if (a1 ==  8) div = 3969;
            else if (a1 == 10) div = 3969;
            else if (a1 == 12) div = 3969;
        }
    } else {
        /* ≤34 MHz */
        if      (a1 ==  3) div = 4224;
        else if (a1 ==  6) div = 4032;
        else if (a1 ==  8) div = 4032;
        else if (a1 == 10) div = 3456;
        else if (a1 == 12) div = 3072;
    }

    /* Write divider into both clock divider registers (lower 20 bits) */
    SCDC_CLK_DIV_A = (SCDC_CLK_DIV_A & 0xFFF00000u) | (uint32_t)div;
    SCDC_CLK_DIV_B = (SCDC_CLK_DIV_B & 0xFFF00000u) | (uint32_t)div;

    return (int)REG_TCB_CURRENT_TASK;
}
