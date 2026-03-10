/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * audio_earc.c — audio format handling, mute control, ARC/eARC protocol glue,
 * FRL/eARC integration, and audio-side DDC helpers.
 *
 * This file was rewritten from decompiler output into structured C.  A few of
 * the original soft-float helper sequences remain approximated with native
 * float/double arithmetic where the decompiler lost ABI details.
 */

#include "include/defs.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================= */
/* Cross-module declarations                                                  */
/* ========================================================================= */

int  ddc_write_request(int dev_addr, int reg_offset, uint16_t *req);
int  ddc_transfer_abort_callback(void);
int  ddc_read_request(int dev_addr, int reg_offset, uint8_t **req);
int  hw_misc_check_state_3(void);
int  hw_misc_set_state_48(void);
int  scdc_write_config_2(void);
int  scdc_write_config_3(void);
int  scdc_write_config_4(void);
int  scdc_update_config_5(char rate);
int  scdc_handle_event_11_32(void);
int  scdc_execute_callback_or_default(void);
int  scdc_calculate_and_set_clock(int rate);
uint64_t hw_misc_process_state_82(uint64_t value);
int  hw_misc_set_state_63(void);
int  hw_misc_process_state_99(void);
int  hw_misc_process_state_152(void);
int  hw_misc_process_state_154(void);
int  scdc_process_pending_reads(void);
int  scdc_enable_feature_1(void);
int  scdc_state_transition_5(void);
int  frl_txpll_enable_lol(void);
int  frl_rate_set_0(void);
int  frl_rate_set_2(void);
int  frl_check_state_1(void);
int  frl_process_state_2(void);
int  frl_check_state_3(void);
int  frl_process_state_4(void);
int  frl_get_rate_params(uint8_t *lane_cfg, uint8_t *rate_code);
int  video_set_hw_state_29(void);
int  video_process_state_33(void);
int  video_set_hw_state_34(int muted, char idx);
int  video_set_state_35(void);
BOOL video_check_state_40(int value);
int  video_check_state_41(void);
int  video_process_state_44(void);
int  video_enqueue_event(int event, int arg0, int arg1);
int  video_get_hw_state_45(void);
int  video_get_hw_state_46(void);
int  video_get_hw_state_47(void);
int  video_get_hw_state_48(void);
int  video_get_hw_state_50(void);
int  video_get_hw_state_53(void);
int  video_get_hw_state_54(void);
int  video_get_hw_state_55(void);
int  video_set_hw_state_75(void);
int  video_math_pow2(int value);
int  video_process_state_77(void);
int  video_print_cs_regs(void);
int  video_process_state_78(void);
int  video_get_state_81(void);
int  video_read_hw_buffer(int offset);
int  video_set_state_112(void);
int  video_process_state_120(void);
unsigned int video_timer_update_122(uint8_t *timestamp, int a2, int a3);
int  video_get_state_125(int base, int offset);
int  video_check_hw_state_126(void);
int  video_check_hw_state_127(void);
int  video_state_machine_2(void);
int  video_process_state_146(void);
int  video_process_state_170(void);

/* Internal forward declarations used before definition order. */
uint64_t audio_float_math_1(uint64_t bits);
int  audio_get_format(void);
int  audio_mute_poll(int event);
int  audio_set_format(char format);
int  audio_update_hw_config(unsigned int cfg);
int  audio_process_state_95(void);
int  audio_set_state_96(int h23);
int  audio_process_state_99(void);
int  audio_process_state_100(void);
int  audio_process_state_104(void);
int  audio_get_hw_state_10(void);
int  audio_get_hw_state_11(void);
int  audio_get_hw_state_12(void);
int  audio_set_hw_state_26(void);
int  audio_set_hw_state_27(void);
int  audio_get_state_81(void);
int  audio_get_state_89(void);
BOOL audio_check_state_84(void);
int  audio_process_state_28(void);
int  audio_process_state_37(void);
int  audio_process_state_58(void);
int  audio_process_state_71(void);
BOOL audio_check_state_74(void);
int  audio_handle_h23t_wr_str_man(void);
int  audio_set_hw_state_24(void);
int  audio_set_hw_state_29(void);
int  audio_set_hw_state_30(void);
int  audio_set_hw_state_38(void);
int  audio_set_hw_state_57(void);
int  audio_set_hw_state_63(void);
int  audio_set_hw_state_65(void);
int  audio_process_state_59(void);
int  audio_process_state_82(void);
int  audio_process_state_83(void);
int  audio_get_state_91(void);
int  audio_get_state_92(void);
int  audio_check_state_42(void);
int  audio_get_state_85(void);
int  audio_process_state_73(void);
int  audio_process_state_90(void);
int  audio_set_hw_state_62(void);
int  audio_set_hw_state_64(void);
int  audio_set_hw_state_79(void);

/* ========================================================================= */
/* Local SRAM / register aliases                                              */
/* ========================================================================= */

#define REG_AUDIO_INFO_CLR              (*(volatile uint8_t  *)0x40100063)
#define REG_AUDIO_N_LOW                 (*(volatile uint8_t  *)0x4010006D)
#define REG_AUDIO_N_MID                 (*(volatile uint8_t  *)0x4010006C)
#define REG_AUDIO_N_HIGH                (*(volatile uint8_t  *)0x4010006B)
#define REG_AUDIO_CTS_LOW               (*(volatile uint8_t  *)0x40100070)
#define REG_AUDIO_CTS_MID               (*(volatile uint8_t  *)0x4010006F)
#define REG_AUDIO_CTS_HIGH              (*(volatile uint8_t  *)0x4010006E)
#define REG_AUDIO_INFO_INDEX            (*(volatile uint8_t  *)0x401000BD)
#define REG_AUDIO_INFO_DATA             (*(volatile uint8_t  *)0x401000BE)
#define REG_AUDIO_CS_CFG                (*(volatile uint32_t *)0x40100D60)
#define REG_AUDIO_FIFO_LIMIT            (*(volatile uint32_t *)0x40100D6C)
#define REG_AUDIO_FIFO_PERIOD           (*(volatile uint32_t *)0x40100D70)
#define REG_AUDIO_FIFO_LOW              (*(volatile uint32_t *)0x40100D64)
#define REG_AUDIO_FIFO_HIGH             (*(volatile uint32_t *)0x40100D68)
#define REG_AUDIO_CTS_STABLE            (*(volatile uint16_t *)0x4025C)
#define REG_AUDIO_SOURCE_FS             (*(volatile uint32_t *)0x40260)
#define REG_AUDIO_8CH_FLAG              (*(volatile uint8_t  *)0x41071)
#define REG_AUDIO_CODE_TYPE             (*(volatile uint8_t  *)0x41074)
#define REG_AUDIO_LAYOUT_3D             (*(volatile uint8_t  *)0x41078)
#define REG_AUDIO_3D_CHANNEL_COUNT      (*(volatile uint8_t  *)0x4107C)

#define REG_AUDIO_FORMAT                (*(volatile uint8_t  *)0x40101697)
#define REG_AUDIO_FREQREADY_TIMER       (*(volatile uint32_t *)0x403AB)
#define REG_AUDIO_FREQREADY_TOGGLE      (*(volatile uint8_t  *)0x403AA)
#define REG_AUDIO_FREQREADY_TS          ((uint8_t *)0x403D0)
#define REG_AUDIO_EN_SEL_TIMER          (*(volatile uint32_t *)0x40398)
#define REG_AUDIO_EN_SEL_PENDING        (*(volatile uint8_t  *)0x40399)
#define REG_AUDIO_EN_SEL_TS             ((uint8_t *)0x403C8)
#define REG_AUDIO_MUTE_EVENT_PENDING    (*(volatile uint8_t  *)0x4039F)
#define REG_AUDIO_MUTE_POLL_SHADOW      (*(volatile uint16_t *)0x403BC)
#define REG_AUDIO_MUTE_ACTIVE           (*(volatile uint8_t  *)0x403A9)
#define REG_AUDIO_MUTE_FREQ_IDX         (*(volatile uint8_t  *)0x403AC)
#define REG_AUDIO_MUTE_FORMAT           (*(volatile uint8_t  *)0x4038F)
#define REG_AUDIO_FORMAT_SHADOW         (*(volatile uint8_t  *)0x401E0)
#define REG_AUDIO_OUTPUT_MODE           (*(volatile uint8_t  *)0x403A2)
#define REG_AUDIO_ROUTE_MODE            (*(volatile uint8_t  *)0x40391)
#define REG_AUDIO_SAMPLE_WIDTH          (*(volatile uint8_t  *)0x40396)
#define REG_AUDIO_NON_PCM_FLAG          (*(volatile uint8_t  *)0x40397)
#define REG_AUDIO_HW_CFG_DIRTY          (*(volatile uint8_t  *)0x40398)
#define REG_AUDIO_IDLE_COUNTDOWN        (*(volatile uint8_t  *)0x40393)
#define REG_AUDIO_IDLE2_ACTIVE          (*(volatile uint8_t  *)0x40392)
#define REG_AUDIO_MUTE_TABLE_A          ((volatile uint16_t *)0x403D8)
#define REG_AUDIO_MUTE_TABLE_B          ((volatile uint16_t *)0x403F6)

#define REG_AUDIO_MODE_FLAGS            (*(volatile uint32_t *)0x40100384)
#define REG_AUDIO_DPCD_CAP              (*(volatile uint8_t  *)0x401004DF)
#define REG_AUDIO_IRQ_EVENT_LOG         (*(volatile uint8_t  *)0x40100EAC)
#define REG_AUDIO_DEBUG_FLAGS           (*(volatile uint8_t  *)0x40100ECC)
#define REG_AUDIO_MUTE_CTRL             (*(volatile uint8_t  *)0x4010161B)
#define REG_AUDIO_MUTE_SETTLE           (*(volatile uint8_t  *)0x40100E10)
#define REG_AUDIO_MUTE_GAIN_A           (*(volatile uint8_t  *)0x40100E54)
#define REG_AUDIO_MUTE_GAIN_B           (*(volatile uint8_t  *)0x40100E5D)
#define REG_AUDIO_MUTE_EVENT_FLAG       (*(volatile uint8_t  *)0x4038B)
#define REG_AUDIO_H23_MANUAL_DATA       (*(volatile uint8_t  *)0x40101994)
#define REG_AUDIO_H23_MANUAL_ADDR       (*(volatile uint8_t  *)0x40101995)
#define REG_AUDIO_H23_MANUAL_CTRL       (*(volatile uint8_t  *)0x40101996)
#define REG_AUDIO_EARC_IRQ_MASK         (*(volatile uint32_t *)0x40101294)
#define REG_AUDIO_SCDC_IRQ_MASK         (*(volatile uint32_t *)0x4010129C)
#define REG_AUDIO_HW_EVENT_MASK         (*(volatile uint32_t *)0x40101290)
#define REG_AUDIO_HW_EVENT_MASK_B       (*(volatile uint32_t *)0x401012AC)
#define REG_AUDIO_I2C_INT_CTRL          (*(volatile uint32_t *)0x40101264)
#define REG_AUDIO_I2C_STATUS            (*(volatile uint32_t *)0x40103740)
#define REG_AUDIO_DDC_FLAGS             (*(volatile uint32_t *)0x40103778)
#define REG_AUDIO_DDC_RESET             (*(volatile uint32_t *)0x40103764)
#define REG_AUDIO_DDC_PENDING           (*(volatile uint8_t  *)0x40444)
#define REG_AUDIO_EXT_CTRL              (*(volatile uint32_t *)0x40100450)
#define REG_AUDIO_APHY_CTRL0            (*(volatile uint8_t  *)0x4010098F)
#define REG_AUDIO_APHY_CTRL1            (*(volatile uint8_t  *)0x40100990)
#define REG_AUDIO_APHY_STATUS           (*(volatile uint32_t *)0x401009B8)
#define REG_AUDIO_APHY_CTRL2            (*(volatile uint32_t *)0x401009B0)
#define REG_AUDIO_SPDIF_CTRL            (*(volatile uint8_t  *)0x40100330)
#define REG_AUDIO_STATUS_LAT            (*(volatile uint8_t  *)0x40438)
#define REG_AUDIO_IRQ_ACK_A             (*(volatile uint8_t  *)0x40101268)
#define REG_AUDIO_IRQ_ACK_B             (*(volatile uint8_t  *)0x401012A8)
#define REG_AUDIO_LINK_FLAGS            (*(volatile uint8_t  *)0x40101C13)
#define REG_AUDIO_LINK_FLAGS_A          (*(volatile uint8_t  *)0x40101C14)
#define REG_AUDIO_LINK_FLAGS_B          (*(volatile uint8_t  *)0x40101C15)
#define REG_AUDIO_LINK_STATUS           (*(volatile uint8_t  *)0x40101C3B)
#define REG_AUDIO_LINK_MODE             (*(volatile uint8_t  *)0x40101C3A)
#define REG_AUDIO_H23_PARAM0            (*(volatile uint8_t  *)0x40101C03)
#define REG_AUDIO_H23_PARAM1            (*(volatile uint8_t  *)0x40101C04)
#define REG_AUDIO_H23_READY             (*(volatile uint8_t  *)0x40101C00)
#define REG_AUDIO_H23_DATA0             (*(volatile uint8_t  *)0x40101CA0)
#define REG_AUDIO_H23_DATA1             (*(volatile uint8_t  *)0x40101CA1)
#define REG_AUDIO_H23_DATA2             (*(volatile uint8_t  *)0x40101CA2)
#define REG_AUDIO_STATUS_REG            (*(volatile uint32_t *)0x40101A80)
#define REG_AUDIO_STATUS_REG_B          (*(volatile uint32_t *)0x40101A84)
#define REG_AUDIO_STATUS_REG_C          (*(volatile uint32_t *)0x40101AE0)
#define REG_AUDIO_STATUS_REG_D          (*(volatile uint32_t *)0x40101AE4)
#define REG_AUDIO_H14_FLAGS             (*(volatile uint8_t  *)0x40100456)
#define REG_AUDIO_FIFO_INDEX            (*(volatile uint32_t *)0x40107800)
#define REG_AUDIO_CS_SAMPLE             (*(volatile uint32_t *)0x40101C25)
#define REG_AUDIO_PORTCFG_LOW           (*(volatile uint8_t  *)0x40100705)
#define REG_AUDIO_PORTCFG_MID           (*(volatile uint8_t  *)0x40100711)
#define REG_AUDIO_PORTCFG_HIGH          (*(volatile uint8_t  *)0x40100718)
#define REG_AUDIO_PORTCFG_FMT           (*(volatile uint8_t  *)0x40100726)
#define REG_AUDIO_PORTCFG_CH            (*(volatile uint8_t  *)0x40100777)
#define REG_AUDIO_PORTCFG_ERR           (*(volatile uint8_t  *)0x40100778)
#define REG_AUDIO_HW_NOTIFY             (*(volatile uint16_t *)0x40100E14)
#define REG_AUDIO_HPD_NOTIFY            (*(volatile uint8_t  *)0x401004DC)
#define REG_AUDIO_READBUF_SEL           (*(volatile uint8_t  *)0x4010198A)
#define REG_AUDIO_READBUF_DATA          (*(volatile uint8_t  *)0x4010197E)
#define REG_AUDIO_H23_RD_CMD            (*(volatile uint8_t  *)0x40101940)
#define REG_AUDIO_H23_RD_CFG            (*(volatile uint8_t  *)0x40101941)
#define REG_AUDIO_H23_RD_LATCH          (*(volatile uint8_t  *)0x40101947)
#define REG_AUDIO_H23_RD_MODE           (*(volatile uint8_t  *)0x40101948)
#define REG_AUDIO_H23_RD_FLAGS          (*(volatile uint8_t  *)0x40101949)
#define REG_AUDIO_H23_RD_CTRL           (*(volatile uint8_t  *)0x4010194E)
#define REG_AUDIO_H23_PKT0              (*(volatile uint8_t  *)0x4010196C)
#define REG_AUDIO_H23_PKT1              (*(volatile uint8_t  *)0x40101973)
#define REG_AUDIO_H23_PKT2              (*(volatile uint8_t  *)0x40101974)
#define REG_AUDIO_H23_PKT3              (*(volatile uint8_t  *)0x40101975)
#define REG_AUDIO_H23_PKT4              (*(volatile uint8_t  *)0x40101976)
#define REG_AUDIO_H23_PKT5              (*(volatile uint8_t  *)0x40101977)
#define REG_AUDIO_H23_FIFO_ARM          (*(volatile uint8_t  *)0x40101984)
#define REG_AUDIO_H23_FIFO_IDX          (*(volatile uint8_t  *)0x40101985)
#define REG_AUDIO_H23_FIFO_B0           (*(volatile uint8_t  *)0x40101986)
#define REG_AUDIO_H23_FIFO_B1           (*(volatile uint8_t  *)0x40101987)
#define REG_AUDIO_H23_FIFO_B2           (*(volatile uint8_t  *)0x40101988)
#define REG_AUDIO_H23_FIFO_B3           (*(volatile uint8_t  *)0x40101989)
#define REG_AUDIO_H23_CLK0              (*(volatile uint16_t *)0x401019A0)
#define REG_AUDIO_H23_CLK1              (*(volatile uint16_t *)0x401019A2)
#define REG_AUDIO_H23_HW_FLAG           (*(volatile uint8_t  *)0x40108234)
#define REG_AUDIO_H23_HW_FLAG_B         (*(volatile uint8_t  *)0x40108404)
#define REG_AUDIO_H23_MISC0             (*(volatile uint8_t  *)0x40100E18)
#define REG_AUDIO_H23_MISC1             (*(volatile uint8_t  *)0x40100E1E)
#define REG_AUDIO_H14_STATUS0           (*(volatile uint16_t *)0x40100388)
#define REG_AUDIO_H14_STATUS1           (*(volatile uint16_t *)0x4010038A)
#define REG_AUDIO_H14_STATUS2           (*(volatile uint16_t *)0x4010038C)
#define REG_AUDIO_H14_RATE_STATE        (*(volatile uint16_t *)0x40101458)
#define REG_AUDIO_H14_DIVISOR           (*(volatile uint16_t *)0x4010140C)
#define REG_AUDIO_H14_SAMPLE_RAW        (*(volatile uint16_t *)0x40101408)
#define REG_AUDIO_H14_SAMPLE_RATE       (*(volatile uint16_t *)0x40101404)
#define REG_AUDIO_PLL_REFCLK            (*(volatile uint32_t *)0x410B0)
#define REG_AUDIO_TMDS_CLOCK            (*(volatile uint16_t *)0x4109C)
#define REG_AUDIO_PIXEL_CLOCK           (*(volatile uint16_t *)0x410A0)
#define REG_AUDIO_TIMING0               (*(volatile uint16_t *)0x410A2)
#define REG_AUDIO_TIMING1               (*(volatile uint16_t *)0x410A4)
#define REG_AUDIO_TIMING2               (*(volatile uint16_t *)0x410A6)
#define REG_AUDIO_TIMING3               (*(volatile uint16_t *)0x410A8)
#define REG_AUDIO_TIMING4               (*(volatile uint16_t *)0x410AA)
#define REG_AUDIO_SAMPLE_RATE_SHADOW    (*(volatile uint16_t *)0x410C6)
#define REG_AUDIO_CLK_RAW               (*(volatile uint16_t *)0x410C8)
#define REG_AUDIO_CLK_DIVISOR           (*(volatile uint16_t *)0x410CA)
#define REG_AUDIO_OVERSAMPLE_MODE       (*(volatile uint16_t *)0x410CC)
#define REG_AUDIO_ROUTE_SEL             (*(volatile uint8_t  *)0x41081)
#define REG_EARC_STATE                  (*(volatile uint8_t  *)0x40458)
#define REG_EARC_CAPABILITY_PENDING     (*(volatile uint8_t  *)0x4045A)
#define REG_EARC_STATUS_LATCH           (*(volatile uint8_t  *)0x4045B)
#define REG_EARC_ENABLE_FLAG            (*(volatile uint8_t  *)0x4045C)
#define REG_FRL_MODE_INDEX              (*(volatile uint8_t  *)0x4045D)
#define REG_DSC_MODE_INDEX              (*(volatile uint8_t  *)0x4045E)
#define REG_AUDIO_EARC_CAPABLE          REG_EARC_CAPABILITY_PENDING
#define REG_AUDIO_EARC_READY            REG_EARC_STATUS_LATCH
#define REG_AUDIO_EARC_ENABLE           REG_EARC_ENABLE_FLAG
#define REG_AUDIO_EARC_LANE_MASK        REG_FRL_MODE_INDEX
#define REG_AUDIO_EARC_RATE             REG_DSC_MODE_INDEX
#define REG_AUDIO_CFG3_PENDING          (*(volatile uint8_t  *)0x4045F)
#define REG_AUDIO_CFG4_PENDING          (*(volatile uint8_t  *)0x40460)
#define REG_AUDIO_FSM_STATE_FRL         REG_EARC_STATE
#define REG_AUDIO_FRL_RETRY_FLAG        (*(volatile uint8_t  *)0x40468)
#define REG_AUDIO_EARC_CB               (*(volatile uint32_t *)0x4046C)
#define REG_AUDIO_FSM_PENDING           (*(volatile uint8_t  *)0x4051A)
#define REG_AUDIO_FSM_PENDING_B         (*(volatile uint8_t  *)0x4051B)
#define REG_AUDIO_EE_PENDING            (*(volatile uint8_t  *)0x4051C)
#define REG_AUDIO_H14_RETRY_TIMER       (*(volatile uint8_t  *)0x4051E)
#define REG_AUDIO_H14_EE_TIMER          (*(volatile uint8_t  *)0x4051F)
#define REG_AUDIO_DEBOUNCE_TIMER        (*(volatile uint8_t  *)0x40520)
#define REG_AUDIO_PROTOCOL_MODE         (*(volatile uint8_t  *)0x40521)
#define REG_AUDIO_PROTOCOL_STATE        (*(volatile uint8_t  *)0x40522)
#define REG_AUDIO_PROTOCOL_TIMER        (*(volatile uint8_t  *)0x40523)
#define REG_AUDIO_H14_RESTART_TIMER     (*(volatile uint8_t  *)0x40528)
#define REG_AUDIO_H23_PATH_OK           (*(volatile uint8_t  *)0x4052C)
#define REG_AUDIO_CAP_FLAGS             (*(volatile uint16_t *)0x4052D)
#define REG_AUDIO_H23_CFG_STATE         (*(volatile uint8_t  *)0x4052E)
#define REG_AUDIO_H23_STATUS_CACHE      (*(volatile uint8_t  *)0x4052F)
#define REG_AUDIO_H23_STATUS_SHADOW     (*(volatile uint8_t  *)0x40530)
#define REG_AUDIO_H23_PENDING_ENABLE    (*(volatile uint8_t  *)0x40531)
#define REG_AUDIO_H23_BF_IA             (*(volatile uint8_t  *)0x40532)
#define REG_AUDIO_CFG2_PENDING          (*(volatile uint8_t  *)0x40533)
#define REG_AUDIO_CFG2_RETRIES          (*(volatile uint8_t  *)0x40534)
#define REG_AUDIO_DISCOVERY_FLAGS       (*(volatile uint16_t *)0x40536)
#define REG_AUDIO_H23_STATE             (*(volatile uint8_t  *)0x40549)
#define REG_AUDIO_H23_SUBSTATE          (*(volatile uint8_t  *)0x4054A)
#define REG_AUDIO_H23_WORK_PENDING      (*(volatile uint8_t  *)0x4054B)
#define REG_AUDIO_H23_UNUSED            (*(volatile uint8_t  *)0x4054C)
#define REG_AUDIO_H23_TIMEOUT           (*(volatile uint8_t  *)0x4054D)
#define REG_AUDIO_H23_REPLAY_PENDING    (*(volatile uint8_t  *)0x4054E)
#define REG_AUDIO_H23_RDVAL0            (*(volatile uint8_t  *)0x4054F)
#define REG_AUDIO_H23_RDVAL1            (*(volatile uint8_t  *)0x40550)
#define REG_AUDIO_H23_FIFO_IMAGE        ((volatile uint8_t  *)0x40551)
#define REG_AUDIO_STATE_CODE            (*(volatile uint8_t  *)0x406D1)
#define REG_AUDIO_CALC_THRESHOLD        (*(volatile uint16_t *)0x40272)
#define REG_AUDIO_PROTOCOL_CB_ENABLE    (*(volatile uint8_t  *)0x41BE2)
#define REG_AUDIO_PROTOCOL_TIMEOUT0     (*(volatile uint16_t *)0x41BDC)
#define REG_AUDIO_PROTOCOL_TIMEOUT1     (*(volatile uint8_t  *)0x41BDA)
#define REG_AUDIO_PROTOCOL_TIMEOUT2     (*(volatile uint8_t  *)0x41BDB)
#define REG_AUDIO_DDC_RATE_MASK         (*(volatile uint32_t *)0x41BF7)
#define REG_AUDIO_DDC_RATE_FAILED       (*(volatile uint32_t *)0x41BF8)
#define REG_AUDIO_DDC_RATE_LATCH        (*(volatile uint8_t  *)0x41BF9)
#define REG_AUDIO_FRL_TRAIN_MODE        (*(volatile uint8_t  *)0x41BFA)
#define REG_AUDIO_EARC_USER_CB          (*(volatile uint32_t *)0x41BFC)
#define REG_AUDIO_EARC_CALLBACK_BUSY    (*(volatile uint8_t  *)0x41C04)
#define REG_AUDIO_EARC_CALLBACK         (*(volatile uint32_t *)0x41C00)
#define REG_AUDIO_FRL_RATE              (*(volatile uint8_t  *)0x41BF5)
#define REG_AUDIO_ROUTE_CHAIN           ((volatile uint32_t *)0x41B70)
#define REG_AUDIO_ROUTE_TRIGGER         (*(volatile uint8_t  *)0x40102208)
#define REG_AUDIO_FIFO_DEBUG            (*(volatile uint8_t  *)0x40100EEA)
#define REG_AUDIO_CALC_READ_PENDING     (*(volatile uint8_t  *)0x404A5)
#define REG_AUDIO_CALC_FLAGS            (*(volatile uint8_t  *)0x404A4)
#define REG_AUDIO_CALC_RESULT           (*(volatile uint32_t *)0x404A8)
#define REG_AUDIO_INFO_READ_REQ         ((uint8_t **)0x404AC)
#define REG_AUDIO_H23_ROUTE_MASK        (*(volatile uint8_t  *)0x4054E)
#define REG_AUDIO_STATE96_TRIGGER       (*(volatile uint8_t  *)0x40101D9E)
#define REG_AUDIO_H14_TIMEOUT_SCALE     (*(volatile uint8_t  *)0x401014C6)
#define REG_AUDIO_H14_STATE_20          20u
#define REG_AUDIO_H23_STATE_34          34u

/* ========================================================================= */
/* Helpers                                                                    */
/* ========================================================================= */

#define AUDIO_TASK_GUARD() uint32_t saved_task = REG_TCB_CURRENT_TASK
#define AUDIO_TASK_CHECK() do { if (REG_TCB_CURRENT_TASK != saved_task) system_halt_clear_flag(); } while (0)
#define AUDIO_TASK_RETURN(value) do { AUDIO_TASK_CHECK(); return (value); } while (0)

static float bits_to_float(uint32_t bits)
{
    union { uint32_t u; float f; } cvt = { bits };
    return cvt.f;
}

static uint32_t float_to_bits(float value)
{
    union { uint32_t u; float f; } cvt;
    cvt.f = value;
    return cvt.u;
}

static double bits_to_double(uint64_t bits)
{
    union { uint64_t u; double d; } cvt = { bits };
    return cvt.d;
}

static uint64_t double_to_bits(double value)
{
    union { uint64_t u; double d; } cvt;
    cvt.d = value;
    return cvt.u;
}

static uint32_t abs_u32_diff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static uint32_t read_u24(volatile uint8_t *hi, volatile uint8_t *mid, volatile uint8_t *lo)
{
    return ((uint32_t)(*hi) << 16) | ((uint32_t)(*mid) << 8) | (uint32_t)(*lo);
}

static void write_debug_string_if_enabled(const char *msg)
{
    if ((REG_AUDIO_DEBUG_FLAGS & 2u) != 0u)
        custom_printf(msg);
}

static uint32_t round_float_to_u32(float value)
{
    if (value <= 0.0f)
        return 0u;
    return (uint32_t)(value + 0.5f);
}

static int highest_set_bit_or_ff(uint32_t mask)
{
    if (mask == 0u)
        return 255;
    return 31 - __clz(mask);
}

static float safe_divf(float num, float den)
{
    if (den == 0.0f)
        return 0.0f;
    return num / den;
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

static uint32_t float64_integral_bits_to_u32(uint64_t bits)
{
    double value = bits_to_double(bits);

    if (value <= 0.0)
        return 0u;
    if (value >= 4294967295.0)
        return 0xFFFFFFFFu;
    return (uint32_t)value;
}

static int audio_process_state_21_log_string(void)
{
    return (int)REG_TCB_CURRENT_TASK;
}

static int audio_process_state_39_log_string(int a1, int a2)
{
    (void)a1;
    (void)a2;
    return (int)REG_TCB_CURRENT_TASK;
}

static int audio_process_state_53_log_string(int a1, int a2, int a3, char a4)
{
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    return (int)REG_TCB_CURRENT_TASK;
}

/* ========================================================================= */
/* Audio parameter helpers                                                    */
/* ========================================================================= */

int audio_calculate_param_1(uint8_t *data)
{
    AUDIO_TASK_GUARD();
    uint32_t sample = data[0];
    double accum = (double)(sample + 2040u);
    double scale = (double)(sample + 32u);
    float result = (float)((accum / (scale > 0.0 ? scale : 1.0)) * 0.934999943f);
    ((uint32_t *)data)[1] = float_to_bits(result);
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_calculate_param_2(int count, unsigned int coeff, int unused_a3,
                             unsigned int limit, unsigned int divisor,
                             int unused_a6, unsigned int scale_bits, int ref_delta)
{
    (void)unused_a3;
    (void)unused_a6;
    AUDIO_TASK_GUARD();

    float alpha = 1.0f - bits_to_float(coeff);
    float frac = safe_divf((float)(uint8_t)count, bits_to_float(divisor));
    float gain = safe_divf(frac, alpha);

    double pll_term = (double)(REG_AUDIO_PLL_REFCLK + (uint32_t)ref_delta);
    double tmds = (double)REG_AUDIO_TMDS_CLOCK;
    double scale = (double)bits_to_float(scale_bits) * 1.020624999;
    float mixed = (float)((tmds + scale) * pll_term);

    float pixel = (float)REG_AUDIO_PIXEL_CLOCK;
    float link = (float)REG_AUDIO_TMDS_CLOCK;
    float base = safe_divf(pixel * link, bits_to_float(limit));
    float delta = safe_divf((float)((int)link - (int)pixel) * link, bits_to_float(limit));

    float ref = (float)REG_AUDIO_PLL_REFCLK * 0.375f;
    float window = ref * gain;
    float prod = (float)REG_AUDIO_PLL_REFCLK * mixed;
    bool use_prod = prod >= window;

    float compare_a = use_prod ? ((float)REG_AUDIO_PLL_REFCLK * mixed) : window;
    float residual = bits_to_float(limit) - compare_a;
    float err = 0.0f;

    if (!use_prod)
        err = compare_a - base;
    (void)delta;
    if (!use_prod)
        err = residual - delta;

    BOOL ok = safe_divf(err, mixed) <= bits_to_float(0x43C80000u);
    AUDIO_TASK_RETURN(ok);
}

unsigned int audio_calculate_param_3(void)
{
    AUDIO_TASK_GUARD();

    uint16_t sample_rate = REG_AUDIO_H14_SAMPLE_RATE & 0x03FFu;
    uint16_t oversample = REG_AUDIO_H14_RATE_STATE & 0x3u;
    float fs = (float)((sample_rate >> 4) + ((sample_rate & 0xFu) / 16.0f));
    if (oversample != 0u)
        fs *= 0.5f;
    if (fs == 0.0f)
        fs = 8.0f;

    uint16_t divisor = REG_AUDIO_H14_DIVISOR;
    if (divisor == 0u)
        divisor = 7680u;

    REG_AUDIO_SAMPLE_RATE_SHADOW = sample_rate;
    REG_AUDIO_OVERSAMPLE_MODE = oversample;
    REG_AUDIO_CLK_RAW = (uint16_t)((REG_AUDIO_H14_SAMPLE_RAW << 8) | (REG_AUDIO_H14_SAMPLE_RAW >> 8));
    REG_AUDIO_CLK_DIVISOR = divisor;

    uint8_t ratio = (uint8_t)(REG_AUDIO_H14_SAMPLE_RAW / divisor);
    float scaled = (((float)divisor / fs) + 7.0f) / 8.0f;
    uint16_t threshold = (uint16_t)(round_float_to_u32(scaled) * ratio);
    REG_AUDIO_CALC_THRESHOLD = threshold;

    AUDIO_TASK_RETURN((threshold + 2u) / 3u);
}

int audio_calculate_param_4(int ctx)
{
    AUDIO_TASK_GUARD();
    uint8_t flags = REG_AUDIO_H14_FLAGS;
    float divisor = (float)(flags & 7u);
    if (divisor == 0.0f)
        divisor = 0.25f;

    int base_fs = (flags & 8u) ? 44100 : 48000;
    float ref = (float)(video_math_pow2(flags >> 4) * base_fs);
    float ratio = safe_divf(ref, divisor);
    float factor = safe_divf(ratio, bits_to_float(*(uint32_t *)(ctx + 12)));
    (void)factor;

    float scaled = ratio * 1000000.0f;
    uint64_t normalized = hw_misc_process_state_82(double_to_bits((double)scaled));
    *(uint16_t *)(ctx + 22) = (uint16_t)(float64_integral_bits_to_u32(normalized) + 512u);
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_calculate_param_5(int a1, int a2, int a3, int a4, int a5, int a6,
                             int a7, uint16_t *out)
{
    (void)a1; (void)a2; (void)a3; (void)a4; (void)a5; (void)a7;
    AUDIO_TASK_GUARD();

    float pixel = (float)REG_AUDIO_PIXEL_CLOCK;
    float delta = ((float)REG_AUDIO_TMDS_CLOCK - pixel) * pixel;
    float ratio = safe_divf((float)REG_AUDIO_PLL_REFCLK, delta);
    uint16_t measured = (uint16_t)float64_integral_bits_to_u32(
        hw_misc_process_state_82(double_to_bits((double)ratio)));
    uint16_t min_value = (uint16_t)(a6 >> 16);
    if (measured < min_value)
        measured = min_value;

    float centered = (float)(uint16_t)a6 - ((float)REG_AUDIO_PLL_REFCLK / 6.0f);
    float candidate = (float)measured;
    if (candidate < centered)
        candidate = centered;

    float result = candidate * 0.25f;
    uint64_t rounded = audio_float_math_1(double_to_bits((double)result));
    uint16_t final = (uint16_t)float64_integral_bits_to_u32(rounded);
    *out = final;
    AUDIO_TASK_RETURN(min_value <= final);
}

int audio_calculate_param_6(unsigned int value_bits, uint8_t *ctx, int a3)
{
    (void)a3;
    AUDIO_TASK_GUARD();

    /* Approximate the original float64 widener + 1.020625 scale sequence. */
    float scale = bits_to_float(value_bits) * 1.020625f;
    float product = (float)REG_AUDIO_TMDS_CLOCK * scale;
    ((uint32_t *)ctx)[3] = float_to_bits(product);

    float ref = ((float)(1000u * ctx[1])) / 0.9997f;
    float step = ref * 18.0f;
    ((uint32_t *)ctx)[4] = float_to_bits(step);

    float divisor = (float)ctx[0];
    float ratio = safe_divf(safe_divf(product, step), divisor);
    uint64_t rounded = audio_float_math_1(double_to_bits((double)ratio));
    ((uint16_t *)ctx)[4] = (uint16_t)float64_integral_bits_to_u32(rounded);
    AUDIO_TASK_RETURN((int)saved_task);
}

/* ========================================================================= */
/* Info-frame / IRQ / mute handling                                           */
/* ========================================================================= */

int audio_process_info_frame(void)
{
    AUDIO_TASK_GUARD();

    uint32_t n_value;
    uint32_t cts_value;

    __disable_irq();
    REG_AUDIO_INFO_CLR = (uint8_t)((REG_AUDIO_INFO_CLR & 0x0Fu) - 64u);
    n_value = read_u24(&REG_AUDIO_N_HIGH, &REG_AUDIO_N_MID, &REG_AUDIO_N_LOW);
    cts_value = read_u24(&REG_AUDIO_CTS_HIGH, &REG_AUDIO_CTS_MID, &REG_AUDIO_CTS_LOW);
    __enable_irq();

    if (abs_u32_diff(n_value, REG_AUDIO_CTS_STABLE) > 10u) {
        REG_AUDIO_FIFO_LIMIT &= ~2u;
        REG_AUDIO_FIFO_LIMIT |= 2u;
    }
    REG_AUDIO_CTS_STABLE = (uint16_t)n_value;

    __disable_irq();
    REG_AUDIO_INFO_INDEX = 105;
    uint8_t info_105 = REG_AUDIO_INFO_DATA;
    __enable_irq();

    if ((info_105 & 7u) <= 1u) {
        REG_AUDIO_PORTCFG_CH &= (uint8_t)~0x20u;
        if ((REG_AUDIO_8CH_FLAG & 0x80u) != 0u)
            REG_AUDIO_8CH_FLAG = 0;
    } else {
        REG_AUDIO_PORTCFG_CH |= 0x20u;
        if ((REG_AUDIO_8CH_FLAG & 0x80u) == 0u)
            REG_AUDIO_8CH_FLAG = 1;
    }

    REG_AUDIO_CODE_TYPE = info_105 >> 4;

    __disable_irq();
    REG_AUDIO_INFO_INDEX = 104;
    REG_AUDIO_LAYOUT_3D = (REG_AUDIO_INFO_DATA & 0x20u) != 0u;
    REG_AUDIO_3D_CHANNEL_COUNT = REG_AUDIO_INFO_DATA & 0x0Fu;
    __enable_irq();

    bool compressed = ((REG_AUDIO_CS_CFG >> 2) & 1u) != 0u;

    uint32_t link_khz;
    switch (*(volatile uint8_t *)0x40100320) {
    case 0x1E: link_khz = 810000u; break;
    case 0x14: link_khz = 540000u; break;
    case 0x0A: link_khz = 270000u; break;
    default:   link_khz = 162000u; break;
    }

    if (cts_value != 0u)
        REG_AUDIO_SOURCE_FS = ((link_khz >> 9) * n_value) / cts_value;
    else
        REG_AUDIO_SOURCE_FS = 0u;

    custom_printf("audio 3d_channel_count = %x\n", REG_AUDIO_3D_CHANNEL_COUNT);
    custom_printf("audio source_fs = %x\n", (unsigned)REG_AUDIO_SOURCE_FS);
    custom_printf("audio aud_is_8ch = %x\n", REG_AUDIO_8CH_FLAG & 1u);

    uint16_t fifo_scale = 127u;
    uint32_t fifo_base = 2u * cts_value;

    switch (REG_AUDIO_CODE_TYPE) {
    case 0:
        custom_printf("audio aud_code_type : LPCM\n");
        break;
    case 1:
        fifo_base = 8u * cts_value;
        REG_AUDIO_SOURCE_FS >>= 2;
        custom_printf("audio aud_code_type : HBR\n");
        break;
    case 2:
        fifo_base = 2u * cts_value;
        if (compressed) {
            fifo_scale = 127u;
        } else if (REG_AUDIO_3D_CHANNEL_COUNT > 0x10u) {
            fifo_scale = 31u;
            REG_AUDIO_SOURCE_FS *= 4u;
        } else {
            fifo_scale = 63u;
            REG_AUDIO_SOURCE_FS *= 2u;
        }
        custom_printf("audio aud_code_type : 3D LPCM\n");
        break;
    case 3:
        fifo_scale = 55u;
        fifo_base = cts_value << 7;
        custom_printf("audio aud_code_type : One Bit\n");
        break;
    case 4:
        fifo_scale = 447u;
        fifo_base = cts_value << 7;
        custom_printf("audio aud_code_type : DST\n");
        break;
    default:
        break;
    }

    REG_AUDIO_FIFO_LIMIT &= 0xFFF0000Fu;
    REG_AUDIO_FIFO_LIMIT |= ((uint32_t)(fifo_scale & 0x3FFu) << 8);

    float period = (float)(REG_AUDIO_FIFO_PERIOD & 0x07FFFFFFu);
    float scaled = (float)(8u * fifo_base);
    float high = (scaled * period) / 131072.0f;
    float low = 6000.0f;

    if (high > 258000.0f) {
        high = 258000.0f;
        low = (period * scaled) / 16777216.0f;
    }

    REG_AUDIO_FIFO_LOW = round_float_to_u32(low) & 0x07FFFFFFu;
    REG_AUDIO_FIFO_HIGH = round_float_to_u32(high) & 0x07FFFFFFu;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_earc_irq_handler(uint8_t *packet)
{
    AUDIO_TASK_GUARD();

    uint8_t code = packet[0];
    uint8_t a = packet[1];
    uint8_t b = packet[2];

    switch (code) {
    case 116:
        if ((a & 2u) != 0u) custom_printf("EARC_CAP_CHNG_CONF\n");
        if ((a & 1u) != 0u) custom_printf("EARC_STAT_CHNG_CONF\n");
        if ((b & 1u) != 0u) custom_printf("EARC_LATREQ\n");
        if ((b & 4u) != 0u) custom_printf("EARC_HEART_BEAT_LOST\n");
        __disable_irq();
        REG_AUDIO_EARC_IRQ_MASK |= 0x000C0500u;
        __enable_irq();
        break;
    case 103:
        if (a == 0) {
            custom_printf("MON_STB_AST\n");
        } else {
            switch (a) {
            case 1: custom_printf("MON_STB_DAST\n"); break;
            case 2: custom_printf("EC_FSM_CHG %02x\n", b); break;
            case 3:
                custom_printf("FMT_STB \n");
                video_process_state_78();
                video_print_cs_regs();
                break;
            case 4: custom_printf("EC_HPD_CHG %02x\n", b); break;
            default: break;
            }
        }
        break;
    case 102:
        custom_printf("I2S_IRQ %02x\n", a);
        break;
    case 106: {
        __disable_irq();
        uint32_t cfg = *(volatile uint8_t *)0x403C2;
        __enable_irq();
        uint8_t old_mode = REG_AUDIO_ROUTE_MODE;
        __disable_irq();
        audio_update_hw_config(cfg);
        __enable_irq();
        if (old_mode != REG_AUDIO_ROUTE_MODE) {
            __disable_irq();
            video_process_state_33();
            __enable_irq();
        }
        custom_printf("DPCD_TDM_CFG chg %02x\n", (unsigned)cfg);
        break;
    }
    case 131:
        REG_AUDIO_MUTE_EVENT_PENDING = 1;
        custom_printf("ESYS_EARC_MUTE_CTRL_CHG %02x\n", a);
        break;
    case 132:
        REG_AUDIO_MUTE_EVENT_PENDING = 1;
        custom_printf("ESYS_STATUS_SAMPLE_WIDTH_CHG_EVT\n");
        break;
    case 107:
        custom_printf("I2S_CH_STA_STB %02x %02x\n", a, b);
        video_print_cs_regs();
        break;
    case 108:
        custom_printf("I2S_CH_STA_UNSTB\n");
        break;
    case 109:
        custom_printf("I2S_CH_STA_B4_CHG %02x -> %02x\n", b, a);
        video_process_state_78();
        video_print_cs_regs();
        break;
    case 127:
        if (b == 0)
            custom_printf("I2S_CH_STA_B0_CHG %02x\n", a);
        else if (b == 5)
            custom_printf("I2S_CH_STA_B5_CHG %02x\n", a);
        else
            custom_printf("I2S_CH_STA_B3_CHG %02x\n", a);
        break;
    case 118: custom_printf("COMP_2IDLE2 %02x\n", a); break;
    case 117: custom_printf("PRE_2IDLE2 %02x\n", a); break;
    case 119: custom_printf("ARC_2IDLE2\n"); break;
    case 120: custom_printf(a ? "ECC 0->1\n" : "ECC 1->0\n"); break;
    case 121:
        custom_printf("EARC BP ERR\n");
        if (audio_get_format() == 4) {
            __disable_irq();
            REG_AUDIO_EARC_IRQ_MASK |= 0x1000u;
            __enable_irq();
        }
        break;
    case 122:
        custom_printf("ARC BP ERR\n");
        if (audio_get_format() == 5) {
            __disable_irq();
            REG_AUDIO_EARC_IRQ_MASK |= 0x800u;
            __enable_irq();
        }
        break;
    case 123: custom_printf("EARC_DPCD: %02x\n", a); break;
    case 128: custom_printf("EARC_2T8_WA_CHANGE: %x\n", a); break;
    case 129: custom_printf("MUTE_INT_EVT\n"); break;
    case 130:
        audio_mute_poll((int)(uint16_t)(a | ((uint16_t)b << 8)));
        custom_printf("MUTE IRQ %02x %02x\n", a, b);
        break;
    case 133:
        custom_printf("FW MUTE Rise %02x mute event %04x\n",
                      ((a | ((uint16_t)b << 8)) >> 11) & 1,
                      a | ((uint16_t)b << 8));
        break;
    case 134: custom_printf("Reload CSB %02x %02x\n", a, b); break;
    case 135: custom_printf("mute_event %04x\n", a | ((uint16_t)b << 8)); break;
    case 139: custom_printf("ARC RX not idle %02x\n", a); break;
    default:
        break;
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_get_format(void)
{
    return REG_AUDIO_FORMAT & 7u;
}

int audio_timer_update(int a1, int a2, int a3)
{
    (void)a1;
    AUDIO_TASK_GUARD();

    if (REG_AUDIO_FREQREADY_TIMER != 0u) {
        uint32_t elapsed = video_timer_update_122(REG_AUDIO_FREQREADY_TS, a2, a3);
        if (REG_AUDIO_FREQREADY_TIMER <= elapsed)
            REG_AUDIO_FREQREADY_TIMER = 0;
        else
            REG_AUDIO_FREQREADY_TIMER -= elapsed;

        if (REG_AUDIO_FREQREADY_TIMER == 0u) {
            __disable_irq();
            *(volatile uint8_t *)0x40100701 &= 0x0Fu;
            __enable_irq();
            delay_loop(2);
            __disable_irq();
            *(volatile uint8_t *)0x40100701 |= 0xF0u;
            REG_AUDIO_FREQREADY_TOGGLE++;
            if (REG_AUDIO_FREQREADY_TOGGLE == 1u)
                REG_AUDIO_FREQREADY_TOGGLE = 0;
            __enable_irq();
            custom_printf("Toggle AudioFreqReady\n");
        }
    }

    if (REG_AUDIO_EN_SEL_TIMER != 0u) {
        uint32_t elapsed = video_timer_update_122(REG_AUDIO_EN_SEL_TS, a2, a3);
        if (REG_AUDIO_EN_SEL_TIMER <= elapsed)
            REG_AUDIO_EN_SEL_TIMER = 0;
        else
            REG_AUDIO_EN_SEL_TIMER -= elapsed;

        if (REG_AUDIO_EN_SEL_TIMER == 0u) {
            __disable_irq();
            video_set_hw_state_29();
            __enable_irq();
            custom_printf("AUD_EN_SEL_S Rel\n");
            REG_AUDIO_FIFO_DEBUG++;
        }
    }

    if (REG_AUDIO_EN_SEL_PENDING != 0u)
        REG_AUDIO_EN_SEL_PENDING = 0;

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_mute_poll(int event)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_MUTE_POLL_SHADOW != (uint16_t)event) {
        if (((REG_AUDIO_MUTE_POLL_SHADOW ^ (uint16_t)event) & 0x8000u) != 0u)
            REG_AUDIO_MUTE_EVENT_PENDING = 1;
        custom_printf("MUTE_POLL %04x\n", event);
        REG_AUDIO_MUTE_POLL_SHADOW = (uint16_t)event;
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_get_freq_constant(char value);

int audio_handle_mute_event(int muted)
{
    AUDIO_TASK_GUARD();
    uint8_t prev = REG_AUDIO_MUTE_ACTIVE;

    if (muted != 0) {
        *(volatile uint8_t *)0x40100ED8 = 0xFFu;
        *(volatile uint8_t *)0x40100ED9 = 0xFFu;

        int freq_constant = audio_get_freq_constant((char)REG_AUDIO_MUTE_FORMAT);
        if (freq_constant == 0xFFFF)
            AUDIO_TASK_RETURN((int)saved_task);

        *(volatile uint16_t *)0x40100ED8 = (uint16_t)freq_constant;
        *(volatile uint8_t *)0x40100EDA = 0xFFu;

        uint8_t idx;
        for (idx = 0; idx < 12u; ++idx) {
            if (REG_AUDIO_MUTE_TABLE_B[idx] == (uint16_t)freq_constant)
                break;
        }

        *(volatile uint8_t *)0x40100EDA = idx;
        *(volatile uint8_t *)0x40100EDB = 0xFFu;
        if (idx == 12u)
            AUDIO_TASK_RETURN((int)saved_task);

        REG_AUDIO_MUTE_ACTIVE = 1;
        *(volatile uint8_t *)0x40100EDB = idx;
        *(volatile uint8_t *)0x40101625 &= (uint8_t)~0x40u;
        video_set_hw_state_34(0, (char)idx);
        if (REG_AUDIO_MUTE_FREQ_IDX != idx)
            REG_AUDIO_FREQREADY_TOGGLE = 0;
        REG_AUDIO_MUTE_FREQ_IDX = idx;
    } else {
        REG_AUDIO_MUTE_ACTIVE = 0;
        REG_AUDIO_FREQREADY_TOGGLE = 0xFFu;
        REG_AUDIO_MUTE_FREQ_IDX = 0xFFu;
        *(volatile uint8_t *)0x40101625 |= 0x40u;
        video_set_hw_state_34(1, 0);
    }

    if (prev != REG_AUDIO_MUTE_ACTIVE) {
        video_process_state_33();
        if (prev != 0 && REG_AUDIO_MUTE_ACTIVE == 0)
            video_enqueue_event(128, 0, 0);
        else
            video_enqueue_event(128, 1, 0);
        REG_AUDIO_EN_SEL_TIMER = 2;
        video_get_state_125(263112, 1);
        video_set_hw_state_29();
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_check_format_change(void)
{
    AUDIO_TASK_GUARD();
    uint8_t format = REG_AUDIO_DPCD_CAP;
    if (REG_AUDIO_FORMAT_SHADOW != format) {
        audio_set_format((char)format);
        REG_AUDIO_FORMAT_SHADOW = format;
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_format(char format)
{
    AUDIO_TASK_GUARD();
    uint8_t prev = REG_AUDIO_OUTPUT_MODE;
    REG_AUDIO_OUTPUT_MODE = format & 0x0Fu;
    if (prev != (uint8_t)(format & 0x0Fu)) {
        __disable_irq();
        hw_misc_process_state_152();
        __enable_irq();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_extract_bit_4(unsigned int value) { return (value >> 4) & 1u; }
int audio_extract_bits_5_6(unsigned int value) { return (value >> 5) & 3u; }
int audio_extract_bits_0_2(char value) { return value & 7; }
int audio_extract_bits_complex(int16_t value) { return ((value & 8) == 0) | ((uint16_t)(value & 0x6000u) >> 12); }
int audio_extract_bits_8_9(unsigned int value) { return (value >> 8) & 3u; }
int audio_extract_bits_10_12(unsigned int value)
{
    uint8_t bits = (value >> 10) & 7u;
    if (bits == 0u)
        return 1;
    if ((unsigned)(bits - 1u) > 5u)
        return 0;
    return bits + 2;
}
int audio_get_highest_bit(unsigned int mask) { return highest_set_bit_or_ff(mask); }

int audio_update_hw_config(unsigned int cfg)
{
    AUDIO_TASK_GUARD();

    uint8_t route_mode = (uint8_t)audio_extract_bits_8_9(cfg);
    uint8_t sample_width = (uint8_t)audio_extract_bits_10_12(cfg);
    bool dirty = false;

    if (route_mode != REG_AUDIO_ROUTE_MODE) {
        REG_AUDIO_ROUTE_MODE = route_mode;
        if (route_mode == 1u)
            REG_AUDIO_HW_NOTIFY |= 0x1000u;
        else
            REG_AUDIO_HW_NOTIFY &= (uint16_t)~0x1000u;
    }

    if (REG_AUDIO_SAMPLE_WIDTH != sample_width) {
        REG_AUDIO_SAMPLE_WIDTH = sample_width;
        dirty = true;
        if (audio_get_format() == 4) {
            REG_AUDIO_HW_CFG_DIRTY = 2;
            video_get_state_125(263112, 1);
            video_set_hw_state_29();
        }
    }

    if (((cfg >> 7) & 1u) != REG_AUDIO_NON_PCM_FLAG) {
        REG_AUDIO_NON_PCM_FLAG = (cfg & 0x80u) != 0u;
        dirty = true;
    }

    if (dirty)
        hw_misc_process_state_154();

    if ((cfg & 0x60u) != 0u) {
        REG_AUDIO_PORTCFG_ERR |= 2u;
    } else {
        REG_AUDIO_PORTCFG_CH &= 0xF8u;
        REG_AUDIO_PORTCFG_CH |= (uint8_t)audio_extract_bits_0_2((char)cfg);
        REG_AUDIO_PORTCFG_FMT &= 0xE0u;
        REG_AUDIO_PORTCFG_FMT |= (uint8_t)audio_extract_bit_4(cfg);
        REG_AUDIO_PORTCFG_MID = (uint8_t)audio_extract_bits_5_6(cfg);
        REG_AUDIO_PORTCFG_LOW &= 0x8Cu;
        REG_AUDIO_PORTCFG_LOW |= (uint8_t)(route_mode | (sample_width << 4));
        REG_AUDIO_PORTCFG_HIGH &= 0x0Fu;
        REG_AUDIO_PORTCFG_HIGH |= (uint8_t)(audio_extract_bits_complex((int16_t)cfg) << 4);
        REG_AUDIO_PORTCFG_ERR &= (uint8_t)~2u;
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_idle_state_transition(void)
{
    AUDIO_TASK_GUARD();
    bool changed = false;
    int format = audio_get_format();

    __disable_irq();
    if (REG_AUDIO_IDLE_COUNTDOWN != 0u) {
        REG_AUDIO_IDLE_COUNTDOWN--;
        if (REG_AUDIO_IDLE_COUNTDOWN == 0u && format == 1 && REG_AUDIO_IDLE2_ACTIVE == 0u) {
            REG_AUDIO_IDLE2_ACTIVE = 1;
            video_process_state_77();
            changed = true;
            REG_AUDIO_HPD_NOTIFY |= 1u;
            REG_AUDIO_HPD_NOTIFY |= 2u;
        }
    }
    __enable_irq();

    if (changed)
        custom_printf("IDLE3->IDLE2\n");

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_is_feature_enabled(void) { return (REG_AUDIO_MODE_FLAGS >> 9) & 1u; }

int audio_read_config_126(void)
{
    AUDIO_TASK_GUARD();
    int result = video_read_hw_buffer(126);
    AUDIO_TASK_RETURN(result);
}

BOOL audio_check_config_0_1(void)
{
    AUDIO_TASK_GUARD();
    BOOL ok = (video_read_hw_buffer(0) == 0) && (video_read_hw_buffer(1) == 255);
    AUDIO_TASK_RETURN(ok);
}

int audio_set_hw_state_1(int enable)
{
    __disable_irq();
    REG_AUDIO_MUTE_CTRL = enable ? 0x80u : 0x81u;
    REG_AUDIO_MUTE_SETTLE = 10;
    REG_AUDIO_MUTE_GAIN_A = 0xC1u;
    REG_AUDIO_MUTE_GAIN_B = 0xA0u;
    REG_AUDIO_MUTE_EVENT_FLAG = 1;
    __enable_irq();
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_hw_state_2(char value)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_H23_MANUAL_CTRL &= (uint8_t)~4u;
    REG_AUDIO_H23_MANUAL_ADDR = 0;
    REG_AUDIO_H23_MANUAL_DATA = (uint8_t)value;
    REG_AUDIO_H23_MANUAL_CTRL &= (uint8_t)~2u;
    REG_AUDIO_H23_MANUAL_CTRL |= 2u;
    delay_loop(10);
    REG_AUDIO_H23_MANUAL_CTRL |= 4u;
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_config_a1(int value)
{
    AUDIO_TASK_GUARD();
    BOOL ok = video_read_hw_buffer(value << 7) == 2;
    AUDIO_TASK_RETURN(ok);
}

uint64_t audio_float_math_1(uint64_t bits)
{
    return float64_trunc_to_integral_bits(bits);
}

int audio_execute_callback(int which)
{
    AUDIO_TASK_GUARD();
    void (*cb)(void) = (void (*)(void))REG_AUDIO_EARC_CALLBACK;

    if (which == 1) {
        if (cb != NULL)
            cb();
        scdc_update_config_5((char)REG_AUDIO_FRL_RATE);
    } else if (which == 2 && REG_AUDIO_EARC_CALLBACK_BUSY == 0u && cb != NULL) {
        cb();
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_event_97(void)
{
    AUDIO_TASK_GUARD();
    video_enqueue_event(97, 0, 0);
    scdc_execute_callback_or_default();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_event_98(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_IRQ_EVENT_LOG = 1;
    video_enqueue_event(98, 0, 0);
    scdc_execute_callback_or_default();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_event_97_alt(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_SCDC_IRQ_MASK |= 1u;
    video_enqueue_event(97, 0, 0);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_event_98_alt(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_SCDC_IRQ_MASK |= 1u;
    video_enqueue_event(98, 0, 0);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_3(int pattern)
{
    AUDIO_TASK_GUARD();
    bool state = false;

    REG_AUDIO_ROUTE_CHAIN[0] = 0x40102018u;
    for (uint8_t i = 0; i < 8u; ++i) {
        bool bit = ((pattern >> i) & 1) != 0;
        REG_AUDIO_ROUTE_CHAIN[i + 1u] = (bit ^ state) ? 0x40102018u : 0x40102008u;
        state = bit;
    }
    REG_AUDIO_ROUTE_CHAIN[9] = state ? 0x40102008u : 0x40102018u;

    REG_AUDIO_ROUTE_TRIGGER = 0xAAu;
    __disable_irq();
    for (uint8_t i = 0; i < 10u; ++i)
        **(volatile uint32_t **)&REG_AUDIO_ROUTE_CHAIN[i] = 0xAAu;
    __enable_irq();

    AUDIO_TASK_RETURN(pattern);
}

int audio_reset_state(void)
{
    AUDIO_TASK_GUARD();
    scdc_enable_feature_1();
    video_process_state_170();
    REG_AUDIO_ROUTE_SEL = 4;
    hw_misc_set_state_48();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_callback(uint16_t *req)
{
    AUDIO_TASK_GUARD();
    if (req != NULL) {
        uint8_t *data = ((uint8_t **)req)[1];
        if (*req == 1 && data != NULL && REG_AUDIO_EARC_CAPABLE != 0u) {
            if ((data[0] & 0x40u) != 0u) {
                REG_AUDIO_EARC_ENABLE = 1;
                REG_AUDIO_EARC_CAPABLE = 0;
                if (REG_AUDIO_EARC_CB != 0u) {
                    ((void (*)(void))REG_AUDIO_EARC_CB)();
                    REG_AUDIO_EARC_CB = 0;
                }
            } else {
                ddc_write_request(0xA8, 64, (uint16_t *)0x40470);
            }
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_callback_alt(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_EARC_CAPABLE != 0u)
        ddc_write_request(0xA8, 64, (uint16_t *)0x40470);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_event_99(int failed, int unused_ptr, int trained_mask)
{
    (void)unused_ptr;
    AUDIO_TASK_GUARD();

    bool changed = false;
    uint8_t rate_param = (uint8_t)trained_mask;

    if (REG_AUDIO_FSM_STATE_FRL != 10)
        video_enqueue_event(99, REG_AUDIO_FSM_STATE_FRL, 0);

    if (REG_AUDIO_FRL_TRAIN_MODE == 1u) {
        changed = true;
        if (failed != 0)
            REG_AUDIO_DDC_RATE_FAILED &= ~(1u << (REG_AUDIO_FRL_RATE - 1u));
        else
            REG_AUDIO_DDC_RATE_FAILED |= 1u << (REG_AUDIO_FRL_RATE - 1u);
    } else if (failed == 255) {
        changed = true;
    }

    if (changed) {
        if (REG_AUDIO_FRL_TRAIN_MODE != 0u) {
            if (REG_AUDIO_FRL_TRAIN_MODE == 1u) {
                if (REG_AUDIO_DDC_RATE_LATCH == 0xFFu) {
                    video_process_state_146();
                    int best = audio_get_highest_bit(REG_AUDIO_DDC_RATE_MASK);
                    if (best == 255) {
                        int fallback = audio_get_highest_bit(REG_AUDIO_DDC_RATE_FAILED);
                        REG_AUDIO_DDC_RATE_LATCH = (uint8_t)fallback;
                        if (fallback != 255) {
                            uint8_t next = (uint8_t)(fallback + 1);
                            if (REG_AUDIO_FRL_RATE != next) {
                                REG_AUDIO_FRL_RATE = next;
                                REG_AUDIO_FSM_STATE_FRL = 2;
                            }
                        }
                    } else {
                        REG_AUDIO_FRL_RATE = (uint8_t)(best + 1);
                        REG_AUDIO_FSM_STATE_FRL = 2;
                    }
                } else {
                    REG_AUDIO_DDC_RATE_LATCH = 0xFFu;
                }
            }
        } else if (--REG_AUDIO_FRL_RATE != 0u) {
            REG_AUDIO_FSM_STATE_FRL = 2;
        }
    }

    if (REG_AUDIO_FSM_STATE_FRL == 2) {
        REG_AUDIO_FRL_RETRY_FLAG = 0;
    } else {
        if (REG_AUDIO_FSM_STATE_FRL == 10 && failed == 0) {
            REG_AUDIO_EARC_READY = 1;
            REG_AUDIO_EARC_LANE_MASK = (uint8_t)REG_AUDIO_DDC_RATE_FAILED;
            REG_AUDIO_EARC_RATE = REG_AUDIO_FRL_RATE;
            uint8_t lane_cfg = 0;
            frl_get_rate_params(&lane_cfg, &rate_param);
            scdc_calculate_and_set_clock(lane_cfg);
        } else {
            REG_AUDIO_EARC_READY = 0;
            REG_AUDIO_EARC_LANE_MASK = 0;
            REG_AUDIO_EARC_RATE = 0;
            video_process_state_120();
        }

        if (REG_AUDIO_EARC_USER_CB != 0u) {
            REG_AUDIO_EARC_CALLBACK_BUSY = 0;
            ((void (*)(void))REG_AUDIO_EARC_USER_CB)();
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_4(void)
{
    *(volatile uint8_t  *)0x40100238 = 4;
    *(volatile uint8_t  *)0x4010023A = 6;
    *(volatile uint8_t  *)0x401002DC = 0x9Fu;
    *(volatile uint8_t  *)0x401002DD = 93;
    *(volatile uint8_t  *)0x401002DE = 93;
    *(volatile uint8_t  *)0x401002F2 = 0xC0u;
    *(volatile uint8_t  *)0x401002F8 = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_process_pending_tasks(void)
{
    AUDIO_TASK_GUARD();
    if (!audio_process_state_104() && !scdc_process_pending_reads() && !audio_process_state_99())
        audio_process_state_95();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_float_math_2(int ctx)
{
    AUDIO_TASK_GUARD();
    float divisor = (float)*(uint16_t *)(ctx + 8);
    float value = 1.0f - bits_to_float(*(uint32_t *)(ctx + 4));
    float result = safe_divf(value, divisor);
    uint64_t rounded = audio_float_math_1(double_to_bits((double)result));
    *(uint16_t *)(ctx + 20) = (uint16_t)float64_integral_bits_to_u32(rounded);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_get_hw_state_5(void) { return REG_AUDIO_CS_SAMPLE; }

int audio_get_freq_constant(char value)
{
    int masked = value & 0xCF;
    switch (masked) {
    case 0x8B: return 1280;
    case 0x8D: return 7040;
    case 0xC5: return 10240;
    case 0xCB: return 5120;
    case 0x45: return 15360;
    case 0x4B: return 2560;
    case 0x4D: return 14080;
    default: {
        unsigned idx = value & 0x0Fu;
        return (idx < 0x0Fu) ? REG_AUDIO_MUTE_TABLE_A[idx] : 0xFFFF;
    }
    }
}

int audio_get_freq_constant_2(unsigned int idx)
{
    return (idx < 12u) ? REG_AUDIO_MUTE_TABLE_B[idx] : 0xFFFF;
}

int audio_get_bit_depth(int value)
{
    if (value == 0) return 16;
    if (value == 1) return 24;
    return 32;
}

int audio_get_hw_state_6(void)
{
    return *(volatile uint32_t *)(0x26DFCu + 4u * ((REG_AUDIO_FIFO_INDEX >> 2) & 3u));
}

int audio_get_hw_state_7(void)
{
    switch ((HIBYTE(REG_AUDIO_MODE_FLAGS) & 3u)) {
    case 3: return 4;
    case 2: return 3;
    case 1: return 2;
    default: return 0;
    }
}

int audio_check_state_array(unsigned int state, uint8_t *array)
{
    bool upper = (array[0] != 0u) || (array[2] != 0u);
    bool lower = (array[1] != 0u) || (array[3] != 0u);
    if (state != 3u && upper)
        return 3;
    if (state <= 1u && lower)
        return 2;
    if (state == 0u && array[4] != 0u)
        return 1;
    return (int)state;
}

int audio_update_hw_states(void)
{
    AUDIO_TASK_GUARD();

    uint16_t h14_clk = (uint16_t)audio_get_hw_state_12();
    if (h14_clk != 0u) {
        REG_AUDIO_TMDS_CLOCK = h14_clk;
        *(volatile uint16_t *)0x4109E = (uint16_t)audio_get_hw_state_11();
        REG_AUDIO_TIMING3 = (uint16_t)(audio_get_hw_state_10() & 0x7FFF);
    } else {
        REG_AUDIO_TMDS_CLOCK = (uint16_t)video_get_hw_state_47();
        *(volatile uint16_t *)0x4109E = (uint16_t)video_get_hw_state_46();
        REG_AUDIO_TIMING3 = (uint16_t)(video_get_hw_state_45() & 0x7FFF);
    }

    REG_AUDIO_PIXEL_CLOCK = (uint16_t)video_get_hw_state_48();
    REG_AUDIO_TIMING0 = (uint16_t)video_get_hw_state_55();
    REG_AUDIO_TIMING1 = (uint16_t)video_get_hw_state_54();
    REG_AUDIO_TIMING2 = (uint16_t)video_get_hw_state_50();
    REG_AUDIO_TIMING4 = (uint16_t)(video_get_hw_state_53() & 0x7FFF);

    if ((int)REG_AUDIO_TIMING0 - (int)REG_AUDIO_TIMING2 - (int)REG_AUDIO_TIMING1 >= 2)
        REG_AUDIO_H14_TIMEOUT_SCALE = 80;
    else
        REG_AUDIO_H14_TIMEOUT_SCALE = 96;

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_get_state_8(void) { return *(volatile uint8_t *)0x40434; }
int audio_set_hw_state_9(void) { *(volatile uint32_t *)0x40100440 |= 0x4000u; return (int)REG_TCB_CURRENT_TASK; }
int audio_get_hw_state_10(void) { return REG_AUDIO_H14_STATUS2; }
int audio_get_hw_state_11(void) { return REG_AUDIO_H14_STATUS1; }
int audio_get_hw_state_12(void) { return REG_AUDIO_H14_STATUS0; }
int audio_get_hw_state_13(void) { return (*(volatile uint32_t *)0x40101AF4 >> 8) & 7u; }
int audio_get_hw_state_14(void) { return (REG_AUDIO_STATUS_REG >> 29) & 1u; }
int audio_get_hw_state_15(void) { return *(volatile uint8_t *)0x40101AF4 & 0x7Fu; }

/* ========================================================================= */
/* H14 / H23 state helpers                                                    */
/* ========================================================================= */

int audio_process_state_16(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_26();
    REG_AUDIO_FSM_PENDING = 1;
    REG_AUDIO_CAP_FLAGS = 0;
    REG_AUDIO_DISCOVERY_FLAGS = 0;
    REG_AUDIO_EXT_CTRL = REG_AUDIO_EXT_CTRL;
    REG_AUDIO_EE_PENDING = 0;
    REG_AUDIO_STATE96_TRIGGER |= 1u;
    audio_set_state_96(0);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_17(void) { REG_AUDIO_APHY_CTRL1 |= 0x300u; return (int)REG_TCB_CURRENT_TASK; }

int audio_process_state_18(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_FSM_PENDING == 1u || REG_AUDIO_FSM_PENDING == 2u)
        audio_set_hw_state_27();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_19(void)
{
    AUDIO_TASK_GUARD();
    bool invalid = false;
    REG_AUDIO_APHY_CTRL0 = 0;
    REG_AUDIO_APHY_CTRL1 = 0;

    uint32_t channels = (uint32_t)audio_get_state_89() + 1u;
    uint32_t lanes = (uint32_t)audio_get_state_81() + 1u;

    if (channels > 0x1Fu) {
        REG_AUDIO_APHY_CTRL0 |= 0x80u;
        invalid = true;
    }
    if (lanes > 7u) {
        REG_AUDIO_APHY_CTRL1 |= 8u;
        invalid = true;
    }

    if (invalid) {
        audio_set_hw_state_27();
        REG_AUDIO_FSM_PENDING = 0;
    } else {
        REG_AUDIO_APHY_CTRL0 |= (uint8_t)channels;
        REG_AUDIO_APHY_CTRL1 |= (uint8_t)lanes;
        audio_set_hw_state_24();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_20(void)
{
    AUDIO_TASK_GUARD();
    if (audio_get_state_92() == 20) {
        if ((REG_AUDIO_APHY_STATUS & 2u) != 0u) {
            audio_set_hw_state_30();
            REG_AUDIO_HW_EVENT_MASK |= 0x02000000u;
            if (REG_AUDIO_FSM_PENDING_B != 0u) {
                audio_set_hw_state_27();
                REG_AUDIO_FSM_PENDING_B = 0;
            }
            REG_AUDIO_STATUS_LAT = 0;
        } else {
            audio_set_hw_state_29();
            REG_AUDIO_STATUS_LAT = 3;
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_h14r_commands(uint8_t *msg)
{
    AUDIO_TASK_GUARD();

    if (msg[0] == 11 && msg[1] == 20) {
        uint8_t flags = msg[2];
        if ((flags & 0x80u) != 0u) {
            audio_set_hw_state_24();
        } else {
            uint32_t discovery = (uint32_t)REG_AUDIO_DISCOVERY_FLAGS << 16;
            if ((flags & 0x40u) != 0u) {
                REG_AUDIO_CAP_FLAGS |= 0x0Au;
                REG_AUDIO_EXT_CTRL |= discovery | ((uint32_t)REG_AUDIO_CAP_FLAGS << 8);
                if ((REG_AUDIO_APHY_STATUS & 2u) != 0u)
                    audio_set_hw_state_27();
                else
                    REG_AUDIO_FSM_PENDING_B = 1;
            } else if ((flags & 0x10u) != 0u) {
                audio_process_state_19();
            } else if ((flags & 0x20u) != 0u) {
                REG_AUDIO_CAP_FLAGS |= 5u;
                REG_AUDIO_EXT_CTRL |= discovery | ((uint32_t)REG_AUDIO_CAP_FLAGS << 8);
                audio_process_state_18();
            }
        }
    }

    if ((REG_AUDIO_DEBUG_FLAGS & 2u) != 0u) {
        switch (msg[0]) {
        case '>': custom_printf("h14r a d\n"); break;
        case '?': custom_printf((REG_AUDIO_APHY_STATUS & 2u) ? "h14r e en\n" : "h14r e dis\n"); break;
        case '@': custom_printf("h14r event\n"); break;
        default: break;
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_21(void) { return audio_process_state_21_log_string(); }

int audio_set_hw_state_22(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_17();
    REG_AUDIO_APHY_CTRL2 |= 0x61u;
    REG_AUDIO_APHY_CTRL2 &= ~0x61u;
    REG_AUDIO_APHY_CTRL2 &= ~0x10u;
    REG_AUDIO_SPDIF_CTRL |= 0x40u;
    REG_AUDIO_FSM_PENDING = 0;
    REG_AUDIO_HW_EVENT_MASK |= 0x04000000u;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_23(void)
{
    AUDIO_TASK_GUARD();
    if ((REG_AUDIO_APHY_STATUS & 4u) != 0u)
        audio_set_hw_state_27();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_24(void)
{
    REG_AUDIO_APHY_CTRL2 |= 2u;
    REG_AUDIO_APHY_CTRL2 &= ~2u;
    REG_AUDIO_HW_EVENT_MASK |= 0x01000000u;
    REG_AUDIO_FSM_PENDING = 2;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_hw_state_25(void)
{
    REG_AUDIO_FSM_PENDING_B = 0;
    REG_AUDIO_HW_EVENT_MASK &= ~0x02000000u;
    REG_AUDIO_FSM_PENDING = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_process_state_58(void);

int audio_set_hw_state_26(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_25();
    audio_process_state_58();
    REG_AUDIO_LINK_FLAGS |= 0x10u;
    REG_AUDIO_LINK_FLAGS &= (uint8_t)~8u;
    REG_AUDIO_LINK_STATUS |= 2u;
    REG_AUDIO_STATE_CODE = 20;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_27(void)
{
    REG_AUDIO_APHY_CTRL2 &= ~0x80000000u;
    REG_AUDIO_APHY_CTRL2 |= 0x80000000u;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_process_state_28(void)
{
    AUDIO_TASK_GUARD();
    int state = ((*(volatile uint32_t *)0x40101000 & 2u) != 0u)
              ? ((int)((*(volatile uint32_t *)0x40101000 << 22) >> 31) + 1)
              : 255;
    if (video_check_state_40(state)) {
        REG_AUDIO_H14_EE_TIMER = 0;
        REG_AUDIO_PROTOCOL_STATE = 0;
        audio_process_state_37();
    } else {
        REG_AUDIO_H14_RETRY_TIMER = 2;
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_29(void)
{
    REG_AUDIO_STATUS_REG_C &= ~0x40000000u;
    REG_AUDIO_IRQ_ACK_B &= (uint8_t)~0x10u;
    REG_AUDIO_IRQ_ACK_A |= 0x10u;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_hw_state_30(void)
{
    *(volatile uint8_t *)0x40101040 &= (uint8_t)~0x20u;
    REG_AUDIO_STATUS_REG_C |= 0x40000000u;
    REG_AUDIO_IRQ_ACK_A |= 0x10u;
    REG_AUDIO_IRQ_ACK_B |= 0x10u;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_handle_h14t_ee(void)
{
    AUDIO_TASK_GUARD();
    write_debug_string_if_enabled("h14t ee\n");

    if (audio_check_state_84()) {
        video_enqueue_event(11, audio_get_state_92(), 128);
        AUDIO_TASK_RETURN((int)saved_task);
    }

    if ((REG_AUDIO_I2C_INT_CTRL & 4u) != 0u || REG_AUDIO_DDC_PENDING != 0u) {
        REG_AUDIO_I2C_INT_CTRL = 4;
        __asm__ volatile("dsb" ::: "memory");
        REG_AUDIO_I2C_STATUS |= 1u;
        __disable_irq();
        ddc_transfer_abort_callback();
        __enable_irq();
        REG_AUDIO_DDC_PENDING = 0;
        write_debug_string_if_enabled("h14t mic h\n");
    }

    REG_AUDIO_STATUS_REG_D |= 0x400u;
    __asm__ volatile("dsb" ::: "memory");
    REG_AUDIO_DDC_RESET |= 0x400u;
    delay_loop(10);
    REG_AUDIO_STATUS_REG_D &= ~0x400u;
    __asm__ volatile("dsb" ::: "memory");
    REG_AUDIO_DDC_RESET &= ~0x400u;
    REG_AUDIO_H14_EE_TIMER = 40;
    REG_AUDIO_H23_RD_CMD &= (uint8_t)~0x80u;
    REG_AUDIO_STATUS_REG &= 0xFFFF3FFFu;
    REG_AUDIO_IRQ_ACK_A = 32;
    audio_process_state_28();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_31(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_38();
    audio_set_hw_state_29();
    delay_loop(20);
    REG_AUDIO_STATUS_REG |= 0xC000u;
    *(volatile uint8_t *)0x40101040 |= 0x20u;
    REG_AUDIO_STATUS_LAT = 0;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_h14t_commands(uint8_t *msg)
{
    AUDIO_TASK_GUARD();
    if (msg[0] == 40 && REG_AUDIO_PROTOCOL_STATE == 5u)
        video_state_machine_2();

    if ((REG_AUDIO_DEBUG_FLAGS & 2u) != 0u) {
        if (msg[0] == 72)
            custom_printf("h14t l i f\n");
        else if (msg[0] == 73)
            custom_printf("h14t au f\n");
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_32(void) { return ((REG_AUDIO_STATUS_REG >> 17) & 3u) == 3u; }

int audio_process_state_33(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_STATE != 0u && REG_AUDIO_PROTOCOL_STATE != 5u)
        video_state_machine_2();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_34(void)
{
    REG_AUDIO_STATUS_REG_C |= 0x28000000u;
    REG_AUDIO_STATUS_REG_D |= 0x1E9u;
    REG_AUDIO_STATUS_REG |= 6u;
    REG_AUDIO_PROTOCOL_MODE = 0xFFu;
    REG_AUDIO_DEBOUNCE_TIMER = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

BOOL audio_check_state_35(void) { return REG_AUDIO_PROTOCOL_STATE != 0u; }
int audio_check_state_36(void) { return (REG_AUDIO_STATUS_REG_B >> 14) & 1u; }

int audio_process_state_37(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_STATE == 0u)
        video_state_machine_2();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_38(void)
{
    REG_AUDIO_IRQ_ACK_B &= 0xCFu;
    REG_AUDIO_H14_RETRY_TIMER = 0;
    REG_AUDIO_PROTOCOL_STATE = 0;
    REG_AUDIO_DEBOUNCE_TIMER = 0;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_timer_update_2(void)
{
    AUDIO_TASK_GUARD();

    if (REG_AUDIO_H14_RETRY_TIMER != 0u && --REG_AUDIO_H14_RETRY_TIMER == 0u)
        audio_process_state_28();
    if (REG_AUDIO_H14_EE_TIMER != 0u && --REG_AUDIO_H14_EE_TIMER == 0u)
        scdc_handle_event_11_32();
    if (REG_AUDIO_H14_RESTART_TIMER != 0u && --REG_AUDIO_H14_RESTART_TIMER == 0u)
        video_state_machine_2();

    if (REG_AUDIO_DEBOUNCE_TIMER != 0u) {
        REG_AUDIO_DEBOUNCE_TIMER--;
        if (REG_AUDIO_DEBOUNCE_TIMER == 0u) {
            if ((REG_AUDIO_APHY_STATUS & 2u) != 0u &&
                (((REG_AUDIO_I2C_INT_CTRL & 4u) != 0u) || REG_AUDIO_DDC_PENDING != 0u)) {
                REG_AUDIO_I2C_INT_CTRL = 4;
                audio_set_hw_state_29();
                __disable_irq();
                video_enqueue_event(11, audio_get_state_92(), 64);
                __enable_irq();
                __disable_irq();
                video_enqueue_event(72, 0, 0);
                __enable_irq();
                write_debug_string_if_enabled("h14 timeout\n");
            } else {
                REG_AUDIO_DEBOUNCE_TIMER = 20;
            }
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_39(int a1, int a2) { return audio_process_state_39_log_string(a1, a2); }

BOOL audio_check_state_40(void)
{
    AUDIO_TASK_GUARD();
    uint32_t cfg = *(volatile uint32_t *)0x40040;
    BOOL ok = ((((cfg >> 22) & 3u) == 1u) || ((cfg & 0x00C00000u) == 0u))
           && (audio_get_hw_state_5() == 1)
           && ((audio_get_state_91() == 1) || !audio_check_state_74());
    AUDIO_TASK_RETURN(ok);
}

int audio_get_state_41(void)
{
    AUDIO_TASK_GUARD();
    int result = audio_check_state_42() ? (REG_AUDIO_H23_RDVAL0 & 7u) : 0;
    AUDIO_TASK_RETURN(result);
}

int audio_check_state_42(void)
{
    REG_AUDIO_READBUF_SEL = 21;
    return REG_AUDIO_READBUF_DATA & 1u;
}

int audio_get_state_43(void)
{
    AUDIO_TASK_GUARD();
    int result = audio_check_state_42() ? (REG_AUDIO_H23_RDVAL0 >> 3) : 0;
    AUDIO_TASK_RETURN(result);
}

int audio_get_state_44(void)
{
    AUDIO_TASK_GUARD();
    int result = audio_check_state_42() ? ((REG_AUDIO_H23_RDVAL1 >> 3) & 1u) : 0;
    AUDIO_TASK_RETURN(result);
}

int audio_get_state_45(void)
{
    AUDIO_TASK_GUARD();
    int result = audio_check_state_42() ? ((REG_AUDIO_H23_RDVAL1 >> 2) & 1u) : 0;
    AUDIO_TASK_RETURN(result);
}

int audio_set_hw_state_46(void)
{
    AUDIO_TASK_GUARD();
    audio_process_state_59();
    REG_AUDIO_HW_EVENT_MASK |= 0x200u;
    REG_AUDIO_H23_BF_IA = 0;
    REG_AUDIO_H23_PATH_OK = 0;
    REG_AUDIO_H23_CFG_STATE = 1;
    REG_AUDIO_EE_PENDING = 0;
    REG_AUDIO_STATE96_TRIGGER |= 8u;
    REG_AUDIO_STATE96_TRIGGER &= (uint8_t)~1u;
    REG_AUDIO_CAP_FLAGS = 0;
    REG_AUDIO_DISCOVERY_FLAGS = 0;
    REG_AUDIO_EXT_CTRL = REG_AUDIO_EXT_CTRL;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_47(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_65();
    audio_set_hw_state_57();
    REG_AUDIO_HW_EVENT_MASK |= 0x20000u;
    REG_AUDIO_H23_CFG_STATE = 3;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_48(void)
{
    REG_AUDIO_H23_READY = 3;
    REG_AUDIO_LINK_MODE = 3;
    REG_AUDIO_H23_DATA0 = 0;
    REG_AUDIO_H23_DATA1 = 0xFFu;
    REG_AUDIO_H23_DATA2 = 32;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_hw_state_49(void)
{
    REG_AUDIO_HW_EVENT_MASK &= 0xFFFDCFFFu;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_process_state_50(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_CFG_STATE == 2u || REG_AUDIO_H23_CFG_STATE == 3u) {
        REG_AUDIO_H23_PARAM0 &= 0x0Fu;
        REG_AUDIO_H23_PARAM1 &= (uint8_t)~1u;
        REG_AUDIO_H23_PARAM1 &= 0xF1u;
        REG_AUDIO_H23_PARAM0 &= 0xF3u;
        if (hw_misc_check_state_3())
            audio_set_hw_state_63();
        audio_set_hw_state_49();
        REG_AUDIO_H23_CFG_STATE = 0;
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_51(void)
{
    AUDIO_TASK_GUARD();
    bool invalid = false;

    if (REG_AUDIO_H23_CFG_STATE == 2u || REG_AUDIO_H23_CFG_STATE == 3u) {
        REG_AUDIO_H23_PARAM0 &= 0xF3u;
        REG_AUDIO_H23_PARAM0 &= 0x0Fu;
        REG_AUDIO_H23_PARAM1 &= (uint8_t)~1u;
        REG_AUDIO_H23_PARAM1 &= 0xF1u;

        uint32_t channels = (uint32_t)audio_get_state_89() + 1u;
        uint32_t lanes = (uint32_t)audio_get_state_81() + 1u;

        if (channels > 0x1Fu) {
            REG_AUDIO_H23_PARAM0 |= 8u;
            invalid = true;
        }
        if (lanes > 4u) {
            REG_AUDIO_H23_PARAM0 |= 4u;
            invalid = true;
        }

        if (invalid) {
            audio_set_hw_state_63();
            audio_set_hw_state_49();
            REG_AUDIO_H23_CFG_STATE = 0;
        } else {
            REG_AUDIO_H23_PARAM0 |= (uint8_t)(channels << 4);
            REG_AUDIO_H23_PARAM1 |= (uint8_t)((channels & 0x10u) != 0u);
            REG_AUDIO_H23_PARAM1 |= (uint8_t)(lanes << 1);
            video_set_hw_state_75();
            audio_set_hw_state_47();
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_52(void)
{
    AUDIO_TASK_GUARD();
    if (audio_get_state_92() == 34) {
        if (video_get_state_81()) {
            if (hw_misc_process_state_99())
                audio_process_state_83();
            else
                audio_process_state_82();
        }

        if ((REG_AUDIO_APHY_STATUS & 2u) != 0u) {
            REG_AUDIO_HW_EVENT_MASK |= 0x1000u;
            if (REG_AUDIO_H23_PENDING_ENABLE != 0u) {
                if (hw_misc_check_state_3())
                    audio_set_hw_state_63();
                REG_AUDIO_H23_PENDING_ENABLE = 0;
            }
            REG_AUDIO_H23_PATH_OK = audio_check_state_40();
            REG_AUDIO_STATUS_LAT = 0;
        } else {
            if (audio_get_state_91() == 1 && audio_check_state_36())
                audio_set_hw_state_29();
            REG_AUDIO_STATUS_LAT = 3;
            REG_AUDIO_H23_PATH_OK = 0;
        }
        video_enqueue_event(142, 0, 0);
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_h23r_commands(uint8_t *msg)
{
    AUDIO_TASK_GUARD();
    uint8_t code = msg[0];

    if (code == 11) {
        if (msg[1] != 20) {
            if ((msg[2] & 0x88u) != 0u) {
                REG_AUDIO_H23_BF_IA = 1;
                audio_set_hw_state_57();
                write_debug_string_if_enabled("h23r n bf ia\n");
            } else if ((msg[2] & 0x11u) != 0u) {
                if (audio_get_state_85()) {
                    if (REG_AUDIO_H23_REPLAY_PENDING != 0u) {
                        REG_AUDIO_H23_REPLAY_PENDING = 0;
                        audio_process_state_51();
                        write_debug_string_if_enabled("h23r n bf h23t ap rep\n");
                    } else {
                        write_debug_string_if_enabled("h23r nn bf sm\n");
                    }
                } else {
                    audio_process_state_51();
                    write_debug_string_if_enabled("h23r n bf h23t ap sink\n");
                }
            } else {
                uint32_t discovery = (uint32_t)REG_AUDIO_DISCOVERY_FLAGS << 16;
                if ((msg[2] & 0x44u) != 0u) {
                    REG_AUDIO_CAP_FLAGS |= 0x0Au;
                    REG_AUDIO_EXT_CTRL |= discovery | ((uint32_t)REG_AUDIO_CAP_FLAGS << 8);
                    if ((REG_AUDIO_APHY_STATUS & 2u) != 0u)
                        audio_set_hw_state_63();
                    else
                        REG_AUDIO_H23_PENDING_ENABLE = 1;
                    REG_AUDIO_H23_PATH_OK = 0;
                    __disable_irq();
                    video_enqueue_event(142, 0, 0);
                    __enable_irq();
                    write_debug_string_if_enabled("h23r ra bf h23t ra\n");
                } else if ((msg[2] & 0x22u) != 0u) {
                    REG_AUDIO_CAP_FLAGS |= 5u;
                    REG_AUDIO_EXT_CTRL |= discovery | ((uint32_t)REG_AUDIO_CAP_FLAGS << 8);
                    audio_process_state_50();
                    write_debug_string_if_enabled("h23r n bf h23t af\n");
                }
            }
        }
    } else if (code == 65) {
        custom_printf("h start machine\n");
        audio_process_state_90();
    } else if (code == 13 && !hw_misc_check_state_3()) {
        audio_set_hw_state_64();
        audio_set_hw_state_65();
        REG_AUDIO_H23_PENDING_ENABLE = 0;
    }

    if ((REG_AUDIO_DEBUG_FLAGS & 2u) != 0u) {
        switch (code) {
        case '7': custom_printf("h23r a i\n"); break;
        case '8': custom_printf("h23r s e\n"); break;
        case '9': custom_printf("h23r s m\n"); break;
        case ':': custom_printf((REG_AUDIO_APHY_STATUS & 2u) ? "h23r e en\n" : "h23r e dis\n"); break;
        case ';': custom_printf("h23r re a\n"); break;
        case '<': custom_printf("h23r event\n"); break;
        default: break;
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_53(int a1, int a2, int a3, char a4) { return audio_process_state_53_log_string(a1, a2, a3, a4); }

int audio_set_hw_state_54(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_HW_EVENT_MASK &= ~0x1000u;
    REG_AUDIO_H23_CFG_STATE = 0;
    audio_set_hw_state_63();
    if ((REG_AUDIO_APHY_STATUS & 2u) != 0u)
        audio_set_hw_state_62();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_55(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_CFG_STATE == 2u || REG_AUDIO_H23_CFG_STATE == 3u) {
        REG_AUDIO_HW_EVENT_MASK &= ~0x2000u;
        REG_AUDIO_H23_CFG_STATE = 0;
        audio_set_hw_state_63();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_56(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_H23_PATH_OK = 0;
    REG_AUDIO_H23_CFG_STATE = 0;
    audio_set_hw_state_48();
    REG_AUDIO_H23_HW_FLAG |= 1u;
    REG_AUDIO_H23_HW_FLAG_B &= (uint8_t)~0x80u;
    REG_AUDIO_H23_MISC0 = 7;
    REG_AUDIO_LINK_MODE = 15;
    REG_AUDIO_LINK_STATUS = 64;
    *(volatile uint8_t *)0x40101C10 &= (uint8_t)~8u;
    *(volatile uint8_t *)0x40101C10 |= 4u;
    *(volatile uint8_t *)0x40101C10 &= (uint8_t)~1u;
    REG_AUDIO_CAP_FLAGS = 0;
    REG_AUDIO_DISCOVERY_FLAGS = 0;
    REG_AUDIO_EXT_CTRL = REG_AUDIO_EXT_CTRL;
    REG_AUDIO_H23_BF_IA = 0;
    REG_AUDIO_H23_STATUS_CACHE = 0;
    REG_AUDIO_H23_STATUS_SHADOW = 0;
    REG_AUDIO_HW_EVENT_MASK |= 0x30u;
    REG_AUDIO_H23_MISC1 |= 4u;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_57(void) { REG_AUDIO_LINK_FLAGS &= (uint8_t)~0x40u; REG_AUDIO_LINK_FLAGS |= 0x40u; return (int)REG_TCB_CURRENT_TASK; }

int audio_process_state_58(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_64();
    audio_set_hw_state_65();
    REG_AUDIO_H23_PENDING_ENABLE = 0;
    audio_set_hw_state_49();
    REG_AUDIO_H23_CFG_STATE = 0;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_59(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_25();
    audio_process_state_58();
    REG_AUDIO_LINK_FLAGS |= 0x10u;
    REG_AUDIO_LINK_FLAGS |= 8u;
    REG_AUDIO_LINK_STATUS &= (uint8_t)~2u;
    REG_AUDIO_APHY_CTRL2 |= 0x40u;
    REG_AUDIO_APHY_CTRL2 &= (uint32_t)~0x40u;
    REG_AUDIO_H23_CLK0 = 0;
    REG_AUDIO_H23_CLK1 = 32;
    REG_AUDIO_STATE_CODE = 34;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_60(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_CFG_STATE == 1u) {
        REG_AUDIO_H23_CFG_STATE = 2;
        REG_AUDIO_HW_EVENT_MASK &= ~0x200u;
        REG_AUDIO_HW_EVENT_MASK |= 0x2000u;
        REG_AUDIO_CFG2_RETRIES = 3;
        scdc_write_config_2();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_61(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_CFG_STATE == 3u) {
        uint8_t status = (uint8_t)audio_get_hw_state_5();
        REG_AUDIO_H23_STATUS_CACHE = status;
        if (status == REG_AUDIO_H23_STATUS_SHADOW ||
            ((REG_AUDIO_H23_STATUS_SHADOW = status), (REG_AUDIO_APHY_STATUS & 2u) == 0u)) {
            if (audio_get_state_91() != 1 && audio_get_state_85() && REG_AUDIO_H23_BF_IA == 0u)
                video_enqueue_event(61, 0, 0);
            REG_AUDIO_HW_EVENT_MASK |= 0x10u;
        } else {
            REG_AUDIO_H23_CFG_STATE = 0;
            audio_set_hw_state_63();
            audio_set_hw_state_62();
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_62(void) { REG_AUDIO_LINK_FLAGS_A &= (uint8_t)~0x10u; REG_AUDIO_LINK_FLAGS_B &= (uint8_t)~0x10u; REG_AUDIO_LINK_FLAGS_B |= 0x10u; return (int)REG_TCB_CURRENT_TASK; }
int audio_set_hw_state_63(void) { REG_AUDIO_LINK_FLAGS_A &= (uint8_t)~8u; REG_AUDIO_LINK_FLAGS_B &= (uint8_t)~8u; REG_AUDIO_LINK_FLAGS_B |= 8u; return (int)REG_TCB_CURRENT_TASK; }
int audio_set_hw_state_64(void) { REG_AUDIO_LINK_FLAGS_A &= (uint8_t)~0x10u; REG_AUDIO_LINK_FLAGS_B &= (uint8_t)~0x10u; return (int)REG_TCB_CURRENT_TASK; }
int audio_set_hw_state_65(void) { REG_AUDIO_LINK_FLAGS_A &= (uint8_t)~8u; REG_AUDIO_LINK_FLAGS_B &= (uint8_t)~8u; return (int)REG_TCB_CURRENT_TASK; }

int audio_process_state_66(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_PKT2 == 57 && (REG_AUDIO_H23_PKT0 & 3u) == 2u) {
        REG_AUDIO_H23_RD_MODE &= (uint8_t)~0x40u;
        hw_misc_set_state_63();
        REG_AUDIO_HW_EVENT_MASK_B &= 0xFFFFFDFBu;
        REG_AUDIO_H23_TIMEOUT = 0;
        video_enqueue_event(11, 0, 2);
        video_set_state_112();
        REG_AUDIO_H23_SUBSTATE = 7;

        if ((REG_AUDIO_H23_PKT5 & 2u) != 0u) REG_AUDIO_DISCOVERY_FLAGS |= 1u;
        if ((REG_AUDIO_H23_PKT4 & 0xC0u) != 0u) REG_AUDIO_DISCOVERY_FLAGS |= 4u;
        if ((REG_AUDIO_H23_PKT4 & 0x10u) != 0u) REG_AUDIO_DISCOVERY_FLAGS |= 0x10u;
        if ((REG_AUDIO_H23_PKT4 & 8u) != 0u) REG_AUDIO_DISCOVERY_FLAGS |= 0x40u;
        if ((REG_AUDIO_H23_PKT4 & 4u) != 0u) REG_AUDIO_DISCOVERY_FLAGS |= 0x100u;

        REG_AUDIO_H23_RD_LATCH |= 0x80u;
        delay_loop(1);
        REG_AUDIO_H23_RD_LATCH &= (uint8_t)~0x80u;

        switch (REG_AUDIO_H23_PKT3) {
        case 8:    REG_AUDIO_DISCOVERY_FLAGS |= 2u; break;
        case 0x16: REG_AUDIO_DISCOVERY_FLAGS |= 8u; break;
        case 0x21:
        case 0x22: REG_AUDIO_DISCOVERY_FLAGS |= 0x20u; break;
        case 0x30: REG_AUDIO_DISCOVERY_FLAGS |= 0x80u; break;
        case 0x38: REG_AUDIO_DISCOVERY_FLAGS |= 0x200u; break;
        default: break;
        }

        if ((REG_AUDIO_DDC_FLAGS & 0x20u) != 0u) {
            REG_AUDIO_DDC_RESET |= 0x800u;
            REG_AUDIO_CAP_FLAGS |= 0x10u;
        }

        audio_process_state_73();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_67(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_SUBSTATE == 4u && (REG_AUDIO_H23_PKT0 & 2u) != 0u) {
        REG_AUDIO_H23_RD_MODE &= (uint8_t)~0x40u;
        hw_misc_set_state_63();
        REG_AUDIO_HW_EVENT_MASK_B &= ~2u;
        REG_AUDIO_HW_EVENT_MASK_B |= 0x600u;
        REG_AUDIO_H23_SUBSTATE = 6;
        video_set_state_112();
        REG_AUDIO_H23_STATE = 20;
        video_enqueue_event(11, 0, 1);
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_68(void)
{
    if (REG_AUDIO_H23_SUBSTATE == 3u || REG_AUDIO_H23_SUBSTATE == 6u) {
        REG_AUDIO_H23_REPLAY_PENDING = 1;
        REG_AUDIO_HW_EVENT_MASK_B &= ~0x400u;
        REG_AUDIO_H23_WORK_PENDING = 1;
        REG_AUDIO_H23_SUBSTATE = 5;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_hw_state_69(void) { REG_AUDIO_H23_RD_FLAGS &= (uint8_t)~2u; return (int)REG_TCB_CURRENT_TASK; }
int audio_set_hw_state_70(void) { REG_AUDIO_H23_RD_FLAGS |= 2u; return (int)REG_TCB_CURRENT_TASK; }

int audio_process_state_71(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_SUBSTATE == 1u) {
        REG_AUDIO_HW_EVENT_MASK_B &= ~0x10u;
        audio_process_state_73();
        REG_AUDIO_H23_SUBSTATE = 0;
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_72(void)
{
    if (REG_AUDIO_H23_SUBSTATE == 1u) {
        REG_AUDIO_HW_EVENT_MASK_B &= ~0x10u;
        REG_AUDIO_H23_TIMEOUT = 0;
        REG_AUDIO_H23_SUBSTATE = 2;
        REG_AUDIO_HW_EVENT_MASK_B |= 8u;
    }
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_handle_h23t_ee(void)
{
    AUDIO_TASK_GUARD();
    write_debug_string_if_enabled("h23t ee\n");

    if (audio_check_state_84()) {
        write_debug_string_if_enabled("h23t is au\n");
        video_enqueue_event(11, 0, 8);
        AUDIO_TASK_RETURN((int)saved_task);
    }

    REG_AUDIO_H23_RD_CMD |= 0x80u;
    audio_set_hw_state_79();

    if ((REG_AUDIO_I2C_INT_CTRL & 4u) != 0u || REG_AUDIO_DDC_PENDING != 0u) {
        REG_AUDIO_I2C_INT_CTRL = 4;
        __asm__ volatile("dsb" ::: "memory");
        REG_AUDIO_I2C_STATUS |= 1u;
        __disable_irq();
        ddc_transfer_abort_callback();
        __enable_irq();
        REG_AUDIO_DDC_PENDING = 0;
        write_debug_string_if_enabled("h23t mic h\n");
    }

    REG_AUDIO_H23_RD_CTRL |= 7u;
    __asm__ volatile("dsb" ::: "memory");
    REG_AUDIO_DDC_RESET |= 0x400u;
    delay_loop(20);
    REG_AUDIO_H23_RD_CTRL &= 0xF8u;
    __asm__ volatile("dsb" ::: "memory");
    REG_AUDIO_DDC_RESET &= ~0x400u;
    REG_AUDIO_HW_EVENT_MASK_B |= 0x214u;
    REG_AUDIO_H23_SUBSTATE = 1;
    REG_AUDIO_H23_RD_CMD &= (uint8_t)~1u;
    REG_AUDIO_H23_TIMEOUT = 100;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_73(void)
{
    AUDIO_TASK_GUARD();
    audio_set_hw_state_79();
    REG_AUDIO_H23_RD_CMD |= 1u;
    REG_AUDIO_H23_RD_CTRL |= 7u;
    REG_AUDIO_STATUS_LAT = 0;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_h23t_commands(uint8_t *msg)
{
    AUDIO_TASK_GUARD();
    if (msg[0] == 61)
        audio_handle_h23t_wr_str_man();

    if ((REG_AUDIO_DEBUG_FLAGS & 2u) != 0u) {
        switch (msg[0]) {
        case 'B': custom_printf("h23t e rd\n"); break;
        case 'C': custom_printf("h23t ks d\n"); break;
        case 'D': custom_printf("h23t rd\n"); break;
        case 'E': custom_printf("h23t a ps\n"); break;
        case 'F': custom_printf("h23t a f\n"); break;
        case 'G': custom_printf("h23t re a\n"); break;
        default: break;
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_74(void) { return (REG_AUDIO_H23_PKT0 & 2u) != 0u; }

int audio_set_hw_state_75(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_H23_RD_CMD |= 1u;
    REG_AUDIO_H23_RD_MODE = 48;
    REG_AUDIO_H23_FIFO_ARM = 1;
    for (uint8_t i = 0; i < 0x60u; ++i) {
        REG_AUDIO_H23_FIFO_IDX = i;
        REG_AUDIO_H23_FIFO_B0 = REG_AUDIO_H23_FIFO_IMAGE[4u * i + 0u];
        REG_AUDIO_H23_FIFO_B1 = REG_AUDIO_H23_FIFO_IMAGE[4u * i + 1u];
        REG_AUDIO_H23_FIFO_B2 = REG_AUDIO_H23_FIFO_IMAGE[4u * i + 2u];
        REG_AUDIO_H23_FIFO_B3 = REG_AUDIO_H23_FIFO_IMAGE[4u * i + 3u];
    }
    REG_AUDIO_H23_FIFO_ARM = 0;
    REG_AUDIO_H23_PKT1 |= 4u;
    REG_AUDIO_H23_RD_CMD |= 6u;
    REG_AUDIO_H23_RD_CTRL &= 0x2Fu;
    REG_AUDIO_H23_RD_CFG &= (uint8_t)~0x20u;
    REG_AUDIO_H23_RD_CFG |= 4u;
    REG_AUDIO_H23_RD_CFG |= 0x40u;
    REG_AUDIO_H23_RD_LATCH |= 0x70u;
    REG_AUDIO_H23_RD_LATCH |= 2u;
    REG_AUDIO_H23_RD_FLAGS |= 0x30u;
    REG_AUDIO_H23_RD_CTRL &= 0x9Fu;
    REG_AUDIO_H23_RD_CTRL |= 0x10u;
    REG_AUDIO_H23_MANUAL_CTRL |= 1u;
    video_set_state_35();
    REG_AUDIO_H23_STATE = 0;
    REG_AUDIO_H23_REPLAY_PENDING = 1;
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_76(void)
{
    return REG_AUDIO_H23_SUBSTATE == 1u || REG_AUDIO_H23_SUBSTATE == 2u ||
           REG_AUDIO_H23_SUBSTATE == 3u || REG_AUDIO_H23_SUBSTATE == 4u;
}

int audio_process_state_77(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_H23_SUBSTATE == 2u) {
        REG_AUDIO_HW_EVENT_MASK_B &= ~8u;
        if (audio_check_state_42()) {
            REG_AUDIO_HW_EVENT_MASK_B |= 0x200u;
            REG_AUDIO_H23_SUBSTATE = 3;
            REG_AUDIO_HW_EVENT_MASK_B |= 0x400u;
        } else {
            hw_misc_set_state_63();
            REG_AUDIO_H23_SUBSTATE = 4;
            REG_AUDIO_HW_EVENT_MASK_B |= 2u;
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_78(void)
{
    AUDIO_TASK_GUARD();
    video_enqueue_event(11, 0, 4);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_hw_state_79(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_HW_EVENT_MASK_B = 0xFFFFF8E1u;
    REG_AUDIO_H23_WORK_PENDING = 0;
    REG_AUDIO_H23_UNUSED = 0;
    REG_AUDIO_H23_TIMEOUT = 0;
    REG_AUDIO_H23_SUBSTATE = 0;
    video_set_state_35();
    REG_AUDIO_H23_STATE = 0;
    REG_AUDIO_H23_REPLAY_PENDING = 1;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_timer_update_3(void)
{
    AUDIO_TASK_GUARD();

    if (REG_AUDIO_H23_WORK_PENDING != 0u)
        video_process_state_44();
    if (REG_AUDIO_H23_TIMEOUT != 0u && --REG_AUDIO_H23_TIMEOUT == 0u)
        audio_process_state_71();
    if (REG_AUDIO_H23_STATE != 0u) {
        REG_AUDIO_H23_STATE--;
        if (REG_AUDIO_H23_STATE == 0u) {
            if ((REG_AUDIO_APHY_STATUS & 2u) != 0u &&
                (((REG_AUDIO_I2C_INT_CTRL & 4u) != 0u) || REG_AUDIO_DDC_PENDING != 0u)) {
                REG_AUDIO_I2C_INT_CTRL = 4;
                __disable_irq();
                video_enqueue_event(11, 0, 4);
                __enable_irq();
                write_debug_string_if_enabled("h23t mic scl l\n");
            } else {
                REG_AUDIO_H23_STATE = 20;
            }
        }
    }

    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_handle_h23t_wr_str_man(void)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_H23_RD_MODE |= 0x40u;
    audio_set_hw_state_2((char)audio_get_hw_state_5());
    REG_AUDIO_H23_SUBSTATE = 4;
    REG_AUDIO_HW_EVENT_MASK_B |= 2u;
    write_debug_string_if_enabled("h23t wr str man\n");
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_80(void)
{
    AUDIO_TASK_GUARD();
    BOOL ok = (audio_get_hw_state_5() == 1) &&
              ((audio_get_state_91() == 1) || !audio_check_state_74());
    AUDIO_TASK_RETURN(ok);
}

int audio_get_state_81(void)
{
    AUDIO_TASK_GUARD();
    int result;
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        result = audio_get_hw_state_13();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        result = audio_get_state_41();
    else
        result = 0;
    AUDIO_TASK_RETURN(result);
}

int audio_process_state_82(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        audio_set_hw_state_29();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        audio_set_hw_state_70();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_83(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        audio_set_hw_state_30();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        audio_set_hw_state_69();
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_84(void)
{
    AUDIO_TASK_GUARD();
    BOOL ok = (REG_AUDIO_PROTOCOL_MODE == 1u) ? audio_check_state_32()
             : (REG_AUDIO_PROTOCOL_MODE == 2u) ? audio_check_state_74() : false;
    AUDIO_TASK_RETURN(ok);
}

int audio_get_state_85(void)
{
    AUDIO_TASK_GUARD();
    int result;
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        result = audio_get_hw_state_14();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        result = audio_check_state_42();
    else
        result = 0;
    AUDIO_TASK_RETURN(result);
}

int audio_process_state_86(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        audio_handle_h14t_ee();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        audio_handle_h23t_ee();
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_87(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        audio_process_state_31();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        audio_process_state_73();
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_88(void)
{
    AUDIO_TASK_GUARD();
    BOOL ok = (REG_AUDIO_PROTOCOL_MODE == 1u) ? audio_check_state_35()
             : (REG_AUDIO_PROTOCOL_MODE == 2u) ? audio_check_state_76() : false;
    AUDIO_TASK_RETURN(ok);
}

int audio_get_state_89(void)
{
    AUDIO_TASK_GUARD();
    int result;
    if (REG_AUDIO_PROTOCOL_MODE == 1u)
        result = audio_get_hw_state_15();
    else if (REG_AUDIO_PROTOCOL_MODE == 2u)
        result = audio_get_state_43();
    else
        result = 0;
    AUDIO_TASK_RETURN(result);
}

int audio_process_state_90(void)
{
    AUDIO_TASK_GUARD();
    if (!audio_check_state_88()) {
        __disable_irq();
        if (video_get_state_81()) {
            REG_AUDIO_EE_PENDING = 0;
            __enable_irq();
            audio_process_state_86();
        } else {
            REG_AUDIO_EE_PENDING = 1;
            __enable_irq();
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_get_state_91(void) { return REG_AUDIO_PROTOCOL_MODE; }
int audio_get_state_92(void) { return REG_AUDIO_STATE_CODE; }

int audio_process_state_93(uint16_t *req)
{
    AUDIO_TASK_GUARD();
    if (req != NULL) {
        uint8_t *data = ((uint8_t **)req)[1];
        if (*req == 1 && data != NULL) {
            if (data[0] != 0u || REG_AUDIO_CFG2_RETRIES == 0u) {
                REG_AUDIO_CFG2_RETRIES = 0;
                if (data[0] != 4u || (REG_AUDIO_EXT_CTRL & 2u) != 0u)
                    audio_set_state_96(0);
                else
                    audio_set_state_96(1);
            } else {
                REG_AUDIO_CFG2_RETRIES--;
                scdc_write_config_2();
            }
        }
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_94(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_CFG2_RETRIES != 0u) {
        REG_AUDIO_CFG2_RETRIES--;
        scdc_write_config_2();
    } else {
        audio_set_state_96(0);
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_95(void)
{
    AUDIO_TASK_GUARD();
    int handled;
    if (REG_AUDIO_CFG2_PENDING != 0u) {
        REG_AUDIO_CFG2_PENDING = 0;
        scdc_write_config_2();
        handled = 1;
    } else {
        handled = 0;
    }
    AUDIO_TASK_RETURN(handled);
}

int audio_timer_update_4(void)
{
    AUDIO_TASK_GUARD();
    if (REG_AUDIO_PROTOCOL_TIMER != 0u) {
        REG_AUDIO_PROTOCOL_TIMER--;
        if (REG_AUDIO_PROTOCOL_TIMER == 0u)
            video_enqueue_event(40, 0, 0);
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_set_state_96(int h23)
{
    AUDIO_TASK_GUARD();
    REG_AUDIO_PROTOCOL_MODE = (h23 != 0) ? 2u : 1u;
    video_enqueue_event(65, 0, 0);
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_init_state_97(void)
{
    AUDIO_TASK_GUARD();
    bzero_align((uint32_t *)0x41BD8, 0xEu);
    REG_AUDIO_PROTOCOL_CB_ENABLE = 1;
    REG_AUDIO_PROTOCOL_TIMEOUT0 = 600;
    REG_AUDIO_PROTOCOL_TIMEOUT1 = 15;
    REG_AUDIO_PROTOCOL_TIMEOUT2 = 15;
    *(volatile uint8_t *)0x40101040 |= 0x20u;
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_98(int a1, int a2)
{
    AUDIO_TASK_GUARD();
    uint8_t flags = (a1 != 0) ? 1u : 0u;
    if (a2 != 0)
        flags |= 2u;
    if (REG_AUDIO_PROTOCOL_CB_ENABLE != 0u) {
        REG_AUDIO_CALC_FLAGS = flags;
        __disable_irq();
        audio_process_state_100();
        __enable_irq();
    }
    AUDIO_TASK_RETURN((int)saved_task);
}

int audio_process_state_99(void)
{
    AUDIO_TASK_GUARD();
    int handled;
    if (REG_AUDIO_CALC_READ_PENDING != 0u) {
        REG_AUDIO_CALC_READ_PENDING = 0;
        audio_process_state_100();
        handled = 1;
    } else {
        handled = 0;
    }
    AUDIO_TASK_RETURN(handled);
}

int audio_process_state_100(void)
{
    AUDIO_TASK_GUARD();
    if (ddc_read_request(0xA8, 32, REG_AUDIO_INFO_READ_REQ) == 255)
        REG_AUDIO_CALC_READ_PENDING = 1;
    AUDIO_TASK_RETURN((int)saved_task);
}

BOOL audio_check_state_101(void)
{
    AUDIO_TASK_GUARD();
    BOOL ok = (!video_check_hw_state_126() && (REG_AUDIO_CALC_RESULT > 1135214592u ||
               (REG_AUDIO_PROTOCOL_CB_ENABLE != 0u && *(volatile uint8_t *)0x41BE5 != 0u))) ||
              (video_check_hw_state_126() && video_check_hw_state_127());
    AUDIO_TASK_RETURN(ok);
}

int audio_set_hw_state_102(int enable_rx, int enable_flag)
{
    uint32_t reg = *(volatile uint32_t *)0x40101000 & 0xFFFFEFF7u;
    uint32_t reg2;

    if (enable_flag != 0) {
        reg |= 8u;
        reg2 = *(volatile uint32_t *)0x40101028 | 0x20u;
    } else {
        reg2 = *(volatile uint32_t *)0x40101028 & ~0x20u;
    }
    *(volatile uint32_t *)0x40101028 = reg2;

    if (enable_rx != 0)
        reg |= 0x1008u;
    *(volatile uint32_t *)0x40101000 = reg;

    __disable_irq();
    if (enable_flag != 0)
        *(volatile uint32_t *)0x40100E70 |= 4u;
    else
        *(volatile uint32_t *)0x40100E70 &= ~4u;
    __enable_irq();

    return (int)REG_TCB_CURRENT_TASK;
}

int audio_set_state_103(int value)
{
    REG_AUDIO_CALC_RESULT = (uint32_t)value;
    return (int)REG_TCB_CURRENT_TASK;
}

int audio_process_state_104(void)
{
    AUDIO_TASK_GUARD();
    int handled;
    if (REG_AUDIO_CFG3_PENDING != 0u) {
        REG_AUDIO_CFG3_PENDING = 0;
        scdc_write_config_3();
        handled = 1;
    } else if (REG_AUDIO_CFG4_PENDING != 0u) {
        REG_AUDIO_CFG4_PENDING = 0;
        scdc_write_config_4();
        handled = 1;
    } else {
        handled = 0;
    }
    AUDIO_TASK_RETURN(handled);
}

int audio_state_machine_1(void)
{
    AUDIO_TASK_GUARD();

    if (REG_AUDIO_FSM_STATE_FRL != 1u && REG_AUDIO_FSM_STATE_FRL != 10u) {
        bool advanced;
        do {
            advanced = false;
            switch (REG_AUDIO_FSM_STATE_FRL) {
            case 2:
                frl_rate_set_2();
                advanced = true;
                break;
            case 3:
                advanced = frl_process_state_2() != 0;
                break;
            case 4:
                scdc_state_transition_5();
                break;
            case 5:
                if (frl_check_state_1()) {
                    REG_AUDIO_FSM_STATE_FRL = 6;
                    advanced = true;
                }
                break;
            case 6:
                frl_txpll_enable_lol();
                advanced = true;
                break;
            case 7:
                if (frl_check_state_3()) {
                    video_check_state_41();
                    if (REG_AUDIO_FRL_RETRY_FLAG != 0u)
                        REG_AUDIO_FSM_STATE_FRL = 8;
                    else
                        frl_rate_set_0();
                }
                break;
            case 8:
                if (REG_AUDIO_FRL_RETRY_FLAG == 0u)
                    frl_rate_set_0();
                break;
            case 9:
                advanced = frl_process_state_4() != 0;
                break;
            default:
                break;
            }
        } while (advanced);
    }

    AUDIO_TASK_RETURN((int)saved_task);
}
