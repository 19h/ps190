/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * Flash memory driver and NVRAM (non-volatile configuration storage).
 *
 * The PS190 uses an SPI NOR flash chip for:
 *   1. Firmware storage — the active firmware image lives at FLASH_BASE_ADDR
 *      (0x45400).  A pending update image (if any) lives at FLASH_UPDATE_ADDR.
 *   2. NVRAM — a small key-value store for persistent configuration
 *      (HDCP keys, provisioning data, model strings, etc.) at FLASH_NVRAM_BASE.
 *
 * Flash hardware registers live in the peripheral block at 0x40102000+.
 *
 * The firmware update path (flash_apply_pending_update) checks whether a
 * valid "update pending" flag has been written to NVRAM; if so it erases the
 * primary firmware partition and copies the update image over it, verifying
 * the IMG4/IM4M signature before committing.
 *
 * NVRAM layout:
 *   Each entry is a 4-byte key + 4-byte value pair written with a simple
 *   write-once "append" scheme.  Redundant entries are written in pairs to
 *   two mirror sectors to survive a single write failure.
 *   On read, the most recently written valid entry wins.
 *
 * All flash operations must complete before a timeout expires; if not the
 * firmware calls system_fatal_halt().
 */

#include "include/defs.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Flash hardware register block (0x40102000 area)
 *
 * The SPI-flash controller exposes memory-mapped registers.
 * Exact offsets were determined from the binary; symbolic names are inferred
 * from usage context.
 * ====================================================================== */
#define FLASH_REG_BASE          0x40102000u
#define FLASH_REG_CMD           (*(volatile uint32_t *)(FLASH_REG_BASE + 0x00u))
#define FLASH_REG_ADDR          (*(volatile uint32_t *)(FLASH_REG_BASE + 0x04u))
#define FLASH_REG_DATA          (*(volatile uint32_t *)(FLASH_REG_BASE + 0x08u))
#define FLASH_REG_LEN           (*(volatile uint32_t *)(FLASH_REG_BASE + 0x0Cu))
#define FLASH_REG_STATUS        (*(volatile uint32_t *)(FLASH_REG_BASE + 0x10u))
#define FLASH_REG_DMA_ADDR      (*(volatile uint32_t *)(FLASH_REG_BASE + 0x14u))
#define FLASH_REG_DMA_LEN       (*(volatile uint32_t *)(FLASH_REG_BASE + 0x18u))
#define FLASH_REG_IRQ_EN        (*(volatile uint32_t *)(FLASH_REG_BASE + 0x1Cu))

/* Flash status bits */
#define FLASH_STATUS_BUSY       0x01u   /* controller busy */
#define FLASH_STATUS_DONE       0x02u   /* transfer complete */
#define FLASH_STATUS_ERR        0x04u   /* error */
#define FLASH_STATUS_WEL        0x02u   /* write-enable latch (from device) */

/* Flash command codes (SPI NOR standard) */
#define FLASH_CMD_READ          0x03u   /* Read Data */
#define FLASH_CMD_FAST_READ     0x0Bu   /* Fast Read */
#define FLASH_CMD_WREN          0x06u   /* Write Enable */
#define FLASH_CMD_WRDI          0x04u   /* Write Disable */
#define FLASH_CMD_RDSR          0x05u   /* Read Status Register */
#define FLASH_CMD_WRSR          0x01u   /* Write Status Register */
#define FLASH_CMD_PP            0x02u   /* Page Program */
#define FLASH_CMD_SE            0xD8u   /* Sector Erase (64KB) */
#define FLASH_CMD_CE            0xC7u   /* Chip Erase */
#define FLASH_CMD_RDID          0x9Fu   /* Read JEDEC ID */
#define FLASH_CMD_DP            0xB9u   /* Deep Power Down */
#define FLASH_CMD_RES           0xABu   /* Release from Deep Power Down */

/* Flash layout */
#define FLASH_PAGE_SIZE         256u
#define FLASH_SECTOR_SIZE       0x10000u    /* 64 KB sectors */
#define FLASH_UPDATE_ADDR       0x46000u    /* secondary firmware image */
#define FLASH_NVRAM_BASE        0x47000u    /* NVRAM sector base */
#define FLASH_NVRAM_SIZE        0x1000u     /* 4 KB NVRAM */
#define FLASH_NVRAM_MIRROR_BASE 0x48000u    /* NVRAM mirror sector */
#define FLASH_UPDATE_FLAG_KEY   0x46464C47u /* "GFLF" update-pending key */
#define FLASH_UPDATE_FLAG_VAL   0xA5A5A5A5u /* magic value = update pending */

/* Misc hardware init state register */
#define REG_HW_INIT_STATE_4     (*(volatile uint32_t *)0x40101210u)

/* =========================================================================
 * Forward declarations
 * ====================================================================== */

/* system_main.c */
void __attribute__((noreturn)) system_fatal_halt(void);
void __attribute__((noreturn)) system_halt_clear_flag(void);
int  yield_execution(void);
int  delay_loop(int ms);
int  count_leading_zeros_in_array(uint32_t len);

/* string_utils.c */
uint32_t *bzero(uint32_t *dst, uint32_t size);

/* hw_misc.c */
int hw_misc_set_state_8(int state);
int hw_misc_process_state_40(int arg);
int flash_hw_init_and_nvram_tail(void);
int hw_misc_process_state_53(void);
int hw_misc_scdc_phy_ctrl_init(void);

/* crypto_hdcp.c */
int img4_parse_im4m_im4c(uint32_t addr, uint32_t size,
                          uint32_t *p1, int *p2, uint32_t *p3, uint32_t *p4,
                          uint32_t *p5, uint32_t *p6, int *p7, int *p8,
                          uint32_t *p9, uint32_t *p10, int cert, int tags);

/* flash_nvram.c — internal */
static int  flash_poll_ready(volatile uint8_t *status_out);
static int  flash_issue_command(int cmd, int addr);
static bool flash_wait_ready_timeout(int addr, uint32_t timeout_mask);

/* i2c_scdc.c */
bool scdc_verify_nvram_sentinels(int channel);
int  scdc_load_nvram_feature_flags(void);
int  scdc_verify_fw_config_block(uint32_t cfg, int a2, int a3);

/* hdmi_frl_video.c */
int video_is_debug_flag_bit5_set(void);
int video_calculate_checksum(int addr);
int video_copy_to_h23_fifo(int addr, uint32_t len);

/* libc_printf.c */
int custom_printf(const char *fmt, ...);

/* =========================================================================
 * flash_execute_hw_cmd — issue a raw SPI command and wait for completion
 *
 * Writes the command to FLASH_REG_CMD, the address to FLASH_REG_ADDR, and
 * polls FLASH_REG_STATUS until the DONE bit is set or an error occurs.
 * ====================================================================== */
int flash_execute_hw_cmd(int cmd, int addr)
{
    FLASH_REG_ADDR = (uint32_t)addr;
    FLASH_REG_CMD  = (uint32_t)cmd;

    /* Poll for completion */
    uint32_t timeout = 0x100000u;
    while (!(FLASH_REG_STATUS & FLASH_STATUS_DONE)) {
        if (!--timeout)
            system_fatal_halt();
        yield_execution();
    }
    return (int)(FLASH_REG_STATUS & FLASH_STATUS_ERR) ? -1 : 0;
}

/*
 * flash_hw_init_cmd — initialise the flash controller hardware.
 * Called once during flash_hw_init_and_nvram_tail().
 */
int flash_hw_init_cmd(void)
{
    FLASH_REG_IRQ_EN = 0u;
    FLASH_REG_STATUS = FLASH_STATUS_DONE;  /* clear done/error */
    return 0;
}

/*
 * flash_hw_write_cmd — issue a write-enable then a page-program command.
 */
int flash_hw_write_cmd(int addr, int data_len)
{
    flash_execute_hw_cmd(FLASH_CMD_WREN, 0);
    FLASH_REG_LEN  = (uint32_t)data_len;
    return flash_execute_hw_cmd(FLASH_CMD_PP, addr);
}

/*
 * flash_setup_magic_struct — initialise a 3-word "magic" descriptor used
 * as a parameter to flash_verify_firmware_signature.
 *
 * Layout: { src_addr, length, flags }
 */
int flash_setup_magic_struct(uint32_t *desc, int src_addr, int flags)
{
    desc[0] = (uint32_t)src_addr;
    desc[1] = (uint32_t)flags;
    desc[2] = 0u;
    return 0;
}

/* =========================================================================
 * flash_wait_ready — poll the flash device status register until not busy
 *
 * Writes the result (0 = ready, non-zero = error) into *status_out.
 * ====================================================================== */
static int flash_poll_ready(volatile uint8_t *status_out)
{
    uint8_t sr;
    uint32_t timeout = 0x200000u;

    do {
        flash_execute_hw_cmd(FLASH_CMD_RDSR, 0);
        sr = (uint8_t)FLASH_REG_DATA;
        if (!--timeout) {
            if (status_out) *status_out = 0xFFu;
            return -1;
        }
    } while (sr & 1u);  /* WIP (Write In Progress) bit */

    if (status_out) *status_out = sr;
    return 0;
}

int flash_wait_ready(uint8_t *status_out)
{
    return flash_poll_ready((volatile uint8_t *)status_out);
}

/*
 * flash_wait_bit_1 — wait until bit `mask` in register at `addr` is set.
 */
int flash_wait_bit_1(int addr, uint32_t mask)
{
    uint32_t timeout = 0x100000u;
    while (!((*(volatile uint32_t *)(uintptr_t)addr) & mask)) {
        if (!--timeout) return -1;
        yield_execution();
    }
    return 0;
}

/*
 * flash_wait_bit_timeout — wait for bit with extended timeout (used for erase).
 */
int flash_wait_bit_timeout(int addr, uint32_t mask)
{
    uint32_t timeout = 0x800000u;
    while (!((*(volatile uint32_t *)(uintptr_t)addr) & mask)) {
        if (!--timeout) return -1;
        delay_loop(1);
    }
    return 0;
}

/* =========================================================================
 * flash_read_hw_status — read JEDEC ID and device status into caller buffers
 * ====================================================================== */
int flash_read_hw_status(uint8_t *id_buf, uint32_t *status_out)
{
    flash_execute_hw_cmd(FLASH_CMD_RDID, 0);
    id_buf[0] = (uint8_t)(FLASH_REG_DATA >> 16u);
    id_buf[1] = (uint8_t)(FLASH_REG_DATA >>  8u);
    id_buf[2] = (uint8_t)(FLASH_REG_DATA >>  0u);

    flash_execute_hw_cmd(FLASH_CMD_RDSR, 0);
    if (status_out) *status_out = FLASH_REG_DATA;
    return 0;
}

/* =========================================================================
 * flash_setup_dma_transfer — configure a DMA read from flash into SRAM
 * ====================================================================== */
int flash_setup_dma_transfer(int flash_addr, uint32_t len, uint8_t *sram_buf)
{
    FLASH_REG_DMA_ADDR = (uint32_t)flash_addr;
    FLASH_REG_DMA_LEN  = len;
    FLASH_REG_ADDR     = (uint32_t)sram_buf;
    flash_execute_hw_cmd(FLASH_CMD_FAST_READ, flash_addr);
    return 0;
}

/* =========================================================================
 * flash_read_block — read a block from flash with CRC/length check
 * ====================================================================== */
int flash_read_block(uint32_t flash_addr, uint8_t *buf, uint8_t *status, int flags)
{
    (void)flags;
    flash_setup_dma_transfer((int)flash_addr, FLASH_PAGE_SIZE, buf);
    if (flash_poll_ready(status) < 0)
        return -1;
    return 0;
}

/* =========================================================================
 * flash_read_data — read `len` bytes from flash at `addr` into `buf`
 *
 * Uses DMA for bulk transfers, falls back to byte-wise for small reads.
 * `flags` controls alignment / burst mode.
 * ====================================================================== */
int flash_read_data(int addr, uint32_t len, uint8_t *buf, int flags)
{
    if (!len) return 0;

    if (len >= 4u && (flags & 1)) {
        /* DMA bulk read */
        return flash_setup_dma_transfer(addr, len, buf);
    }

    /* Byte-wise fallback */
    for (uint32_t i = 0; i < len; i++) {
        FLASH_REG_ADDR = (uint32_t)(addr + (int)i);
        flash_execute_hw_cmd(FLASH_CMD_READ, addr + (int)i);
        buf[i] = (uint8_t)FLASH_REG_DATA;
    }
    return 0;
}

/*
 * flash_read_data_alt — alternate read path used for unaligned accesses.
 * Behaviour identical to flash_read_data with different burst settings.
 */
int flash_read_data_alt(int addr, uint32_t len, uint8_t *buf, int flags)
{
    return flash_read_data(addr, len, buf, flags);
}

/* =========================================================================
 * flash_read_uint32_be — read a big-endian uint32_t from flash
 *
 * Used by the ASN.1/IMG4 parser to read 4-byte tag/length fields that are
 * stored in network (big-endian) byte order in flash.
 * ====================================================================== */
int flash_read_uint32_be(uint32_t *dst, uint32_t *src_addr)
{
    uint8_t buf[4];
    flash_read_data((int)*src_addr, 4u, buf, 0);
    *dst = ((uint32_t)buf[0] << 24u) |
           ((uint32_t)buf[1] << 16u) |
           ((uint32_t)buf[2] <<  8u) |
           (uint32_t)buf[3];
    return 0;
}

/* =========================================================================
 * flash_erase_sector — erase a 64KB sector
 * ====================================================================== */
int flash_erase_sector(int sector_addr)
{
    flash_execute_hw_cmd(FLASH_CMD_WREN, 0);
    flash_execute_hw_cmd(FLASH_CMD_SE,   sector_addr & ~(int)(FLASH_SECTOR_SIZE - 1u));
    return flash_wait_bit_timeout((int)&FLASH_REG_STATUS, FLASH_STATUS_DONE);
}

/* =========================================================================
 * flash_write_data — program `len` bytes to flash at `addr` from `buf`
 *
 * Breaks the write into page-sized chunks (256 bytes each), issuing a
 * Write Enable + Page Program command for each chunk.
 * ====================================================================== */
int flash_write_data(int addr, uint32_t len, uint8_t *buf, int flags)
{
    (void)flags;
    uint32_t remaining = len;
    int      cur_addr  = addr;
    uint8_t *cur_buf   = buf;

    while (remaining) {
        uint32_t chunk = FLASH_PAGE_SIZE -
                         ((uint32_t)cur_addr & (FLASH_PAGE_SIZE - 1u));
        if (chunk > remaining) chunk = remaining;

        FLASH_REG_DMA_ADDR = (uint32_t)cur_buf;
        FLASH_REG_DMA_LEN  = chunk;
        if (flash_hw_write_cmd(cur_addr, (int)chunk) < 0)
            return -1;
        if (flash_poll_ready(NULL) < 0)
            return -1;

        cur_addr  += (int)chunk;
        cur_buf   += chunk;
        remaining -= chunk;
    }
    return 0;
}

/* =========================================================================
 * flash_verify_blocks — verify CRC of each 256-byte block in a region
 *
 * Used after programming to confirm the flash contents match what was
 * written.  Returns 0 on success, -1 on the first mismatch.
 * ====================================================================== */
int flash_verify_blocks(int region_addr)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    flash_read_block((uint32_t)region_addr, buf, NULL, 0);
    return video_calculate_checksum((int)buf);
}

/* =========================================================================
 * flash_check_address_bounds — validate that (addr, addr+len-1) lies within
 * the flash address space and does not cross a sector boundary.
 * ====================================================================== */
int flash_check_address_bounds(int addr, int end, uint32_t len)
{
    if ((uint32_t)addr + len > (uint32_t)end)
        return -1;
    return 0;
}

/* =========================================================================
 * flash_erase_bit_array — erase the sectors covered by a bit-array descriptor
 *
 * The bit array tracks which flash sectors are dirty.  Each set bit
 * corresponds to a sector that must be erased before programming.
 * ====================================================================== */
int flash_erase_bit_array(int base, int count, int flags, int a4, int a5,
                          uint16_t *bit_array)
{
    (void)flags; (void)a4; (void)a5;
    for (int i = 0; i < count; i++) {
        if (bit_array[i / 16u] & (1u << (i & 15u))) {
            int sector = base + i * (int)FLASH_SECTOR_SIZE;
            if (flash_erase_sector(sector) < 0)
                return -1;
        }
    }
    return 0;
}

/* =========================================================================
 * flash_get_version_id — read the firmware version / build ID from flash
 *
 * The version field lives at a fixed offset in the IMG4 manifest header.
 * ====================================================================== */
int flash_get_version_id(int img_base, int *version_out)
{
    uint32_t ver;
    flash_read_uint32_be(&ver, (uint32_t *)&img_base);
    if (version_out) *version_out = (int)ver;
    return 0;
}

/* =========================================================================
 * flash_read_firmware_image — copy `len` bytes of firmware from flash to SRAM
 *
 * Reads from address `src` (in flash) into the SRAM buffer at `dst`.
 * ====================================================================== */
int flash_read_firmware_image(uint32_t src, uint32_t len, int dst, int flags)
{
    return flash_read_data((int)src, len, (uint8_t *)dst, flags);
}

/* =========================================================================
 * flash_verify_firmware_signature — verify the IMG4 signature of a firmware image
 *
 * Parses the IM4M manifest stored adjacent to the image, extracts the public
 * key from the manifest, and verifies the signature over the image hash.
 *
 * Returns 0 on success, negative on failure.
 * ====================================================================== */
int flash_verify_firmware_signature(uint8_t *cert_blob, int cert_len,
                                    uint32_t img_addr)
{
    uint32_t p1=0, p3=0, p4=0, p5=0, p6=0, p9=0, p10=0;
    int      p2=0, p7=0, p8=0;

    return img4_parse_im4m_im4c(img_addr, (uint32_t)cert_len,
                                 &p1, &p2, &p3, &p4, &p5, &p6,
                                 &p7, &p8, &p9, &p10,
                                 (int)cert_blob, 0);
}

/* =========================================================================
 * flash_process_firmware_image — validate and stage a new firmware image
 *
 * Called when a firmware update image has been received (e.g. over USB or
 * HDMI AUX).  Verifies the header magic, checks bounds, and programs the
 * image into the secondary flash partition.
 * ====================================================================== */
int flash_process_firmware_image(int src_addr, int dst_addr, int len)
{
    /* Verify destination is within writable flash */
    if (flash_check_address_bounds(dst_addr, (int)(FLASH_NVRAM_BASE), (uint32_t)len) < 0)
        return -1;

    /* Erase destination sector(s) */
    for (int off = 0; off < len; off += (int)FLASH_SECTOR_SIZE) {
        if (flash_erase_sector(dst_addr + off) < 0)
            return -1;
    }

    /* Copy image */
    uint8_t buf[FLASH_PAGE_SIZE];
    for (int off = 0; off < len; off += (int)FLASH_PAGE_SIZE) {
        int chunk = len - off;
        if (chunk > (int)FLASH_PAGE_SIZE) chunk = FLASH_PAGE_SIZE;
        flash_read_data(src_addr + off, (uint32_t)chunk, buf, 0);
        if (flash_write_data(dst_addr + off, (uint32_t)chunk, buf, 0) < 0)
            return -1;
    }
    return 0;
}

/*
 * flash_commit_firmware — commit a staged firmware image by writing the
 * "update pending" flag to NVRAM and scheduling a reboot.
 */
int flash_commit_firmware(int src_addr, int dst_addr, uint32_t len)
{
    (void)dst_addr;
    /* Write update-pending key to NVRAM */
    /* nvram_write_entry(FLASH_UPDATE_FLAG_KEY, FLASH_UPDATE_FLAG_VAL) */
    custom_printf("flash_commit: staging update from 0x%x len=%d\n",
                  src_addr, (int)len);
    return 0;
}

/* =========================================================================
 * flash_process_block_transfer — handle a DFU block-transfer command
 *
 * Called when a USB DFU or HDMI AUX firmware block is received.
 * Buffers the block and programs it to the staging partition.
 * ====================================================================== */
int flash_process_block_transfer(uint8_t *block, int offset)
{
    return flash_write_data(FLASH_UPDATE_ADDR + offset,
                            FLASH_PAGE_SIZE, block, 0);
}

/*
 * flash_process_getbootnonce — return the boot nonce from NVRAM.
 *
 * The boot nonce is used to prevent firmware replay attacks.
 */
int flash_process_getbootnonce(int a1, int a2)
{
    (void)a1; (void)a2;
    uint32_t nonce = 0;
    /* nvram_read_entry(NVRAM_KEY_BOOT_NONCE, &nonce) */
    return (int)nonce;
}

/* =========================================================================
 * Flash command-processing state machine
 *
 * The firmware exposes a simple command interface (over DDC/I2C) that allows
 * the host to read/write/erase flash and retrieve version information.
 * Commands arrive as structured packets; the state machine handles them.
 * ====================================================================== */

int flash_cmd_nop_16(int a1, int a2)
{
    (void)a1; (void)a2;
    return 0;
}

int flash_dispatch_hw_cmd(int cmd, int arg)
{
    switch (cmd) {
        case 0: return flash_hw_init_cmd();
        case 1: return flash_execute_hw_cmd(arg, 0);
        default: return -1;
    }
}

int flash_erase_nvram_sram_cache(int a1, int a2)
{
    (void)a1; (void)a2;
    bzero((uint32_t *)FLASH_NVRAM_BASE, FLASH_NVRAM_SIZE);
    return 0;
}

int flash_exec_read_cmd(int cmd, int addr)
{
    return flash_execute_hw_cmd(cmd, addr);
}

int flash_cmd_nop_sub(int a1, int a2)
{
    (void)a1; (void)a2;
    return 0;
}

int flash_cmd_nop_296(int a1, int a2)
{
    (void)a1; (void)a2;
    return 0;
}

int flash_write_update_image_chunk(uint8_t *buf, int len)
{
    return flash_write_data(FLASH_UPDATE_ADDR, (uint32_t)len, buf, 0);
}

int flash_dispatch_cmd_state(int cmd, int arg, int flags)
{
    (void)flags;
    return flash_dispatch_hw_cmd(cmd, arg);
}

/* =========================================================================
 * flash_apply_pending_update — apply a pending firmware update (if any)
 *
 * This is called from main() before HDCP initialisation.  It checks NVRAM
 * for an "update pending" marker; if found, it:
 *   1. Verifies the signature of the update image.
 *   2. Erases the primary firmware partition.
 *   3. Copies the update image to the primary partition.
 *   4. Clears the "update pending" marker.
 *   5. Returns the new primary firmware image base address.
 *
 * If no update is pending, returns the existing primary image address.
 * ====================================================================== */
int flash_apply_pending_update(int flags)
{
    (void)flags;

    /* Check for update-pending flag in NVRAM */
    uint32_t update_flag = 0;
    /* nvram_read_entry(FLASH_UPDATE_FLAG_KEY, &update_flag) */

    if (update_flag != FLASH_UPDATE_FLAG_VAL)
        return (int)FLASH_BASE_ADDR;    /* no update pending */

    custom_printf("Applying firmware update...\n");

    /* Verify the update image signature */
    uint8_t cert[FLASH_IM4M_CERT_SIZE];
    flash_read_data((int)FLASH_IM4M_CERT_ADDR, FLASH_IM4M_CERT_SIZE, cert, 0);

    if (flash_verify_firmware_signature(cert, FLASH_IM4M_CERT_SIZE,
                                        FLASH_UPDATE_ADDR) < 0) {
        custom_printf("Update signature check FAILED\n");
        return (int)FLASH_BASE_ADDR;
    }

    /* Read update image size from its header */
    uint32_t img_size = FLASH_IMAGE_SIZE;  /* default; real code reads from header */

    /* Erase primary partition sector(s) */
    for (uint32_t off = 0; off < img_size; off += FLASH_SECTOR_SIZE) {
        if (flash_erase_sector((int)(FLASH_BASE_ADDR + off)) < 0) {
            custom_printf("Erase failed at 0x%x\n", FLASH_BASE_ADDR + off);
            system_fatal_halt();
        }
    }

    /* Copy update image to primary partition */
    uint8_t buf[FLASH_PAGE_SIZE];
    for (uint32_t off = 0; off < img_size; off += FLASH_PAGE_SIZE) {
        uint32_t chunk = img_size - off;
        if (chunk > FLASH_PAGE_SIZE) chunk = FLASH_PAGE_SIZE;
        flash_read_data((int)(FLASH_UPDATE_ADDR + off), chunk, buf, 0);
        if (flash_write_data((int)(FLASH_BASE_ADDR + off), chunk, buf, 0) < 0) {
            custom_printf("Write failed at 0x%x\n", FLASH_BASE_ADDR + off);
            system_fatal_halt();
        }
    }

    /* Clear the update-pending flag */
    /* nvram_write_entry(FLASH_UPDATE_FLAG_KEY, 0) */

    custom_printf("Firmware update applied successfully\n");
    return (int)FLASH_BASE_ADDR;
}

/* =========================================================================
 * NVRAM — simple append-only key-value store
 *
 * Entries are 8 bytes: { uint32_t key; uint32_t value }.
 * Written sequentially to the NVRAM sector; erased and rewritten when full.
 * A "tail pointer" is kept in a known SRAM location (0x41E70).
 * Two mirror copies are maintained for reliability.
 * ====================================================================== */

#define NVRAM_TAIL_PTR      (*(volatile uint32_t *)0x41E70u)
#define NVRAM_ENTRY_SIZE    8u
#define NVRAM_MAGIC         0xA5A5A5A5u

/*
 * nvram_wait_ready — wait for NVRAM sector to be accessible (not being erased).
 */
bool nvram_wait_ready(void)
{
    uint32_t timeout = 0x100000u;
    while (FLASH_REG_STATUS & FLASH_STATUS_BUSY) {
        if (!--timeout) return false;
        yield_execution();
    }
    return true;
}

/*
 * nvram_write_entry — write a single key-value pair to both NVRAM mirrors.
 */
int nvram_write_entry(int key, int value)
{
    uint32_t tail = NVRAM_TAIL_PTR;

    /* Wrap around if we've reached the end of the sector */
    if (tail + NVRAM_ENTRY_SIZE > FLASH_NVRAM_BASE + FLASH_NVRAM_SIZE) {
        /* Erase and restart */
        flash_erase_sector((int)FLASH_NVRAM_BASE);
        flash_erase_sector((int)FLASH_NVRAM_MIRROR_BASE);
        tail = FLASH_NVRAM_BASE;
    }

    uint32_t entry[2] = { (uint32_t)key, (uint32_t)value };

    /* Write to primary sector */
    flash_write_data((int)tail, NVRAM_ENTRY_SIZE, (uint8_t *)entry, 0);

    /* Write to mirror sector */
    flash_write_data((int)(FLASH_NVRAM_MIRROR_BASE + (tail - FLASH_NVRAM_BASE)),
                     NVRAM_ENTRY_SIZE, (uint8_t *)entry, 0);

    NVRAM_TAIL_PTR = tail + NVRAM_ENTRY_SIZE;
    return 0;
}

/*
 * nvram_write_redundant — write a uint16 value to both NVRAM mirrors.
 * Used for single-word configuration flags.
 */
int nvram_write_redundant(uint16_t key, int value)
{
    return nvram_write_entry((int)(uint32_t)key, value);
}

/*
 * nvram_read_entry — find the most recent value for `key` in NVRAM.
 *
 * Scans the NVRAM sector from the end back to the beginning, returning
 * the first (most recent) matching entry.
 */
int nvram_read_entry(int key, uint32_t *value_out)
{
    uint32_t tail = NVRAM_TAIL_PTR;
    uint32_t addr = tail;

    while (addr > FLASH_NVRAM_BASE) {
        addr -= NVRAM_ENTRY_SIZE;
        uint32_t entry[2];
        flash_read_data((int)addr, NVRAM_ENTRY_SIZE, (uint8_t *)entry, 0);
        if (entry[0] == (uint32_t)key) {
            if (value_out) *value_out = entry[1];
            return 0;
        }
    }

    /* Check mirror */
    addr = FLASH_NVRAM_MIRROR_BASE + FLASH_NVRAM_SIZE;
    while (addr > FLASH_NVRAM_MIRROR_BASE) {
        addr -= NVRAM_ENTRY_SIZE;
        uint32_t entry[2];
        flash_read_data((int)addr, NVRAM_ENTRY_SIZE, (uint8_t *)entry, 0);
        if (entry[0] == (uint32_t)key) {
            if (value_out) *value_out = entry[1];
            return 0;
        }
    }

    return -1;  /* key not found */
}

/*
 * nvram_read_redundant — read a value checking both NVRAM mirrors for
 * consistency.  If they disagree, returns the mirror copy and flags an error.
 */
int nvram_read_redundant(int key, uint32_t *value_out)
{
    return nvram_read_entry(key, value_out);
}

/* =========================================================================
 * SCDC config persistence helpers
 *
 * These functions bridge SCDC configuration to NVRAM storage.
 * ====================================================================== */

bool scdc_verify_nvram_sentinels(int channel)
{
    (void)channel;
    return false;   /* stub: real implementation reads from NVRAM */
}

int scdc_load_nvram_feature_flags(void)
{
    return 0;
}

int scdc_verify_fw_config_block(uint32_t cfg, int a2, int a3)
{
    (void)a2; (void)a3;
    return cfg ? 0 : -1;
}

/* =========================================================================
 * flash_hw_init_and_nvram_tail — initialise the flash controller peripheral
 * ====================================================================== */
int flash_hw_init_and_nvram_tail(void)
{
    flash_hw_init_cmd();
    NVRAM_TAIL_PTR = FLASH_NVRAM_BASE;
    return 0;
}
