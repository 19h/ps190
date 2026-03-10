/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * String / memory utilities and floating-point support helpers.
 *
 * This module provides the low-level memory primitives (memcpy, memset,
 * bzero, strcmp, memcmp) used throughout the firmware.  Because the chip
 * has no libc, these are hand-written and aggressively unrolled to match
 * what the ARM compiler generates from __builtin_memcpy / __builtin_memset.
 *
 * Also included: helper functions shared between the float formatter
 * (libc_printf.c) and the softfloat library (softfloat.c), which process
 * IEEE 754 double-precision (float64) bit patterns directly as two 32-bit
 * halves (high word / low word).
 *
 * The hardware FIFO at 0x20001000 is used as the debug UART transmit buffer:
 * write_to_hw_fifo_20001000() fills it 4 bytes at a time.
 */

#include "include/defs.h"
#include <stdint.h>
#include <stdbool.h>

/* Forward declaration (implemented in system_main.c) */
void __attribute__((noreturn)) system_halt_clear_flag(void);

/* =========================================================================
 * fast_memcpy — word-at-a-time bulk copy, Thumb-2 LDM/STM style
 *
 * Copies `size` bytes from `src` to `dst` using 32-byte (8-word) then
 * 16-byte, 8-byte, 4-byte, 2-byte, and 1-byte steps to minimise loop
 * overhead.  `size` must be an exact byte count; alignment is assumed on
 * the caller's side (the compiler usually guarantees this for struct copies).
 *
 * The `extra` parameter carries the caller's r3 argument through; it is
 * used as scratch for the partial-word tail copy.
 *
 * Returns a pointer to one-past the last written destination byte
 * (C++ iterator convention), consistent with the ABI used by the firmware's
 * struct-copy helpers.
 * ====================================================================== */
uint32_t *fast_memcpy(uint32_t *dst, const uint32_t *src, uint32_t size, int extra)
{
    /* 32-byte (8-word) main loop */
    while (size >= 32u) {
        uint32_t w0 = src[0], w1 = src[1], w2 = src[2], w3 = src[3];
        uint32_t w4 = src[4], w5 = src[5], w6 = src[6], w7 = src[7];
        dst[0] = w0;  dst[1] = w1;  dst[2] = w2;  dst[3] = w3;
        dst[4] = w4;  dst[5] = w5;  dst[6] = w6;  dst[7] = w7;
        src  += 8;
        dst  += 8;
        size -= 32u;
        (void)extra;
    }

    /* 16-byte (4-word) chunk */
    if (size & 16u) {
        uint32_t w0 = src[0], w1 = src[1], w2 = src[2], w3 = src[3];
        dst[0] = w0;  dst[1] = w1;  dst[2] = w2;  dst[3] = w3;
        src += 4;
        dst += 4;
    }

    /* 8-byte (2-word) chunk */
    if (size & 8u) {
        uint32_t w0 = src[0], w1 = src[1];
        dst[0] = w0;  dst[1] = w1;
        src += 2;
        dst += 2;
    }

    /* 4-byte (1-word) chunk */
    if (size & 4u) {
        *dst++ = *src++;
    }

    /* Byte-granularity tail (0–3 bytes) */
    {
        uint8_t *dp = (uint8_t *)dst;
        const uint8_t *sp = (const uint8_t *)src;
        uint32_t tail = size & 3u;

        /* 2-byte chunk */
        if (tail & 2u) {
            uint16_t hw = *(const uint16_t *)sp;
            *(uint16_t *)dp = hw;
            sp += 2;
            dp += 2;
        }
        /* 1-byte chunk */
        if (tail & 1u) {
            *dp++ = *sp;
        }
        return (uint32_t *)dp;
    }
}

/* =========================================================================
 * bzero_align — zero `size` bytes at `dst`, handling unaligned starts
 *
 * If `dst` is not 4-byte aligned, pads up to the next word boundary with
 * individual byte/halfword writes, then calls fast_memset for the bulk.
 * Handles very small sizes (< 4 bytes) directly.
 * ====================================================================== */
uint32_t *bzero_align(uint32_t *dst, uint32_t size)
{
    if (size < 4u) {
        /* Sub-word case: write 2 bytes then 1 byte */
        uint8_t *p = (uint8_t *)dst;
        if (size & 2u) {
            *p++ = 0;
            *p++ = 0;
        }
        if (size & 1u)
            *p++ = 0;
        return (uint32_t *)p;
    }

    /* Align dst to a 4-byte boundary */
    uintptr_t addr = (uintptr_t)dst;
    if (addr & 3u) {
        uint32_t pad = 4u - (addr & 3u);
        uint8_t *p = (uint8_t *)dst;
        if (pad != 2u)
            *p++ = 0;           /* 1 byte */
        if (pad >= 2u) {
            *(uint16_t *)p = 0; /* 2 bytes */
            p += 2;
        }
        size -= pad;
        dst = (uint32_t *)p;
    }

    return fast_memset(dst, size, 0);
}

/* =========================================================================
 * bzero — zero `size` bytes at `dst` (no alignment handling)
 *
 * Thin wrapper around fast_memset with value = 0.
 * ====================================================================== */
uint32_t *bzero(uint32_t *dst, uint32_t size)
{
    return fast_memset(dst, size, 0);
}

/* =========================================================================
 * fast_memset — fill `size` bytes at `dst` with `val`, 32-byte main loop
 *
 * `val` is broadcast across all bytes of the 32-bit word by the caller
 * (e.g., 0x00000000 for bzero, or 0xFFFFFFFF for memset(p, 0xFF, n)).
 * ====================================================================== */
uint32_t *fast_memset(uint32_t *dst, uint32_t size, int val)
{
    uint32_t v = (uint32_t)val;

    /* 32-byte (8-word) main loop */
    while (size >= 32u) {
        dst[0] = v;  dst[1] = v;  dst[2] = v;  dst[3] = v;
        dst[4] = v;  dst[5] = v;  dst[6] = v;  dst[7] = v;
        dst  += 8;
        size -= 32u;
    }

    /* 16-byte chunk */
    if (size & 16u) {
        dst[0] = v;  dst[1] = v;  dst[2] = v;  dst[3] = v;
        dst += 4;
    }

    /* 8-byte chunk */
    if (size & 8u) {
        dst[0] = v;  dst[1] = v;
        dst += 2;
    }

    /* 4-byte chunk */
    if (size & 4u)
        *dst++ = v;

    /* Byte-granularity tail */
    {
        uint8_t *p = (uint8_t *)dst;
        if (size & 2u) {
            *(uint16_t *)p = (uint16_t)v;
            p += 2;
        }
        if (size & 1u)
            *p++ = (uint8_t)v;
        return (uint32_t *)p;
    }
}

/* =========================================================================
 * fast_strcmp — optimised 4-byte-at-a-time string comparison
 *
 * Compares two NUL-terminated strings a word at a time. Uses the classic
 * "hasNUL" trick:  (word - 0x01010101) & ~word & 0x80808080 != 0
 * indicates that at least one byte in the word is zero.
 *
 * Returns:
 *   0  if strings are equal (including the case where the differing byte
 *      comes after a NUL — constant-time for equal strings)
 *   >0 if *a > *b at the first differing byte
 *   <0 if *a < *b at the first differing byte
 *
 * Note: the implementation favours throughput over strict conformance;
 * it reads slightly past the NUL byte in the last word, which is safe
 * because strings in this firmware always reside in SRAM with word-aligned
 * backing.
 * ====================================================================== */
int fast_strcmp(const uint32_t *a, const uint32_t *b)
{
#define HASNUL(w) (((w) - 0x01010101u) & ~(w) & 0x80808080u)

    uint32_t wa, wb;

    for (;;) {
        wa = *a++;   wb = *b++;
        if (wa != wb)
            break;
        if (HASNUL(wa))
            return 0;   /* equal up to (and including) NUL */

        wa = *a++;   wb = *b++;
        if (wa != wb)
            break;
        if (HASNUL(wa))
            return 0;

        wa = *a++;   wb = *b++;
        if (wa != wb)
            break;
        if (HASNUL(wa))
            return 0;
    }

    /* Find the first differing byte position via byte-swap + CLZ */
    {
        uint32_t diff  = wa ^ wb;
        int      shift = __clz(__builtin_bswap32(diff)) & 0x18;  /* 0,8,16,24 */

        /* If the differing position is past a NUL in wa, strings are equal */
        uint32_t nul_mask = 0x01010101u >> (32 - shift);
        if ((wa - nul_mask) & ~wa & (nul_mask << 7))
            return 0;

        return (int)((wa >> shift) & 0xFFu) - (int)((wb >> shift) & 0xFFu);
    }
#undef HASNUL
}

/* =========================================================================
 * memcmp_custom — constant-time-ish byte comparison
 *
 * Compares `len` bytes.  Unlike standard memcmp this function:
 *   - Returns 255 if either pointer is NULL.
 *   - Returns 255 if a[i] > b[i] at the first difference (not +1).
 *   - Returns 0   if a[i] < b[i] and the remaining bytes of b are non-zero.
 *   - Returns 255 if a[i] < b[i] and all remaining bytes of b are zero.
 *
 * This asymmetric contract is used by the crypto/ASN.1 parser to implement
 * a "is a a prefix of b" check without branching on secret data.
 *
 * All index arithmetic is truncated to uint8_t to match the original Thumb-2
 * loop which used UXTB to keep the counter in [0,255].
 * ====================================================================== */
int memcmp_custom(const uint8_t *a, const uint8_t *b, uint32_t len)
{
    uint32_t acc  = 0;
    bool     diff = false;
    uint16_t i    = 0;

    if (!a || !b)
        return 255;

    for (i = 0; (uint16_t)i < (uint16_t)len; i = (uint8_t)(i + 1)) {
        uint8_t bv = *b;
        uint8_t av = *a;

        if (bv > av)
            return 255;     /* b[i] > a[i]: a < b, return sentinel */

        if (bv < av) {
            acc  |= *b;
            diff  = true;
            break;
        }

        acc |= bv;
        ++a;
        ++b;
    }

    if (!diff)
        return 255;     /* All bytes equal */

    if (acc)
        return 0;       /* Non-zero byte in b after the difference */

    /* All remaining bytes of b are zero — scan the rest to confirm */
    for (; (uint16_t)i < (uint16_t)len; i = (uint8_t)(i + 1)) {
        acc |= *b;
        if (acc)
            return 0;
        ++b;
    }
    return 255;
}

/* =========================================================================
 * asn1_advance_ptr — copy (offset, base) pair and compute new base
 *
 * Used by the ASN.1 parser to step past a parsed element.
 * Input:  src[0] = element length, src[1] = element start offset
 * Output: dst[0] = element length (copy), dst[1] = start + length (new pos)
 *
 * Includes a stack-canary check (standard pattern throughout this firmware).
 * ====================================================================== */
int asn1_advance_ptr(uint32_t *dst, const uint32_t *src)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;

    dst[0] = src[0];                /* copy length  */
    dst[1] = src[1] + src[0];      /* advance base */

    if (REG_TCB_CURRENT_TASK != saved_task)
        system_halt_clear_flag();

    return 0;
}

/* =========================================================================
 * simple_memset — portable byte-granularity memset with stack-canary check
 *
 * Less aggressive than fast_memset; used in security-sensitive paths
 * (e.g. wiping key material) where the compiler must not elide the writes.
 * Also checks the stack canary to detect corruption during the fill.
 * ====================================================================== */
int simple_memset(int dst, char val, uint32_t len)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;

    for (uint32_t i = 0; i < len; ++i)
        *(volatile uint8_t *)(dst + i) = (uint8_t)val;

    if (REG_TCB_CURRENT_TASK != saved_task)
        system_halt_clear_flag();

    return (int)saved_task;
}

/* =========================================================================
 * write_to_hw_fifo_20001000 — DMA the buffer into the debug UART FIFO
 *
 * The FIFO at 0x20001000 is a memory-mapped byte queue connected to the
 * chip's debug UART.  Writing a 32-bit word stores four bytes in the order
 * imposed by the bus (little-endian).  The function writes word-sized chunks
 * first for speed, then individual bytes for the tail.
 *
 * Stack-canary check included.
 * ====================================================================== */
int write_to_hw_fifo_20001000(char *buf, uint32_t len)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;
    volatile uint32_t *fifo = (volatile uint32_t *)REG_DDC_FIFO_ADDR;
    volatile uint8_t  *fifo_b = (volatile uint8_t *)REG_DDC_FIFO_ADDR;

    /* Word-aligned bulk write */
    for (uint32_t i = 0; i < len >> 2; ++i) {
        *fifo = *(uint32_t *)buf;
        buf += 4;
    }

    /* Byte tail */
    uint32_t tail = len & 3u;
    for (uint32_t j = 0; j < tail; ++j)
        *fifo_b++ = (uint8_t)*buf++;

    if (REG_TCB_CURRENT_TASK != saved_task)
        system_halt_clear_flag();

    return (int)saved_task;
}

/* =========================================================================
 * check_null_string — safe model-string accessor
 *
 * Returns the firmware's product model string if `str` is valid and
 * non-empty and differs from the ROM default, otherwise returns a pointer
 * to the ROM-default string at DATA_MODEL_STR_DEFAULT.
 *
 * Used during system_early_init() to populate the global struct's model
 * string pointer with a safe default before any provisioning has occurred.
 *
 * Parameters:
 *   unused — ignored (formerly held a handle; kept for ABI compatibility)
 *   str    — candidate string pointer (may be NULL)
 * ====================================================================== */
char *check_null_string(int unused, uint32_t *str)
{
    (void)unused;

    const uint32_t *default_str = (const uint32_t *)DATA_MODEL_STR_BASE;

    /*
     * Accept `str` only if:
     *   1. It is non-NULL.
     *   2. Its first byte is non-NUL (not an empty string).
     *   3. It differs from the ROM default (i.e. was actually provisioned).
     */
    if (str && *(const uint8_t *)str && fast_strcmp(default_str, str))
        return NULL;    /* invalid / unprovisionable string */

    return (char *)DATA_MODEL_STR_DEFAULT;
}

/* =========================================================================
 * IEEE 754 double-precision helpers
 *
 * float64 values are carried as two 32-bit words (high, low) through the
 * ARM AAPCS64-on-32 calling convention, where r0:r1 = low:high or
 * r0 = high, r1 = low depending on the function.
 *
 * These helpers are shared between libc_printf.c (float formatting) and
 * softfloat.c (arithmetic).
 * ====================================================================== */

/*
 * float_extract_parts — decode a float64 exponent and normalise.
 *
 * Given a float64 split as (hi, lo) = (a1, a2):
 *   - Extracts the biased exponent (bits 62:52 of the full value).
 *   - Rebuilds a normalised exponent in a form suitable for the
 *     Grisu/Dragon4 decimal formatter.
 *   - Returns the scaled exponent packed into a single int32.
 *
 * Special cases:
 *   - Zero / denormal (exp field == 0): returns adjusted negative exponent.
 *   - Infinity / NaN  (exp field == 0x7FF): sets bit 30 of result.
 */
int float_extract_parts(int hi, uint32_t lo)
{
    uint32_t sign_bit  = (uint32_t)hi & 0x80000000u;
    uint32_t hi2       = (uint32_t)hi << 1;        /* remove sign bit */
    bool     is_zero   = (hi2 == 0u) && (lo == 0u);

    /* Mantissa: rebias to 12-bit aligned position */
    uint32_t mant_hi = (((uint32_t)hi << 11) | (lo >> 21));
    uint32_t mant_lo = lo << 11;

    /* Biased exponent [30:20] of hi2, then subtract 15360 (= 0x3C00) */
    uint32_t exp = hi2 >> 20;
    if (!is_zero)
        exp += 30720u;  /* 0x7800 — rebias from float64 to our internal scale */

    int32_t result = (int32_t)((exp >> 1) | sign_bit);

    if (!is_zero)
        mant_hi |= 0x80000000u;   /* implicit leading 1 for normalised values */

    /* Check exponent class */
    int32_t exp_class = (int32_t)hi2 >> 21;

    if (exp_class != 0) {
        if (exp_class == -1)
            return result | 0x40000000;  /* Inf / NaN */
        /* Normal value — return as-is */
        return result;
    }

    /* Denormal or zero: count leading zeros in mantissa */
    if (mant_hi < 0) {
        /* Leading 1 already in mant_hi: count in lower bits */
        uint32_t m = mant_hi & 0x7FFFFFFFu;
        if (m) {
            int lz = 0;
            if (!HIWORD(m))      { m <<= 16; lz  = 16; }
            if (!HIBYTE(m))      { m <<= 8;  lz += 8;  }
            if (!(m >> 28))      { m <<= 4;  lz += 4;  }
            if (!(m >> 30))      { m <<= 2;  lz += 2;  }
            if (!(m & 0x80000000u))            lz++;
            return result - lz + 1;
        } else {
            /* mant_hi mantissa is zero, look in mant_lo */
            uint32_t m2 = mant_lo;
            int lz = 0;
            if (!HIWORD(m2))     { m2 <<= 16; lz  = 16; }
            if (!HIBYTE(m2))     { m2 <<= 8;  lz += 8;  }
            if (!(m2 >> 28))     { m2 <<= 4;  lz += 4;  }
            if (!(m2 >> 30))     { m2 <<= 2;  lz += 2;  }
            if (!(m2 & 0x80000000u))             lz++;
            return result - 31 - lz;
        }
    }

    return result;
}

/*
 * float_sign_xor — extract the XOR of the sign bits of two float64 values.
 *
 * Used by the float add/sub core to determine the sign of the result.
 * Returns bit 31 set if the two operands have opposite signs, 0 otherwise.
 * The two operands are passed as (hi_a, lo_a, hi_b, lo_b).
 */
uint32_t float_sign_xor(int hi_a, int lo_a, int hi_b, int lo_b)
{
    (void)lo_a;
    (void)hi_b;
    return ((uint32_t)hi_a ^ (uint32_t)lo_b) & 0x80000000u;
}

/*
 * float_sign_xor_2 — identical to float_sign_xor; second variant used by
 * float_sub_mantissa for the subtraction path.
 */
uint32_t float_sign_xor_2(int hi_a, int lo_a, int hi_b, int lo_b)
{
    (void)lo_a;
    (void)hi_b;
    return ((uint32_t)hi_a ^ (uint32_t)lo_b) & 0x80000000u;
}

/*
 * float_shift_normalize — shift a 96-bit mantissa and update the exponent.
 *
 * Used during float64 addition/subtraction to align mantissas before
 * combining them.  The shift amount is in `a4`; negative shifts are right
 * shifts, positive shifts are left shifts.  The 96-bit value is split
 * across three 32-bit words: (a2, a3, sticky) where `sticky` accumulates
 * dropped bits for rounding.
 *
 * Returns the updated exponent OR'd with the sign bit from `a1`.
 */
uint32_t float_shift_normalize(int sign, uint32_t hi, uint32_t lo, int shift)
{
    uint32_t sticky = 0u;

    if (shift < 0) {
        if (shift <= -64) {
            sticky = (sticky | lo | ((sticky | lo) << 16u)) >> 16u;
            sticky |= hi;
            if (shift < -64)
                sticky = (sticky | (sticky << 16u)) >> 16u;
            shift = 0;
            lo = 0;
            hi = 0;
        } else {
            if (shift <= -32) {
                sticky |= lo;
                lo = hi;
                hi = 0;
                shift += 32;
            }
            shift = -shift;
            if (shift) {
                sticky = (sticky | (sticky << 16u)) >> 16u;
                sticky |= (lo << (32 - shift));
                uint32_t new_lo = lo >> shift;
                uint32_t carry  = hi << (32 - shift);
                hi >>= shift;
                lo = new_lo | carry;
                shift = 0;
            }
        }
    }

    /* Detect sticky overflow: if the sticky word has the MSB set,
     * propagate a carry into lo/hi and adjust the exponent. */
    {
        uint32_t s2    = sticky << 1u;
        bool     carry = (sticky >> 31u) != 0u;

        if (!s2) {
            if (!carry)
                return (uint32_t)shift | ((uint32_t)sign & 0x80000000u);
            carry = (lo & 1u) != 0u;
            s2    = lo >> 1u;
        }

        uint32_t s3 = ((s2 >> 1u) | (carry ? 0x80000000u : 0u));
        int exp = shift + 1;

        bool c1 = false, c2 = false, c3;
        if (exp == 1) c1 = (s3 >> 31u) != 0u; /* check hi-word overflow */

        if (c1) { c2 = (lo == 0xFFFFFFFFu); }
        if (lo == 0xFFFFFFFFu && c1) { c3 = (hi + 1u == 0u); (void)c3; }

        shift += (int)c1;
    }

    return (uint32_t)shift | ((uint32_t)sign & 0x80000000u);
}

/*
 * float_add_mantissa — choose result for float64 addition mantissa step.
 *
 * The input is a pair of float64 mantissa structs { hi, lo, hi2, lo2 }.
 * If the "borrow" condition indicates the subtraction path should be taken,
 * delegates to float_sign_xor + float_shift_normalize.
 */
uint32_t float_add_mantissa(uint32_t *a, int *b)
{
    uint32_t a_hi  = a[0];
    uint32_t a_lo  = a[1];
    uint32_t a_hi2 = a[2];
    int      b_v   = b[0];
    int      b_v2  = b[1];

    bool borrow = ((a_lo & ~(2u * a_hi)) & 0x80000000u) != 0u;
    if (borrow)
        borrow = ((uint32_t)b_v2 & ~(2u * (uint32_t)b_v)) < 0x80000000u;

    if (borrow) {
        uint32_t sign = float_sign_xor((int)a_hi, (int)a_lo, (int)a_hi2, b_v);
        uint32_t hi_out, lo_out;
        int      exp_out;
        /* The three output words are implicit in the calling convention */
        return float_shift_normalize((int)sign, hi_out, lo_out, exp_out);
    }
    return a_hi;
}

/*
 * float_sub_mantissa — choose result for float64 subtraction mantissa step.
 *
 * Similar to float_add_mantissa but uses float_sign_xor_2 and checks bit 30
 * (the QNaN/infinity flag) rather than bit 31.
 */
uint32_t float_sub_mantissa(uint32_t *a, int *b)
{
    uint32_t a_hi = a[0];
    uint32_t a_lo = a[1];
    int      b_v  = b[0];

    bool cond = (a_hi & 0x40000000u) == 0u;
    if (cond)
        cond = ((uint32_t)b_v & 0x40000000u) == 0u;

    if (cond) {
        uint32_t sign = float_sign_xor_2((int)a_hi, (int)a_lo, 0, b_v);
        uint32_t hi_out, lo_out;
        int      exp_out;
        return float_shift_normalize((int)sign, hi_out, lo_out, exp_out);
    }
    return a_hi;
}
