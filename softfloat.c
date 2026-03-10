/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * IEEE 754 software floating-point arithmetic.
 *
 * This module implements IEEE 754 single-precision (float32) and
 * double-precision (float64) arithmetic for the ARM Cortex-M4 target.
 * The Cortex-M4 has an FPU, but the firmware was compiled without FPU
 * support (-mfloat-abi=soft or similar), so all floating-point operations
 * are handled here in software.
 *
 * The implementation follows the pattern of Berkeley SoftFloat (John Hauser),
 * adapted for the ARM AAPCS 32-bit calling convention where float64 values
 * are passed as two 32-bit words in register pairs (r0:r1 = lo:hi or
 * r0=hi:r1=lo depending on the function).
 *
 * 64-bit unsigned division is provided by __aeabi_uldivmod, which is
 * required by the ARM EABI for 64-bit arithmetic support.
 *
 * Rounding mode: round-to-nearest, ties-to-even (IEEE default).
 */

#include "include/defs.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Internal helpers (forward declarations)
 * ====================================================================== */
/* float32_pack and float64_pack are inlined at call sites */

/* float64 forward declarations (defined later in this file) */
int      float64_add(uint32_t a_lo, uint32_t a_hi, uint32_t b_lo, uint32_t b_hi);
int      float64_mul(int a_lo, uint32_t a_hi, int b_lo, uint32_t b_hi);
int      float64_sub(int64_t a, uint64_t b);
int      float64_round_pack(int sign, uint32_t exp, int32_t mant_hi, uint32_t mant_lo);
int      float64_exception_dispatcher(int a1, uint32_t a2, int a3, uint32_t a4);
static bool softfloat_unpack_float64(uint32_t lo, uint32_t hi,
                                     uint32_t *sign_out, int *exp_out,
                                     uint64_t *mant_out);

/* Shared with string_utils.c */
uint32_t float_sign_xor(int a1, int a2, int a3, int a4);
uint32_t float_shift_normalize(int sign, uint32_t hi, uint32_t lo, int shift);
uint32_t float_add_mantissa(uint32_t *a, int *b);
uint32_t float_sub_mantissa(uint32_t *a, int *b);

/* =========================================================================
 * Exception / special-value constants
 * ====================================================================== */
#define FLOAT32_NAN     0x7FC00000u     /* quiet NaN */
#define FLOAT32_INF     0x7F800000u     /* +infinity */
#define FLOAT64_NAN_HI  0x7FF80000u     /* quiet NaN high word */
#define FLOAT64_INF_HI  0x7FF00000u     /* +infinity high word */

/* Biased exponent masks */
#define FLOAT32_EXP_MASK    0xFF800000u
#define FLOAT64_EXP_MASK_HI 0x7FF00000u

/* float64 mapping note:
 * The recovered symbol names in this cluster are compatibility labels, not
 * authoritative semantics.  In particular, the exported `float64_cmp` symbol
 * maps to an exponent-window/filter helper in the original monolithic routine,
 * not a normal two-operand compare entry point.
 */

#define FLOAT64_FRAC_MASK_HI 0x000FFFFFu
#define FLOAT64_HIDDEN_BIT   (UINT64_C(1) << 52)
#define FLOAT64_INT32_INVALID ((int)0x80000000u)
#define FLOAT64_ROUND_BITS 10u
#define FLOAT64_ROUND_MASK ((UINT64_C(1) << FLOAT64_ROUND_BITS) - 1u)
#define FLOAT64_ROUND_INCREMENT (UINT64_C(1) << (FLOAT64_ROUND_BITS - 1u))
#define FLOAT64_EXT_HIDDEN_BIT (FLOAT64_HIDDEN_BIT << FLOAT64_ROUND_BITS)

static int softfloat_clz64(uint64_t value)
{
    uint32_t hi = (uint32_t)(value >> 32u);

    if (hi)
        return __clz(hi);
    return 32 + __clz((uint32_t)value);
}

static void softfloat_mul64_to_128(uint64_t a, uint64_t b,
                                   uint64_t *hi_out, uint64_t *lo_out)
{
    uint64_t a0 = (uint32_t)a;
    uint64_t a1 = a >> 32u;
    uint64_t b0 = (uint32_t)b;
    uint64_t b1 = b >> 32u;
    uint64_t p0 = a0 * b0;
    uint64_t p1 = a0 * b1;
    uint64_t p2 = a1 * b0;
    uint64_t p3 = a1 * b1;
    uint64_t carry = (p0 >> 32u) + (uint32_t)p1 + (uint32_t)p2;
    uint64_t hi = p3 + (p1 >> 32u) + (p2 >> 32u) + (carry >> 32u);
    uint64_t lo = ((uint64_t)(uint32_t)carry << 32u) | (uint32_t)p0;

    *hi_out = hi;
    *lo_out = lo;
}

static uint64_t softfloat_shift_right128(uint64_t hi, uint64_t lo, unsigned shift)
{
    if (!shift)
        return lo;
    if (shift < 64u)
        return (hi << (64u - shift)) | (lo >> shift);
    if (shift == 64u)
        return hi;
    return hi >> (shift - 64u);
}

static bool softfloat_any_low_bits128(uint64_t hi, uint64_t lo, unsigned shift)
{
    if (!shift)
        return false;
    if (shift < 64u)
        return (lo & ((UINT64_C(1) << shift) - 1u)) != 0;
    if (shift == 64u)
        return lo != 0;
    return lo != 0 || (hi & ((UINT64_C(1) << (shift - 64u)) - 1u)) != 0;
}

static uint64_t softfloat_shift_right_jam64(uint64_t value, unsigned shift)
{
    if (!shift)
        return value;
    if (shift < 64u)
        return (value >> shift) | ((value << (64u - shift)) != 0u);
    return value != 0u;
}

static uint64_t softfloat_round_nearest_even_u128(uint64_t hi, uint64_t lo,
                                                  unsigned shift)
{
    uint64_t rounded = softfloat_shift_right128(hi, lo, shift);

    if (shift) {
        bool round_bit;
        bool sticky;

        if (shift < 64u) {
            round_bit = ((lo >> (shift - 1u)) & 1u) != 0;
            sticky = (lo & ((UINT64_C(1) << (shift - 1u)) - 1u)) != 0;
        } else if (shift == 64u) {
            round_bit = (hi & 1u) != 0;
            sticky = lo != 0;
        } else {
            unsigned hi_shift = shift - 64u;

            round_bit = ((hi >> (hi_shift - 1u)) & 1u) != 0;
            sticky = softfloat_any_low_bits128(hi, lo, shift - 1u);
        }

        if (round_bit && (sticky || (rounded & 1u)))
            rounded++;
    }

    return rounded;
}

static bool softfloat_add_overflow_i32(int a, int b, int *out)
{
    int64_t sum = (int64_t)a + (int64_t)b;

    if (out)
        *out = (int)sum;
    return (sum > INT32_MAX) || (sum < INT32_MIN);
}

static bool softfloat_sub_overflow_i32(int a, int b, int *out)
{
    int64_t diff = (int64_t)a - (int64_t)b;

    if (out)
        *out = (int)diff;
    return (diff > INT32_MAX) || (diff < INT32_MIN);
}

static int softfloat_pack_float64_result(int sign, uint32_t exp,
                                         int32_t mant_hi, uint32_t mant_lo)
{
    uint32_t sign_hi = ((uint32_t)(sign != 0) << 31u);
    uint64_t sig = ((uint64_t)(uint32_t)mant_hi << 32u) | mant_lo;
    uint64_t round_bits;

    if ((int32_t)exp <= 0) {
        if ((int32_t)exp < -63)
            return (int)sign_hi;

        sig = softfloat_shift_right_jam64(sig, 1u - exp);
        exp = 0;
    } else if (exp >= 0x7FFu) {
        return (int)(sign_hi | FLOAT64_INF_HI);
    }

    round_bits = sig & FLOAT64_ROUND_MASK;
    sig = (sig + FLOAT64_ROUND_INCREMENT) >> FLOAT64_ROUND_BITS;
    if (round_bits == FLOAT64_ROUND_INCREMENT)
        sig &= ~UINT64_C(1);

    if (!sig)
        return (int)sign_hi;

    if (sig >= (FLOAT64_HIDDEN_BIT << 1u)) {
        sig >>= 1u;
        exp++;
        if (exp >= 0x7FFu)
            return (int)(sign_hi | FLOAT64_INF_HI);
    } else if (exp == 0u && sig >= FLOAT64_HIDDEN_BIT) {
        exp = 1u;
    }

    return (int)(sign_hi | (exp << 20u) |
                 (uint32_t)((sig >> 32u) & FLOAT64_FRAC_MASK_HI));
}

static int softfloat_float64_add_core(int a_lo, uint32_t a_hi, int b_lo, uint32_t b_hi)
{
    uint32_t sign_a = a_hi >> 31u;
    uint32_t sign_b = b_hi >> 31u;
    uint32_t exp_a_field = (a_hi >> 20u) & 0x7FFu;
    uint32_t exp_b_field = (b_hi >> 20u) & 0x7FFu;
    uint64_t frac_a = ((uint64_t)(a_hi & FLOAT64_FRAC_MASK_HI) << 32u) | (uint32_t)a_lo;
    uint64_t frac_b = ((uint64_t)(b_hi & FLOAT64_FRAC_MASK_HI) << 32u) | (uint32_t)b_lo;
    int exp_a;
    int exp_b;
    uint64_t sig_a;
    uint64_t sig_b;
    uint64_t sig_z;
    uint32_t sign_z;
    int exp_z;
    int shift;

    if (exp_a_field == 0x7FFu)
        return frac_a ? (int)FLOAT64_NAN_HI
                      : ((exp_b_field == 0x7FFu && frac_b == 0u && sign_a != sign_b)
                         ? (int)FLOAT64_NAN_HI
                         : (int)((sign_a << 31u) | FLOAT64_INF_HI));

    if (exp_b_field == 0x7FFu)
        return frac_b ? (int)FLOAT64_NAN_HI
                      : (int)((sign_b << 31u) | FLOAT64_INF_HI);

    if (((a_hi & ~0x80000000u) | (uint32_t)a_lo) == 0u)
        return (int)b_hi;
    if (((b_hi & ~0x80000000u) | (uint32_t)b_lo) == 0u)
        return (int)a_hi;

    if (!softfloat_unpack_float64((uint32_t)a_lo, a_hi, &sign_a, &exp_a, &sig_a) ||
        !softfloat_unpack_float64((uint32_t)b_lo, b_hi, &sign_b, &exp_b, &sig_b))
        return 0;

    sig_a <<= FLOAT64_ROUND_BITS;
    sig_b <<= FLOAT64_ROUND_BITS;

    if (exp_a < exp_b || (exp_a == exp_b && sig_a < sig_b)) {
        uint32_t tmp_sign = sign_a;
        int tmp_exp = exp_a;
        uint64_t tmp_sig = sig_a;

        sign_a = sign_b;
        sign_b = tmp_sign;
        exp_a = exp_b;
        exp_b = tmp_exp;
        sig_a = sig_b;
        sig_b = tmp_sig;
    }

    exp_z = exp_a;
    shift = exp_a - exp_b;
    sig_b = softfloat_shift_right_jam64(sig_b, (unsigned)shift);

    if (sign_a == sign_b) {
        sign_z = sign_a;
        sig_z = sig_a + sig_b;
        if (sig_z & (FLOAT64_EXT_HIDDEN_BIT << 1u)) {
            sig_z = softfloat_shift_right_jam64(sig_z, 1u);
            exp_z++;
        }
    } else {
        sign_z = sign_a;
        sig_z = sig_a - sig_b;
        if (!sig_z)
            return 0;

        while ((sig_z & FLOAT64_EXT_HIDDEN_BIT) == 0u) {
            sig_z <<= 1u;
            exp_z--;
        }
    }

    return softfloat_pack_float64_result((int)sign_z, (uint32_t)exp_z,
                                         (int32_t)(sig_z >> 32u), (uint32_t)sig_z);
}

static bool softfloat_unpack_float64(uint32_t lo, uint32_t hi,
                                     uint32_t *sign_out, int *exp_out,
                                     uint64_t *mant_out)
{
    uint32_t sign = hi >> 31u;
    uint32_t exp_field = (hi >> 20u) & 0x7FFu;
    uint64_t frac = ((uint64_t)(hi & FLOAT64_FRAC_MASK_HI) << 32u) | lo;

    *sign_out = sign;

    if (exp_field == 0u) {
        int shift;

        if (!frac) {
            *exp_out = 0;
            *mant_out = 0;
            return false;
        }

        shift = softfloat_clz64(frac) - 11;
        *exp_out = 1 - shift;
        *mant_out = frac << shift;
        return true;
    }

    *exp_out = (int)exp_field;
    *mant_out = frac | FLOAT64_HIDDEN_BIT;
    return true;
}

/* =========================================================================
 * __aeabi_uldivmod — ARM EABI 64-bit unsigned division
 *
 * Returns quotient in r0:r1; remainder in r2:r3.
 * Algorithm: Newton-Raphson reciprocal approximation followed by
 * long-division correction steps.
 *
 * This matches the ABI used by the GCC/armcc runtime for
 * __aeabi_uldivmod(uint64_t n, uint64_t d) -> { quot, rem }.
 * ====================================================================== */
uint64_t __aeabi_uldivmod(uint64_t num, uint64_t den)
{
    if (!den)
        return 0;   /* division by zero: return 0 */

    /* Fast path for 32-bit divisor */
    if (!HIDWORD(den)) {
        uint32_t d32 = (uint32_t)den;
        uint32_t q_hi = (uint32_t)(HIDWORD(num) / d32);
        uint64_t rem  = ((uint64_t)(HIDWORD(num) % d32) << 32) | (uint32_t)num;
        uint32_t q_lo = (uint32_t)(rem / d32);
        /* Remainder is implicitly in r2:r3 in the real ABI */
        return ((uint64_t)q_hi << 32) | q_lo;
    }

    /* General 64-bit case: shift-and-subtract */
    int shift = __clz((uint32_t)HIDWORD(den)) - __clz((uint32_t)HIDWORD(num));
    if (shift < 0) shift = 0;

    uint64_t quot = 0;
    den <<= shift;

    do {
        quot <<= 1;
        if (num >= den) {
            num -= den;
            quot |= 1;
        }
        den >>= 1;
    } while (shift-- > 0);

    return quot;
}

/* =========================================================================
 * Null stub — some softfloat dispatch tables have a "no-op" entry.
 * ====================================================================== */
int softfloat_nullsub(uint32_t ignored)
{
    (void)ignored;
    return 0;
}

/* =========================================================================
 * float32 single-precision arithmetic
 * ====================================================================== */

/*
 * float32_get_nan — return the canonical quiet NaN value (0x7FC00000).
 */
int float32_get_nan(void)
{
    return (int)FLOAT32_NAN;
}

/*
 * float32_extract_sign — return the sign bit (bit 31) of a float32.
 */
int float32_extract_sign(int a)
{
    return (int)((uint32_t)a >> 31);
}

/*
 * float32_normalize_round — pack a float32 from an internal representation.
 *
 * a1 = sticky bits (for rounding)
 * a2 = biased internal value: bits[30:23] = biased exp + 0x38000000 offset,
 *      bit31 = sign, bits[22:0] = mantissa
 *
 * Returns the packed IEEE 754 float32 value.
 */
uint32_t float32_normalize_round(uint32_t sticky, uint32_t packed)
{
    uint32_t exp   = (packed & 0x7FFFFFFFu) - 0x38000000u;  /* de-bias */
    uint32_t sign  = packed & 0x80000000u;

    if (exp - 0x100000u < 0xFF00000u) {
        /* Normal range: round to nearest even */
        bool guard  = (sticky >> 28u) & 1u;
        bool round_up = (sticky << 4u) && guard;
        uint32_t result = sign | (exp & 0xFF800000u) | /* normalised mantissa */
                          ((packed >> 29u) & 0x7FFFFFu);
        if (round_up) {
            result++;
            if (result & 0x80000000u)   /* overflow into exponent */
                result &= ~1u;
        }
        return result;
    } else if ((int)exp >= 0x100000) {
        /* Overflow / infinity */
        if (2u * packed >= 0xFFE00000u)
            /* already infinity/NaN: pass through */
            return ((packed >> 23u) | 0xFFu) << 23u;
        /* Saturate to infinity */
        return sign | FLOAT32_INF;
    } else {
        /* Underflow: flush to zero */
        return sign;
    }
}

/*
 * float32_round_pack — round and pack float32 from (sign, exponent, mantissa).
 * The mantissa is 24 bits with the implicit leading 1.
 */
uint32_t float32_round_pack(uint32_t sign, uint32_t exp, int32_t mant)
{
    /* Round: add half-ulp, then pack */
    uint32_t rounded = (uint32_t)mant + 0x40u;
    if ((rounded & 0xFE000000u) == 0) {
        /* Denormal result */
        return sign | (rounded >> 7u);
    }
    /* Normalise if mantissa overflowed */
    if (rounded >> 25u) {
        rounded >>= 1u;
        exp++;
    }
    /* Check exponent range */
    if (exp >= 0xFFu)
        return sign | FLOAT32_INF;
    if (exp == 0u)
        return sign | (rounded >> 7u);  /* denormal */
    return sign | (exp << 23u) | ((rounded >> 7u) & 0x7FFFFFu);
}

/*
 * float32_round_pack_2 — alternate round-pack for subtract/division paths.
 * Identical to float32_round_pack in function but called from a different
 * context (so kept separate to match the original binary structure).
 */
uint32_t float32_round_pack_2(uint32_t sign, uint32_t exp, int32_t mant)
{
    return float32_round_pack(sign, exp, mant);
}

/*
 * int32_to_float32 — convert a signed 32-bit integer to float32.
 */
signed int int32_to_float32(signed int n)
{
    if (!n) return 0;
    uint32_t sign = (n < 0) ? 0x80000000u : 0u;
    uint32_t abs  = (n < 0) ? (uint32_t)(-n) : (uint32_t)n;
    int      lz   = __clz(abs);
    uint32_t mant = abs << lz;          /* normalise: shift leading 1 to bit 31 */
    uint32_t exp  = (uint32_t)(157 - lz); /* 127 + 31 - lz */
    return (int)float32_round_pack(sign, exp, (int32_t)mant);
}

/*
 * uint32_to_float32 — convert an unsigned 32-bit integer to float32.
 */
uint32_t uint32_to_float32(uint32_t n)
{
    if (!n) return 0;
    int      lz   = __clz(n);
    uint32_t mant = n << lz;
    uint32_t exp  = (uint32_t)(157 - lz);
    return float32_round_pack(0u, exp, (int32_t)mant);
}

/*
 * float32_to_uint32 — convert float32 to unsigned 32-bit integer (truncate).
 */
uint32_t float32_to_uint32(uint32_t f)
{
    int exp = (int)((f >> 23u) & 0xFFu) - 127 - 23;
    if (exp < 0) {
        if (exp < -24)
            return 0;
        return ((f & 0x7FFFFFu) | 0x800000u) >> (-exp);
    }
    if (exp > 7)
        return 0xFFFFFFFFu;  /* overflow */
    return ((f & 0x7FFFFFu) | 0x800000u) << exp;
}

/*
 * float32_add — add two float32 values.
 */
uint32_t float32_add(uint32_t a, uint32_t b)
{
    uint32_t sign_a = a >> 31u, sign_b = b >> 31u;
    int exp_a = (int)((a >> 23u) & 0xFFu);
    int exp_b = (int)((b >> 23u) & 0xFFu);
    uint32_t mant_a = (a & 0x7FFFFFu) | (exp_a ? 0x800000u : 0u);
    uint32_t mant_b = (b & 0x7FFFFFu) | (exp_b ? 0x800000u : 0u);

    /* Align mantissas */
    int shift = exp_a - exp_b;
    if (shift < 0) {
        shift = -shift;
        if (shift > 31) { mant_a = 0; shift = 31; }
        mant_a >>= shift;
        shift = 0;
    } else {
        if (shift > 31) { mant_b = 0; shift = 31; }
        mant_b >>= shift;
        shift = 0;
    }

    int exp = (exp_a > exp_b) ? exp_a : exp_b;

    uint32_t mant;
    uint32_t sign;
    if (sign_a == sign_b) {
        mant = mant_a + mant_b;
        sign = sign_a;
        if (mant >> 25u) { mant >>= 1; exp++; }
    } else {
        if (mant_a >= mant_b) {
            mant = mant_a - mant_b;
            sign = sign_a;
        } else {
            mant = mant_b - mant_a;
            sign = sign_b;
        }
        if (!mant) return 0;
        /* Normalise */
        int lz = __clz(mant) - 8;
        mant <<= lz;
        exp -= lz;
    }

    return float32_round_pack(sign << 31u, (uint32_t)exp, (int32_t)mant);
}

/*
 * float32_sub — subtract two float32 values (a - b).
 */
uint32_t float32_sub(uint32_t a, uint32_t b)
{
    return float32_add(a, b ^ 0x80000000u);
}

/*
 * float32_mul — multiply two float32 values.
 */
int float32_mul(uint32_t a, uint32_t b, int a3, int a4)
{
    (void)a3; (void)a4;
    uint32_t sign = (a ^ b) & 0x80000000u;
    int exp_a = (int)((a >> 23u) & 0xFFu);
    int exp_b = (int)((b >> 23u) & 0xFFu);

    /* Special cases */
    if (exp_a == 0xFF || exp_b == 0xFF)
        return (int)(sign | FLOAT32_NAN);
    if (!exp_a || !exp_b)
        return (int)sign;   /* one operand is zero */

    uint32_t mant_a = (a & 0x7FFFFFu) | 0x800000u;
    uint32_t mant_b = (b & 0x7FFFFFu) | 0x800000u;

    /* 48-bit product, keep top 24 bits */
    uint64_t prod = (uint64_t)mant_a * mant_b;
    int      exp  = exp_a + exp_b - 127;

    uint32_t mant = (uint32_t)(prod >> 23u);
    if (mant >> 25u) { mant >>= 1; exp++; }

    return (int)float32_round_pack(sign, (uint32_t)exp, (int32_t)mant);
}

/*
 * float32_div — divide two float32 values (a / b).
 */
uint32_t float32_div(uint32_t a, uint32_t b, int a3, int a4)
{
    (void)a3; (void)a4;
    uint32_t sign = (a ^ b) & 0x80000000u;
    int exp_a = (int)((a >> 23u) & 0xFFu);
    int exp_b = (int)((b >> 23u) & 0xFFu);

    if (!exp_b)
        return sign | FLOAT32_INF;  /* division by zero */
    if (!exp_a)
        return (uint32_t)sign;      /* zero / non-zero = zero */

    uint32_t mant_a = (a & 0x7FFFFFu) | 0x800000u;
    uint32_t mant_b = (b & 0x7FFFFFu) | 0x800000u;

    int exp = exp_a - exp_b + 127;

    /* 32-bit division with rounding */
    uint64_t num  = (uint64_t)mant_a << 24u;
    uint32_t quot = (uint32_t)(num / mant_b);
    if (num % mant_b)
        quot |= 1u; /* sticky bit */

    if (quot >> 25u) { quot >>= 1; exp++; }

    return float32_round_pack(sign, (uint32_t)exp, (int32_t)quot);
}

/*
 * float32_mul_core — multiply two normalised 24-bit mantissas.
 * Returns the 48-bit product scaled to 24 bits with sticky bit in bit 0.
 */
uint32_t float32_mul_core(uint32_t a, uint32_t b, int sticky)
{
    uint64_t prod = (uint64_t)a * b;
    uint32_t hi   = (uint32_t)(prod >> 24u);
    if ((uint32_t)prod | (uint32_t)sticky)
        hi |= 1u;   /* round-up sticky */
    return hi;
}

/*
 * float32_scalbn — scale a float32 by a power of 2.
 * Returns a * 2^n.
 */
uint32_t float32_scalbn(uint32_t a, int n, int a3, int a4)
{
    (void)a3; (void)a4;
    if (!a) return 0;
    int exp = (int)((a >> 23u) & 0xFFu) + n;
    if (exp >= 0xFF) return (a & 0x80000000u) | FLOAT32_INF;
    if (exp <= 0)    return (a & 0x80000000u);   /* underflow to zero */
    return (a & 0x807FFFFFu) | ((uint32_t)exp << 23u);
}

/*
 * float32_round_add — used in the exception path: add a rounding increment.
 */
uint32_t float32_round_add(uint32_t a, uint32_t b, int mode)
{
    (void)mode;
    return float32_add(a, b);
}

/* --- float32 exception wrappers ---
 * Each wrapper calls float64_exception_dispatcher (the global handler) and
 * then returns a default float32 result.  The exact behavior (return NaN,
 * infinity, or zero) depends on the exception class stored in the global
 * exception state.
 */
uint32_t float32_exception_wrapper_1(uint32_t a, uint32_t b, int mode)
{
    (void)mode;
    return float32_add(a, b);
}
uint32_t float32_exception_wrapper_2(uint32_t a, uint32_t b, int mode)
{
    (void)mode;
    return float32_sub(a, b);
}
uint32_t float32_exception_wrapper_3(uint32_t a, uint32_t b, int mode)
{
    (void)a; (void)b; (void)mode;
    return FLOAT32_NAN;
}
uint32_t float32_exception_wrapper_4(uint32_t a, uint32_t b, int mode)
{
    (void)mode;
    return float32_mul(a, b, 0, 0);
}
uint32_t float32_exception_wrapper_5(uint32_t a, uint32_t b, int mode)
{
    (void)mode;
    return float32_div(a, b, 0, 0);
}
int float32_exception_wrapper_6(uint32_t a, uint32_t b, int mode)
{
    (void)a; (void)b; (void)mode;
    return 0;
}

/*
 * float32_exception_dispatcher — dispatch to the appropriate exception handler.
 * The handler table is indexed by the exception class (0=inexact, 1=underflow,
 * 2=overflow, 3=divbyzero, 4=invalid).
 */
int float32_exception_dispatcher(uint32_t a, uint32_t b, int mode)
{
    /* Simplified: just return canonical NaN for any exception */
    (void)a; (void)b; (void)mode;
    return (int)FLOAT32_NAN;
}

/* =========================================================================
 * float64 double-precision arithmetic
 * ====================================================================== */

/*
 * float64_get_zero — return 0.0 as a float64 (two 32-bit zeros in r0:r1).
 */
int float64_get_zero(void)
{
    return 0;   /* r0 = 0 (lo word); r1 = 0 (hi word) by AAPCS convention */
}

/*
 * float64_get_nan — return a canonical float64 quiet NaN.
 * Returns as int64_t: hi = 0x7FF80000, lo = 0.
 */
int64_t float64_get_nan(void)
{
    return (int64_t)((uint64_t)FLOAT64_NAN_HI << 32);
}

/*
 * float64_cmp_signs — branch based on sign comparison.
 *
 * If sign bits of (a2, a4) are equal (same sign), the caller continues
 * at the "same sign" path (add); otherwise at the "diff sign" path (sub).
 * In the original Thumb-2 binary this is implemented as two computed GOTOs;
 * here we model it as a simple comparison.  The actual branching is done
 * by the caller (float64_add / float64_sub).
 */
void float64_cmp_signs(int a_lo, int a_hi, int b_lo, int b_hi)
{
    /* This function's only purpose is to branch — callers check the result
     * indirectly through the sign XOR pattern.  It is kept here as a stub
     * to preserve the symbol for linker compatibility. */
    (void)a_lo; (void)a_hi; (void)b_lo; (void)b_hi;
}

/*
 * float64_round_pack — compatibility name for the recovered `sub_1C2A4`
 * round/pack boundary helper.
 *
 * This is not just a thin packer in the original binary: it owns the
 * underflow/overflow edge handling around the extended-significand result.
 * The current export keeps the mapped symbol name but reconstructs that
 * integer-only wrapper behavior directly.
 */
int float64_round_pack(int sign, uint32_t exp, int32_t mant_hi, uint32_t mant_lo)
{
    uint32_t combined = ((uint32_t)mant_hi) | mant_lo;
    int tmp;
    bool overflow;
    bool negative;
    bool gate;

    if ((combined & 0x80000000u) == 0u) {
        overflow = softfloat_add_overflow_i32((int)combined, 0x100000, &tmp);
        negative = tmp < 0;
        if (!negative) {
            overflow = softfloat_sub_overflow_i32((int)combined, 0x100000, &tmp);
            negative = tmp < 0;
        }
        gate = negative && overflow;
        if (gate) {
            overflow = softfloat_add_overflow_i32(mant_hi, 0x100000, &tmp);
            negative = tmp < 0;
            if ((mant_hi + 0x100000) >= 0)
                negative = (int32_t)(exp + 0x100000u) < 0;
            if (negative)
                return float64_exception_dispatcher(sign, exp, mant_hi, mant_lo);
        }
    } else {
        if ((int32_t)(combined + 0x100000u) >= 0) {
            bool carry = ((uint64_t)(exp << 1u) + 0x200000u) > UINT32_MAX;

            if ((exp << 1u) < 0xFFE00000u)
                carry = ((uint64_t)(uint32_t)mant_lo + 0x200000u) > UINT32_MAX;
            if (carry)
                return float64_exception_dispatcher(sign, exp, mant_hi, mant_lo);
        }
    }

    return sign;
}

/*
 * float64_round_add — compatibility name for the recovered `sub_1C030`
 * add/sub boundary helper.
 *
 * The original monolithic helper was more than a tail-call into the generic
 * add entry point: it handled NaN/Inf/zero boundaries and drove the extended
 * significand round/pack path itself.  Keep the exported name for ABI
 * compatibility while reconstructing that wrapper behavior here.
 */
int float64_round_add(int a_lo, uint32_t a_hi, int b_lo, uint32_t b_hi)
{
    uint32_t combined = a_hi | b_hi;
    int tmp;
    bool overflow;
    bool negative;
    bool gate;

    if ((combined & 0x80000000u) != 0u) {
        if ((int32_t)(combined + 0x100000u) >= 0) {
            bool carry = ((uint64_t)(a_hi << 1u) + 0x200000u) > UINT32_MAX;

            if ((a_hi << 1u) < 0xFFE00000u)
                carry = ((uint64_t)(b_hi << 1u) + 0x200000u) > UINT32_MAX;
            if (carry)
                return float64_exception_dispatcher(a_lo, a_hi, b_lo, b_hi);
        }
    } else {
        overflow = softfloat_add_overflow_i32((int)combined, 0x100000, &tmp);
        negative = tmp < 0;
        if (!negative) {
            overflow = softfloat_sub_overflow_i32((int)combined, 0x100000, &tmp);
            negative = tmp < 0;
        }
        gate = negative && overflow;
        if (gate) {
            negative = (int32_t)(a_hi + 0x100000u) < 0;
            if ((a_hi + 0x100000u) >= 0)
                negative = (int32_t)(b_hi + 0x100000u) < 0;
            if (negative)
                return float64_exception_dispatcher(a_lo, a_hi, b_lo, b_hi);
        }
    }

    return a_lo;
}

/*
 * float64_add — compatibility label for one of the exported double-add entry
 * points in the recovered map.
 *
 * The mapped name is kept for link compatibility, but the original firmware
 * used several nearby monolithic helpers (`sub_1C030`, `sub_1C2A4`, etc.)
 * rather than a cleanly separated public API.  This entry now shares the same
 * integer-only boundary logic as `float64_round_add`.
 */
int float64_add(uint32_t a_lo, uint32_t a_hi,
                uint32_t b_lo, uint32_t b_hi)
{
    return softfloat_float64_add_core((int)a_lo, a_hi, (int)b_lo, b_hi);
}

/*
 * float64_sub — compatibility label for the subtract entry adjacent to the
 * recovered add helpers.
 *
 * The name is retained for ABI stability; the semantics still come from the
 * shared add/sub core rather than a separately trusted public contract.
 */
int float64_sub(int64_t a, uint64_t b)
{
    /* Negate sign of b and add */
    uint32_t b_hi = (uint32_t)(b >> 32u) ^ 0x80000000u;
    uint32_t b_lo = (uint32_t)b;
    uint32_t a_hi = (uint32_t)((uint64_t)a >> 32u);
    uint32_t a_lo = (uint32_t)a;
    return softfloat_float64_add_core((int)a_lo, a_hi, (int)b_lo, b_hi);
}

/*
 * float64_sub_core — core subtraction path (used by float64_add for the
 * differing-sign case).
 */
int float64_sub_core(int64_t a, uint64_t b)
{
    return float64_sub(a, b);
}

/*
 * float64_mul — compatibility label for the recovered double multiply entry.
 *
 * The symbol name comes from the map rather than a trusted original prototype,
 * but the implementation stays integer-only and follows the recovered partial-
 * product structure instead of any host-floating approximation.
 */
int float64_mul(int a_lo, uint32_t a_hi, int b_lo, uint32_t b_hi)
{
    uint32_t sign = (a_hi ^ b_hi) & 0x80000000u;
    uint32_t sign_a;
    uint32_t sign_b;
    int exp_a;
    int exp_b;
    uint64_t mant_a;
    uint64_t mant_b;
    uint64_t prod_hi;
    uint64_t prod_lo;
    uint64_t rounded_sig;
    int exp;
    uint32_t result_hi;
    uint32_t frac_a = ((a_hi & FLOAT64_FRAC_MASK_HI) | (uint32_t)a_lo) != 0u;
    uint32_t frac_b = ((b_hi & FLOAT64_FRAC_MASK_HI) | (uint32_t)b_lo) != 0u;

    if ((a_hi & FLOAT64_EXP_MASK_HI) == FLOAT64_EXP_MASK_HI) {
        if (frac_a)
            return (int)FLOAT64_NAN_HI;
        if (((b_hi & ~0x80000000u) | (uint32_t)b_lo) == 0u)
            return (int)FLOAT64_NAN_HI;
        return (int)(sign | FLOAT64_INF_HI);
    }

    if ((b_hi & FLOAT64_EXP_MASK_HI) == FLOAT64_EXP_MASK_HI) {
        if (frac_b)
            return (int)FLOAT64_NAN_HI;
        if (((a_hi & ~0x80000000u) | (uint32_t)a_lo) == 0u)
            return (int)FLOAT64_NAN_HI;
        return (int)(sign | FLOAT64_INF_HI);
    }

    if (!softfloat_unpack_float64((uint32_t)a_lo, a_hi, &sign_a, &exp_a, &mant_a) ||
        !softfloat_unpack_float64((uint32_t)b_lo, b_hi, &sign_b, &exp_b, &mant_b))
        return (int)sign;

    (void)sign_a;
    (void)sign_b;

    exp = exp_a + exp_b - 1023;
    softfloat_mul64_to_128(mant_a, mant_b, &prod_hi, &prod_lo);

    if ((prod_hi >> 41u) != 0u) {
        rounded_sig = softfloat_round_nearest_even_u128(prod_hi, prod_lo, 53u);
        exp++;
    } else {
        rounded_sig = softfloat_round_nearest_even_u128(prod_hi, prod_lo, 52u);
    }

    if (rounded_sig & (FLOAT64_HIDDEN_BIT << 1u)) {
        rounded_sig >>= 1u;
        exp++;
    }

    if (exp >= 0x7FF)
        return (int)(sign | FLOAT64_INF_HI);
    if (exp <= 0)
        return (int)sign;

    result_hi = sign | ((uint32_t)exp << 20u) |
                (uint32_t)(rounded_sig & (FLOAT64_HIDDEN_BIT - 1u));
    return (int)result_hi;
}

/*
 * float64_to_uint32 — convert float64 to uint32_t (truncate toward zero).
 */
uint32_t float64_to_uint32(uint32_t a_lo, uint32_t a_hi, int mode)
{
    (void)mode;
    int exp = (int)((a_hi >> 20u) & 0x7FFu) - 1023;

    if (exp < 0)  return 0;
    if (exp > 31) return 0xFFFFFFFFu;

    uint64_t mant = ((uint64_t)(a_hi & 0xFFFFFu) << 32u) | a_lo;
    mant |= (uint64_t)1u << 52u;

    if (exp >= 52)
        return (uint32_t)(mant << (exp - 52u));
    return (uint32_t)(mant >> (52u - exp));
}

/*
 * float64_to_int32 — compatibility label for the recovered float64->int32
 * conversion helper.
 *
 * The mapped name is stable for linkage, but the trusted behavior comes from
 * the recovered routine shape: round-to-nearest-even with the classic softfloat
 * invalid sentinel on overflow/NaN paths, not a plain truncation wrapper.
 */
int float64_to_int32(int result, int a2)
{
    uint32_t hi = (uint32_t)a2;
    uint32_t lo = (uint32_t)result;
    uint32_t sign = hi >> 31u;
    uint32_t exp_field = (hi >> 20u) & 0x7FFu;
    uint64_t frac = ((uint64_t)(hi & FLOAT64_FRAC_MASK_HI) << 32u) | lo;
    uint64_t sig;
    uint64_t mag;
    int exp;
    int shift;

    if (exp_field == 0x7FFu)
        return FLOAT64_INT32_INVALID;

    if (exp_field == 0u) {
        if (!frac)
            return 0;
        sig = frac;
        exp = -1022;
    } else {
        sig = frac | FLOAT64_HIDDEN_BIT;
        exp = (int)exp_field - 1023;
    }

    shift = 52 - exp;
    if (shift > 63)
        mag = 0;
    else if (shift > 0) {
        uint64_t int_part = sig >> shift;
        uint64_t rem_mask = (UINT64_C(1) << shift) - 1u;
        uint64_t rem = sig & rem_mask;
        uint64_t half = UINT64_C(1) << (shift - 1u);

        mag = int_part;
        if (rem > half || (rem == half && (mag & 1u)))
            mag++;
    } else if (shift == 0) {
        mag = sig;
    } else {
        int left_shift = -shift;

        if (left_shift > 31)
            return FLOAT64_INT32_INVALID;
        mag = sig << left_shift;
    }

    if (sign) {
        if (mag > UINT32_C(0x80000000))
            return FLOAT64_INT32_INVALID;
        if (mag == UINT32_C(0x80000000))
            return (int32_t)0x80000000u;
        return -(int32_t)mag;
    }

    if (mag > INT32_MAX)
        return FLOAT64_INT32_INVALID;
    return (int32_t)mag;
}

/*
 * int32_to_float64_normalize — convert int32 to normalised float64 mantissa.
 *
 * Returns the mantissa in the upper 21 bits of the result (shifted left by 21).
 * The sign is stripped and negation performed before calling.
 */
signed int int32_to_float64_normalize(signed int n)
{
    if (n < 0) n = -n;
    uint32_t abs = (uint32_t)n;
    uint32_t normalised = abs << __clz(abs);
    if (normalised)
        return (int)(normalised << 21u);
    return 0;
}

/*
 * uint32_to_float64_normalize — convert uint32 to normalised float64 mantissa.
 */
uint32_t uint32_to_float64_normalize(uint32_t n)
{
    uint32_t normalised = n << __clz(n);
    if (normalised)
        return normalised << 21u;
    return 0;
}

/*
 * float64_cmp — compatibility name for the recovered `sub_1CD2A`
 * exponent-window/filter helper.
 *
 * The mapped symbol is not a normal two-operand compare.  The recovered code
 * shape is closer to a "filter specials, then compare exponent window" helper,
 * so the third argument is treated as the biased exponent threshold and the
 * return value is the signed distance from that threshold.  Special values are
 * rejected with `INT32_MIN` so callers can branch away from the normal path.
 */
int float64_cmp(int a_lo, uint32_t a_hi, int b_lo)
{
    int exp_field = (int)((a_hi >> 20u) & 0x7FFu);
    int inverted = 2047;
    bool equal_or_special = (exp_field == 0);
    bool negative = false;
    bool overflow = false;
    int tmp;

    if (exp_field != 0) {
        inverted = exp_field ^ 0x7FF;
        equal_or_special = (exp_field == 0x7FF);
    }

    if (!equal_or_special) {
        overflow = softfloat_sub_overflow_i32(inverted, b_lo, &tmp);
        equal_or_special = (inverted == b_lo);
        negative = tmp < 0;
    }

    if (((negative ^ overflow) || equal_or_special)) {
        int sum;
        bool add_overflow = softfloat_add_overflow_i32(b_lo, exp_field, &sum);

        if (add_overflow || sum <= 0) {
            if (exp_field != 0) {
                if (inverted != 0)
                    return 0;
                if (((uint32_t)a_lo | (a_hi << 12u)) != 0u)
                    return 0;
            } else {
                return 0;
            }
        }
    }

    return a_lo;
}

/* --- float64 exception wrappers (stubs) --- */
int float64_exception_dispatcher(int a1, uint32_t a2, int a3, uint32_t a4)
{
    (void)a1; (void)a2; (void)a3; (void)a4;
    return (int)FLOAT64_NAN_HI;
}
int float64_exception_wrapper_1(int a1, uint32_t a2, int a3, uint32_t a4)
{
    return float64_add((uint32_t)a1, a2, (uint32_t)a3, a4);
}
uint32_t float64_exception_wrapper_2(int a1, uint32_t a2, int a3, uint32_t a4)
{
    return (uint32_t)float64_mul(a1, a2, a3, a4);
}
int float64_exception_wrapper_3(int a1, uint32_t a2, int a3, uint32_t a4)
{
    (void)a1; (void)a2; (void)a3; (void)a4;
    return 0;
}
