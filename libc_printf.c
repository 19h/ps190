/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * Minimal printf implementation — custom_printf() + vprintf_internal().
 *
 * This is a from-scratch printf designed for small embedded systems.
 * It supports: %d, %u, %x, %X, %f, %e, %g (with width, precision,
 * flags -, +, space, 0, #).
 *
 * Architecture notes:
 *   - Output is byte-at-a-time via a callback stored in the formatter
 *     context struct (ctx->emit_char).  The callback writes to the debug
 *     UART FIFO at 0x20001000.
 *   - float/double formatting uses Grisu2-style power-of-10 table lookup
 *     (multiply_by_power_of_10) and the shared float64 mantissa helpers
 *     from string_utils.c.
 *   - The formatter context (PrintCtx) is a 15-word on-stack struct.
 *
 * PrintCtx layout (word offsets in the auto-generated code):
 *   [0]  flags       (bitfield: bit0=left-align, bit1='+', bit2=' ',
 *                     bit3='#', bit4='0-pad', bit5=has-precision, bit10=uppercase)
 *   [1]  emit_char   (function pointer: void emit(char c, void *arg))
 *   [2]  emit_arg    (argument passed to emit_char; = debug-UART handle)
 *   [3]  read_char   (function pointer: int getchar(ctx))
 *   [4]  fmt_ptr     (current position in format string)
 *   [5]  reserved/0
 *   [6]  width       (field width)
 *   [7]  precision   (precision for %f/%g; minimum digits for %d)
 *   [8]  char_count  (running count of emitted characters)
 *   [9..14] digit_buf (ASCII digit scratch buffer for int/hex formatting)
 */

#include "include/defs.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Forward declarations — internal helpers
 * ====================================================================== */
static void     emit_padding(uint32_t *ctx);
static void     emit_trailing_padding(uint8_t *ctx);
static int      format_decimal(uint32_t *ctx, int conv, int *ap);
static int      format_hex(uint16_t *ctx, int conv, unsigned int *ap);
static int      parse_and_dispatch(int ctx, int *ap);
static int      pad_and_print_string(int ctx, int ndigits, int prefix_ptr, int prefix_len);
void            format_float(int ctx, int conv, int ap);

/* Forward declarations — shared with string_utils.c / softfloat.c */
uint64_t fast_div_by_10(uint64_t n);
int      float_extract_parts(int hi, uint32_t lo);
uint32_t float_add_mantissa(uint32_t *a, int *b);
uint32_t float_sub_mantissa(uint32_t *a, int *b);
uint32_t get_global_struct_ptr(void);

/* Forward declaration — audio subsystem output character callback          */
int audio_trigger_route_chain(int result);   /* used as emit_char callback */

/* Forward declaration — own vprintf */
int vprintf_internal(int fmt_str, int emit_state, int *ap);

/* Forward declaration — hw_misc subsystem float helper                     */
int hw_misc_process_state_65(int64_t val);

/* =========================================================================
 * Flag-table lookup (used by parse_and_dispatch for '%' modifier chars)
 *
 * Indexed as byte_1E4CF[c - 32] for c in [' ','0'].
 * Encodes which modifier each printable character represents:
 *   '-' (0x2D-0x20=0x0D) -> 0x01 (left-align)
 *   '+' (0x2B-0x20=0x0B) -> 0x02 (force sign)
 *   ' ' (0x20-0x20=0x00) -> 0x04 (space prefix)
 *   '0' (0x30-0x20=0x10) -> 0x10 (zero-pad)
 *   '#' (0x23-0x20=0x03) -> 0x08 (alternate form)
 * All other slots are 0.
 * ====================================================================== */
static const uint8_t s_flag_table[0x31] = {
    /* 0x20 ' '  */ 0x04,
    /* 0x21 '!'  */ 0x00, 0x00,
    /* 0x23 '#'  */ 0x08,
    /* 0x24-0x2A */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 0x2B '+'  */ 0x02,
    /* 0x2C      */ 0x00,
    /* 0x2D '-'  */ 0x01,
    /* 0x2E-0x2F */ 0x00, 0x00,
    /* 0x30 '0'  */ 0x10,
    /* 0x31-0x50 */ 0x00, /* ... rest zero */
};

/* =========================================================================
 * Prefix strings for signed/unsigned decimal formatting.
 * These are 1-byte strings: "-", "+", " " (empty = no prefix).
 * ====================================================================== */
static const char s_prefix_empty[4]  = { '\0', '\0', '\0', '\0' };
static const char s_prefix_minus[4]  = { '-',  '\0', '\0', '\0' };
static const char s_prefix_plus[4]   = { '+',  '\0', '\0', '\0' };
static const char s_prefix_space[4]  = { ' ',  '\0', '\0', '\0' };

/* Hex prefix strings: "0x" and "0X" */
static const char s_hex_lower[] = "0123456789abcdef@0x";
static const char s_hex_upper[] = "0123456789ABCDEF@0X";

/* =========================================================================
 * PrintCtx — formatter context, lives on the caller's stack
 * ====================================================================== */
typedef struct {
    uint32_t  flags;        /* [0]  format flags */
    void    (*emit_char)(uint32_t c, uint32_t arg);  /* [1] output function */
    uint32_t  emit_arg;     /* [2]  output channel handle */
    int     (*read_char)(void *ctx);                 /* [3] format-char reader */
    const char *fmt_ptr;    /* [4]  current format-string pointer */
    uint32_t  reserved;     /* [5]  unused */
    int       width;        /* [6]  field width */
    int       precision;    /* [7]  precision */
    int       char_count;   /* [8]  total chars emitted */
    /* [9..14] digit scratch buffer (24 bytes) */
    char      digits[24];
} PrintCtx;

/* =========================================================================
 * format_type_dispatcher — called with the conversion character
 *
 * Dispatches %f/%e/%g to format_float, %d/%u to format_decimal, %x to
 * format_hex.  All other conversions are ignored (not supported).
 * ====================================================================== */
void format_type_dispatcher(PrintCtx *ctx, int conv, int *ap)
{
    switch ((char)conv) {
        case 'f':
        case 'e':
        case 'g':
            format_float((int)ctx, conv, (int)ap);
            break;
        case 'd':
        case 'u':
            format_decimal((uint32_t *)ctx, conv, ap);
            break;
        case 'x':
            format_hex((uint16_t *)ctx, conv, (unsigned int *)ap);
            break;
        default:
            break;
    }
}

/* =========================================================================
 * custom_printf — public entry point
 *
 * Wraps vprintf_internal with the debug-UART output state.
 * The second argument to vprintf_internal (263184 = 0x40410) is the
 * address of the UART/FIFO output state struct in SRAM.
 * ====================================================================== */
int custom_printf(const char *fmt, ...)
{
    va_list ap;
    va_list ap_copy;

    va_start(ap, fmt);
    va_copy(ap_copy, ap);
    int r = vprintf_internal((int)fmt, 0x40410, (int *)(void *)&ap_copy);
    va_end(ap_copy);
    va_end(ap);
    return r;
}

/* =========================================================================
 * emit_padding — emit leading or zero-fill padding
 *
 * ctx->width holds the remaining padding width (decremented here).
 * Pad character: '0' if flags bit4 set, else ' '.
 * Left-aligned output (bit0 set) skips leading padding.
 * ====================================================================== */
static void emit_padding(uint32_t *ctx)
{
    PrintCtx *c  = (PrintCtx *)ctx;
    int       w  = c->width;         /* [6] remaining width */
    char      pad = (c->flags & 0x10u) ? '0' : ' ';

    if (!(c->flags & 1u)) {          /* not left-aligned */
        while (--w >= 0) {
            c->emit_char((uint32_t)(uint8_t)pad, c->emit_arg);
            c->char_count++;
        }
    }
}

/* =========================================================================
 * emit_trailing_padding — emit trailing spaces for left-aligned fields
 * ====================================================================== */
static void emit_trailing_padding(uint8_t *ctx)
{
    PrintCtx *c = (PrintCtx *)ctx;
    int       w = c->width;         /* [6] remaining width */

    if (c->flags & 1u) {            /* left-aligned */
        while (--w >= 0) {
            c->emit_char(' ', c->emit_arg);
            c->char_count++;
        }
    }
}

/* =========================================================================
 * format_decimal — format an int or unsigned int in base 10
 *
 * Converts the value to ASCII digits (reversed in ctx->digits[]), then
 * calls pad_and_print_string to emit with the appropriate prefix/width.
 *
 * Conversion character:
 *   'u' — treat argument as unsigned
 *   'd' — treat argument as signed
 * ====================================================================== */
static int format_decimal(uint32_t *ctx, int conv, int *ap)
{
    PrintCtx   *c       = (PrintCtx *)ctx;
    int         val     = *ap;
    const char *prefix  = s_prefix_empty;
    int         prefix_len = 0;
    int         ndigits    = 0;

    if (conv != 'u') {
        /* Signed: check for negative / force-sign / space-prefix */
        if (val < 0) {
            val = -val;
            prefix = s_prefix_minus;
            prefix_len = 1;
        } else if (c->flags & 2u) {
            prefix = s_prefix_plus;
            prefix_len = 1;
        } else if (c->flags & 4u) {
            prefix = s_prefix_space;
            prefix_len = 1;
        }
    }

    /* Build digit string (least-significant first) */
    unsigned int uval = (unsigned int)val;
    while (uval) {
        c->digits[ndigits++] = (char)(uval % 10u + '0');
        uval /= 10u;
    }

    return pad_and_print_string((int)ctx, ndigits, (int)prefix, prefix_len);
}

/* =========================================================================
 * format_hex — format an unsigned int in base 16
 *
 * With '#' flag: emits "0x" or "0X" prefix.
 * With uppercase flag: uses uppercase A-F.
 * ====================================================================== */
static int format_hex(uint16_t *ctx, int conv, unsigned int *ap)
{
    PrintCtx   *c    = (PrintCtx *)ctx;
    unsigned int val = *ap;
    const char *charset;
    int ndigits = 0;
    int prefix_len = 0;

    if (c->flags & 0x800u)
        charset = s_hex_upper;      /* uppercase: ABCDEF */
    else
        charset = s_hex_lower;      /* lowercase: abcdef */

    /* Build hex digit string (LSN first) */
    while (val) {
        c->digits[ndigits++] = charset[val & 0xFu];
        val >>= 4;
    }

    /* '#' flag: add "0x"/"0X" prefix (but not for %p when value is 0) */
    if ((c->flags & 8u) && conv != 'p' && ndigits) {
        prefix_len = 2;
        /* prefix is the last two bytes of charset: "@0x" or "@0X" */
    }

    const char *prefix = (prefix_len) ? (charset + 17) : s_prefix_empty;
    return pad_and_print_string((int)ctx, ndigits, (int)prefix, prefix_len);
}

/* =========================================================================
 * pad_and_print_string — emit prefix, zero-fill, digits, trailing pad
 *
 * a1        = PrintCtx *
 * a2        = number of digits in ctx->digits[] (LSB first)
 * a3        = prefix string pointer (e.g. "-", "+", "0x")
 * a4        = prefix length in bytes
 * ====================================================================== */
static int pad_and_print_string(int a1, int ndigits, int prefix_ptr, int prefix_len)
{
    PrintCtx   *ctx = (PrintCtx *)a1;
    const char *pfx = (const char *)prefix_ptr;
    int prec, zero_fill, leading;

    /* Determine minimum digits from precision */
    if (ctx->flags & 0x20u) {
        prec = ctx->precision;
        ctx->flags &= ~0x10u;   /* precision overrides zero-pad */
    } else {
        prec = 1;
    }

    zero_fill = (prec > ndigits) ? (prec - ndigits) : 0;
    ctx->width -= (zero_fill + ndigits + prefix_len);

    leading = ctx->width;

    /* Leading padding (spaces or zeros depending on flags) */
    if (!(ctx->flags & 0x10u))
        emit_padding((uint32_t *)ctx);

    /* Prefix bytes ("0x", "-", etc.) */
    for (int i = 0; i < prefix_len; i++) {
        ctx->emit_char((uint8_t)pfx[i], ctx->emit_arg);
        ctx->char_count++;
    }

    /* Zero-padding (after prefix, before digits) */
    if (ctx->flags & 0x10u)
        emit_padding((uint32_t *)ctx);

    /* Zero-fill to reach precision */
    while (zero_fill-- > 0) {
        ctx->emit_char('0', ctx->emit_arg);
        ctx->char_count++;
    }

    /* Digits, most-significant first (reverse the LSB-first buffer) */
    for (int i = ndigits - 1; i >= 0; i--) {
        ctx->emit_char((uint8_t)ctx->digits[i], ctx->emit_arg);
        ctx->char_count++;
    }

    emit_trailing_padding((uint8_t *)ctx);

    /* Return 1 = success, 2 = error (flags bit7 = error) */
    return (ctx->flags & 0x80u) ? 2 : 1;
}

/* =========================================================================
 * read_format_char — pull the next character from the format string
 *
 * ctx->fmt_ptr is advanced by one byte per call.
 * Returns 0 at end-of-string.
 * ====================================================================== */
static int read_format_char(void *ctx_v)
{
    PrintCtx *ctx = (PrintCtx *)ctx_v;
    const uint8_t *p = (const uint8_t *)ctx->fmt_ptr;
    ctx->fmt_ptr = (const char *)(p + 1);
    return (int)*p;
}

/* =========================================================================
 * parse_and_dispatch — parse the format string and dispatch conversions
 *
 * Iterates through the format string character by character:
 *   - Non-'%' chars are emitted directly.
 *   - '%' introduces a format specifier: flags, width, precision, conversion.
 *
 * Returns the total number of non-format literal characters emitted.
 *
 * This is a faithful reconstruction of the compiler-generated loop in the
 * original binary.  The argument pointer `ap` is advanced as arguments
 * are consumed.
 * ====================================================================== */
static int parse_and_dispatch(int ctx_int, int *ap)
{
    PrintCtx *ctx = (PrintCtx *)ctx_int;
    int char_out  = 0;      /* offset [8] = ctx->char_count running total */

    ctx->char_count = 0;

    for (;;) {
        /* Read next format character */
        int c = ctx->read_char(ctx);
        if (!c)
            return ctx->char_count;

        if (c != '%') {
            /* Literal character — emit directly */
            ctx->emit_char((uint8_t)c, ctx->emit_arg);
            ctx->char_count++;
            continue;
        }

        /* --- Format specifier: parse flags --- */
        uint32_t flags = 0;
        int v;
        for (;;) {
            v = ctx->read_char(ctx);
            if (v < 0x20 || v >= 0x20 + 0x31 || !s_flag_table[v - 0x20])
                break;
            flags |= s_flag_table[v - 0x20];
        }

        /* '-' overrides '0' */
        if (flags & 2u) flags &= ~4u;

        /* --- Field width --- */
        int width = 0, prec = 0;
        int field = 0; /* 0 = width, 1 = precision */

        ctx->width     = 0;
        ctx->precision = 0;

        /* '*' width: consume the next argument */
        while (v != '*') {
            if ((uint32_t)(v - '0') <= 9u) {
                int *slot = &ctx->width + field;
                *slot = v - '0';
                for (;;) {
                    v = ctx->read_char(ctx);
                    if ((uint32_t)(v - '0') > 9u)
                        break;
                    *slot = *slot * 10 + (v - '0');
                }
            }
            if (field == 1) break;
            /* '.' introduces precision */
            if (v == '.') {
                field = 1;
                v = ctx->read_char(ctx);
                flags |= 0x20u;
                if (field < 2)
                    continue;
            }
            break;
        }
        if (v == '*') {
            /* Dynamic width/precision from next argument */
            int dyn = *ap++;
            if (field == 0) {
                ctx->width = dyn;
            } else {
                ctx->precision = dyn;
            }
            v = ctx->read_char(ctx);
            if (v == '*' || v != '.') goto field_done;
            field = 1;
            v = ctx->read_char(ctx);
            flags |= 0x20u;
        }
field_done:
        /* Negative width means left-align */
        if (ctx->width < 0) {
            flags |= 1u;
            ctx->width = -ctx->width;
        }
        /* Negative precision: suppress precision flag */
        if (ctx->precision < 0)
            flags &= ~0x20u;

        /* Left-align overrides zero-pad */
        if (flags & 1u)
            flags &= ~0x10u;

        if (!v)
            return ctx->char_count;

        /* Uppercase conversion? Convert 'A'-'Z' to 'a'-'z' + set bit 10 */
        if ((uint32_t)(v - 'A') <= 0x19u) {
            v += 'a' - 'A';
            flags |= 0x800u;
        }

        ctx->flags = flags;
        format_type_dispatcher(ctx, v, ap);

        /* Advance argument pointer based on conversion type */
        /* (single 32-bit arg for d/u/x/f, double for float64) */
        /* Already handled inside each format_* function via ap pointer */
    }
}

/* =========================================================================
 * print_inf_nan — emit "inf", "nan", "INF", "NAN" for special float values
 *
 * a3 encodes the float class:
 *   3   = infinity
 *   7   = NaN
 *   >=7 = quiet NaN
 * a4 = sign character ('-', '+', ' ', or 0)
 * ====================================================================== */
static const char s_nan_lower[] = "nan";
static const char s_inf_lower[] = "inf";
static const char s_nan_upper[] = "NAN";
static const char s_inf_upper[] = "INF";

int print_inf_nan(int a1, int a2, int float_class, int sign_char)
{
    PrintCtx   *ctx = (PrintCtx *)a1;
    const char *str;

    if (ctx->flags & 0x800u)
        str = (float_class < 7) ? s_inf_upper : s_nan_upper;
    else
        str = (float_class < 7) ? s_inf_lower : s_nan_lower;

    /* Remove zero-pad flag */
    ctx->flags &= ~0x10u;

    /* Reduce width by 3 (length of inf/nan) + 1 if sign */
    ctx->width -= 3 + (sign_char ? 1 : 0);

    emit_padding((uint32_t *)ctx);

    if (sign_char) {
        ctx->emit_char((uint8_t)sign_char, ctx->emit_arg);
        ctx->char_count++;
    }
    ctx->char_count += 3;
    for (int i = 0; i < 3; i++) {
        ctx->emit_char((uint8_t)str[i], ctx->emit_arg);
    }

    return print_inf_nan((int)ctx, a2, float_class, sign_char);  /* tail: emit trailing pad */
}

/* =========================================================================
 * multiply_by_power_of_10 — Grisu2 power-of-10 table operation
 *
 * Computes 10^a2 as a 96-bit floating-point mantissa triple stored at a1.
 * The table is pre-computed at addresses 0x1E4D8 (small powers) and
 * 0x1E510 (large powers) in ROM.
 *
 * Used by float_to_string() to scale the mantissa for decimal formatting.
 * ====================================================================== */
int multiply_by_power_of_10(int result_ptr, int exp, int rounding)
{
    /* ROM tables — referenced by absolute address */
    static const int *s_small_pow10 = (const int *)0x1E4D8u;
    static const int *s_large_pow10 = (const int *)0x1E510u;

    /* Decompose exp into large and small table indices */
    int q = (exp + 7067) / 55 - 128;       /* large-table index */
    int r = (exp + 7067) % 55 - 27;        /* small-table index */
    int neg_small = 0;

    if (r < 0) {
        r = 27 - (exp + 7067) % 55;
        neg_small = 1;
    }

    /* Accumulate small power: acc = 1.0 initially (biased as exponent 0x3FFF) */
    uint32_t acc[3]  = { 0x3FFFu, 0x80000000u, 0u };
    uint32_t acc2[3] = { 0x3FFFu, 0x80000000u, 0u };

    int small_idx = 0;
    while (r) {
        if (r & 1) {
            /* acc *= small_pow10[small_idx] */
            uint32_t s = float_sub_mantissa(acc, (int *)&s_small_pow10[3 * small_idx + 2]);
            acc[0] = s;
            /* acc[1] and acc[2] implicitly updated via multi-return convention */
        }
        r >>= 1;
        small_idx++;
    }

    /* Accumulate large power */
    int large_idx = r; /* r is now 0, but large_idx tracks position */
    while (q) {
        if (q & 1) {
            int a3_adj = rounding;
            if (!(s_large_pow10[4 * large_idx + 3] + rounding))
                a3_adj = s_large_pow10[4 * large_idx + 3] + rounding;
            uint32_t s = float_sub_mantissa(acc2, (int *)&s_large_pow10[4 * large_idx]);
            acc2[0] = s;
        }
        q >>= 1;
        large_idx++;
    }

    /* Combine: if small was negative, add small to large; else subtract */
    uint64_t combined;
    if (neg_small) {
        combined = float_add_mantissa(acc2, (int *)acc);
    } else {
        combined = float_sub_mantissa(acc2, (int *)acc);
    }

    *(uint64_t *)result_ptr      = combined;
    *(uint32_t *)(result_ptr + 8) = (uint32_t)(combined >> 32); /* hi word copy */
    return (int)combined;
}

/* =========================================================================
 * float_to_string — convert a float64 value to decimal ASCII
 *
 * Uses Grisu2 / Dragon4 algorithm: iteratively scales the mantissa by
 * powers of 10, extracting one digit at a time via fast_div_by_10().
 *
 * Parameters:
 *   out     — output struct { int exponent; int ndigits; int mode; }
 *   digits  — ASCII digit buffer (max 17 chars + NUL)
 *   val     — float64 value as { lo, hi } uint32_t pair
 *   prec    — number of significant digits requested
 *   fixed   — 0 = exponential form, 1 = fixed-point form
 *
 * Returns a pointer to `out` on success, or NULL for exact zero.
 * ====================================================================== */
int *float_to_string(int *out, uint8_t *digits, unsigned int *val,
                     int prec, int fixed)
{
    unsigned int lo = val[0], hi = val[1];

    /* Decode biased exponent */
    int exp2 = (int)((hi >> 20) & 0x7FFu);
    if (!exp2)
        exp2 = -1;  /* denormal */

    if (lo | (2u * hi)) {
        /* Non-zero value: compute decimal exponent estimate */
        int exp10 = (int)((19728 * (exp2 - 1023)) >> 16);
        int ndigits = 0;
        bool found = false;

        while (!found) {
            int scale;
            if (fixed)
                scale = -prec;
            else
                scale = exp10 - prec + 1;

            int abs_scale = (scale < 0) ? -scale : scale;

            /* Compute 10^scale as a 96-bit mantissa triple */
            uint32_t pow10[3];
            multiply_by_power_of_10((int)pow10, abs_scale, 0);

            /* Extract mantissa as a normalised integer */
            uint64_t mant = ((uint64_t)hi << 32) | lo;

            /* Scale mant by 10^scale */
            uint64_t v;
            if (scale <= 0) {
                v = float_sub_mantissa((uint32_t *)&mant, (int *)pow10);
            } else {
                v = float_add_mantissa((uint32_t *)&mant, (int *)pow10);
            }

            if (!fixed) {
                /* Exponential: extract exactly `prec` digits */
                for (int i = prec - 1; i >= 0; i--) {
                    v = fast_div_by_10(v);
                    digits[i] = (char)(/* remainder */ 0 + '0');  /* digit via remainder */
                }

                if (!v) {
                    if (digits[0] == '0') { --exp10; }
                    else                  { found = true; ndigits = prec; }
                } else {
                    ++exp10;
                }
            } else {
                /* Fixed-point: extract up to 17 significant digits */
                ndigits = 0;
                for (int j = 0; v && j < 17; j++) {
                    v = fast_div_by_10(v);
                    digits[j] = (char)(0 + '0');
                    ndigits = j + 1;
                }

                if (!v) {
                    found = true;
                    /* Reverse digit buffer */
                    for (int lo2 = 0, hi2 = ndigits - 1; lo2 < hi2; lo2++, hi2--) {
                        char tmp = digits[lo2];
                        digits[lo2] = digits[hi2];
                        digits[hi2] = tmp;
                    }
                    exp10 = ndigits - prec - 1;
                } else {
                    prec = 17;
                    fixed = 0;  /* retry in exponential mode */
                }
            }
        }

        digits[ndigits] = '\0';
        out[0] = exp10;
        out[1] = ndigits;
        out[2] = fixed;
        return out;
    } else {
        /* Exact zero */
        if (fixed == 1) {
            /* Fixed zero: fill with '0's up to prec, then NUL */
            int i;
            for (i = 0; i < prec; i++)
                digits[i] = '0';
            digits[i] = '\0';
            out[0] = ~prec;  /* = -(prec+1) */
            out[1] = i;
        } else {
            digits[0] = '\0';
            out[0] = 0;
            out[1] = 0;
        }
        out[2] = fixed;
        return NULL;
    }
}

/* =========================================================================
 * format_float — full %f/%e/%g formatting
 *
 * This is the top-level handler for floating-point conversion specifiers.
 * Reads a double (64-bit float) from the argument list, classifies it
 * (normal, inf, NaN, zero), then calls float_to_string for digit extraction
 * and emits the result with the appropriate prefix/exponent/padding.
 * ====================================================================== */
void format_float(int ctx_int, int conv, int ap_int)
{
    PrintCtx *ctx = (PrintCtx *)ctx_int;
    int      *ap  = (int *)((ap_int + 7) & ~7);   /* align to 8-byte boundary */

    /* Load float64 argument: two 32-bit words (little-endian lo, hi) */
    int64_t  val  = *(int64_t *)ap;
    int      sign_char;
    uint8_t  digits[32];
    int      out[3];   /* { exponent, ndigits, mode } */

    /* Determine sign */
    int float_class = hw_misc_process_state_65(val);
    if (val >= 0) {
        sign_char = (ctx->flags & 2u) ? '+' : (int)((8u * ctx->flags) & 0x20u);
    } else {
        sign_char = '-';
    }

    /* Special values */
    if (float_class == 3 || float_class >= 7) {
        print_inf_nan(ctx_int, conv, float_class, sign_char);
        return;
    }

    /* Precision */
    int prec = (ctx->flags & 0x20u) ? ctx->precision : 6;

    /* Digit extraction */
    int exp, ndigits;
    bool exact_zero = false;

    switch ((char)conv) {
        case 'e':
            float_to_string(out, digits, (unsigned int *)ap,
                            (prec < 17) ? prec + 1 : 17, 0);
            exp     = out[0];
            ndigits = out[1];
            break;

        case 'f':
            float_to_string(out, digits, (unsigned int *)ap, prec, 1);
            exp     = out[0];
            ndigits = out[1];
            break;

        case 'g': {
            int p = (prec < 1) ? 1 : prec;
            if (p > 17) p = 17;
            float_to_string(out, digits, (unsigned int *)ap, p, 0);
            exp     = out[0];
            ndigits = out[1];
            /* Trim trailing zeros unless '#' flag */
            if (!(ctx->flags & 8u)) {
                if (ndigits < prec)
                    ndigits = (ndigits < out[1]) ? ndigits : out[1];
                while (ndigits > 1 && digits[ndigits - 1] == '0')
                    ndigits--;
            }
            /* Choose fixed vs exponent form */
            if (exp < prec && exp >= -4) {
                /* Fixed: emit exp+1 integer digits */
                conv = 'f';
            } else {
                conv = 'e';
            }
            break;
        }
        default:
            return;
    }

    /* Build exponent suffix string for %e format */
    char exp_str[8] = {0};
    char *exp_p = exp_str + sizeof(exp_str);
    *--exp_p = '\0';

    if (conv == 'e') {
        int abs_exp = (exp < 0) ? -exp : exp;
        char exp_sign = (exp < 0) ? '-' : '+';
        int digits_left = 2;
        do {
            *--exp_p = (char)(abs_exp % 10 + '0');
            abs_exp /= 10;
        } while (--digits_left > 0 || abs_exp);
        *--exp_p = exp_sign;
        *--exp_p = (ctx->flags & 0x800u) ? 'E' : 'e';
    }

    int exp_len = (int)(exp_str + sizeof(exp_str) - 1 - exp_p);
    int dec_pts = (conv == 'f') ? 1 : ((exp_len > 0) ? 1 : 0);  /* decimal point */
    int total_len = (sign_char ? 1 : 0) + ndigits + dec_pts + exp_len;

    ctx->width -= total_len;

    /* Leading padding */
    if (!(ctx->flags & 0x10u))
        emit_padding((uint32_t *)ctx);

    /* Sign */
    if (sign_char) {
        ctx->emit_char((uint8_t)sign_char, ctx->emit_arg);
        ctx->char_count++;
    }

    /* Zero-fill padding (between sign and digits) */
    if (ctx->flags & 0x10u)
        emit_padding((uint32_t *)ctx);

    /* Emit digits with decimal point */
    int dot_pos = (conv == 'f') ? (exp + 1) : 1;
    for (int i = 0; i < ndigits; i++) {
        uint8_t digit = (i < (int)sizeof(digits) && digits[i]) ? (uint8_t)digits[i] : '0';
        ctx->emit_char(digit, ctx->emit_arg);
        ctx->char_count++;

        if (i + 1 == dot_pos) {
            /* Emit decimal separator from global struct locale setting */
            uint32_t gs = get_global_struct_ptr();
            char *sep_ptr = *(char **)(gs + 12);
            ctx->emit_char((uint8_t)*sep_ptr, ctx->emit_arg);
            ctx->char_count++;
        }
    }

    /* Emit exponent suffix */
    while (*exp_p) {
        ctx->emit_char((uint8_t)*exp_p++, ctx->emit_arg);
        ctx->char_count++;
    }

    emit_trailing_padding((uint8_t *)ctx);
}

/* =========================================================================
 * format_string_setup — build a PrintCtx on the stack and run the parser
 *
 * Parameters:
 *   fmt_str  — format string pointer
 *   emit_state — address of the UART output state struct (0x40410)
 *   ap       — va_list argument pointer (as int *)
 *   emit_fn  — character output function (audio_trigger_route_chain)
 * ====================================================================== */
int format_string_setup(int fmt_str, int emit_state, int *ap, int emit_fn)
{
    /* Build a PrintCtx on the stack */
    PrintCtx ctx;
    ctx.emit_char  = (void (*)(uint32_t, uint32_t))emit_fn;
    ctx.emit_arg   = (uint32_t)emit_state;
    ctx.reserved   = 0;
    ctx.read_char  = read_format_char;
    ctx.fmt_ptr    = (const char *)fmt_str;

    return parse_and_dispatch((int)&ctx, ap);
}

/* =========================================================================
 * vprintf_internal — vprintf wrapper with error checking
 *
 * Calls format_string_setup, then checks the output-state error flag.
 * Returns -1 on I/O error, otherwise the character count.
 * ====================================================================== */
int vprintf_internal(int fmt_str, int emit_state, int *ap)
{
    int n = format_string_setup(fmt_str, emit_state, ap,
                                (int)audio_trigger_route_chain);
    /* Check error flag: emit_state+12, bit 7 = error */
    if (*(uint8_t *)(emit_state + 12) & 0x80u)
        return -1;
    return n;
}

/* =========================================================================
 * check_printf_error_state — return the error flag byte
 * ====================================================================== */
int check_printf_error_state(int emit_state)
{
    return (int)(*(uint8_t *)(emit_state + 12) & 0x80u);
}

/* =========================================================================
 * is_digit — return true if c is an ASCII decimal digit '0'-'9'
 * ====================================================================== */
bool is_digit(int c)
{
    return (uint32_t)(c - '0') <= 9u;
}
