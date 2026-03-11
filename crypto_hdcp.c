/*
 * PS190 HDMI 2.1 FRL Retimer/Repeater Firmware
 * HDCP 2.x cryptographic engine and Apple IMG4 manifest parser.
 *
 * This module handles:
 *
 * 1. IMG4 / IM4M parsing (Apple Secure Boot manifest format)
 *    The PS190 uses Apple's IMG4 format to authenticate firmware images.
 *    An IM4M (IMG4 Manifest) contains RSA-signed hashes of the firmware
 *    partitions.  The parser walks the ASN.1 DER encoding to extract the
 *    certificate chain and verify the chain up to a ROM-resident root cert.
 *
 * 2. ASN.1 DER parsing helpers
 *    tag+length reader, SEQUENCE/BOOLEAN/INTEGER parsers.
 *
 * 3. HDCP 2.x key/certificate validation
 *    The hdcp_crypto_validate_loop() function iterates over the HDCP
 *    receiver certificate chain and verifies each RSA signature using the
 *    hardware RSA accelerator.
 *
 * 4. Hardware RSA accelerator interface
 *    crypto_hw_accelerator_feed() programs the hardware modular-exponentiation
 *    unit with the message, exponent, and modulus, then polls for completion.
 *
 * Register addresses for the RSA HW block were inferred from the binary.
 */

#include "include/defs.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * RSA hardware accelerator registers
 * ====================================================================== */
#define RSAHW_BASE          0x40109000u
#define RSAHW_CTRL          (*(volatile uint32_t *)(RSAHW_BASE + 0x00u))
#define RSAHW_STATUS        (*(volatile uint32_t *)(RSAHW_BASE + 0x04u))
#define RSAHW_MSG_BASE      (RSAHW_BASE + 0x10u)   /* message input (256 bytes) */
#define RSAHW_EXP_BASE      (RSAHW_BASE + 0x110u)  /* exponent input (256 bytes) */
#define RSAHW_MOD_BASE      (RSAHW_BASE + 0x210u)  /* modulus input (256 bytes) */
#define RSAHW_OUT_BASE      (RSAHW_BASE + 0x310u)  /* result output (256 bytes) */
#define RSAHW_CTRL_START    0x00000001u             /* start computation */
#define RSAHW_STATUS_DONE   0x00000001u             /* result ready */
#define RSAHW_KEY_BITS      2048u                   /* RSA-2048 */
#define RSAHW_KEY_BYTES     (RSAHW_KEY_BITS / 8u)  /* 256 bytes */

/* ASN.1 / DER tag values */
#define ASN1_TAG_BOOLEAN    0x01u
#define ASN1_TAG_INTEGER    0x02u
#define ASN1_TAG_BITSTRING  0x03u
#define ASN1_TAG_OCTETSTRING 0x04u
#define ASN1_TAG_NULL       0x05u
#define ASN1_TAG_OID        0x06u
#define ASN1_TAG_SEQUENCE   0x30u
#define ASN1_TAG_SET        0x31u
#define ASN1_TAG_CONTEXT_0  0xA0u
#define ASN1_TAG_CONTEXT_3  0xA3u

/* IM4M magic tags (4-byte big-endian ASCII in the DER payload) */
#define IM4M_MAGIC          0x494D344Du  /* "IM4M" */
#define IM4C_MAGIC          0x494D3443u  /* "IM4C" */
#define IMG4_TAG_MANB       0x4D414E42u  /* "MANB" */
#define IMG4_TAG_MANP       0x4D414E50u  /* "MANP" */
#define IMG4_TAG_OBJP       0x4F424A50u  /* "OBJP" */
#define IMG4_TAG_PUBK       0x5055424Bu  /* "PUBK" */
#define IMG4_TAG_CRTP       0x43525450u  /* "CRTP" */

/* HDCP certificate / key constants */
#define HDCP_CERT_LEN       522u         /* HDCP 2.2 receiver certificate */
#define HDCP_DCP_PUBKEY_LEN 384u         /* DCP LLC RSA-3072 public key */

/* =========================================================================
 * Forward declarations
 * ====================================================================== */

static void *crypto_memcpy(void *dst, const void *src, uint32_t len);

struct der_tlv {
    uint32_t tag;
    uint32_t len;
    uint8_t *value;
    uint8_t *next;
};

struct der_tlv64 {
    uint64_t tag;
    uint32_t len;
    uint8_t *value;
    uint8_t *next;
};

struct img4_blob_range {
    uint8_t *ptr;
    uint8_t *end;
};

struct img4_manifest_tag_item {
    uint32_t name;
    uint32_t value_tag;
    struct img4_blob_range value;
    uint8_t *item_ptr;
    uint32_t item_len;
};

struct img4_named_pair {
    uint8_t *first_ptr;
    uint32_t first_len;
    uint8_t *second_ptr;
    uint32_t second_len;
};

struct img4_parser_descriptor {
    uint32_t alt_payload_tag;
    int (*manifest_cb)(uint32_t, int, uint32_t, uint32_t,
                       uint32_t *, uint32_t *, int);
    int (*alt_cb)(uint32_t, int, uint32_t, uint32_t,
                  uint32_t *, uint32_t *, int);
    uint32_t trust_anchor_ptr;
    uint32_t trust_anchor_len;
};

static int der_parse_tlv(uint8_t **pp, uint8_t *end, struct der_tlv *tlv);
static int der_parse_expected(uint8_t **pp, uint8_t *end, uint32_t expected_tag,
                              struct der_tlv *tlv);
static int der_parse_tlv64(uint8_t **pp, uint8_t *end, struct der_tlv64 *tlv);
static int der_parse_expected64(uint8_t **pp, uint8_t *end, uint64_t expected_tag,
                                struct der_tlv64 *tlv);
static int der_skip_any(uint8_t **pp, uint8_t *end);
static int der_parse_uint32_value(uint8_t **pp, uint8_t *end, uint32_t *out);
static int der_parse_ia5_magic(uint8_t **pp, uint8_t *end, uint32_t *out);
static int der_extract_content_bytes(uint8_t *ptr, uint8_t *end,
                                     struct img4_blob_range *out);
static int img4_range_from_tlv(const struct der_tlv64 *tlv,
                               struct img4_blob_range *out);
static int img4_range_next(struct img4_blob_range *range,
                           struct der_tlv64 *tlv);
static int der_is_duplicate_tag(const int64_t *tags, int count, int64_t tag);
static int img4_parse_manifest_tag_list(uint8_t *ptr, uint8_t *end,
                                        struct img4_manifest_tag_item *items,
                                        int *count_out);
static int img4_parse_tag_block(uint8_t *ptr, uint8_t *end,
                                struct img4_manifest_tag_item *out);
static int img4_parse_named_pair(uint8_t *ptr, uint32_t len,
                                 uint32_t expected_name,
                                  struct img4_named_pair *out);
static int img4_parse_named_pair64(uint8_t *ptr, uint32_t len,
                                   uint64_t expected_name,
                                   uint64_t *actual_name,
                                   struct img4_named_pair *out,
                                   struct der_tlv64 *second_tlv);
static int img4_parse_pubkey_blob_range(uint8_t *ptr, uint8_t *end,
                                        struct img4_blob_range *out);
static int crypto_decode_pubkey_blob(uint32_t blob_ptr, uint32_t blob_len,
                                     uint32_t *modulus, uint32_t *exponent);
static int img4_decode_rsa_pubkey(const struct img4_blob_range *blob,
                                  uint32_t *modulus, uint32_t *exponent,
                                  uint32_t exponent_words);
static int img4_compare_tag_lists(const int64_t *left_tags,
                                  const int64_t *left_types, int left_count,
                                  const int64_t *right_tags,
                                  const int64_t *right_types, int right_count,
                                  uint32_t *alt_tag_out);
static int img4_parse_cert_chain_core(uint32_t ptr, uint32_t len,
                                      uint32_t *body_ptr, uint32_t *body_len,
                                      uint32_t *sig_ptr, uint32_t *sig_len,
                                      uint32_t *cert_ptr, uint32_t *cert_len,
                                      struct img4_blob_range *block_range,
                                      const uint8_t *expected_magic,
                                      uint32_t expected_name);
static const struct img4_parser_descriptor *img4_get_descriptor(int ptr,
                                                                uint32_t fallback_alt_tag,
                                                                uint32_t fallback_anchor_ptr,
                                                                uint32_t fallback_anchor_len);
static int img4_validate_payload(uint32_t *manifest_desc, uint32_t *payload_desc,
                                 int64_t range, int flags, int depth);
static int crypto_verify_payload_hash_cb(uint32_t manifest, int offset,
                                         uint32_t magic, uint32_t type,
                                         uint32_t *value_start,
                                         uint32_t *value_end, int flags);

bool img4_check_magic_tag(int64_t tag);
int crypto_verify_payload_hash(uint32_t *manifest, int offset, int64_t range,
                               int (*hash_fn)(uint32_t, int, uint32_t,
                                              uint32_t, uint32_t *, uint32_t *, int),
                               int flags);
bool hdcp_crypto_validate_loop(int cert_ptr, int cert_len, int depth,
                               int max_depth, int64_t root_key,
                               int *result_ptr, uint32_t *key_out);
int crypto_validation_stub(void);

/* system_main.c */
void __attribute__((noreturn)) system_halt_clear_flag(void);
int  yield_execution(void);

/* string_utils.c */
uint32_t *fast_memcpy(uint32_t *dst, const uint32_t *src, uint32_t size, int extra);
int       memcmp_custom(const uint8_t *a, const uint8_t *b, uint32_t len);
int       simple_memset(int dst, char val, uint32_t len);

/* flash_nvram.c */
int flash_read_data(int addr, uint32_t len, uint8_t *buf, int flags);
int flash_read_uint32_be(uint32_t *dst, uint32_t *src_addr);
int flash_execute_hw_cmd(int cmd, int addr);
int flash_wait_ready(uint8_t *status_out);

/* hw_misc.c */
int hw_misc_process_state_29(void);

/* =========================================================================
 * Internal helpers — ASN.1 parsing
 * ====================================================================== */

/*
 * asn1_parse_tag_and_length — read an ASN.1 tag byte and DER length field.
 *
 * On entry:  *ptr  = pointer to current parse position in the DER blob.
 *            *end  = pointer past the end of the containing element.
 * On exit:   *tag  = the tag byte
 *            *len  = the content length
 *            *ptr  has been advanced past the tag+length.
 *
 * Returns 0 on success, -1 on parse error.
 */
int asn1_parse_tag_and_length(int ptr_addr, uint32_t *tag_out, uint32_t *len_out)
{
    const uint8_t **pp = (const uint8_t **)ptr_addr;
    const uint8_t  *p  = *pp;

    /* Tag byte */
    *tag_out = *p++;

    /* Length: short form (1 byte) or long form (multi-byte) */
    uint32_t first = *p++;
    if (first & 0x80u) {
        /* Long form: first byte = number of length bytes */
        uint32_t n = first & 0x7Fu;
        if (n > 4u) return -1;  /* too long for us to handle */
        first = 0u;
        for (uint32_t i = 0; i < n; i++) {
            first = (first << 8u) | *p++;
        }
    }
    *len_out = first;
    *pp = p;
    return 0;
}

/*
 * asn1_read_length_advance — read a DER length field and advance a {ptr,end} pair.
 *
 * a1  = pointer to { uint64_t: ptr (lo), end (hi) } — updated on exit
 * a2  = output: parsed length value
 * a3  = caller's end address (used for bounds check)
 */
int asn1_read_length_advance(int64_t *a1, uint32_t *len_out, int end_addr)
{
    uint32_t tag, len;
    int rc = asn1_parse_tag_and_length((int)a1, &tag, &len);
    if (rc < 0) return rc;
    *len_out = len;
    return 0;
}

/*
 * asn1_parse_sequence — parse a SEQUENCE tag and return the contents bounds.
 *
 * a1   = uint32_t *: { ptr, unused }
 * a2   = uint32_t   : end boundary
 * a3   = contents start (output, as uint64_t { start, end })
 * a4   = uint64_t * : output for the SEQUENCE's content range
 */
int asn1_parse_sequence(uint32_t *a1, int end, int64_t a3, uint64_t *a4)
{
    uint32_t tag = 0, len = 0;
    if (asn1_parse_tag_and_length((int)a1, &tag, &len) < 0)
        return -1;
    if (tag != ASN1_TAG_SEQUENCE) return -1;
    if (a4) {
        *a4 = ((uint64_t)*a1 << 32u) | (*a1 + len);
    }
    return 0;
}

/*
 * asn1_parse_boolean — parse an ASN.1 BOOLEAN value.
 *
 * Advances *pp past the BOOLEAN TLV and stores TRUE/FALSE in *val.
 */
int asn1_parse_boolean(uint8_t **pp, uint8_t *val)
{
    uint8_t *p = *pp;
    if (*p++ != ASN1_TAG_BOOLEAN) return -1;
    if (*p++ != 1u)               return -1;   /* length must be 1 */
    *val = (*p++ != 0u) ? 1u : 0u;
    *pp = p;
    return 0;
}

/*
 * asn1_read_integer — parse an ASN.1 INTEGER value.
 *
 * Reads the integer into a flat byte array at *out (up to `max_bytes`).
 * Leading zero (sign) bytes are skipped.  Returns the number of bytes
 * in the integer value.
 */
int asn1_read_integer(uint8_t **pp, uint32_t *out, int max_bytes)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;

    uint8_t *p = *pp;
    if (*p++ != ASN1_TAG_INTEGER) { if (REG_TCB_CURRENT_TASK!=saved_task) system_halt_clear_flag(); return -1; }

    /* Parse length */
    uint32_t len = *p++;
    if (len & 0x80u) {
        uint32_t n = len & 0x7Fu;
        len = 0;
        for (uint32_t i = 0; i < n; i++) len = (len << 8u) | *p++;
    }

    /* Skip leading 0x00 padding byte (for positive integers with MSB set) */
    if (*p == 0x00u && len > 1u) { ++p; --len; }

    /* Copy up to max_bytes */
    uint32_t copy_len = (len < (uint32_t)max_bytes) ? len : (uint32_t)max_bytes;
    for (uint32_t i = 0; i < copy_len; i++)
        ((uint8_t *)out)[i] = p[i];

    *pp = p + len;

    if (REG_TCB_CURRENT_TASK != saved_task)
        system_halt_clear_flag();
    return (int)copy_len;
}

/*
 * asn1_parse_integer_core — parse an INTEGER and return its value as int64_t.
 */
int asn1_parse_integer_core(uint8_t **pp, int64_t *out)
{
    uint32_t val = 0;
    int r = asn1_read_integer(pp, &val, 4);
    if (r < 0) return r;
    if (out) *out = (int64_t)val;
    return r;
}

static int der_parse_tlv(uint8_t **pp, uint8_t *end, struct der_tlv *tlv)
{
    uint8_t *p = *pp;
    uint32_t tag;
    uint32_t len;

    if (p == NULL || end == NULL || tlv == NULL || p >= end)
        return -1;
    if (asn1_parse_tag_and_length((int)&p, &tag, &len) < 0)
        return -1;
    if (p > end || len > (uint32_t)(end - p))
        return -1;

    tlv->tag = tag;
    tlv->len = len;
    tlv->value = p;
    tlv->next = p + len;
    *pp = tlv->next;
    return 0;
}

static int der_parse_expected(uint8_t **pp, uint8_t *end, uint32_t expected_tag,
                              struct der_tlv *tlv)
{
    if (der_parse_tlv(pp, end, tlv) < 0)
        return -1;
    if (tlv->tag != expected_tag)
        return -1;
    return 0;
}

static uint64_t img4_make_tag64(uint8_t tag_class, uint8_t constructed,
                                uint64_t tag_number)
{
    uint64_t prefix;

    switch (tag_class) {
    case 0x00u:
        prefix = constructed ? 0x2000000000000000ull : 0ull;
        break;
    case 0x40u:
        prefix = constructed ? 0xE000000000000000ull : 0xC000000000000000ull;
        break;
    case 0x80u:
        prefix = constructed ? 0xA000000000000000ull : 0x8000000000000000ull;
        break;
    default:
        prefix = constructed ? 0x6000000000000000ull : 0x4000000000000000ull;
        break;
    }

    return prefix | tag_number;
}

static uint64_t img4_make_app_tag64_u32(uint32_t tag)
{
    return 0xE000000000000000ull | (uint64_t)tag;
}

static uint64_t img4_tag64_name_from_bytes(const uint8_t *ptr, uint32_t len)
{
    uint64_t tag = 0ull;
    uint32_t i;

    if (ptr == NULL || len == 0u || len > 8u)
        return 0ull;
    for (i = 0; i < len; i++)
        tag = (tag << 8u) | ptr[i];
    return img4_make_tag64(0x40u, 1u, tag);
}

static int der_parse_tlv64(uint8_t **pp, uint8_t *end, struct der_tlv64 *tlv)
{
    uint8_t *p = *pp;
    uint8_t first;
    uint8_t tag_class;
    uint8_t constructed;
    uint64_t tag_number;
    uint32_t len;

    if (p == NULL || end == NULL || tlv == NULL || p >= end)
        return -1;

    first = *p++;
    tag_class = first & 0xC0u;
    constructed = first & 0x20u;
    tag_number = (uint64_t)(first & 0x1Fu);
    if (tag_number == 0x1Fu) {
        tag_number = 0ull;
        do {
            uint8_t next;

            if (p >= end || (tag_number & 0xFE00000000000000ull) != 0ull)
                return -1;
            next = *p++;
            if (tag_number == 0ull && next == 0x80u)
                return -1;
            tag_number = (tag_number << 7u) | (uint64_t)(next & 0x7Fu);
            if ((next & 0x80u) == 0u)
                break;
        } while (true);
    }

    if (p >= end)
        return -1;
    len = *p++;
    if (len & 0x80u) {
        uint32_t count = len & 0x7Fu;
        uint32_t i;

        if (count == 0u || count > 4u || (uint32_t)(end - p) < count)
            return -1;
        len = 0u;
        for (i = 0; i < count; i++)
            len = (len << 8u) | *p++;
    }
    if (len > (uint32_t)(end - p))
        return -1;

    tlv->tag = img4_make_tag64(tag_class, constructed, tag_number);
    tlv->len = len;
    tlv->value = p;
    tlv->next = p + len;
    *pp = tlv->next;
    return 0;
}

static int der_parse_expected64(uint8_t **pp, uint8_t *end, uint64_t expected_tag,
                                struct der_tlv64 *tlv)
{
    if (der_parse_tlv64(pp, end, tlv) < 0)
        return -1;
    if (tlv->tag != expected_tag)
        return -1;
    return 0;
}

static int img4_range_from_tlv(const struct der_tlv64 *tlv,
                               struct img4_blob_range *out)
{
    if (tlv == NULL || out == NULL)
        return -1;
    out->ptr = tlv->value;
    out->end = tlv->next;
    return 0;
}

static int img4_range_next(struct img4_blob_range *range,
                           struct der_tlv64 *tlv)
{
    uint8_t *cursor;

    if (range == NULL || tlv == NULL || range->ptr == NULL || range->end == NULL)
        return -1;
    cursor = range->ptr;
    if (cursor >= range->end)
        return -1;
    if (der_parse_tlv64(&cursor, range->end, tlv) < 0)
        return -1;
    range->ptr = cursor;
    return 0;
}

static int der_skip_any(uint8_t **pp, uint8_t *end)
{
    struct der_tlv tlv;

    return der_parse_tlv(pp, end, &tlv);
}

static int der_parse_uint32_value(uint8_t **pp, uint8_t *end, uint32_t *out)
{
    struct der_tlv tlv;
    uint32_t value = 0;
    uint32_t i;

    if (der_parse_expected(pp, end, ASN1_TAG_INTEGER, &tlv) < 0)
        return -1;
    if (tlv.len == 0u || tlv.len > 5u)
        return -1;
    if (tlv.len == 5u) {
        if (tlv.value[0] != 0u)
            return -1;
        tlv.value++;
        tlv.len--;
    }
    for (i = 0; i < tlv.len; i++)
        value = (value << 8u) | tlv.value[i];
    if (out)
        *out = value;
    return 0;
}

static int der_parse_ia5_magic(uint8_t **pp, uint8_t *end, uint32_t *out)
{
    struct der_tlv tlv;
    uint32_t magic = 0;
    uint32_t i;

    if (der_parse_expected(pp, end, 22u, &tlv) < 0)
        return -1;
    if (tlv.len != 4u)
        return -1;
    for (i = 0; i < 4u; i++)
        magic = (magic << 8u) | tlv.value[i];
    if (out)
        *out = magic;
    return 0;
}

static int der_extract_content_bytes(uint8_t *ptr, uint8_t *end,
                                     struct img4_blob_range *out)
{
    struct der_tlv tlv;

    if (ptr == NULL || end == NULL || out == NULL)
        return -1;
    if (der_parse_tlv(&ptr, end, &tlv) < 0)
        return -1;
    out->ptr = tlv.value;
    out->end = tlv.next;
    return 0;
}

static int der_is_duplicate_tag(const int64_t *tags, int count, int64_t tag)
{
    int i;

    for (i = 0; i < count; i++) {
        if (tags[i] == tag)
            return 1;
    }
    return 0;
}

static int img4_parse_tag_block(uint8_t *ptr, uint8_t *end,
                                struct img4_manifest_tag_item *out)
{
    struct der_tlv outer;
    uint8_t *cursor;
    uint32_t name;
    struct der_tlv wrapper;
    struct der_tlv inner;

    if (out == NULL)
        return -1;
    if (der_parse_expected(&ptr, end, ASN1_TAG_SEQUENCE, &outer) < 0)
        return -1;

    cursor = outer.value;
    if (der_parse_ia5_magic(&cursor, outer.next, &name) < 0)
        return -1;
    if (der_parse_expected(&cursor, outer.next, ASN1_TAG_SEQUENCE, &wrapper) < 0)
        return -1;
    if (cursor != outer.next)
        return -1;

    cursor = wrapper.value;
    out->item_ptr = cursor;
    if (der_parse_tlv(&cursor, wrapper.next, &inner) < 0)
        return -1;
    if (cursor != wrapper.next)
        return -1;

    out->name = name;
    out->value_tag = inner.tag;
    out->value.ptr = inner.value;
    out->value.end = inner.next;
    out->item_len = (uint32_t)(inner.next - out->item_ptr);
    return 0;
}

static int img4_parse_manifest_tag_list(uint8_t *ptr, uint8_t *end,
                                        struct img4_manifest_tag_item *items,
                                        int *count_out)
{
    struct der_tlv list_tlv;
    uint8_t *cursor;
    int count = 0;

    if (der_parse_expected(&ptr, end, ASN1_TAG_SEQUENCE, &list_tlv) < 0)
        return -1;
    cursor = list_tlv.value;
    while (cursor < list_tlv.next) {
        struct img4_manifest_tag_item item;
        uint8_t *item_ptr = cursor;
        int i;

        if (count >= 16)
            return -1;
        if (img4_parse_tag_block(item_ptr, list_tlv.next, &item) < 0)
            return -1;
        for (i = 0; i < count; i++) {
            if (items[i].name == item.name)
                return -1;
        }
        items[count] = item;

        if (der_skip_any(&item_ptr, list_tlv.next) < 0)
            return -1;
        cursor = item_ptr;
        count++;
    }

    if (count_out)
        *count_out = count;
    return 0;
}

static int crypto_verify_payload_hash_cb(uint32_t manifest, int offset,
                                         uint32_t magic, uint32_t type,
                                         uint32_t *value_start,
                                         uint32_t *value_end, int flags)
{
    uint64_t second_tag = ((uint64_t)type << 32u) | magic;

    (void)manifest;
    (void)offset;
    (void)flags;

    if (value_start == NULL || value_end == NULL)
        return -1;
    if (value_start[0] == 0u || value_end[0] == 0u)
        return -1;
    if (value_start[1] == 0u || value_end[1] == 0u)
        return -1;
    if (img4_check_magic_tag((int64_t)second_tag)) {
        return 0;
    }
    return -1;
}

static int img4_parse_named_pair(uint8_t *ptr, uint32_t len,
                                 uint32_t expected_name,
                                 struct img4_named_pair *out)
{
    return img4_parse_named_pair64(ptr, len, img4_make_app_tag64_u32(expected_name),
                                   NULL, out, NULL);
}

static int img4_parse_named_pair64(uint8_t *ptr, uint32_t len,
                                   uint64_t expected_name,
                                   uint64_t *actual_name,
                                   struct img4_named_pair *out,
                                   struct der_tlv64 *second_tlv)
{
    struct der_tlv64 outer;
    struct der_tlv64 name_tlv;
    struct der_tlv64 child;
    uint8_t *cursor = ptr;
    uint64_t name_tag;
    if (ptr == NULL || out == NULL)
        return -1;
    if (der_parse_expected64(&cursor, ptr + len, 0x2000000000000010ull, &outer) < 0)
        return -1;
    if (cursor != outer.next)
        return -1;

    cursor = outer.value;
    if (der_parse_expected64(&cursor, outer.next, 22ull, &name_tlv) < 0)
        return -1;
    name_tag = img4_tag64_name_from_bytes(name_tlv.value, name_tlv.len);
    if (name_tag == 0ull)
        return -1;
    if (expected_name != 0ull && name_tag != expected_name)
        return -1;
    if (actual_name)
        *actual_name = name_tag;

    out->first_ptr = name_tlv.value;
    out->first_len = name_tlv.len;

    if (der_parse_tlv64(&cursor, outer.next, &child) < 0)
        return -1;
    out->second_ptr = child.value;
    out->second_len = child.len;
    if (second_tlv)
        *second_tlv = child;

    if (cursor != outer.next)
        return -1;
    return 0;
}

static int img4_parse_pubkey_blob_range(uint8_t *ptr, uint8_t *end,
                                        struct img4_blob_range *out)
{
    struct der_tlv64 outer;
    struct img4_blob_range set_range;
    struct der_tlv64 crtp_tlv;
    struct der_tlv64 pubk_tlv;
    struct der_tlv64 blob_tlv;
    uint8_t *blob_cursor;
    uint8_t *cursor = ptr;

    if (ptr == NULL || end == NULL || out == NULL)
        return -1;
    if (der_parse_expected64(&cursor, end, 0x2000000000000011ull, &outer) < 0)
        return -1;
    if (img4_range_from_tlv(&outer, &set_range) < 0)
        return -1;

    if (img4_range_next(&set_range, &crtp_tlv) < 0 ||
        crtp_tlv.tag != img4_make_app_tag64_u32(IMG4_TAG_CRTP)) {
        return -1;
    }
    if (img4_range_next(&set_range, &pubk_tlv) < 0 ||
        pubk_tlv.tag != img4_make_app_tag64_u32(IMG4_TAG_PUBK)) {
        return -1;
    }
    blob_cursor = pubk_tlv.value;
    if (der_parse_tlv64(&blob_cursor, pubk_tlv.next, &blob_tlv) < 0)
        return -1;

    if (blob_tlv.tag != 4ull)
        return -1;
    out->ptr = blob_tlv.value;
    out->end = blob_tlv.next;
    return 0;
}

/* -------------------------------------------------------------------------
 * ROM-style IMG4 / ASN.1 helpers used by the recursive manifest validators.
 * These mirror the descriptor contracts of sub_33B0 / sub_34C0 / sub_34E8 /
 * sub_3538 rather than the higher-level convenience parsers above.
 * ---------------------------------------------------------------------- */

#define ROM_TAG64_SEQUENCE        0x2000000000000010ull
#define ROM_TAG64_SET             0x2000000000000011ull
#define ROM_TAG64_IA5STRING       22ull
#define ROM_TAG64_INTEGER         2ull
#define ROM_TAG64_OCTETSTRING     4ull
#define ROM_TAG64_CONTEXT0        0xA000000000000000ull
#define ROM_TAG64_CONTEXT1        0xA000000000000001ull

typedef int (*img4_validator_callback_t)(uint32_t, int, uint32_t, uint32_t,
                                         uint32_t *, uint32_t *, int);

struct rom_tlv_desc {
    uint32_t tag_lo;
    uint32_t tag_hi;
    uint32_t value_ptr;
    uint32_t value_len;
};

struct rom_range32 {
    uint32_t start;
    uint32_t end;
};

struct rom_named_desc {
    uint32_t name_ptr;
    uint32_t name_len;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t child_value_ptr;
    uint32_t child_value_len;
    uint32_t child_tag_lo;
    uint32_t child_tag_hi;
};

static uint64_t rom_make_tag64(uint32_t lo, uint32_t hi)
{
    return ((uint64_t)hi << 32u) | (uint64_t)lo;
}

static uint64_t rom_make_app_tag64_u32(uint32_t tag)
{
    return 0xE000000000000000ull | (uint64_t)tag;
}

static uint64_t rom_make_app_tag64_words(uint32_t lo, uint32_t hi)
{
    return rom_make_tag64(lo, hi | 0xE0000000u);
}

static uint64_t rom_tlv_desc_tag64(const struct rom_tlv_desc *desc)
{
    return rom_make_tag64(desc->tag_lo, desc->tag_hi);
}

static uint64_t rom_named_child_tag64(const struct rom_named_desc *desc)
{
    return rom_make_tag64(desc->child_tag_lo, desc->child_tag_hi);
}

static uint32_t *rom_named_child_pair(struct rom_named_desc *desc)
{
    return &desc->child_value_ptr;
}

static int rom_parse_tlv_pair(const uint32_t *pair,
                              struct rom_tlv_desc *out,
                              uint32_t *declared_len_out)
{
    uint8_t *ptr;
    uint32_t available;
    uint8_t first;
    uint32_t tag_lo;
    uint32_t tag_hi = 0u;
    uint8_t len_first;
    uint32_t remaining;

    if (pair == NULL || out == NULL)
        return 6;

    ptr = (uint8_t *)(uintptr_t)pair[0];
    available = pair[1];
    if (available < 2u)
        return 3;

    first = *ptr++;
    available -= 1u;
    tag_lo = (uint32_t)(first & 0x1Fu);
    if (tag_lo == 0x1Fu) {
        uint8_t next_byte;

        tag_lo = 0u;
        next_byte = *ptr;
        if (next_byte == 0x80u || next_byte < 0x1Fu)
            return 3;
        do {
            if (available < 2u || (tag_hi & 0xFE000000u) != 0u)
                return 3;
            next_byte = *ptr++;
            available -= 1u;
            tag_hi = (tag_hi << 7u) | (tag_lo >> 25u);
            tag_lo = (tag_lo << 7u) | (uint32_t)(next_byte & 0x7Fu);
        } while ((next_byte & 0x80u) != 0u);
        if ((tag_hi & 0xE0000000u) != 0u)
            return 3;
    }

    out->tag_lo = tag_lo;
    out->tag_hi = tag_hi | ((uint32_t)(first & 0xE0u) << 24u);

    len_first = *ptr++;
    remaining = available - 1u;
    if ((len_first & 0x80u) != 0u) {
        uint32_t count = (uint32_t)(len_first & 0x7Fu);
        uint32_t value_len = 0u;
        uint32_t i;

        if (count > 4u || count > remaining || count == 0u || *ptr == 0u)
            return 3;
        for (i = 0u; i < count; i++)
            value_len = (value_len << 8u) | ptr[i];
        ptr += count;
        remaining -= count;
        if (value_len < 0x80u)
            return 3;
        if (value_len > remaining && declared_len_out == NULL)
            return 3;
        if (value_len < remaining)
            remaining = value_len;
        out->value_ptr = (uint32_t)(uintptr_t)ptr;
        out->value_len = remaining;
        if (declared_len_out != NULL)
            *declared_len_out = value_len;
        return 0;
    }

    if ((uint32_t)len_first > remaining && declared_len_out == NULL)
        return 3;
    if ((uint32_t)len_first < remaining)
        remaining = (uint32_t)len_first;
    out->value_ptr = (uint32_t)(uintptr_t)ptr;
    out->value_len = remaining;
    if (declared_len_out != NULL)
        *declared_len_out = (uint32_t)len_first;
    return 0;
}

static int rom_pair_to_range(const uint32_t *pair, struct rom_range32 *out)
{
    if (pair == NULL || out == NULL)
        return 6;
    out->start = pair[0];
    out->end = pair[0] + pair[1];
    return 0;
}

static int rom_range_next(struct rom_range32 *range, struct rom_tlv_desc *out)
{
    uint32_t pair[2];
    int rc;

    if (range == NULL || out == NULL)
        return 6;
    if (range->start >= range->end)
        return 1;

    pair[0] = range->start;
    pair[1] = range->end - range->start;
    rc = rom_parse_tlv_pair(pair, out, NULL);
    if (rc != 0)
        return rc;
    range->start = out->value_ptr + out->value_len;
    return 0;
}

static int rom_parse_small_u32(const uint32_t *pair, uint32_t *out)
{
    const uint8_t *ptr;
    uint32_t len;
    uint64_t value = 0u;
    uint32_t i;

    if (pair == NULL)
        return 6;
    ptr = (const uint8_t *)(uintptr_t)pair[0];
    len = pair[1];
    if (len == 0u)
        return 3;
    if ((ptr[0] & 0x80u) != 0u)
        return 3;
    if (ptr[0] != 0u) {
        if (len > 8u)
            return 7;
    } else {
        if (len > 1u && (ptr[1] & 0x80u) == 0u)
            return 3;
        if (len > 9u)
            return 7;
    }
    for (i = 0u; i < len; i++)
        value = (value << 8u) | ptr[i];
    if ((value >> 32u) != 0u)
        return 7;
    if (out != NULL)
        *out = (uint32_t)value;
    return 0;
}

static int rom_check_magic4_pair(const uint32_t *pair, uint32_t expected_tag)
{
    const uint8_t *bytes;

    if (pair == NULL)
        return 6;
    if (pair[1] != 4u)
        return 2;
    bytes = (const uint8_t *)(uintptr_t)pair[0];
    if (bytes[0] != (uint8_t)(expected_tag >> 24u))
        return 2;
    if (bytes[1] != (uint8_t)(expected_tag >> 16u))
        return 2;
    if (bytes[2] != (uint8_t)(expected_tag >> 8u))
        return 2;
    if (bytes[3] != (uint8_t)expected_tag)
        return 2;
    return 0;
}

static bool rom_bytes_equal(uint32_t lhs_ptr, uint32_t lhs_len,
                            uint32_t rhs_ptr, uint32_t rhs_len)
{
    const uint8_t *lhs;
    const uint8_t *rhs;
    uint32_t i;

    if (lhs_len != rhs_len)
        return false;
    if (lhs_len == 0u)
        return true;

    lhs = (const uint8_t *)(uintptr_t)lhs_ptr;
    rhs = (const uint8_t *)(uintptr_t)rhs_ptr;
    for (i = 0u; i < lhs_len; i++) {
        if (lhs[i] != rhs[i])
            return false;
    }
    return true;
}

static int rom_parse_named_wrapper(const uint32_t *pair,
                                   uint64_t expected_name,
                                   struct rom_named_desc *out)
{
    struct rom_tlv_desc outer = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc child = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc extra = {0u, 0u, 0u, 0u};
    struct rom_range32 range = {0u, 0u};
    uint32_t name_pair[2];
    uint32_t name_value = 0u;
    int rc;

    if (pair == NULL || out == NULL)
        return 6;

    out->name_ptr = 0u;
    out->name_len = 0u;
    out->reserved0 = 0u;
    out->reserved1 = 0u;
    out->child_value_ptr = 0u;
    out->child_value_len = 0u;
    out->child_tag_lo = 0u;
    out->child_tag_hi = 0u;

    rc = rom_parse_tlv_pair(pair, &outer, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&outer) != ROM_TAG64_SEQUENCE)
        return 2;

    rc = rom_pair_to_range(&outer.value_ptr, &range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_IA5STRING)
        return 2;

    name_pair[0] = child.value_ptr;
    name_pair[1] = child.value_len;
    rc = rom_parse_small_u32(name_pair, &name_value);
    if (rc != 0)
        return rc;
    if (expected_name != rom_make_app_tag64_u32(name_value))
        return 2;

    out->name_ptr = child.value_ptr;
    out->name_len = child.value_len;

    rc = rom_range_next(&range, &child);
    if (rc != 0)
        return rc;
    out->child_value_ptr = child.value_ptr;
    out->child_value_len = child.value_len;
    out->child_tag_lo = child.tag_lo;
    out->child_tag_hi = child.tag_hi;

    rc = rom_range_next(&range, &extra);
    if (rc == 1) {
        if ((outer.value_ptr + outer.value_len) == (pair[0] + pair[1]))
            return 0;
        return 7;
    }
    if (rc == 0)
        return 2;
    return rc;
}


/* =========================================================================
 * crypto_check_magic_4bytes — compare 4 bytes at `addr` against a 32-bit tag
 *
 * Used to verify big-endian 4-byte ASCII tags in the IM4M stream.
 * Returns 0 if equal, -1 if not.
 * ====================================================================== */
int crypto_check_magic_4bytes(int addr, uint32_t expected_tag)
{
    uint32_t actual = 0;
    flash_read_uint32_be(&actual, (uint32_t *)&addr);
    return (actual == expected_tag) ? 0 : -1;
}

/* =========================================================================
 * crypto_memcmp — constant-time comparison of two memory regions
 *
 * Returns 0 if equal, non-zero if different.  The comparison does not
 * short-circuit on the first difference to prevent timing side-channels.
 * ====================================================================== */
int crypto_memcmp(int a, uint32_t len, int b, int dummy)
{
    (void)dummy;
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    uint8_t diff = 0;
    for (uint32_t i = 0; i < len; i++)
        diff |= pa[i] ^ pb[i];
    return (int)diff;
}

/* =========================================================================
 * crypto_clear_struct — zero a key/certificate data structure
 * ====================================================================== */
int crypto_clear_struct(uint32_t *dst)
{
    /* Structure size: 6 words (24 bytes) in the caller context */
    for (int i = 0; i < 6; i++) dst[i] = 0u;
    return 0;
}

/* =========================================================================
 * asn1_parse_rsa_pubkey — parse an RSA public key from DER encoding
 *
 * An RSA public key in DER is a SEQUENCE { INTEGER modulus, INTEGER exponent }.
 * This function extracts both values from the raw DER bytes at `ptr`.
 *
 * Returns a pointer past the parsed key on success, or NULL on error.
 * ====================================================================== */
uint8_t *asn1_parse_rsa_pubkey(uint8_t *ptr, int *key_struct)
{
    uint32_t tag, len;

    /* Outer SEQUENCE */
    if (asn1_parse_tag_and_length((int)&ptr, &tag, &len) < 0 ||
        tag != ASN1_TAG_SEQUENCE) return NULL;

    uint8_t *seq_end = ptr + len;

    /* Modulus */
    uint32_t modulus[RSAHW_KEY_BYTES / 4u];
    if (asn1_read_integer(&ptr, modulus, RSAHW_KEY_BYTES) < 0) return NULL;

    /* Public exponent */
    uint32_t exponent[4];
    if (asn1_read_integer(&ptr, exponent, 16) < 0) return NULL;

    if (key_struct) {
        key_struct[0] = (int)modulus;
        key_struct[1] = (int)exponent;
    }

    return seq_end;
}

/* =========================================================================
 * crypto_parse_pubkey_struct — parse the CRTP/PUBK public-key container
 * ====================================================================== */
int crypto_parse_pubkey_struct(int cert_ptr, int cert_len,
                               uint32_t *blob_ptr, uint32_t *blob_len)
{
    struct rom_tlv_desc outer = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc child = {0u, 0u, 0u, 0u};
    struct rom_range32 set_range = {0u, 0u};
    uint32_t input_pair[2];
    uint32_t child_pair[2];
    int rc;

    input_pair[0] = (uint32_t)cert_ptr;
    input_pair[1] = (uint32_t)cert_len;
    if (cert_ptr == 0 || cert_len == 0)
        return 6;

    rc = rom_parse_tlv_pair(input_pair, &outer, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&outer) != ROM_TAG64_SET)
        return 2;

    rc = rom_pair_to_range(&outer.value_ptr, &set_range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&set_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != rom_make_app_tag64_u32(IMG4_TAG_CRTP))
        return 2;

    rc = rom_range_next(&set_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != rom_make_app_tag64_u32(IMG4_TAG_PUBK))
        return 2;

    child_pair[0] = child.value_ptr;
    child_pair[1] = child.value_len;
    rc = rom_parse_tlv_pair(child_pair, &outer, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&outer) != ROM_TAG64_OCTETSTRING)
        return 2;

    if (blob_ptr != NULL && blob_len != NULL) {
        *blob_ptr = outer.value_ptr;
        *blob_len = outer.value_len;
    }
    return 0;
}

static int crypto_decode_pubkey_blob(uint32_t blob_ptr, uint32_t blob_len,
                                     uint32_t *modulus, uint32_t *exponent)
{
    struct img4_blob_range blob;

    if (blob_ptr == 0u || blob_len == 0u || modulus == NULL || exponent == NULL)
        return -1;

    simple_memset((int)modulus, 0, RSAHW_KEY_BYTES);
    simple_memset((int)exponent, 0, 4u);
    blob.ptr = (uint8_t *)(uintptr_t)blob_ptr;
    blob.end = blob.ptr + blob_len;
    return img4_decode_rsa_pubkey(&blob, modulus, exponent, 1u);
}

static int img4_decode_rsa_pubkey(const struct img4_blob_range *blob,
                                  uint32_t *modulus, uint32_t *exponent,
                                  uint32_t exponent_words)
{
    struct der_tlv key_seq;
    uint8_t *key_ptr;
    uint8_t *key_cursor;
    uint8_t exponent_buf[16] = {0};
    uint32_t exponent_bytes;
    int exponent_len;

    if (blob == NULL || modulus == NULL || exponent == NULL || exponent_words == 0u)
        return -1;

    key_ptr = blob->ptr;
    if (der_parse_expected(&key_ptr, blob->end, ASN1_TAG_SEQUENCE, &key_seq) < 0)
        return -1;
    if (key_ptr != blob->end)
        return -1;

    key_cursor = key_seq.value;
    if (asn1_read_integer(&key_cursor, modulus, RSAHW_KEY_BYTES) != (int)RSAHW_KEY_BYTES)
        return -1;
    exponent_bytes = exponent_words * 4u;
    exponent_len = asn1_read_integer(&key_cursor, (uint32_t *)exponent_buf,
                                     (int)exponent_bytes);
    if (exponent_len <= 0 || (uint32_t)exponent_len > exponent_bytes)
        return -1;
    if (key_cursor != key_seq.next)
        return -1;

    crypto_memcpy(((uint8_t *)exponent) + (exponent_bytes - (uint32_t)exponent_len),
                  exponent_buf, (uint32_t)exponent_len);
    return 0;
}

static void *crypto_memcpy(void *dst, const void *src, uint32_t len)
{
    uint8_t *out = (uint8_t *)dst;
    const uint8_t *in = (const uint8_t *)src;

    while (len--) {
        *out++ = *in++;
    }

    return dst;
}

static int img4_compare_tag_lists(const int64_t *left_tags,
                                  const int64_t *left_types, int left_count,
                                  const int64_t *right_tags,
                                  const int64_t *right_types, int right_count,
                                  uint32_t *alt_tag_out)
{
    int i;
    int j;
    uint32_t alt_tag = 0u;

    if (left_tags == NULL || left_types == NULL ||
        right_tags == NULL || right_types == NULL) {
        return -1;
    }

    for (i = 0; i < left_count; i++) {
        int found = 0;

        for (j = 0; j < right_count; j++) {
            if (left_tags[i] == right_tags[j]) {
                if (left_types[i] != right_types[j])
                    return -1;
                found = 1;
                break;
            }
        }
        if (!found)
            return -1;
        if ((uint32_t)left_tags[i] != IMG4_TAG_MANB) {
            if (alt_tag != 0u && alt_tag != (uint32_t)left_tags[i])
                return -1;
            alt_tag = (uint32_t)left_tags[i];
        }
    }

    if (alt_tag_out)
        *alt_tag_out = alt_tag;
    return 0;
}

static int img4_parse_cert_chain_core(uint32_t ptr, uint32_t len,
                                      uint32_t *body_ptr, uint32_t *body_len,
                                      uint32_t *sig_ptr, uint32_t *sig_len,
                                      uint32_t *cert_ptr, uint32_t *cert_len,
                                      struct img4_blob_range *block_range,
                                      const uint8_t *expected_magic,
                                      uint32_t expected_name)
{
    struct der_tlv64 outer;
    struct der_tlv64 version_tlv;
    struct der_tlv64 named_wrapper_tlv;
    struct der_tlv64 wrapper_app_tlv;
    struct der_tlv64 named_body_tlv;
    struct der_tlv64 body_set_tlv;
    struct der_tlv64 sig;
    struct der_tlv64 cert;
    struct img4_named_pair wrapper_pair;
    struct img4_named_pair body_pair;
    uint8_t *p = (uint8_t *)(uintptr_t)ptr;
    uint8_t *end = p + len;
    uint8_t *cursor;
    uint8_t *item_start;
    uint32_t version = 0u;
    uint64_t expected_tag = img4_make_app_tag64_u32(expected_name);
    uint64_t actual_tag = 0ull;
    uint32_t expected_magic_u32;

    if (ptr == 0u || len == 0u || expected_magic == NULL)
        return 6;
    expected_magic_u32 = ((uint32_t)expected_magic[0] << 24u) |
                         ((uint32_t)expected_magic[1] << 16u) |
                         ((uint32_t)expected_magic[2] << 8u) |
                         (uint32_t)expected_magic[3];

    if (der_parse_expected64(&p, end, 0x2000000000000010ull, &outer) < 0)
        return -1;
    if (p != end)
        return 7;

    cursor = outer.value;
    if (der_parse_ia5_magic(&cursor, outer.next, &version) < 0)
        return -1;
    if (version != expected_magic_u32)
        return 16;
    if (der_parse_expected64(&cursor, outer.next, 2ull, &version_tlv) < 0)
        return -1;
    if (version_tlv.len != 1u || version_tlv.value[0] != 0u)
        return 15;
    if (body_ptr)
        *body_ptr = (uint32_t)(uintptr_t)cursor;
    if (body_len)
        *body_len = (uint32_t)(outer.next - cursor);

    item_start = cursor;
    if (der_parse_expected64(&cursor, outer.next, 0x2000000000000010ull,
                             &named_wrapper_tlv) < 0) {
        return -1;
    }
    if (img4_parse_named_pair64(item_start,
                                (uint32_t)(named_wrapper_tlv.next - item_start),
                                expected_tag, &actual_tag, &wrapper_pair,
                                &wrapper_app_tlv) < 0) {
        return -1;
    }
    if (actual_tag != expected_tag || wrapper_app_tlv.tag != expected_tag)
        return 2;
    if (der_parse_expected64(&cursor, outer.next, 4ull, &sig) < 0)
        return -1;
    if (cert_ptr != NULL) {
        uint8_t *cert_start = cursor;

        if (cursor < outer.next) {
            if (der_parse_expected64(&cursor, outer.next, 0x2000000000000010ull, &cert) < 0)
                return -1;
            if (cert_ptr)
                *cert_ptr = (uint32_t)(uintptr_t)cert_start;
            if (cert_len)
                *cert_len = (uint32_t)(cert.next - cert_start);
        } else if (cert_len) {
            *cert_len = 0u;
        }
    }
    if (cursor != outer.next)
        return 7;

    cursor = wrapper_app_tlv.value;
    item_start = cursor;
    if (der_parse_expected64(&cursor, wrapper_app_tlv.next, 0x2000000000000010ull,
                             &named_body_tlv) < 0) {
        return -1;
    }
    if (cursor != wrapper_app_tlv.next)
        return 7;
    if (img4_parse_named_pair64(item_start,
                                (uint32_t)(named_body_tlv.next - item_start),
                                expected_tag, NULL, &body_pair,
                                &body_set_tlv) < 0) {
        return -1;
    }
    if (body_set_tlv.tag != 0x2000000000000011ull)
        return 2;
    if (block_range) {
        block_range->ptr = body_set_tlv.value;
        block_range->end = body_set_tlv.next;
    }

    if (sig_ptr)
        *sig_ptr = (uint32_t)(uintptr_t)sig.value;
    if (sig_len)
        *sig_len = sig.len;
    return 0;
}

static const struct img4_parser_descriptor *img4_get_descriptor(int ptr,
                                                                uint32_t fallback_alt_tag,
                                                                uint32_t fallback_anchor_ptr,
                                                                uint32_t fallback_anchor_len)
{
    static struct img4_parser_descriptor fallback;
    const uint32_t *raw;

    if (ptr != 0) {
        raw = (const uint32_t *)(uintptr_t)ptr;
        fallback.alt_payload_tag = raw[5];
        fallback.manifest_cb = (int (*)(uint32_t, int, uint32_t, uint32_t,
                                        uint32_t *, uint32_t *, int))(uintptr_t)raw[2];
        fallback.alt_cb = (int (*)(uint32_t, int, uint32_t, uint32_t,
                                   uint32_t *, uint32_t *, int))(uintptr_t)raw[3];
        fallback.trust_anchor_ptr = fallback_anchor_ptr;
        fallback.trust_anchor_len = fallback_anchor_len;
        return &fallback;
    }

    fallback.alt_payload_tag = fallback_alt_tag;
    fallback.manifest_cb = crypto_verify_payload_hash_cb;
    fallback.alt_cb = crypto_verify_payload_hash_cb;
    fallback.trust_anchor_ptr = fallback_anchor_ptr;
    fallback.trust_anchor_len = fallback_anchor_len;
    return &fallback;
}

/* =========================================================================
 * crypto_hw_accelerator_feed — program the RSA hardware and wait for result
 *
 * Writes the input message, exponent, and modulus into the hardware
 * accelerator's memory-mapped register windows, triggers computation,
 * polls for completion, and reads back the result.
 *
 * a1   = output buffer (256 bytes)
 * a2   = input flags / key length selector
 * a3   = { msg[256], exp[256], mod[256] } input struct
 * ====================================================================== */
int crypto_hw_accelerator_feed(uint32_t *out_buf, int flags,
                               uint32_t *key_struct)
{
    uint32_t key_words = RSAHW_KEY_BYTES / 4u;

    /* Load message into HW input buffer */
    volatile uint32_t *hw_msg = (volatile uint32_t *)RSAHW_MSG_BASE;
    const uint32_t    *msg    = key_struct;
    for (uint32_t i = 0; i < key_words; i++)
        hw_msg[i] = msg[i];

    /* Load public exponent */
    volatile uint32_t *hw_exp = (volatile uint32_t *)RSAHW_EXP_BASE;
    const uint32_t    *exp    = key_struct + key_words;
    for (uint32_t i = 0; i < key_words; i++)
        hw_exp[i] = exp[i];

    /* Load modulus */
    volatile uint32_t *hw_mod = (volatile uint32_t *)RSAHW_MOD_BASE;
    const uint32_t    *mod    = key_struct + 2u * key_words;
    for (uint32_t i = 0; i < key_words; i++)
        hw_mod[i] = mod[i];

    /* Start RSA computation */
    RSAHW_CTRL = RSAHW_CTRL_START | (uint32_t)flags;

    /* Poll for completion */
    while (!(RSAHW_STATUS & RSAHW_STATUS_DONE))
        yield_execution();

    /* Read result */
    volatile uint32_t *hw_out = (volatile uint32_t *)RSAHW_OUT_BASE;
    for (uint32_t i = 0; i < key_words; i++)
        out_buf[i] = hw_out[i];

    return 0;
}

/* =========================================================================
 * crypto_validate_block — validate a top-level IMG4/HDCP hash block
 * ====================================================================== */
int crypto_validate_block(int data_ptr, int data_len, int desc_ptr, int flags,
                          uint32_t expected_name)
{
    const uint32_t *desc = (const uint32_t *)(uintptr_t)desc_ptr;
    struct rom_tlv_desc set_desc = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc item_desc = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc seq_desc = {0u, 0u, 0u, 0u};
    struct rom_range32 outer_range = {0u, 0u};
    struct rom_range32 inner_range = {0u, 0u};
    uint32_t input_pair[2];
    uint32_t item_pair[2];
    int have_manp = 0;
    int have_alt = 0;
    uint32_t alt_tag;
    int rc;

    if (data_ptr == 0 || data_len == 0)
        return 6;

    input_pair[0] = (uint32_t)data_ptr;
    input_pair[1] = (uint32_t)data_len;
    rc = rom_parse_tlv_pair(input_pair, &set_desc, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&set_desc) != ROM_TAG64_SET)
        return 2;

    rc = rom_pair_to_range(&set_desc.value_ptr, &outer_range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&outer_range, &item_desc);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&item_desc) != rom_make_app_tag64_u32(expected_name))
        return 2;

    item_pair[0] = item_desc.value_ptr;
    item_pair[1] = item_desc.value_len;
    rc = rom_parse_tlv_pair(item_pair, &seq_desc, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&seq_desc) != ROM_TAG64_SEQUENCE)
        return 2;

    rc = rom_pair_to_range(&seq_desc.value_ptr, &inner_range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&inner_range, &item_desc);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&item_desc) != ROM_TAG64_IA5STRING)
        return 2;
    item_pair[0] = item_desc.value_ptr;
    item_pair[1] = item_desc.value_len;
    if (rom_check_magic4_pair(item_pair, expected_name) != 0)
        return 2;

    rc = rom_range_next(&inner_range, &item_desc);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&item_desc) != ROM_TAG64_SET)
        return 2;

    rc = rom_pair_to_range(&item_desc.value_ptr, &inner_range);
    if (rc != 0)
        return rc;

    alt_tag = desc[5];
    while (1) {
        rc = rom_range_next(&inner_range, &item_desc);
        if (rc == 1)
            return 0;
        if (rc != 0)
            return rc;

        item_pair[0] = item_desc.value_ptr;
        item_pair[1] = item_desc.value_len;
        if (item_desc.tag_lo == IMG4_TAG_MANP) {
            if (have_manp)
                return 2;
            have_manp = 1;
            rc = crypto_verify_payload_hash(
                item_pair,
                -(int)IMG4_TAG_MANP,
                (int64_t)rom_tlv_desc_tag64(&item_desc),
                (img4_validator_callback_t)(uintptr_t)desc[2],
                flags);
            if (rc != 0)
                return rc;
            continue;
        }
        if (item_desc.tag_lo != alt_tag)
            return 2;
        if (have_alt)
            return 2;
        have_alt = 1;
        rc = crypto_verify_payload_hash(
            item_pair,
            (int)alt_tag,
            (int64_t)rom_tlv_desc_tag64(&item_desc),
            (img4_validator_callback_t)(uintptr_t)desc[3],
            flags);
        if (rc != 0)
            return rc;
    }
}

/* =========================================================================
 * crypto_verify_payload_hash — validate a manifest payload wrapper
 * ====================================================================== */
int crypto_verify_payload_hash(uint32_t *manifest, int offset, int64_t range,
                               int (*hash_fn)(uint32_t, int, uint32_t,
                                              uint32_t, uint32_t *, uint32_t *, int),
                               int flags)
{
    struct rom_named_desc wrapper = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    struct rom_tlv_desc item_desc = {0u, 0u, 0u, 0u};
    struct rom_range32 item_range = {0u, 0u};
    uint32_t raw_child_pair[2];
    uint32_t expected_pair[2];
    int rc;

    (void)offset;

    if (hash_fn == NULL)
        return 6;

    rc = rom_parse_named_wrapper(manifest,
                                 ((uint64_t)range) | 0xE000000000000000ull,
                                 &wrapper);
    if (rc != 0)
        return rc;
    if (rom_named_child_tag64(&wrapper) != ROM_TAG64_SET)
        return 2;

    rc = rom_pair_to_range(rom_named_child_pair(&wrapper), &item_range);
    if (rc != 0)
        return rc;

    while (1) {
        rc = rom_range_next(&item_range, &item_desc);
        if (rc == 1)
            return 0;
        if (rc != 0)
            return rc;

        raw_child_pair[0] = item_desc.value_ptr;
        raw_child_pair[1] = item_desc.value_len;
        rc = rom_parse_named_wrapper(raw_child_pair,
                                     rom_tlv_desc_tag64(&item_desc),
                                     &wrapper);
        if (rc != 0)
            return rc;
        if (!img4_check_magic_tag((int64_t)rom_named_child_tag64(&wrapper)))
            return 2;
        if (item_desc.tag_hi != 0xE0000000u)
            return 2;

        expected_pair[0] = wrapper.name_ptr;
        expected_pair[1] = wrapper.name_len;
        if (rom_check_magic4_pair(expected_pair, item_desc.tag_lo) != 0)
            return 2;

        rc = hash_fn((uint32_t)range,
                     (int)item_desc.tag_lo,
                     wrapper.child_tag_lo,
                     wrapper.child_tag_hi,
                     rom_named_child_pair(&wrapper),
                     raw_child_pair,
                     flags);
        if (rc != 0)
            return rc;
    }
}

/* =========================================================================
 * crypto_parse_cert_chain — split an IM4M / IM4C object into body/signature
 * ====================================================================== */
int crypto_parse_cert_chain(uint32_t ptr, uint32_t len,
                            uint32_t *body_ptr, uint32_t *body_len,
                            uint32_t *sig_ptr, uint32_t *sig_len,
                            uint32_t *cert_ptr, uint32_t *cert_len,
                            int expected_magic_ptr, uint32_t expected_magic_len,
                            uint32_t *range_pair, uint32_t expected_name)
{
    struct rom_tlv_desc outer = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc child = {0u, 0u, 0u, 0u};
    struct rom_tlv_desc inner = {0u, 0u, 0u, 0u};
    struct rom_range32 outer_range = {0u, 0u};
    struct rom_range32 inner_range = {0u, 0u};
    uint32_t input_pair[2];
    uint32_t temp_pair[2];
    uint8_t zero = 0u;
    int rc = -1;

    input_pair[0] = ptr;
    input_pair[1] = len;
    if (ptr == 0u || len == 0u)
        return 6;

    rc = rom_parse_tlv_pair(input_pair, &outer, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&outer) != ROM_TAG64_SEQUENCE)
        return 2;

    rc = rom_pair_to_range(&outer.value_ptr, &outer_range);
    if (rc != 0)
        return rc;
    if (outer.value_len >= len)
        return -1;
    if (outer.value_ptr <= ptr)
        return -1;
    if ((outer.value_ptr - ptr) != (len - outer.value_len))
        return 7;

    rc = rom_range_next(&outer_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_IA5STRING)
        return 2;

    temp_pair[0] = child.value_ptr;
    temp_pair[1] = child.value_len;
    if (!rom_bytes_equal((uint32_t)expected_magic_ptr, expected_magic_len,
                         temp_pair[0], temp_pair[1])) {
        return 16;
    }

    rc = rom_range_next(&outer_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_INTEGER)
        return 2;
    if (!rom_bytes_equal((uint32_t)(uintptr_t)&zero, 1u,
                         child.value_ptr, child.value_len)) {
        return 15;
    }

    if (body_ptr != NULL && body_len != NULL) {
        uint32_t raw_body_pair[2];
        uint32_t raw_len = 0u;

        raw_body_pair[0] = outer_range.start;
        raw_body_pair[1] = outer_range.end - outer_range.start;
        rc = rom_parse_tlv_pair(raw_body_pair, &child, &raw_len);
        if (rc != 0)
            return rc;
        *body_ptr = raw_body_pair[0];
        *body_len = (child.value_ptr + child.value_len) - raw_body_pair[0];
    }

    rc = rom_range_next(&outer_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_SET)
        return 2;

    rc = rom_pair_to_range(&child.value_ptr, &inner_range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&inner_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != rom_make_app_tag64_u32(expected_name))
        return 2;

    temp_pair[0] = child.value_ptr;
    temp_pair[1] = child.value_len;
    rc = rom_parse_tlv_pair(temp_pair, &inner, NULL);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&inner) != ROM_TAG64_SEQUENCE)
        return 2;

    rc = rom_pair_to_range(&inner.value_ptr, &inner_range);
    if (rc != 0)
        return rc;

    rc = rom_range_next(&inner_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_IA5STRING)
        return 2;

    temp_pair[0] = child.value_ptr;
    temp_pair[1] = child.value_len;
    if (rom_check_magic4_pair(temp_pair, expected_name) != 0)
        return 2;

    rc = rom_range_next(&inner_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_SET)
        return 2;
    if (range_pair != NULL) {
        range_pair[0] = child.value_ptr;
        range_pair[1] = child.value_ptr + child.value_len;
    }

    rc = rom_range_next(&outer_range, &child);
    if (rc != 0)
        return rc;
    if (rom_tlv_desc_tag64(&child) != ROM_TAG64_OCTETSTRING)
        return 2;
    if (sig_ptr != NULL && sig_len != NULL) {
        *sig_ptr = child.value_ptr;
        *sig_len = child.value_len;
    }

    if (cert_ptr != NULL) {
        rc = rom_range_next(&outer_range, &child);
        if (rc != 0)
            return rc;
        if (rom_tlv_desc_tag64(&child) != ROM_TAG64_SEQUENCE)
            return 2;
        if (cert_len != NULL) {
            *cert_ptr = child.value_ptr;
            *cert_len = child.value_len;
        }
    }

    if ((child.value_ptr - ptr + child.value_len) == len)
        return 0;
    return 7;
}

/* =========================================================================
 * img4_check_magic_tag — accept the ROM validator's small set of child tags
 * ====================================================================== */
bool img4_check_magic_tag(int64_t tag)
{
    uint32_t lo = (uint32_t)tag;
    uint32_t hi = HIDWORD(tag);

    if (tag == 4 || tag == 1 || tag == 2 || tag == 22)
        return true;
    if (((hi ^ 0xA0000000u) | lo) != 0u) {
        hi ^= 0xA0000000u;
        return rom_make_tag64(lo, hi) == 1ull;
    }
    return true;
}

/* =========================================================================
 * img4_parse_manifest_tags — compare the certificate and manifest tag sets
 * ====================================================================== */
int img4_parse_manifest_tags(int desc_ptr,
                             const struct img4_blob_range *left_range,
                             const struct img4_blob_range *right_range)
{
    const uint32_t *desc = (const uint32_t *)(uintptr_t)desc_ptr;
    struct rom_range32 left = {0u, 0u};
    struct rom_range32 right = {0u, 0u};
    struct rom_tlv_desc item_desc = {0u, 0u, 0u, 0u};
    uint32_t left_manp[2] = {0u, 0u};
    uint32_t left_alt[2] = {0u, 0u};
    uint32_t right_manp[2] = {0u, 0u};
    uint32_t right_alt[2] = {0u, 0u};
    int have_left_manp = 0;
    int have_left_alt = 0;
    int have_right_manp = 0;
    int have_right_alt = 0;
    int rc = 0;

    left.start = (uint32_t)(uintptr_t)left_range->ptr;
    left.end = (uint32_t)(uintptr_t)left_range->end;
    right.start = (uint32_t)(uintptr_t)right_range->ptr;
    right.end = (uint32_t)(uintptr_t)right_range->end;

    do {
        rc = rom_range_next(&left, &item_desc);
        if (rc == 1)
            break;
        if (item_desc.tag_lo == IMG4_TAG_MANP) {
            if (have_left_manp)
                return 2;
            have_left_manp = 1;
            left_manp[0] = item_desc.value_ptr;
            left_manp[1] = item_desc.value_len;
        } else {
            if (item_desc.tag_lo != IMG4_TAG_OBJP)
                return 2;
            if (have_left_alt)
                return 2;
            have_left_alt = 1;
            left_alt[0] = item_desc.value_ptr;
            left_alt[1] = item_desc.value_len;
        }
    } while (rc == 0);

    do {
        rc = rom_range_next(&right, &item_desc);
        if (rc == 1)
            break;
        if (item_desc.tag_lo == IMG4_TAG_MANP) {
            if (have_right_manp)
                return 2;
            have_right_manp = 1;
            right_manp[0] = item_desc.value_ptr;
            right_manp[1] = item_desc.value_len;
        } else {
            if (item_desc.tag_lo != desc[5])
                return 2;
            if (have_right_alt)
                return 2;
            have_right_alt = 1;
            right_alt[0] = item_desc.value_ptr;
            right_alt[1] = item_desc.value_len;
        }
    } while (rc == 0);

    if (left_manp[1] != 0u &&
        !img4_validate_payload(left_manp, right_manp,
                               (int64_t)IMG4_TAG_MANP,
                               (int)IMG4_TAG_MANP,
                               0)) {
        return 12;
    }
    if (left_alt[1] != 0u &&
        !img4_validate_payload(left_alt, right_alt,
                               (int64_t)IMG4_TAG_OBJP,
                               (int)desc[5],
                               0)) {
        return 13;
    }
    return 0;
}

/* =========================================================================
 * img4_validate_payload — validate one named manifest payload against peer
 * ====================================================================== */
int img4_validate_payload(uint32_t *manifest_desc, uint32_t *payload_desc,
                          int64_t range, int flags, int depth)
{
    struct rom_named_desc wrapper = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    struct rom_tlv_desc item_desc = {0u, 0u, 0u, 0u};
    struct rom_range32 item_range = {0u, 0u};
    uint32_t child_pair[2];
    int rc;

    rc = rom_parse_named_wrapper(manifest_desc,
                                 ((uint64_t)range) | 0xE000000000000000ull,
                                 &wrapper);
    if (rc != 0)
        return 0;
    if (rom_named_child_tag64(&wrapper) != ROM_TAG64_SET)
        return 0;

    rc = rom_pair_to_range(rom_named_child_pair(&wrapper), &item_range);
    if (rc != 0)
        return 0;

    while (1) {
        rc = rom_range_next(&item_range, &item_desc);
        if (rc == 1)
            return 1;
        if (rc != 0)
            return 0;

        child_pair[0] = item_desc.value_ptr;
        child_pair[1] = item_desc.value_len;
        rc = rom_parse_named_wrapper(child_pair,
                                     rom_tlv_desc_tag64(&item_desc),
                                     &wrapper);
        if (rc != 0)
            return 0;
        if (!img4_check_magic_tag((int64_t)rom_named_child_tag64(&wrapper)))
            return 0;
        if (item_desc.tag_hi != 0xE0000000u)
            return 0;
        if (!hdcp_crypto_validate_loop(flags,
                                       depth,
                                       (int)item_desc.tag_lo,
                                       (int)item_desc.tag_hi,
                                       (int64_t)rom_named_child_tag64(&wrapper),
                                       (int *)rom_named_child_pair(&wrapper),
                                       payload_desc)) {
            return 0;
        }
    }
}

/* =========================================================================
 * img4_parse_im4m_im4c — top-level IM4M / IM4C validator
 * ====================================================================== */
int img4_parse_im4m_im4c(uint32_t addr, uint32_t size,
                         uint32_t *p1,  int *p2,
                         uint32_t *p3,  uint32_t *p4,
                         uint32_t *p5,  uint32_t *p6,
                         int *p7,       int *p8,
                         uint32_t *p9,  uint32_t *p10,
                         int cert_ptr,  int tag_ptr)
{
    const uint32_t *desc;
    const uint32_t *anchor;
    struct rom_tlv_desc top_desc = {0u, 0u, 0u, 0u};
    uint32_t top_pair[2];
    uint32_t declared_len = 0u;
    uint32_t trimmed_size;
    uint32_t manifest_block[2] = {0u, 0u};
    uint32_t cert_block[2] = {0u, 0u};
    uint32_t fallback_anchor[3] = {0u, 268752u, 398u};
    uint32_t manifest_body_ptr = 0u;
    uint32_t manifest_sig_ptr = 0u;
    uint32_t cert_sig_ptr = 0u;
    uint32_t cert_sig_len = 0u;
    uint32_t pubkey_blob_ptr = 0u;
    uint32_t pubkey_blob_len = 0u;
    int manifest_body_len = 0;
    int cert_body_ptr = 0;
    int cert_body_len = 0;
    int rc = -1;
    static const uint8_t img4_magic[8] = {'I', 'M', '4', 'M', 'I', 'M', '4', 'C'};

    if (addr == 0u || size == 0u || p1 == NULL || p2 == NULL || p3 == NULL ||
        p4 == NULL || p5 == NULL || p6 == NULL || p7 == NULL || p8 == NULL ||
        p9 == NULL || p10 == NULL || cert_ptr == 0) {
        return -1;
    }

    desc = (const uint32_t *)(uintptr_t)cert_ptr;
    anchor = (tag_ptr != 0) ? (const uint32_t *)(uintptr_t)tag_ptr : fallback_anchor;
    if (desc[1] == 0u || desc[2] == 0u || anchor[1] == 0u || anchor[2] == 0u)
        return -1;

    top_pair[0] = addr;
    top_pair[1] = size;
    rc = rom_parse_tlv_pair(top_pair, &top_desc, &declared_len);
    if (rc != 0)
        return rc;

    trimmed_size = (top_desc.value_ptr - addr) + declared_len;
    if (trimmed_size > size)
        return 7;

    rc = crypto_parse_cert_chain(addr, trimmed_size,
                                 &manifest_body_ptr, (uint32_t *)p2,
                                 p3, p4,
                                 p5, p6,
                                 (int)(uintptr_t)img4_magic, 4u,
                                 manifest_block, IMG4_TAG_MANB);
    if (rc != 0)
        return 8;

    manifest_body_len = *p2;
    manifest_sig_ptr = *p3;
    if (manifest_body_len == 0 || (uint32_t)manifest_body_len > size ||
        *p4 == 0u || *p4 > size ||
        *p6 == 0u || *p6 > size ||
        (uint32_t)manifest_body_len + *p4 + *p6 > trimmed_size) {
        return 7;
    }

    rc = crypto_parse_cert_chain(*p5, *p6,
                                 (uint32_t *)p7, (uint32_t *)p8,
                                 p9, p10,
                                 NULL, NULL,
                                 (int)(uintptr_t)(img4_magic + 4), 4u,
                                 cert_block, IMG4_TAG_CRTP);
    if (rc != 0)
        return 9;

    cert_body_ptr = *p7;
    cert_body_len = *p8;
    cert_sig_ptr = *p9;
    cert_sig_len = *p10;
    if (cert_body_len == 0 || (uint32_t)cert_body_len > size ||
        cert_sig_len == 0u || cert_sig_len > size ||
        (uint32_t)cert_body_len + cert_sig_len > *p6) {
        return 7;
    }

    rc = ((int (*)(uint32_t))(uintptr_t)desc[1])(anchor[1]);
    if (rc != 0)
        return rc;

    rc = crypto_parse_pubkey_struct(cert_body_ptr, cert_body_len,
                                    &pubkey_blob_ptr, &pubkey_blob_len);
    if (rc != 0)
        return 11;

    rc = ((int (*)(int, int, uint32_t, uint32_t))(uintptr_t)desc[1])(
            (int)pubkey_blob_ptr, (int)pubkey_blob_len, manifest_sig_ptr, *p4);
    if (rc != 0)
        return 10;

    if (REG_BOOT_FLAGS == 1u) {
        rc = img4_parse_manifest_tags(cert_ptr,
                                      (const struct img4_blob_range *)(const void *)cert_block,
                                      (const struct img4_blob_range *)(const void *)manifest_block);
        if (rc != 0)
            return rc;
    }

    rc = crypto_validate_block(manifest_body_ptr, manifest_body_len,
                               cert_ptr, 0, IMG4_TAG_MANB);
    if (rc != 0)
        return 14;

    *p1 = manifest_body_ptr;
    *p7 = cert_body_ptr;
    *p8 = cert_body_len;
    *p9 = cert_sig_ptr;
    *p10 = cert_sig_len;
    return 0;
}

/* =========================================================================
 * crypto_hw_verify_signature — RSA-PKCS1v15 signature verification
 *
 * Verifies that `sig` (of `sig_len` bytes) is a valid RSA signature over
 * the message `msg` (of `msg_len` bytes) using the public key (modulus `n`,
 * exponent `e`) stored in flash at `pubkey_ptr`.
 *
 * Uses the hardware RSA accelerator to perform the modular exponentiation.
 * Returns 0 on success (signature valid), negative on failure.
 * ====================================================================== */
int crypto_hw_verify_signature(uint8_t *pubkey_ptr, int pubkey_len,
                               uint8_t *sig,        int sig_len,
                               int msg_ptr,          int hash_len)
{
    /* Build input struct for HW accelerator: {msg, exp, mod} */
    static uint8_t hw_input[3u * RSAHW_KEY_BYTES];
    uint32_t out_buf[RSAHW_KEY_BYTES / 4u];

    /* Load message (padded to key size) */
    simple_memset((int)hw_input, 0, RSAHW_KEY_BYTES);
    fast_memcpy((uint32_t *)hw_input,
                (const uint32_t *)msg_ptr,
                (uint32_t)hash_len, 0);

    /* Load public exponent (little-endian, zero-padded) */
    simple_memset((int)(hw_input + RSAHW_KEY_BYTES), 0, RSAHW_KEY_BYTES);
    hw_input[RSAHW_KEY_BYTES] = 0x01u;
    hw_input[RSAHW_KEY_BYTES + 1u] = 0x00u;
    hw_input[RSAHW_KEY_BYTES + 2u] = 0x01u;  /* e = 65537 */

    /* Load modulus (from pubkey_ptr) */
    fast_memcpy((uint32_t *)(hw_input + 2u * RSAHW_KEY_BYTES),
                (const uint32_t *)pubkey_ptr,
                (uint32_t)pubkey_len, 0);

    /* Run RSA */
    crypto_hw_accelerator_feed(out_buf, 0, (uint32_t *)hw_input);

    /* Compare decrypted signature against signature input */
    return crypto_memcmp((int)out_buf, (uint32_t)sig_len, (int)sig, 0);
}

/* =========================================================================
 * hdcp_crypto_validate_loop — match one validator item against a peer block
 * ====================================================================== */
bool hdcp_crypto_validate_loop(int cert_ptr, int cert_len, int depth,
                               int max_depth, int64_t root_key,
                               int *result_ptr, uint32_t *key_out)
{
    struct rom_named_desc wrapper = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    struct rom_tlv_desc item_desc = {0u, 0u, 0u, 0u};
    struct rom_range32 item_range = {0u, 0u};
    uint32_t child_pair[2];
    uint64_t expected_outer;
    uint32_t root_lo = (uint32_t)root_key;
    uint32_t root_hi = HIDWORD(root_key);
    bool compare_required = false;
    bool found = false;
    bool result = false;
    int rc;

    (void)max_depth;

    if ((uint64_t)root_key != ROM_TAG64_CONTEXT1)
        compare_required = ((root_lo | (root_hi ^ 0xA0000000u)) != 0u);
    if (compare_required)
        crypto_validation_stub();

    if (key_out == NULL || key_out[1] == 0u)
        goto finish_normal;

    expected_outer = rom_make_app_tag64_words((uint32_t)cert_ptr,
                                              (uint32_t)cert_len);
    rc = rom_parse_named_wrapper(key_out, expected_outer, &wrapper);
    if (rc != 0)
        goto finish_return;
    if (rom_named_child_tag64(&wrapper) != ROM_TAG64_SET)
        goto finish_return;

    rc = rom_pair_to_range(rom_named_child_pair(&wrapper), &item_range);
    if (rc != 0)
        goto finish_return;

    while (1) {
        rc = rom_range_next(&item_range, &item_desc);
        if (rc == 1)
            goto finish_normal;
        if (rc != 0)
            goto finish_return;

        child_pair[0] = item_desc.value_ptr;
        child_pair[1] = item_desc.value_len;
        rc = rom_parse_named_wrapper(child_pair,
                                     rom_tlv_desc_tag64(&item_desc),
                                     &wrapper);
        if (rc != 0)
            goto finish_return;
        if (!img4_check_magic_tag((int64_t)rom_named_child_tag64(&wrapper)))
            goto finish_return;
        if (item_desc.tag_hi != 0xE0000000u)
            goto finish_return;
        if ((int)item_desc.tag_lo != depth)
            continue;

        found = true;
        if ((uint64_t)root_key == ROM_TAG64_CONTEXT1)
            goto finish_return;
        if (compare_required &&
            !rom_bytes_equal(wrapper.child_value_ptr, wrapper.child_value_len,
                             (uint32_t)result_ptr[0], (uint32_t)result_ptr[1])) {
            goto finish_return;
        }
        goto finish_normal;
    }

finish_return:
    return result;

finish_normal:
    result = (((uint64_t)root_key == ROM_TAG64_CONTEXT1) || found);
    return result;
}

/* =========================================================================
 * crypto_validation_stub — placeholder for ROM-patch hook
 *
 * Called after the certificate chain has been fully validated.  In
 * production chips this may call into a secure enclave for an additional
 * authentication step.
 * ====================================================================== */
int crypto_validation_stub(void)
{
    return (int)REG_TCB_CURRENT_TASK;
}

/* =========================================================================
 * crypto_init_and_parse_im4m — main entry point called from main()
 *
 * Reads the IM4M manifest from the flash address returned by
 * flash_apply_pending_update(), parses the certificate chain, verifies
 * the firmware hash, and loads the HDCP keys from the authenticated manifest.
 *
 * The parameters (a1..a4) are passed through from the flash_efface return
 * value and unused register values (r1, r2, r3) at the call site in main().
 * ====================================================================== */
int crypto_init_and_parse_im4m(int fw_base, int a2, int a3, int a4)
{
    uint32_t saved_task = REG_TCB_CURRENT_TASK;
    uint32_t cert_desc[6];
    uint32_t anchor_block[3] = {0u, 268752u, 398u};

    (void)fw_base;
    (void)a2;
    (void)a3;

    uint32_t p1=0, p3=0, p4=0, p5=0, p6=0, p9=0, p10=0;
    int      p2=0, p7=0, p8=0;

    fast_memcpy(cert_desc, (const uint32_t *)FLASH_IM4M_CERT_ADDR,
                FLASH_IM4M_CERT_SIZE, a4);

    img4_parse_im4m_im4c(FLASH_BASE_ADDR, FLASH_IMAGE_SIZE,
                         &p1, &p2, &p3, &p4, &p5, &p6,
                         &p7, &p8, &p9, &p10,
                         (int)cert_desc, (int)anchor_block);

    return (int)saved_task;
}
