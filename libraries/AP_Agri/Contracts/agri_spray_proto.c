#include "agri_spray_proto.h"
#include <string.h>
uint16_t asp_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8); }
uint32_t asp_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
void asp_put16(uint8_t *p, uint16_t x) {
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
}
void asp_put32(uint8_t *p, uint32_t x) {
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
    p[2] = (uint8_t)(x >> 16);
    p[3] = (uint8_t)(x >> 24);
}
uint16_t asp_crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (unsigned j = 0; j < 8; j++)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}
uint32_t asp_crc32(const uint8_t *p, size_t n) {
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (unsigned j = 0; j < 8; j++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
    }
    return crc ^ 0xffffffff;
}
void asp_message_init(asp_message *m, uint8_t type) {
    memset(m, 0, sizeof(*m));
    m->type = type;
    if (type > 0 && type < sizeof(asp_lengths))
        m->length = asp_lengths[type];
}
void asp_decoder_init(asp_decoder *d, uint32_t timeout) {
    memset(d, 0, sizeof(*d));
    d->timeout_ms = timeout ? timeout : 20;
}
size_t asp_encode(const asp_message *m, uint8_t *out, size_t capacity) {
    if (!m || !out || m->type == 0 || m->type >= sizeof(asp_lengths) ||
        m->length != asp_lengths[m->type] || capacity < (size_t)m->length + 8)
        return 0;
    uint8_t raw[70];
    raw[0] = ASP_VERSION;
    raw[1] = m->type;
    asp_put16(raw + 2, m->length);
    memcpy(raw + 4, m->payload, m->length);
    size_t n = m->length + 6;
    asp_put16(raw + n - 2, asp_crc16(raw, n - 2));
    size_t code_at = 0, pos = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < n; i++) {
        if (raw[i] == 0) {
            out[code_at] = code;
            code_at = pos++;
            code = 1;
        } else {
            out[pos++] = raw[i];
            code++;
        }
    }
    out[code_at] = code;
    out[pos++] = 0;
    return pos;
}
asp_result asp_feed(asp_decoder *d, uint8_t b, uint32_t now, asp_message *m) {
    if (d->count && (uint32_t)(now - d->last_ms) > d->timeout_ms) {
        d->count = 0;
        d->discard = b != 0;
        d->last_ms = now;
        return ASP_TIMEOUT;
    }
    d->last_ms = now;
    if (d->discard) {
        if (!b)
            d->discard = false;
        return ASP_NONE;
    }
    if (b) {
        if (d->count >= sizeof(d->encoded)) {
            d->count = 0;
            d->discard = true;
            return ASP_OVERFLOW;
        }
        d->encoded[d->count++] = b;
        return ASP_NONE;
    }
    if (!d->count)
        return ASP_NONE;
    uint8_t raw[70];
    size_t n = 0, i = 0, count = d->count;
    d->count = 0;
    while (i < count) {
        uint8_t code = d->encoded[i++];
        if (!code || i + (size_t)code - 1 > count)
            return ASP_BAD_COBS;
        for (unsigned j = 1; j < code; j++) {
            if (n >= sizeof(raw))
                return ASP_BAD_LENGTH;
            raw[n++] = d->encoded[i++];
        }
        if (code != 255 && i < count) {
            if (n >= sizeof(raw))
                return ASP_BAD_LENGTH;
            raw[n++] = 0;
        }
    }
    if (n < 6)
        return ASP_BAD_LENGTH;
    if (asp_crc16(raw, n - 2) != asp_u16(raw + n - 2))
        return ASP_BAD_CRC;
    if (raw[0] != ASP_VERSION)
        return ASP_BAD_VERSION;
    uint8_t type = raw[1];
    if (!type || type >= sizeof(asp_lengths) || asp_u16(raw + 2) != asp_lengths[type] ||
        n != (size_t)asp_lengths[type] + 6)
        return ASP_BAD_LENGTH;
    asp_message_init(m, type);
    memcpy(m->payload, raw + 4, m->length);
    return ASP_MESSAGE;
}
uint32_t asp_session_id(uint32_t cube, uint32_t mcu) {
    uint8_t data[12] = {'A', 'G', 'S', '1'};
    asp_put32(data + 4, cube);
    asp_put32(data + 8, mcu);
    return asp_crc32(data, 12);
}
bool asp_session_begin(asp_session *s, uint32_t cube, uint32_t mcu, uint32_t now) {
    memset(s, 0, sizeof(*s));
    if (!cube || !mcu)
        return false;
    s->id = asp_session_id(cube, mcu);
    s->cube_nonce = cube;
    s->mcu_nonce = mcu;
    s->started_ms = now;
    s->need_inhibit = true;
    return s->id != 0;
}
asp_reject asp_session_check(const asp_session *s, const asp_message *m, uint32_t now) {
    if (m->type != ASP_PERMIT && m->type != ASP_GROUND_TEST)
        return ASP_SEMANTIC;
    if (!s->id || asp_u32(m->payload) != s->id)
        return ASP_SESSION;
    uint32_t seq = asp_u32(m->payload + 4);
    if (!seq || seq <= s->seq)
        return ASP_SEQUENCE;
    uint16_t flags =
        asp_u16(m->payload + (m->type == ASP_PERMIT ? ASP_PERMIT_FLAGS : ASP_GROUND_TEST_FLAGS));
    if (!s->ready &&
        ((uint32_t)(now - s->started_ms) >= 1000 || m->type != ASP_PERMIT || flags || seq != 1))
        return ASP_HANDSHAKE;
    if (s->need_inhibit && (m->type != ASP_PERMIT || flags))
        return ASP_HANDSHAKE;
    if (m->type == ASP_PERMIT) {
        uint16_t q = asp_u16(m->payload + ASP_PERMIT_Q_TARGET_ML_MIN),
                 valid = asp_u16(m->payload + ASP_PERMIT_VALIDITY_MS);
        if (asp_u16(m->payload + ASP_PERMIT_RESERVED) || flags > 1 ||
            (flags && (!q || !valid || !asp_u32(m->payload + ASP_PERMIT_PACKAGE_ID))) ||
            (!flags && (q || valid)))
            return ASP_SEMANTIC;
    } else {
        uint16_t q = asp_u16(m->payload + ASP_GROUND_TEST_Q_TARGET_ML_MIN),
                 valid = asp_u16(m->payload + ASP_GROUND_TEST_VALIDITY_MS),
                 duration = asp_u16(m->payload + ASP_GROUND_TEST_DURATION_MS);
        if (asp_u32(m->payload + ASP_GROUND_TEST_RESERVED) || flags > 1 ||
            (flags && (!q || !valid || !duration)) || (!flags && (q || valid || duration)))
            return ASP_SEMANTIC;
    }
    return ASP_ACCEPT;
}
void asp_session_commit(asp_session *s, const asp_message *m) {
    s->seq = asp_u32(m->payload + 4);
    s->ready = true;
    if (m->type == ASP_PERMIT) {
        s->last_permit_seq = s->seq;
        if (!asp_u16(m->payload + ASP_PERMIT_FLAGS))
            s->need_inhibit = false;
    } else if (m->type == ASP_GROUND_TEST && !asp_u16(m->payload + ASP_GROUND_TEST_FLAGS)) {
        s->need_inhibit = false;
    }
}
void asp_session_inhibit(asp_session *s) { s->need_inhibit = true; }
