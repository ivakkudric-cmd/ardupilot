#ifndef AGRI_SPRAY_PROTO_H
#define AGRI_SPRAY_PROTO_H
#include "agri_spray_layout.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ASP_MAX_PAYLOAD 64u
#define ASP_MAX_WIRE 72u
typedef struct {
    uint8_t type, length, payload[ASP_MAX_PAYLOAD];
} asp_message;
typedef enum {
    ASP_NONE = 0,
    ASP_MESSAGE,
    ASP_BAD_COBS,
    ASP_BAD_CRC,
    ASP_BAD_VERSION,
    ASP_BAD_LENGTH,
    ASP_TIMEOUT,
    ASP_OVERFLOW
} asp_result;
typedef struct {
    uint8_t encoded[71];
    uint8_t count;
    bool discard;
    uint32_t last_ms, timeout_ms;
} asp_decoder;
void asp_decoder_init(asp_decoder *, uint32_t timeout_ms);
asp_result asp_feed(asp_decoder *, uint8_t byte, uint32_t now, asp_message *);
size_t asp_encode(const asp_message *, uint8_t *wire, size_t capacity);
uint16_t asp_crc16(const uint8_t *, size_t);
uint32_t asp_crc32(const uint8_t *, size_t);
uint16_t asp_u16(const uint8_t *);
uint32_t asp_u32(const uint8_t *);
void asp_put16(uint8_t *, uint16_t);
void asp_put32(uint8_t *, uint32_t);
void asp_message_init(asp_message *, uint8_t type);
uint32_t asp_session_id(uint32_t cube_nonce, uint32_t mcu_nonce);
typedef struct {
    uint32_t id, cube_nonce, mcu_nonce, seq, last_permit_seq, started_ms;
    bool ready, need_inhibit;
} asp_session;
typedef enum { ASP_ACCEPT = 0, ASP_SESSION, ASP_SEQUENCE, ASP_HANDSHAKE, ASP_SEMANTIC } asp_reject;
bool asp_session_begin(asp_session *, uint32_t cube, uint32_t mcu, uint32_t now);
asp_reject asp_session_check(const asp_session *, const asp_message *, uint32_t now);
void asp_session_commit(asp_session *, const asp_message *);
void asp_session_inhibit(asp_session *);
#endif
