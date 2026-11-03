#include "AP_AgriSpray.h"
#include <AP_SerialManager/AP_SerialManager.h>
extern const AP_HAL::HAL &hal;
void AP_AgriSpray::init() {
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_AgriSpray, 0);
    if (!uart || AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_AgriSpray, 1)) {
        uart = nullptr;
        return;
    }
    uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_AgriSpray, 0),
                144, 144);
    asp_decoder_init(&decoder, 20);
    restart(AP_HAL::millis());
}
void AP_AgriSpray::inhibit() {
    requested = false;
    request_pending = false;
}
void AP_AgriSpray::set_authorization(bool allow, uint16_t q, uint32_t package, uint32_t now) {
    requested = allow;
    request_q = q;
    request_package = package;
    request_time = now;
    request_pending = true;
}
bool AP_AgriSpray::healthy(uint32_t now) const {
    return ready && session && uint32_t(now - last_status) < 150 && faults == 0;
}
bool AP_AgriSpray::send(const asp_message &m) {
    uint8_t wire[ASP_MAX_WIRE];
    size_t n = asp_encode(&m, wire, sizeof(wire));
    return uart && n && uart->txspace() >= n && uart->write(wire, n) == n;
}
void AP_AgriSpray::restart(uint32_t now) {
    inhibit();
    ready = false;
    session = seq = status_seq = 0;
    mcu_nonce = 0;
    have_hello = false;
    cube_nonce = 0;
    last_hello = now - 1000;
    if (uart)
        uart->discard_input();
    asp_decoder_init(&decoder, 20);
    // VERIFY(agri): ChibiOS 4.5.7 RNG API can fall back to PRNG; validate reboot
    // uniqueness in E11.
    if (!hal.util->get_random_vals(reinterpret_cast<uint8_t *>(&cube_nonce), sizeof(cube_nonce)))
        cube_nonce = 0;
}
void AP_AgriSpray::receive(const asp_message &m, uint32_t now) {
    if (m.type == ASP_HELLO) {
        if (m.payload[ASP_HELLO_ROLE] != 2 || m.payload[ASP_HELLO_RESERVED] ||
            asp_u16(m.payload + ASP_HELLO_PROTOCOL_VERSION) != 1)
            return;
        if (session) {
            restart(now);
            return;
        }
        mcu_nonce = asp_u32(m.payload);
        have_hello = mcu_nonce != 0;
        return;
    }
    if (m.type == ASP_SESSION_ACK && !session && have_hello && cube_nonce) {
        uint32_t candidate = asp_session_id(cube_nonce, mcu_nonce);
        if (!candidate || asp_u32(m.payload) != candidate ||
            asp_u32(m.payload + ASP_SESSION_ACK_CUBE_NONCE) != cube_nonce ||
            asp_u32(m.payload + ASP_SESSION_ACK_MCU_NONCE) != mcu_nonce)
            return;
        session = candidate;
        seq = 1;
        asp_message inhibit_msg;
        asp_message_init(&inhibit_msg, ASP_PERMIT);
        asp_put32(inhibit_msg.payload, session);
        asp_put32(inhibit_msg.payload + 4, seq);
        if (!send(inhibit_msg)) {
            restart(now);
            return;
        }
        last_send = now;
        last_status = now;
        return;
    }
    if (m.type != ASP_STATUS || !session || asp_u32(m.payload) != session)
        return;
    uint32_t incoming = asp_u32(m.payload + ASP_STATUS_SEQ),
             ack = asp_u32(m.payload + ASP_STATUS_LAST_PERMIT_SEQ);
    if (!incoming || incoming <= status_seq || ack == 0 || ack > seq ||
        asp_u16(m.payload + ASP_STATUS_RESERVED) || m.payload[ASP_STATUS_PERMIT_ACTIVE] > 1)
        return;
    status_frame = m;
    status_seq = incoming;
    last_status = now;
    faults = asp_u32(m.payload + ASP_STATUS_FAULTS);
    q_measured = asp_u16(m.payload + ASP_STATUS_Q_MEASURED_ML_MIN);
    pressure = asp_u16(m.payload + ASP_STATUS_PRESSURE_DKPA);
    const uint8_t state = m.payload[ASP_STATUS_STATE];
    if (faults || q_measured == 65535 || pressure == 65535 || state == MCU_STATE_FAULT_LATCHED ||
        state == MCU_STATE_INHIBITED) {
        restart(now);
        return;
    }
    if (state != MCU_STATE_READY && state != MCU_STATE_APPLYING) {
        restart(now);
        return;
    }
    if (state == MCU_STATE_APPLYING &&
        asp_u32(m.payload + ASP_STATUS_PACKAGE_ID) != request_package) {
        restart(now);
        return;
    }
    ready = true;
}
void AP_AgriSpray::update(uint32_t now, uint16_t hz, uint16_t validity) {
    if (!uart)
        return;
    if (hz < 5 || hz > 50 || validity < 20 || validity > 200 || 1000U / hz >= validity) {
        restart(now);
        return;
    }
    if (uart->available() > 144) {
        restart(now);
        return;
    }
    for (unsigned i = 0; i < 144 && uart->available() > 0; i++) {
        int16_t byte = uart->read();
        if (byte < 0)
            break;
        asp_message m;
        if (asp_feed(&decoder, uint8_t(byte), now, &m) == ASP_MESSAGE)
            receive(m, now);
    }
    if (session && uint32_t(now - last_status) >= 150) {
        restart(now);
        return;
    }
    if (!session) {
        if (uint32_t(now - last_hello) >= 1000) {
            if (!cube_nonce) {
                restart(now);
                return;
            }
            asp_message hello;
            asp_message_init(&hello, ASP_HELLO);
            asp_put32(hello.payload, cube_nonce);
            asp_put16(hello.payload + ASP_HELLO_PROTOCOL_VERSION, 1);
            hello.payload[ASP_HELLO_ROLE] = 1;
            if (send(hello))
                last_hello = now;
        }
        request_pending = false;
        return;
    }
    if (ready && uint32_t(now - last_send) >= 1000U / hz) {
        if (seq == UINT32_MAX) {
            restart(now);
            return;
        }
        bool allow = request_pending && requested && request_time == now && healthy(now) &&
                     request_q > 0 && request_package > 0;
        asp_message m;
        asp_message_init(&m, ASP_PERMIT);
        asp_put32(m.payload, session);
        asp_put32(m.payload + ASP_PERMIT_SEQ, ++seq);
        asp_put32(m.payload + ASP_PERMIT_PACKAGE_ID, request_package);
        asp_put16(m.payload + ASP_PERMIT_FLAGS, allow ? 1 : 0);
        asp_put16(m.payload + ASP_PERMIT_Q_TARGET_ML_MIN, allow ? request_q : 0);
        asp_put16(m.payload + ASP_PERMIT_VALIDITY_MS, allow ? validity : 0);
        if (!send(m)) {
            restart(now);
            return;
        }
        last_send = now;
    }
    request_pending = false;
}
