#pragma once
#include <AP_HAL/AP_HAL.h>
extern "C" {
#include "../Contracts/agri_mcu_codes.h"
#include "../Contracts/agri_spray_proto.h"
}
class AP_Agri;
class AP_AgriSupervisor;
class AP_AgriSpray {
  public:
    void init();
    void update(uint32_t now, uint16_t hz, uint16_t validity);
    bool healthy(uint32_t now) const;
    void inhibit();
    uint32_t session_id() const { return session; }
    uint32_t sent_sequence() const { return seq; }
    uint32_t status_time() const { return last_status; }
    // Pair with healthy() and status_time(); retained data alone never authorizes.
    const asp_message &last_status_frame() const { return status_frame; }
    uint32_t fault_mask() const { return faults; }
    uint16_t measured_flow() const { return q_measured; }
    uint16_t measured_pressure() const { return pressure; }

  private:
    friend class AP_Agri;
    friend class AP_AgriSupervisor;
    void set_authorization(bool allow, uint16_t q, uint32_t package, uint32_t now);
    bool send(const asp_message &);
    void restart(uint32_t now);
    void receive(const asp_message &, uint32_t now);
    AP_HAL::UARTDriver *uart = nullptr;
    asp_decoder decoder{};
    asp_message status_frame{};
    uint32_t cube_nonce = 0, mcu_nonce = 0, session = 0, seq = 0, status_seq = 0, last_status = 0,
             last_hello = 0, last_send = 0;
    uint32_t request_time = 0, request_package = 0, faults = 0;
    uint16_t request_q = 0, q_measured = 65535, pressure = 65535;
    bool ready = false, requested = false, request_pending = false, have_hello = false;
};
