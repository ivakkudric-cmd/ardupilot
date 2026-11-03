#include "AP_Agri.h"
#if AP_AGRI_ENABLED
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
AP_Agri *AP_Agri::singleton;
const AP_Param::GroupInfo AP_Agri::var_info[] = {
    // @Param: FILT_SER
    // @DisplayName: Companion serial port
    // @Description: Protected SERIALn, requires reboot. SERIAL2 remains
    // protected as fallback.
    // @Range: 2 9
    // @User: Advanced
    AP_GROUPINFO("FILT_SER", 1, AP_Agri, filt_ser, 2),
    // @Param: JET_SYSID
    // @DisplayName: Expected companion system ID
    // @Description: Expected MAVLink system ID on the protected channel.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("JET_SYSID", 2, AP_Agri, jet_sysid, 191),
    // @Param: JET_CMPID
    // @DisplayName: Expected companion component ID
    // @Description: Expected MAVLink component ID on the protected channel.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("JET_CMPID", 3, AP_Agri, jet_compid, 191),
    // @Param: PMT_HZ
    // @DisplayName: Spray permit frequency
    // @Description: Permission refresh rate, bounded by 50Hz task.
    // @Range: 5 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("PMT_HZ", 4, AP_Agri, pmt_hz, 25),
    // @Param: PMT_VAL
    // @DisplayName: Spray permit validity
    // @Description: Local MCU expiry, must exceed refresh interval, at most
    // 200ms.
    // @Range: 20 200
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("PMT_VAL", 5, AP_Agri, pmt_val, 120),
    // @Param: HB_HZ
    // @DisplayName: Companion heartbeat rate limit
    // @Description: Token refill rate, burst two frames. CALIBRAR link budget.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("HB_HZ", 6, AP_Agri, rate_heartbeat, 2),
    // @Param: SYNC_HZ
    // @DisplayName: Companion timesync rate limit
    // @Description: Token refill rate, burst two frames. CALIBRAR link budget.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("SYNC_HZ", 7, AP_Agri, rate_timesync, 10),
    // @Param: TIME_HZ
    // @DisplayName: Companion system time rate limit
    // @Description: Token refill rate. Zero disables SYSTEM_TIME by default.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("TIME_HZ", 8, AP_Agri, rate_time, 0),
    // @Param: OBS_HZ
    // @DisplayName: Companion obstacle rate limit
    // @Description: Token refill rate, burst two frames. CALIBRAR UART budget.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("OBS_HZ", 9, AP_Agri, rate_obstacle, 100),
    // @Param: HLTH_HZ
    // @DisplayName: Companion health rate limit
    // @Description: Token refill rate, burst two frames. CALIBRAR link budget.
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("HLTH_HZ", 10, AP_Agri, rate_health, 20),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Param: DBG_PERMIT
    // @DisplayName: SITL-only spray authorization stub
    // @Description: Requires 0 then 1 after every new MCU session. Simulation
    // only.
    // @Values: 0:Inhibit,1:Request
    // @User: Advanced
    AP_GROUPINFO("DBG_PERMIT", 11, AP_Agri, debug_permit, 0),
#endif
    AP_GROUPEND};
AP_Agri::AP_Agri() {
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;
}
void AP_Agri::init() {
    if (filt_ser.get() >= 2 && filt_ser.get() < 10)
        filter_port = uint8_t(filt_ser.get());
    spray.init();
}
bool AP_Agri::accept(uint8_t serial, const mavlink_message_t &m) {
    if (!protects(serial))
        return true;
    AgriMavFilter::Config cfg = {uint8_t(jet_sysid.get()),
                                 uint8_t(jet_compid.get()),
                                 {rate_heartbeat.get(), rate_timesync.get(), rate_time.get(),
                                  rate_obstacle.get(), rate_health.get()}};
    if (jet_sysid.get() < 1 || jet_sysid.get() > 255 || jet_compid.get() < 1 ||
        jet_compid.get() > 255)
        return false;
    if (!filter.accept(m, AP_HAL::millis(), cfg))
        return false;
    if (m.msgid == MAVLINK_MSG_ID_AGRI_PERCEPTION_HEALTH) {
        mavlink_msg_agri_perception_health_decode(&m, &last_health);
        last_health_ms = AP_HAL::millis();
    }
    return true;
}
void AP_Agri::update() {
    const uint32_t now = AP_HAL::millis();
    const auto *state = AP::serialmanager().get_state_by_id(filter_port);
    valid_config = state && filt_ser.get() == filter_port && filter_port >= 2 &&
                   (state->get_protocol() == AP_SerialManager::SerialProtocol_MAVLink ||
                    state->get_protocol() == AP_SerialManager::SerialProtocol_MAVLink2) &&
                   state->option_enabled(AP_HAL::UARTDriver::OPTION_MAVLINK_NO_FORWARD) &&
                   jet_sysid.get() > 0 && jet_sysid.get() <= 255 && jet_compid.get() > 0 &&
                   jet_compid.get() <= 255;
    spray.inhibit();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (debug_session != spray.session_id() || !spray.healthy(now)) {
        debug_active = false;
        debug_saw_low = false;
        debug_session = spray.session_id();
    }
    if (debug_permit.get() == 0) {
        debug_active = false;
        debug_saw_low = true;
    }
    if (debug_permit.get() == 1 && debug_saw_low && spray.healthy(now) && valid_config)
        debug_active = true;
    // CALIBRAR: synthetic E3 plant recipe. Not built into hardware firmware.
    if (debug_active && valid_config)
        spray.set_authorization(true, 1000, 42, now);
#endif
    spray.update(now, uint16_t(pmt_hz.get()), uint16_t(pmt_val.get()));
    if (uint32_t(now - last_log) >= 1000) {
        const uint32_t *c = filter.counters();
        AP::logger().Write("AGFL", "TimeUS,Port,Id,Src,Len,Data,Rate,Ver", "QBIIIIII",
                           AP_HAL::micros64(), filter_port, c[0], c[1], c[2], c[3], c[4], c[5]);
        AP::logger().Write("AGSL", "TimeUS,Sess,Seq,LastMS,Fault,Q,Press", "QIIIIHH",
                           AP_HAL::micros64(), spray.session_id(), spray.sent_sequence(),
                           spray.status_time(), spray.fault_mask(), spray.measured_flow(),
                           spray.measured_pressure());
        last_log = now;
    }
}
#endif
