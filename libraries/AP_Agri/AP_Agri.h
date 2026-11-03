#pragma once
#include "AP_Agri_config.h"
#if AP_AGRI_ENABLED
#include "MavFilter/MavFilter.h"
#include "Spray/AP_AgriSpray.h"
#include <AP_Param/AP_Param.h>
class AP_Agri {
  public:
    AP_Agri();
    static AP_Agri *get_singleton() { return singleton; }
    static bool protected_port(uint8_t serial) {
        return singleton ? singleton->protects(serial) : serial == 2;
    }
    static const AP_Param::GroupInfo var_info[];
    void init();
    void update();
    bool protects(uint8_t serial) const { return serial == filter_port || serial == 2; }
    bool accept(uint8_t serial, const mavlink_message_t &);
    bool configured() const { return valid_config; }

  private:
    static AP_Agri *singleton;
    AP_Int8 filt_ser, pmt_hz;
    AP_Int16 pmt_val, jet_sysid, jet_compid;
    AP_Float rate_heartbeat, rate_timesync, rate_time, rate_obstacle, rate_health;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_Int8 debug_permit;
    bool debug_saw_low = false, debug_active = false;
    uint32_t debug_session = 0;
#endif
    bool valid_config = false;
    uint8_t filter_port = 2;
    uint32_t last_log = 0, last_health_ms = 0;
    mavlink_agri_perception_health_t last_health{};
    AgriMavFilter filter;
    AP_AgriSpray spray;
};
#endif
