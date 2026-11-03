#include "MavFilter.h"
#include <math.h>
bool AgriMavFilter::accept(const mavlink_message_t &m, uint32_t now, const Config &cfg) {
    static const uint32_t ids[5] = {MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_TIMESYNC,
                                    MAVLINK_MSG_ID_SYSTEM_TIME, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D,
                                    MAVLINK_MSG_ID_AGRI_PERCEPTION_HEALTH};
    static const uint8_t lengths[5] = {
        MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN,
        MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_AGRI_PERCEPTION_HEALTH_LEN};
    if (m.magic != MAVLINK_STX)
        return reject(VERSION);
    unsigned slot = 5;
    for (unsigned i = 0; i < 5; i++)
        if (ids[i] == m.msgid)
            slot = i;
    if (slot == 5)
        return reject(ID);
    if (!cfg.sysid || !cfg.compid || m.sysid != cfg.sysid || m.compid != cfg.compid)
        return reject(SOURCE);
    // MAVLink2 legitimately truncates trailing zero bytes; decoder zero-fills
    // fields.
    if (m.len == 0 || m.len > lengths[slot])
        return reject(LENGTH);
    if (m.msgid == MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D) {
        mavlink_obstacle_distance_3d_t p{};
        mavlink_msg_obstacle_distance_3d_decode(&m, &p);
        double distance = sqrt(double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z);
        if (p.frame != MAV_FRAME_BODY_FRD || !isfinite(p.min_distance) ||
            !isfinite(p.max_distance) || !isfinite(distance) || p.min_distance < 0 ||
            p.max_distance <= p.min_distance || distance <= 0 || distance < p.min_distance ||
            distance > p.max_distance)
            return reject(DATA);
    }
    if (m.msgid == MAVLINK_MSG_ID_AGRI_PERCEPTION_HEALTH) {
        mavlink_agri_perception_health_t p{};
        mavlink_msg_agri_perception_health_decode(&m, &p);
        if (p.sensor_count == 0 || p.sensor_count > 4 || (p.flags & ~uint32_t(3)) ||
            p.unknown_forward_sectors > 72)
            return reject(DATA);
    }
    float rate = cfg.rate[slot];
    if (!isfinite(rate) || rate <= 0 || rate > 200)
        return reject(RATE);
    Bucket &b = buckets[slot];
    // Burst capacity two frames, no unbounded catch-up after idle.
    if (!b.initialized) {
        b.tokens = 2;
        b.time = now;
        b.initialized = true;
    }
    b.tokens = fminf(2, b.tokens + float(uint32_t(now - b.time)) * 0.001f * rate);
    b.time = now;
    if (b.tokens < 1)
        return reject(RATE);
    b.tokens -= 1;
    return true;
}
