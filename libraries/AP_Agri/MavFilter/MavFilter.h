#pragma once
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
class AgriMavFilter {
  public:
    enum Reject : uint8_t { ID = 0, SOURCE, LENGTH, DATA, RATE, VERSION, REJECT_COUNT };
    struct Config {
        uint8_t sysid, compid;
        float rate[5];
    };
    bool accept(const mavlink_message_t &, uint32_t now, const Config &);
    const uint32_t *counters() const { return rejected; }

  private:
    struct Bucket {
        float tokens;
        uint32_t time;
        bool initialized;
    } buckets[5]{};
    uint32_t rejected[REJECT_COUNT]{};
    bool reject(Reject r) {
        if (rejected[r] != UINT32_MAX)
            ++rejected[r];
        return false;
    }
};
