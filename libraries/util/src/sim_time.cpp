#include "util/sim_time.hpp"

static double _time = 0.0;

namespace sim_time {
    void setTime(double time) {
        _time = time;
    }

    double now() {
        return _time;
    }
}
