#include "util/Profiler.hpp"

namespace util {
    Clock::Clock(const string & name)
        : name(name) {
        reset();
    }

    void Clock::reset() {
        lastPoint = clock::now();
    }

    Clock::duration Clock::tick() {
        auto now = clock::now();
        delta = now - lastPoint;
        lastPoint = now;
        return delta;
    }
}

namespace util {
    Profiler::Profiler() {}

    Clock::SharedPtr Profiler::getClock(const string & name) {
        for (auto & clock : clocks) {
            if (clock->name == name) {
                return clock;
            }
        }
        return clocks.emplace_back(Clock::make_shared(name));
    }

    json Profiler::getTelemetry() const {
        using seconds = chrono::duration<double>;

        json ds;
        for (auto & clock : clocks) {
            auto timestamp = clock->lastPoint.time_since_epoch();
            auto sec = chrono::duration_cast<seconds>(clock->delta);
            auto time = chrono::duration_cast<seconds>(timestamp);
            ds[clock->name] = {
                {"dt", sec.count()},
                {"last", time.count()},
            };
        }
        return ds;
    }
}
