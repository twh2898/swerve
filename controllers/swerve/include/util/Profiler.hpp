#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Telemetry.hpp"

#define R_DEF_CLOCK(PROFILER, VAR, NAME) auto VAR = (PROFILER).getClock(NAME)

#define R_PROFILE_STEP(CLOCK, ACTION) \
    {                                 \
        (CLOCK)->reset();              \
        ACTION;                       \
        (CLOCK)->tick();               \
    }

namespace util {
    using std::string;
    using std::vector;
    using std::shared_ptr;
    using std::make_shared;

    struct Clock {
        using Ptr = shared_ptr<Clock>;
        using ConstPtr = const shared_ptr<Clock>;

        using clock = std::chrono::high_resolution_clock;

        clock::duration delta;
        clock::time_point lastPoint;
        string name;

        Clock(string name) : name(name) {
            reset();
        }

        Clock(Clock && other) = default;

        Clock & operator=(Clock && other) = default;

        Clock(const Clock &) = delete;
        Clock & operator=(const Clock &) = delete;

        void reset() {
            lastPoint = clock::now();
        }

        clock::duration tick() {
            auto now = clock::now();
            delta = now - lastPoint;
            lastPoint = now;
            return delta;
        }
    };

    struct Profiler : public TelemetrySender {
        vector<Clock::Ptr> clocks;

        Profiler() {}

        Clock::Ptr getClock(const string & name) {
            for (auto & clock : clocks) {
                if (clock->name == name) {
                    return clock;
                }
            }
            return clocks.emplace_back(make_shared<Clock>(name));
        }

        json getTelemetry() const override {
            using seconds = chrono::duration<double>;

            json ds;
            for (auto & clock : clocks) {
                auto sec = chrono::duration_cast<seconds>(clock->delta);
                ds[clock->name] = sec.count();
            }
            return ds;
        }
    };
}
