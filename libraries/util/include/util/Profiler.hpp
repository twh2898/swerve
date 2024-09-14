#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Telemetry.hpp"

#define R_DEF_CLOCK(PROFILER, VAR, NAME) auto VAR = (PROFILER).getClock(NAME)

#define R_PROFILE_STEP(CLOCK, ACTION) \
    {                                 \
        (CLOCK)->reset();             \
        ACTION;                       \
        (CLOCK)->tick();              \
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
        using duration = clock::duration;
        using time_point = clock::time_point;

        duration delta;
        time_point lastPoint;
        string name;

        Clock(const string & name);

        Clock(Clock && other) = default;

        Clock & operator=(Clock && other) = default;

        Clock(const Clock &) = delete;
        Clock & operator=(const Clock &) = delete;

        void reset();

        duration tick();
    };

    struct Profiler : public TelemetrySender {
        vector<Clock::Ptr> clocks;

        Profiler();

        Clock::Ptr getClock(const string & name);

        json getTelemetry() const override;
    };
}
