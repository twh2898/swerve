#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Telemetry.hpp"
#include "rclcpp/macros.hpp"

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

    struct Clock {
        RCLCPP_SMART_PTR_DEFINITIONS(Clock)

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
        RCLCPP_SMART_PTR_DEFINITIONS(Profiler)

        vector<Clock::SharedPtr> clocks;

        Profiler();

        Clock::SharedPtr getClock(const string & name);

        json getTelemetry() const override;
    };
}
