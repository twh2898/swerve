#include "util/log.hpp"

namespace util::Logging {
    Logger::SharedPtr Main;
    Logger::SharedPtr Config;
    Logger::SharedPtr MC;
    Logger::SharedPtr Planning;

    void init_logging(spdlog::level::level_enum level) {
        spdlog::set_level(level);
        Logging::Main = Logging::Logger::make_shared("Main");
        Logging::Config = Logging::Logger::make_shared("Config");
        Logging::MC = Logging::Logger::make_shared("MotionControl");
        Logging::Planning = Logging::Logger::make_shared("Planning");
    }
}
