#include "util/log.hpp"

namespace util::Logging {
    using std::make_shared;

    Logger::Ptr Main;
    Logger::Ptr Config;
    Logger::Ptr MC;
    Logger::Ptr Planning;

    void init_logging(spdlog::level::level_enum level) {
        spdlog::set_level(level);
        Logging::Main = make_shared<Logging::Logger>("Main");
        Logging::Config = make_shared<Logging::Logger>("Config");
        Logging::MC = make_shared<Logging::Logger>("MotionControl");
        Logging::Planning = make_shared<Logging::Logger>("Planning");
    }
}
