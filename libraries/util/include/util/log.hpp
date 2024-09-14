#pragma once

#include <memory>
#include <vector>

#ifndef SPDLOG_ACTIVE_LEVEL
#ifdef NDEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#else
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif
#endif

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace util::Logging {
    using std::vector;
    using std::shared_ptr;
    using std::make_shared;

    /**
     * Wrapper for a logging subsystem.
     */
    class Logger {
    public:
        /**
         * The log message level.
         */
        enum Level {
            /// Used for development, should be removed before a release
            Trace,
            /// What's happening in the program, useful for debugging
            Debug,
            /// User events
            Info,
            /// Something that can be recovered from
            Warning,
            /// Something but cannot be recovered from
            Error,
            /// Something that forces the program to crash
            Critical,
        };

    private:
        shared_ptr<spdlog::logger> logger;
        Level level;

    public:
        using Ptr = std::shared_ptr<Logger>;
        using ConstPtr = std::shared_ptr<const Logger>;

        /**
         * Create a new Logger with subsystem name and the default level.
         *
         * The default level is Debug if the compile configuration is Debug,
         * otherwise it is Info.
         *
         * @param subsystem the name of the logging subsystem
         */
        template<typename FormatString>
        Logger(const FormatString & subsystem)
            : Logger(subsystem,
#ifndef NDEBUG
                     Debug
#else
                     Info
#endif
            ) {
        }

        /**
         * Create a new Logger with subsystem name and the given level.
         *
         * @param subsystem the name of the logging subsystem
         * @param level the logging level
         */
        template<typename FormatString>
        Logger(const FormatString & subsystem, Level level) {
            vector<spdlog::sink_ptr> sinks;
            sinks.push_back(make_shared<spdlog::sinks::stdout_color_sink_mt>());
            sinks.push_back(
                make_shared<spdlog::sinks::basic_file_sink_mt>("swerve.log"));
            logger =
                make_shared<spdlog::logger>(subsystem, begin(sinks), end(sinks));

            setLevel(level);
        }

        /**
         * Log a Trace level message.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        void trace(spdlog::format_string_t<Args...> fmt, Args &&... args) {
            logger->trace(fmt, std::forward<Args>(args)...);
        }

        /**
         * Log a Debug level message.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        void debug(spdlog::format_string_t<Args...> fmt, Args &&... args) {
            logger->debug(fmt, std::forward<Args>(args)...);
        }

        /**
         * Log a Info level message.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        void info(spdlog::format_string_t<Args...> fmt, Args &&... args) {
            logger->info(fmt, std::forward<Args>(args)...);
        }

        /**
         * Log a Warning level message.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        void warning(spdlog::format_string_t<Args...> fmt, Args &&... args) {
            logger->warn(fmt, std::forward<Args>(args)...);
        }

        /**
         * Log a Error level message.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        void error(spdlog::format_string_t<Args...> fmt, Args &&... args) {
            logger->error(fmt, std::forward<Args>(args)...);
        }

        /**
         * Log a Critical level message. This method calls std::terminate and
         * should only be used to terminate the program.
         *
         * @param fmt the format string
         * @param args... the format arguments
         */
        template<typename... Args>
        [[noreturn]] void critical(spdlog::format_string_t<Args...> fmt,
                                   Args &&... args) noexcept {
            logger->critical(fmt, std::forward<Args>(args)...);
            std::terminate();
        }

        /**
         * Get the current log level.
         *
         * @return the current log level
         */
        Level getLevel() const {
            return level;
        }

        /**
         * Set the log level.
         *
         * @param level the new log level
         */
        void setLevel(Level level) {
            this->level = level;
            switch (level) {
                case Trace:
                    logger->set_level(spdlog::level::trace);
                    break;
                case Debug:
                    logger->set_level(spdlog::level::debug);
                    break;
                case Info:
                    logger->set_level(spdlog::level::info);
                    break;
                case Warning:
                    logger->set_level(spdlog::level::warn);
                    break;
                case Error:
                    logger->set_level(spdlog::level::err);
                    break;
                case Critical:
                    logger->set_level(spdlog::level::critical);
                    break;
            }
        }
    };

    extern Logger::Ptr Main;
    extern Logger::Ptr Config;
    extern Logger::Ptr MC;
    extern Logger::Ptr Planning;

    void init_logging(spdlog::level::level_enum level);
}
