#pragma once

#include <arpa/inet.h>

#include <chrono>
#include <string>

#include "json.hpp"

namespace util {
    namespace chrono = std::chrono;
    using json = nlohmann::json;

    using std::string;

    /**
     * @brief Interface for classes to send data though Telemetry.
     */
    class TelemetrySender {
    public:
        /**
         * @brief Get a json object with telemetry data.
         *
         * @return json telemetry object
         */
        virtual json getTelemetry() const = 0;
    };

    /**
     * @brief UDP Telemetry Broadcaster.
     */
    class Telemetry {
        int m_socket;
        sockaddr_in m_addr;

    public:
        /**
         * @brief Connect to Telemetry
         *
         * @param udpPort the UDP telemetry port
         * @param udpAddress the UDP telemetry network address
         *
         * @throw std::runtime_error
         */
        Telemetry(int udpPort, string udpAddress);

        /**
         * @brief Send arbitrary telemetry data
         *
         * @param data json telemetry object
         */
        void send(json data) const;

        /**
         * @brief Send telemetry from a TelemetrySender via getTelemetry().
         *
         * @param sender the TelemetrySender
         */
        void send(const TelemetrySender * sender) const;
    };
}
