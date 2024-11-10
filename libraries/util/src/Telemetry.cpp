#include "util/Telemetry.hpp"

#include <sys/socket.h>

#include <exception>

namespace util {
    using std::runtime_error;

    Telemetry::Telemetry(int udpPort, const string & udpAddress)
        : m_socket(socket(AF_INET, SOCK_DGRAM, 0)), m_addr() {
        if (m_socket == -1) {
            throw runtime_error("Could not create socket");
        }
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(udpPort);
        inet_pton(AF_INET, udpAddress.c_str(), &m_addr.sin_addr);
    }

    void Telemetry::send(json data) const {
        string message = data.dump();
        size_t message_length = message.length();
        sendto(m_socket, message.c_str(), message_length, 0,
               reinterpret_cast<const sockaddr *>(&m_addr), sizeof(m_addr));
    }

    void Telemetry::send(const TelemetrySender * sender) const {
        send(sender->getTelemetry());
    }
}
