// UdpPeer.hpp
#ifndef UDP_PEER_HPP
#define UDP_PEER_HPP

#pragma once

#include "RobotConfig.h" // For UdpConfig
#include "LoggingMacros.h"
#include <string>
#include <vector>
#include <sys/types.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno> // For errno
#endif

namespace RDT {

/**
 * @class UdpPeer
 * @brief A lightweight, cross-platform wrapper for basic UDP send/receive operations.
 * This class encapsulates platform-specific socket logic.
 * It is not exception-safe by design; all error-prone operations return error codes.
 */
class UdpPeer {
public:
    /**
     * @brief Constructs the UdpPeer and initializes platform-specific networking APIs (WSA).
     * This constructor is lightweight and does not perform operations that can fail,
     * such as socket creation or binding.
     * @param config The UDP network configuration.
     */
    explicit UdpPeer(const UdpConfig& config) : config_(config) {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            // This is a catastrophic failure of the networking stack.
            // Logging is the only reasonable action before returning an error in connect().
            RDT_LOG_CRITICAL("UdpPeer", "WSAStartup failed. UDP will not be available.");
            wsa_startup_failed_ = true;
        }
#endif
    }

    /**
     * @brief Destructor. Ensures the socket is closed and networking APIs are cleaned up.
     */
    ~UdpPeer() {
        disconnect();
#ifdef _WIN32
        if (!wsa_startup_failed_) {
            WSACleanup();
        }
#endif
    }

    // Non-copyable, non-movable due to managing a raw socket handle.
    UdpPeer(const UdpPeer&) = delete;
    UdpPeer& operator=(const UdpPeer&) = delete;
    UdpPeer(UdpPeer&&) = delete;
    UdpPeer& operator=(UdpPeer&&) = delete;

    /**
     * @brief Creates, configures, and binds the UDP socket.
     * @return 0 on success, or a negative error code on failure.
     *         -1: Socket creation failed.
     *         -2: Socket bind failed.
     *         -3: WSAStartup failed (Windows-only).
     */
    [[nodiscard]] int connect() {
#ifdef _WIN32
        if (wsa_startup_failed_) return -3;
#endif
        // 1. Create socket
#ifdef _WIN32
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) return -1;
#else
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_ < 0) return -1;
#endif

        // 2. Configure local address for binding
        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(config_.local_port);
        local_addr.sin_addr.s_addr = INADDR_ANY;

        // 3. Bind the socket
        if (bind(socket_, (const sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            disconnect();
            return -2;
        }

        // 4. Configure remote address for sending
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(config_.remote_port);
        inet_pton(AF_INET, config_.remote_ip.c_str(), &remote_addr_.sin_addr);

        // 5. Set receive timeout
#ifdef _WIN32
        DWORD timeout = config_.timeout_ms;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
        struct timeval tv;
        tv.tv_sec = config_.timeout_ms / 1000;
        tv.tv_usec = (config_.timeout_ms % 1000) * 1000;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
        return 0;
    }

    /**
     * @brief Closes the socket. Safe to call multiple times.
     */
    void disconnect() {
#ifdef _WIN32
        if (socket_ != INVALID_SOCKET) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
#else
        if (socket_ >= 0) {
            close(socket_);
            socket_ = -1;
        }
#endif
    }

    /**
     * @brief Sends data to the configured remote peer.
     * @param data The data to send.
     * @return Number of bytes sent, or -1 on error.
     */
    int send(const std::vector<char>& data) {
        if (data.empty()) return 0;
        return sendto(socket_, data.data(), static_cast<int>(data.size()), 0, (const sockaddr*)&remote_addr_, sizeof(remote_addr_));
    }

    /**
     * @brief Receives data, blocking up to the configured timeout.
     * @param buffer A buffer to be filled with the received data. The buffer will be resized.
     * @return Number of bytes received, 0 on timeout, or -1 on a non-timeout error.
     */
    int receive(std::vector<char>& buffer) {
        buffer.resize(2048); // Max expected packet size
        sockaddr_in from_addr;
#ifdef _WIN32
        int from_len = sizeof(from_addr);
        int bytes_received = recvfrom(socket_, buffer.data(), static_cast<int>(buffer.size()), 0, (sockaddr*)&from_addr, &from_len);
#else
        socklen_t from_len = sizeof(from_addr);
        ssize_t bytes_received = recvfrom(socket_, buffer.data(), buffer.size(), 0, (sockaddr*)&from_addr, &from_len);
#endif

        if (bytes_received > 0) {
            buffer.resize(bytes_received);
        } else {
            buffer.clear();
#ifdef _WIN32
            if (bytes_received == SOCKET_ERROR && WSAGetLastError() == WSAETIMEDOUT) {
                return 0; // Timeout
            }
#else
            if (bytes_received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return 0; // Timeout
            }
#endif
        }
        return static_cast<int>(bytes_received);
    }

private:
#ifdef _WIN32
    SOCKET socket_ = INVALID_SOCKET;
    bool wsa_startup_failed_ = false;
#else
    int socket_ = -1;
#endif
    UdpConfig config_;
    sockaddr_in remote_addr_{};
};

} // namespace RDT

#endif // UDP_PEER_HPP