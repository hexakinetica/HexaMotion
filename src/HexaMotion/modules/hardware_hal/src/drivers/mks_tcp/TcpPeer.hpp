// TcpPeer.hpp
#ifndef TCP_PEER_HPP
#define TCP_PEER_HPP

#pragma once

#include "RobotConfig.h" // For MksTcpConfig
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
#include <netinet/tcp.h> // For TCP_NODELAY
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno> // For errno
#endif

namespace RDT {

/**
 * @class TcpPeer
 * @brief A lightweight, cross-platform wrapper for a blocking TCP client socket.
 *
 * Mirrors the design of @ref UdpPeer: all error-prone operations return error codes
 * (no exceptions). It connects to a single remote server, sends raw bytes and reads
 * raw bytes with a receive timeout. Framing (newline-delimited JSON) is the caller's
 * responsibility. TCP_NODELAY is enabled for low-latency robotics traffic.
 */
class TcpPeer {
public:
    explicit TcpPeer(const MksTcpConfig& config) : config_(config) {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            RDT_LOG_CRITICAL("TcpPeer", "WSAStartup failed. TCP will not be available.");
            wsa_startup_failed_ = true;
        }
#endif
    }

    ~TcpPeer() {
        disconnect();
#ifdef _WIN32
        if (!wsa_startup_failed_) {
            WSACleanup();
        }
#endif
    }

    // Non-copyable, non-movable due to managing a raw socket handle.
    TcpPeer(const TcpPeer&) = delete;
    TcpPeer& operator=(const TcpPeer&) = delete;
    TcpPeer(TcpPeer&&) = delete;
    TcpPeer& operator=(TcpPeer&&) = delete;

    /**
     * @brief Creates the TCP socket and connects to the configured server.
     * @return 0 on success, or a negative error code on failure.
     *         -1: Socket creation failed.
     *         -2: Connect failed (e.g. connection refused / server not running).
     *         -3: WSAStartup failed (Windows-only).
     *         -4: Invalid IP address.
     */
    [[nodiscard]] int connect() {
#ifdef _WIN32
        if (wsa_startup_failed_) return -3;
        socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socket_ == INVALID_SOCKET) return -1;
#else
        socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) return -1;
#endif

        sockaddr_in remote_addr{};
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(config_.port);
        if (inet_pton(AF_INET, config_.ip.c_str(), &remote_addr.sin_addr) != 1) {
            disconnect();
            return -4;
        }

        if (::connect(socket_, (const sockaddr*)&remote_addr, sizeof(remote_addr)) < 0) {
            disconnect();
            return -2;
        }

        // Low-latency: disable Nagle's algorithm.
        int flag = 1;
        setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (const char*)&flag, sizeof(flag));

        // Receive timeout so the worker loop can wake up to send telemetry polls.
        int rcv_timeout_ms = config_.telemetry_poll_ms > 0 ? config_.telemetry_poll_ms : 50;
        if (config_.timeout_ms > 0 && config_.timeout_ms < rcv_timeout_ms) {
            rcv_timeout_ms = config_.timeout_ms;
        }
#ifdef _WIN32
        DWORD timeout = rcv_timeout_ms;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
        struct timeval tv;
        tv.tv_sec = rcv_timeout_ms / 1000;
        tv.tv_usec = (rcv_timeout_ms % 1000) * 1000;
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
     * @brief Sends all bytes to the server, handling TCP partial sends.
     * @param data The data to send.
     * @return Number of bytes sent, or -1 on error.
     */
    int send(const std::vector<char>& data) {
        if (data.empty()) return 0;
        size_t total_sent = 0;
        while (total_sent < data.size()) {
#ifdef _WIN32
            int n = ::send(socket_, data.data() + total_sent,
                           static_cast<int>(data.size() - total_sent), 0);
            if (n == SOCKET_ERROR) return -1;
#else
            ssize_t n = ::send(socket_, data.data() + total_sent,
                               data.size() - total_sent, MSG_NOSIGNAL);
            if (n < 0) {
                if (errno == EINTR) continue;
                return -1;
            }
#endif
            if (n == 0) return -1; // connection closed
            total_sent += static_cast<size_t>(n);
        }
        return static_cast<int>(total_sent);
    }

    /**
     * @brief Receives data, blocking up to the configured timeout.
     * @param buffer A buffer to be filled with the received data. The buffer will be resized.
     * @return Number of bytes received, 0 on timeout, or -1 on error / peer disconnect.
     */
    int receive(std::vector<char>& buffer) {
        buffer.resize(4096);
#ifdef _WIN32
        int bytes_received = recv(socket_, buffer.data(), static_cast<int>(buffer.size()), 0);
#else
        ssize_t bytes_received = recv(socket_, buffer.data(), buffer.size(), 0);
#endif
        if (bytes_received > 0) {
            buffer.resize(bytes_received);
            return static_cast<int>(bytes_received);
        }

        buffer.clear();
        if (bytes_received == 0) {
            return -1; // Peer performed an orderly shutdown.
        }
#ifdef _WIN32
        if (WSAGetLastError() == WSAETIMEDOUT) {
            return 0; // Timeout
        }
#else
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // Timeout
        }
        if (errno == EINTR) {
            return 0; // Interrupted; treat as a benign wake-up.
        }
#endif
        return -1; // Real error
    }

private:
#ifdef _WIN32
    SOCKET socket_ = INVALID_SOCKET;
    bool wsa_startup_failed_ = false;
#else
    int socket_ = -1;
#endif
    MksTcpConfig config_;
};

} // namespace RDT

#endif // TCP_PEER_HPP
