#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace radar {

/// Thin RAII wrapper around a POSIX IPv4 UDP socket.
///
/// Errors during setup throw std::system_error; per-datagram errors are
/// reported through return values so a single bad packet never tears down a
/// worker thread.
class UdpSocket {
public:
    /// Creates an unbound socket suitable for sending.
    UdpSocket();

    /// Creates a socket bound to `bind_address:port`. Port 0 picks a free port.
    static UdpSocket bind(std::uint16_t port, const std::string& bind_address = "0.0.0.0");

    ~UdpSocket();

    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    /// Sets the default destination used by send().
    void connect(const std::string& host, std::uint16_t port);

    /// Sends one datagram to the connected destination. Returns false on error.
    bool send(const std::vector<std::uint8_t>& datagram);

    /// Waits up to `timeout` for a datagram. Returns the received bytes, or
    /// std::nullopt on timeout or error. The timeout lets receiver threads
    /// poll a stop flag instead of blocking forever in recv().
    std::optional<std::vector<std::uint8_t>> receive(std::chrono::milliseconds timeout);

    /// Port the socket is bound to (useful after binding to port 0).
    std::uint16_t localPort() const;

    int nativeHandle() const { return fd_; }

private:
    explicit UdpSocket(int fd) : fd_(fd) {}
    void close() noexcept;

    int fd_ = -1;
};

}  // namespace radar
