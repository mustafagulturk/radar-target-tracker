#include "radar/udp_socket.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>
#include <utility>

namespace radar {

namespace {

[[noreturn]] void throwErrno(const char* what) {
    throw std::system_error(errno, std::generic_category(), what);
}

int openSocket() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        throwErrno("socket");
    }
    return fd;
}

sockaddr_in resolve(const std::string& host, std::uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) == 1) {
        return addr;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    if (::getaddrinfo(host.c_str(), nullptr, &hints, &result) != 0 || result == nullptr) {
        throw std::system_error(std::make_error_code(std::errc::host_unreachable),
                                "cannot resolve host '" + host + "'");
    }
    addr.sin_addr = reinterpret_cast<sockaddr_in*>(result->ai_addr)->sin_addr;
    ::freeaddrinfo(result);
    return addr;
}

}  // namespace

UdpSocket::UdpSocket() : fd_(openSocket()) {}

UdpSocket UdpSocket::bind(std::uint16_t port, const std::string& bind_address) {
    UdpSocket sock(openSocket());

    const int reuse = 1;
    ::setsockopt(sock.fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // A larger receive buffer absorbs bursts while the tracker thread is busy.
    const int rcvbuf = 4 * 1024 * 1024;
    ::setsockopt(sock.fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    const sockaddr_in addr = resolve(bind_address, port);
    if (::bind(sock.fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        throwErrno("bind");
    }
    return sock;
}

UdpSocket::~UdpSocket() {
    close();
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
    if (this != &other) {
        close();
        fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
}

void UdpSocket::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void UdpSocket::connect(const std::string& host, std::uint16_t port) {
    const sockaddr_in addr = resolve(host, port);
    if (::connect(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        throwErrno("connect");
    }
}

bool UdpSocket::send(const std::vector<std::uint8_t>& datagram) {
    const ssize_t sent = ::send(fd_, datagram.data(), datagram.size(), 0);
    return sent == static_cast<ssize_t>(datagram.size());
}

std::optional<std::vector<std::uint8_t>> UdpSocket::receive(std::chrono::milliseconds timeout) {
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
    if (ready <= 0 || (pfd.revents & POLLIN) == 0) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> buffer(65536);
    const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
    if (n < 0) {
        return std::nullopt;
    }
    buffer.resize(static_cast<std::size_t>(n));
    return buffer;
}

std::uint16_t UdpSocket::localPort() const {
    sockaddr_in addr{};
    socklen_t len = sizeof(addr);
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        return 0;
    }
    return ntohs(addr.sin_port);
}

}  // namespace radar
