#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#ifndef SIGNALK_MINI_BINARY
#error "SIGNALK_MINI_BINARY must point to the built signalk-mini executable"
#endif

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

namespace {

class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { reset(); }
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    Fd& operator=(Fd&& other) noexcept {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }
    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
    void reset(int fd = -1) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = fd;
    }
private:
    int fd_ = -1;
};

Fd connect_loopback(uint16_t port) {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
    if (!fd.valid()) return Fd{};
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (::connect(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return Fd{};
    return fd;
}

bool wait_for_tcp_port(uint16_t port, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        Fd fd = connect_loopback(port);
        if (fd.valid()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return false;
}

bool send_all(int fd, const std::string& data) {
    const char* ptr = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
        const ssize_t n = ::send(fd, ptr, remaining, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        ptr += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

bool recv_contains(int fd, const char* needle, int timeout_ms) {
    std::string buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50000;
        const int rc = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (rc == 0) continue;
        char chunk[512];
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) return false;
        buffer.append(chunk, static_cast<size_t>(n));
        if (buffer.find(needle) != std::string::npos) return true;
    }
    return false;
}

bool wait_for_websocket_upgrade(uint16_t port, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        Fd fd = connect_loopback(port);
        if (fd.valid()) {
            const std::string request =
                "GET /signalk/v1/stream?subscribe=none HTTP/1.1\r\n"
                "Host: 127.0.0.1\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                "Sec-WebSocket-Version: 13\r\n"
                "\r\n";
            if (send_all(fd.get(), request) && recv_contains(fd.get(), "HTTP/1.1 101 Switching Protocols", 1000)) return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return false;
}

bool file_exists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    REQUIRE(in.good());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void stop_child(pid_t pid) {
    if (pid <= 0) return;
    ::kill(pid, SIGTERM);
    for (int i = 0; i < 40; ++i) {
        int status = 0;
        const pid_t rc = ::waitpid(pid, &status, WNOHANG);
        if (rc == pid) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    ::kill(pid, SIGKILL);
    int status = 0;
    (void)::waitpid(pid, &status, 0);
}

} // namespace

int main() {
    char home_template[] = "/tmp/signalk-mini-default-home-XXXXXX";
    char* home = ::mkdtemp(home_template);
    REQUIRE(home != nullptr);

    const pid_t pid = ::fork();
    REQUIRE(pid >= 0);
    if (pid == 0) {
        ::setenv("HOME", home, 1);
        ::execl(SIGNALK_MINI_BINARY, "signalk-mini", static_cast<char*>(nullptr));
        ::_exit(127);
    }

    const std::string config_path = std::string(home) + "/.signalk-mini/signalk-mini.conf";
    bool config_created = false;
    for (int i = 0; i < 80; ++i) {
        if (file_exists(config_path)) {
            config_created = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    const bool tcp_started = wait_for_tcp_port(20223, 5000);
    const bool websocket_started = wait_for_websocket_upgrade(3001, 5000);

    stop_child(pid);

    REQUIRE(config_created);
    const std::string config_text = read_file(config_path);
    REQUIRE(config_text.find("port = 20223;") != std::string::npos);
    REQUIRE(config_text.find("websocket = {") != std::string::npos);
    REQUIRE(config_text.find("port = 3001;") != std::string::npos);
    REQUIRE(config_text.find("listen_port = 10110;") != std::string::npos);

    REQUIRE(tcp_started);
    REQUIRE(websocket_started);
    return 0;
}
