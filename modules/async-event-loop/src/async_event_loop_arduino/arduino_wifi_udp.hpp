#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include <string.h>

#include "async_event_loop/udp.hpp"

namespace async_event_loop {

class ArduinoWiFiUdpDatagramStream final : public IUdpDatagramStream {
public:
    bool bind(uint16_t local_port, bool reuse_address = true) override {
        (void)reuse_address;
        bound_ = udp_.begin(local_port) == 1;
        return bound_;
    }

    void close() {
        udp_.stop();
        packet_size_ = 0;
        bound_ = false;
        has_remote_ = false;
        remote_port_ = 0;
    }

    bool set_remote(const char* host, uint16_t port) override {
        if (!host || port == 0) {
            return false;
        }
        IPAddress ip;
        if (strcmp(host, "255.255.255.255") == 0) {
            ip = IPAddress(255, 255, 255, 255);
        } else if (!ip.fromString(host)) {
            return false;
        }
        remote_ip_ = ip;
        remote_port_ = port;
        has_remote_ = true;
        return true;
    }

    int write(const uint8_t* src, size_t len) override {
        if (!has_remote_) {
            return 0;
        }
        return send_to_ip(src, len, remote_ip_, remote_port_);
    }

    int send_to(const uint8_t* src, size_t len, const UdpEndpoint& endpoint) override {
        IPAddress ip;
        if (strcmp(endpoint.host, "255.255.255.255") == 0) {
            ip = IPAddress(255, 255, 255, 255);
        } else if (!ip.fromString(endpoint.host)) {
            return -1;
        }
        return send_to_ip(src, len, ip, endpoint.port);
    }

    int read(uint8_t* dst, size_t max_len) override {
        return recv_from(dst, max_len, nullptr);
    }

    int recv_from(uint8_t* dst, size_t max_len, UdpEndpoint* source) override {
        if (!bound_ || !dst || max_len == 0) {
            return 0;
        }
        if (packet_size_ <= 0) {
            packet_size_ = udp_.parsePacket();
        }
        if (packet_size_ <= 0) {
            return 0;
        }
        const int n = udp_.read(dst, max_len);
        packet_size_ = 0;
        if (source && n >= 0) {
            IPAddress ip = udp_.remoteIP();
            snprintf(source->host, sizeof(source->host), "%u.%u.%u.%u",
                     static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                     static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]));
            source->port = udp_.remotePort();
        }
        return n;
    }

    bool readable() const override {
        if (!bound_) {
            return false;
        }
        if (packet_size_ <= 0) {
            packet_size_ = udp_.parsePacket();
        }
        return packet_size_ > 0;
    }

    bool valid() const override { return bound_; }

private:
    int send_to_ip(const uint8_t* src, size_t len, IPAddress ip, uint16_t port) {
        if (!bound_ || !src || len == 0 || port == 0) {
            return 0;
        }
        if (!udp_.beginPacket(ip, port)) {
            return -1;
        }
        const size_t written = udp_.write(src, len);
        if (!udp_.endPacket()) {
            return -1;
        }
        return static_cast<int>(written);
    }

    mutable WiFiUDP udp_;
    mutable int packet_size_ = 0;
    bool bound_ = false;
    bool has_remote_ = false;
    IPAddress remote_ip_;
    uint16_t remote_port_ = 0;
};

} // namespace async_event_loop
#endif
