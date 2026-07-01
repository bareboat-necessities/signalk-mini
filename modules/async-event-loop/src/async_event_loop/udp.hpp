#pragma once

#include <stddef.h>
#include <stdint.h>

#include "async_event_loop/event_loop.hpp"

namespace async_event_loop {

struct UdpEndpoint {
    char host[64]{};
    uint16_t port = 0;
};

class IUdpDatagramStream : public IByteStream {
public:
    bool is_datagram() const override { return true; }
    bool writable() const override { return valid(); }

    virtual bool bind(uint16_t local_port, bool reuse_address = true) = 0;
    virtual bool set_remote(const char* host, uint16_t port) = 0;
    virtual int send_to(const uint8_t* src, size_t len, const UdpEndpoint& endpoint) = 0;
    virtual int recv_from(uint8_t* dst, size_t max_len, UdpEndpoint* source) = 0;
};

} // namespace async_event_loop
