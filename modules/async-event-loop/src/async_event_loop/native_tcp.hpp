#pragma once

#if defined(__linux__)
#include "async_event_loop_linux/linux_tcp_server.hpp"
#include "async_event_loop_linux/linux_tcp_client.hpp"
#endif

#if defined(ARDUINO) && defined(ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP)
#include "async_event_loop_arduino/arduino_wifi_tcp.hpp"
#endif

namespace async_event_loop {

#if defined(__linux__)
using NativeTcpServer = LinuxTcpServer;
using NativeTcpConnection = LinuxTcpConnection;
using NativeTcpClient = LinuxTcpClient;
#endif

#if defined(ARDUINO) && defined(ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP)
using NativeTcpServer = ArduinoWiFiTcpServer;
using NativeTcpConnection = ArduinoWiFiTcpConnection;
using NativeTcpClient = ArduinoWiFiTcpClient;
#endif

} // namespace async_event_loop
