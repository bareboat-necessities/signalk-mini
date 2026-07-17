#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

#include <signalk_mini/config.hpp>
#include "../linux/config_loader.hpp"

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string write_config() {
    char path[] = "/tmp/signalk-mini-ubx-config-XXXXXX";
    const int fd = ::mkstemp(path);
    REQUIRE(fd >= 0);
    ::close(fd);
    std::ofstream out(path);
    out <<
        "signalk = { host=\"127.0.0.1\"; port=20223; websocket={enabled=false;}; };\n"
        "connectors = (\n"
        " { enabled=true; label=\"ubx-net\"; protocol=\"ubx\"; transport=\"tcp_client\";\n"
        "   tcp_client={host=\"127.0.0.1\";port=4242;}; ubx={configure_receiver=false;};\n"
        "   reconnect={enabled=true;initial_delay_ms=25;maximum_delay_ms=200;}; },\n"
        " { enabled=false; label=\"gpsd\"; protocol=\"gpsd\"; transport=\"tcp_client\";\n"
        "   tcp_client={host=\"127.0.0.1\";port=2947;}; gpsd={device=\"/dev/ttyACM0\";include_sky=false;include_gst=true;}; }\n"
        ");\n";
    out.close();
    return path;
}

int main() {
    const std::string path = write_config();
    signalk_mini_linux::ConfigLoader loader;
    std::string error;
    REQUIRE(loader.load_file(path.c_str(), error));
    ::unlink(path.c_str());
    REQUIRE(loader.config.connector_count == 2);

    const auto& ubx = loader.config.connectors[0];
    REQUIRE(ubx.protocol.kind == signalk_mini::ConnectorProtocol::Ubx);
    REQUIRE(!ubx.protocol.ubx.configure_receiver);
    REQUIRE(ubx.transport.kind == signalk_mini::ConnectorTransport::TcpClient);
    REQUIRE(std::string(ubx.transport.tcp_client.host) == "127.0.0.1");
    REQUIRE(ubx.transport.tcp_client.port == 4242);
    REQUIRE(ubx.reconnect.enabled);
    REQUIRE(ubx.reconnect.initial_delay_ms == 25);
    REQUIRE(ubx.reconnect.maximum_delay_ms == 200);

    const auto& gpsd = loader.config.connectors[1];
    REQUIRE(gpsd.protocol.kind == signalk_mini::ConnectorProtocol::Gpsd);
    REQUIRE(std::string(gpsd.protocol.gpsd.device) == "/dev/ttyACM0");
    REQUIRE(!gpsd.protocol.gpsd.include_sky);
    REQUIRE(gpsd.protocol.gpsd.include_gst);
    return 0;
}
