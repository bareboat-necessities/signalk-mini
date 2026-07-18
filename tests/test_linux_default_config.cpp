#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#ifndef SIGNALK_MINI_SOURCE_DIR
#define SIGNALK_MINI_SOURCE_DIR "."
#endif

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string read_file(const char* path) {
    std::ifstream in(path);
    REQUIRE(in.good());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static std::string default_config_text_block(const std::string& main_cpp) {
    const std::string begin_marker = "const char* default_config_text()";
    const std::string end_marker = "bool ensure_user_config_dir";
    const size_t begin = main_cpp.find(begin_marker);
    const size_t end = main_cpp.find(end_marker);
    REQUIRE(begin != std::string::npos);
    REQUIRE(end != std::string::npos);
    REQUIRE(begin < end);
    return main_cpp.substr(begin, end - begin);
}

static void require_current_linux_defaults(const std::string& text) {
    REQUIRE(text.find("signalk") != std::string::npos);
    REQUIRE(text.find("host = \\\"0.0.0.0\\\";") != std::string::npos || text.find("host = \"0.0.0.0\";") != std::string::npos);
    REQUIRE(text.find("port = 20223;") != std::string::npos);
    REQUIRE(text.find("signalk_version = \\\"1.8.2\\\";") != std::string::npos || text.find("signalk_version = \"1.8.2\";") != std::string::npos);
    REQUIRE(text.find("vessels.urn:mrn:signalk:uuid:") != std::string::npos);
    REQUIRE(text.find("send_current_values_on_connect = true;") != std::string::npos);
    REQUIRE(text.find("current_value_timeout_ms = 0;") != std::string::npos);

    REQUIRE(text.find("websocket") != std::string::npos);
    REQUIRE(text.find("enabled = true;") != std::string::npos);
    REQUIRE(text.find("port = 3001;") != std::string::npos);
    REQUIRE(text.find("max_connections = 8;") != std::string::npos);
    REQUIRE(text.find("allow_rx = true;") != std::string::npos);
    REQUIRE(text.find("allow_tx = true;") != std::string::npos);

    REQUIRE(text.find("connectors") != std::string::npos);
    REQUIRE(text.find("protocol = \\\"nmea0183\\\";") != std::string::npos || text.find("protocol = \"nmea0183\";") != std::string::npos);
    REQUIRE(text.find("transport = \\\"udp\\\";") != std::string::npos || text.find("transport = \"udp\";") != std::string::npos);
    REQUIRE(text.find("listen_port = 10110;") != std::string::npos);
}

int main() {
    const std::string main_cpp = read_file(SIGNALK_MINI_SOURCE_DIR "/linux/main.cpp");
    const std::string generated_block = default_config_text_block(main_cpp);
    require_current_linux_defaults(generated_block);

    const std::string example_config = read_file(SIGNALK_MINI_SOURCE_DIR "/examples/linux/signalk-mini.conf");
    require_current_linux_defaults(example_config);

    return 0;
}
