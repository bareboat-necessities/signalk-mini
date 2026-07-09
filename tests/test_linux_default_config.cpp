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

int main() {
    const std::string main_cpp = read_file(SIGNALK_MINI_SOURCE_DIR "/linux/main.cpp");
    const std::string block = default_config_text_block(main_cpp);

    REQUIRE(block.find("signalk = {") != std::string::npos);
    REQUIRE(block.find("  host = \\\"0.0.0.0\\\";") != std::string::npos);
    REQUIRE(block.find("  port = 20223;") != std::string::npos);

    REQUIRE(block.find("  websocket = {") != std::string::npos);
    REQUIRE(block.find("    enabled = true;") != std::string::npos);
    REQUIRE(block.find("    host = \\\"0.0.0.0\\\";") != std::string::npos);
    REQUIRE(block.find("    port = 3001;") != std::string::npos);
    REQUIRE(block.find("    max_connections = 8;") != std::string::npos);
    REQUIRE(block.find("    allow_rx = true;") != std::string::npos);
    REQUIRE(block.find("    allow_tx = true;") != std::string::npos);

    REQUIRE(block.find("connectors = (") != std::string::npos);
    REQUIRE(block.find("protocol = \\\"nmea0183\\\";") != std::string::npos);
    REQUIRE(block.find("transport = \\\"udp\\\";") != std::string::npos);
    REQUIRE(block.find("listen_port = 10110;") != std::string::npos);

    return 0;
}
