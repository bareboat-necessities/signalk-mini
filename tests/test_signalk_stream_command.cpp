#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

using signalk_mini::SignalKStreamCommand;

static SignalKStreamCommand command(const char* text) {
    return signalk_mini::signalk_stream_command(text, std::strlen(text));
}

int main() {
    REQUIRE(command("*") == SignalKStreamCommand::SubscribeAll);
    REQUIRE(command("  subscribe all\r\n") == SignalKStreamCommand::SubscribeAll);
    REQUIRE(command("SUBSCRIBE") == SignalKStreamCommand::SubscribeAll);
    REQUIRE(command("{\"context\":\"vessels.self\",\"subscribe\":[{\"path\":\"*\"}]}") == SignalKStreamCommand::SubscribeAll);

    // An unsubscribe contains "subscribe" but must stop the stream, never start it.
    REQUIRE(command("{\"context\":\"*\",\"unsubscribe\":[{\"path\":\"*\"}]}") == SignalKStreamCommand::Unsubscribe);
    REQUIRE(command("unsubscribe") == SignalKStreamCommand::Unsubscribe);
    REQUIRE(command("UNSUBSCRIBE all") == SignalKStreamCommand::Unsubscribe);

    REQUIRE(command("") == SignalKStreamCommand::None);
    REQUIRE(command("hello") == SignalKStreamCommand::None);
    REQUIRE(command("{\"requestId\":\"1\",\"put\":{\"path\":\"a\",\"value\":1}}") == SignalKStreamCommand::None);

    // Length-bounded: bytes past len are ignored even without a NUL.
    const char line[] = {'*', 'u', 'n', 's'};
    REQUIRE(signalk_mini::signalk_stream_command(line, 1) == SignalKStreamCommand::SubscribeAll);

    std::puts("test_signalk_stream_command passed");
    return 0;
}
