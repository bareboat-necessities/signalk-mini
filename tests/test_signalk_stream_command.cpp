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

    // Stall detection: an ESP32 ring that drained fully and was refilled to the
    // same 700 bytes before the next sample has made progress.
    REQUIRE(signalk_mini::stream_output_progressed(700, 700, 700));
    REQUIRE(signalk_mini::stream_output_progressed(0, 700, 0));
    REQUIRE(signalk_mini::stream_output_progressed(500, 700, 0));
    // Nothing sent: same queue, nothing added.
    REQUIRE(!signalk_mini::stream_output_progressed(700, 700, 0));
    // Nothing sent, 300 added: queue grew by exactly that.
    REQUIRE(!signalk_mini::stream_output_progressed(1000, 700, 300));

    std::puts("test_signalk_stream_command passed");
    return 0;
}
