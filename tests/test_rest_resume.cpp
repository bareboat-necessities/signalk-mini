#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

namespace {

using signalk_mini::RestPassResult;
using signalk_mini::RestResumeJob;

void load_fixture(signalk_mini::ModelStore<float>& store, const char* name) {
    signalk_mini::Nmea0183Input<float> input(store);
    std::ifstream in(std::filesystem::path(__FILE__).parent_path() / "fixtures" / "nmea" / name);
    REQUIRE(in.good());
    std::string line;
    uint64_t now_us = 1000000ULL;
    while (std::getline(in, line)) {
        const auto start = line.find_first_of("$!");
        if (start == std::string::npos) continue;
        line.erase(0, start);
        while (!line.empty() && line.back() == '\r') line.pop_back();
        input.feed_line(line.c_str(), 7, now_us += 1000, false);
    }
}

void collect_keys(JsonVariantConst value, const std::string& prefix, std::set<std::string>& keys) {
    if (value.is<JsonObjectConst>()) {
        for (JsonPairConst member : value.as<JsonObjectConst>()) {
            const std::string key = prefix + "/" + member.key().c_str();
            keys.insert(key);
            collect_keys(member.value(), key, keys);
        }
    } else if (value.is<JsonArrayConst>()) {
        size_t i = 0;
        for (JsonVariantConst item : value.as<JsonArrayConst>()) collect_keys(item, prefix + "[" + std::to_string(i++) + "]", keys);
    }
}

std::set<std::string> keys_of(const std::string& json) {
    JsonDocument doc;
    REQUIRE(!deserializeJson(doc, json));
    std::set<std::string> keys;
    collect_keys(doc.as<JsonVariantConst>(), "", keys);
    return keys;
}

std::string write_once(const signalk_mini::ModelStore<float>& store, const signalk_mini::SignalKMiniConfig& config, const char* path) {
    std::string out;
    auto sink = [](void* context, const char* data, size_t len) {
        static_cast<std::string*>(context)->append(data, len);
        return true;
    };
    signalk_mini::SignalKJsonStreamWriter writer(sink, &out);
    signalk_mini::SignalKFullModelWriter<float> model_writer;
    REQUIRE(model_writer.write_api_path(writer, path, store, config, 2000000ULL) == signalk_mini::SignalKModelWriteResult::Written);
    return out;
}

// Runs one pass that may write at most `budget` bytes.
RestPassResult pass(RestResumeJob& job,
                    const signalk_mini::ModelStore<float>& store,
                    const signalk_mini::SignalKMiniConfig& config,
                    size_t budget,
                    std::string& client) {
    size_t written = 0;
    auto write = [&](const char* data, size_t len) {
        client.append(data, len);
        written += len;
        return true;
    };
    auto room = [&]() { return budget > written ? budget - written : 0; };
    signalk_mini::RestResumeSink<decltype(write), decltype(room)> sink(job, write, room);
    signalk_mini::SignalKJsonStreamWriter out(&decltype(sink)::write, &sink);
    signalk_mini::SignalKFullModelWriter<float> model_writer;
    const auto result = model_writer.write_api_path(out, job.path, store, config, job.now_us);
    if (sink.result() != RestPassResult::Complete) return sink.result();
    return result == signalk_mini::SignalKModelWriteResult::Written ? sink.finish() : RestPassResult::Failed;
}

void test_lexer_marks_leaves_only() {
    signalk_mini::JsonLeafLexer lexer;
    const char* json = "{\"a\\\"]\":\"x,}\\\"\",\"b\":[1,true,{\"c\":-2.5e3}],\"d\":{}}";
    int leaves = 0;
    std::string structure;
    for (const char* p = json; *p; ++p) {
        const auto step = lexer.feed(*p);
        if (step.leaf_ends_before) ++leaves;
        if (step.leaf_ends_after) ++leaves;
        if (step.structural) structure += *p;
    }
    REQUIRE(!lexer.finish_leaf());
    REQUIRE(leaves == 4);
    // Keys and punctuation only; the string value "x,}\"" and the numbers are excluded.
    REQUIRE(structure == "{\"a\\\"]\":,\"b\":[,,{\"c\":}],\"d\":{}}");
}

void test_resume_survives_changing_values() {
    signalk_mini::SignalKMiniConfig config;
    signalk_mini::ModelStore<float> store;
    load_fixture(store, "tripmate_850_gps_sample.nmea");
    store.model().wind.apparent.speed_kn.set(10.0f, 1500000ULL);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 7, 1500000ULL);
    const std::string reference = write_once(store, config, "");
    REQUIRE(reference.size() > 1500);

    RestResumeJob job;
    job.connection = &job;
    job.now_us = 2000000ULL;
    std::string client;
    int passes = 0;
    RestPassResult result = RestPassResult::Paused;
    while (result == RestPassResult::Paused) {
        result = pass(job, store, config, 200, client);
        ++passes;
        REQUIRE(passes < 1000);
        // The value changes between passes, also changing its printed length.
        store.model().wind.apparent.speed_kn.set(10.0f + static_cast<float>(passes) * 1.37f, 1500000ULL);
    }
    REQUIRE(result == RestPassResult::Complete);
    REQUIRE(passes > 5);
    REQUIRE(keys_of(client) == keys_of(reference));
}

void test_structure_change_aborts() {
    signalk_mini::SignalKMiniConfig config;
    signalk_mini::ModelStore<float> store;
    load_fixture(store, "tripmate_850_gps_sample.nmea");

    RestResumeJob job;
    job.connection = &job;
    job.now_us = 2000000ULL;
    std::string client;
    REQUIRE(pass(job, store, config, 300, client) == RestPassResult::Paused);
    REQUIRE(job.leaves_sent > 0);

    // The position, already partly sent, disappears: the structure of the
    // sent prefix no longer matches, so resuming by position would be wrong.
    store.model().gnss.fix.fix_lat_deg.valid = false;
    store.model().gnss.fix.fix_lon_deg.valid = false;
    RestPassResult result = RestPassResult::Paused;
    for (int i = 0; i < 1000 && result == RestPassResult::Paused; ++i) result = pass(job, store, config, 300, client);
    REQUIRE(result == RestPassResult::Failed);
}

// Runs one pass over a fixed JSON text instead of the model writer.
RestPassResult pass_text(RestResumeJob& job, const char* json, size_t budget, std::string& client) {
    size_t written = 0;
    auto write = [&](const char* data, size_t len) {
        client.append(data, len);
        written += len;
        return true;
    };
    auto room = [&]() { return budget > written ? budget - written : 0; };
    signalk_mini::RestResumeSink<decltype(write), decltype(room)> sink(job, write, room);
    signalk_mini::SignalKJsonStreamWriter out(&decltype(sink)::write, &sink);
    if (!out.append_raw(json)) return sink.result();
    return sink.finish();
}

void test_shorter_document_aborts() {
    RestResumeJob job;
    job.connection = &job;
    std::string client;
    // Two leaf segments of 6 bytes fit; the third does not.
    REQUIRE(pass_text(job, "{\"a\":1,\"b\":2,\"c\":3}", 12, client) == RestPassResult::Paused);
    REQUIRE(job.leaves_sent == 2);
    // The document lost values before the resume point: it must not complete.
    REQUIRE(pass_text(job, "{\"a\":1}", 100, client) == RestPassResult::Failed);
}

} // namespace

int main() {
    test_lexer_marks_leaves_only();
    test_resume_survives_changing_values();
    test_structure_change_aborts();
    test_shorter_document_aborts();
    std::puts("test_rest_resume passed");
    return 0;
}
