#include <stdio.h>
#include <string.h>
#include <ArduinoJson.h>
#include <signalk_mini/subscription.hpp>

using signalk_mini::SignalKSubscriptionSet;
using signalk_mini::SubscriptionApplyResult;
using signalk_mini::SubscriptionPolicy;

#define CHECK(expression) do { \
    if (!(expression)) { \
        fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__, #expression); \
        return __LINE__; \
    } \
} while (false)

int main() {
    SignalKSubscriptionSet<4> subscriptions;
    CHECK(subscriptions.all());
    CHECK(subscriptions.matches("navigation.speedThroughWater"));

    const char unsubscribe_all[] = R"({"context":"*","unsubscribe":[{"path":"*"}]})";
    {
        JsonDocument document;
        const DeserializationError error = deserializeJson(document, unsubscribe_all, strlen(unsubscribe_all));
        CHECK(!error);
        JsonObject root = document.as<JsonObject>();
        CHECK(!root.isNull());
        const char* context = root["context"].as<const char*>();
        CHECK(context != nullptr);
        CHECK(strcmp(context, "*") == 0);
        JsonArray array = root["unsubscribe"].as<JsonArray>();
        CHECK(!array.isNull());
        CHECK(array.size() == 1);
        JsonObject entry = array[0].as<JsonObject>();
        CHECK(!entry.isNull());
        const char* path = entry["path"].as<const char*>();
        CHECK(path != nullptr);
        CHECK(strcmp(path, "*") == 0);
    }
    CHECK(subscriptions.apply_message(unsubscribe_all, strlen(unsubscribe_all)) == SubscriptionApplyResult::Applied);
    CHECK(subscriptions.empty());

    const char subscribe[] = R"({"context":"vessels.self","subscribe":[{"path":"navigation.speedThroughWater","period":250,"minPeriod":50,"policy":"instant","format":"delta"},{"path":"environment.*.temperature","policy":"fixed"}]})";
    CHECK(subscriptions.apply_message(subscribe, strlen(subscribe)) == SubscriptionApplyResult::Applied);
    CHECK(subscriptions.size() == 2);
    CHECK(subscriptions.matches("navigation.speedThroughWater"));
    CHECK(!subscriptions.matches("navigation.speedOverGround"));
    CHECK(subscriptions.matches("environment.inside.temperature"));
    CHECK(!subscriptions.matches("environment.inside.humidity"));
    CHECK(subscriptions[0].timing.period_ms == 250);
    CHECK(subscriptions[0].timing.min_period_ms == 50);
    CHECK(subscriptions[0].timing.policy == SubscriptionPolicy::Instant);

    const char trailing_wildcard[] = R"({"subscribe":[{"path":"propulsion.port.*"}]})";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(trailing_wildcard, strlen(trailing_wildcard)) == SubscriptionApplyResult::Applied);
    CHECK(subscriptions.matches("propulsion.port.oilTemperature"));
    CHECK(subscriptions.matches("propulsion.port.transmission.oilTemperature"));
    CHECK(!subscriptions.matches("propulsion.starboard.oilTemperature"));

    const char malformed[] = R"({"subscribe":[{"path":)";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(malformed, strlen(malformed)) == SubscriptionApplyResult::FallbackToAll);
    CHECK(subscriptions.all());

    const char unsupported_context[] = R"({"context":"vessels.123456789","subscribe":[{"path":"navigation.position"}]})";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(unsupported_context, strlen(unsupported_context)) == SubscriptionApplyResult::FallbackToAll);
    CHECK(subscriptions.all());

    const char unsupported_policy[] = R"({"subscribe":[{"path":"navigation.position","policy":"sometimes"}]})";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(unsupported_policy, strlen(unsupported_policy)) == SubscriptionApplyResult::FallbackToAll);
    CHECK(subscriptions.all());

    const char over_capacity[] = R"({"subscribe":[{"path":"a"},{"path":"b"},{"path":"c"},{"path":"d"},{"path":"e"}]})";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(over_capacity, strlen(over_capacity)) == SubscriptionApplyResult::FallbackToAll);
    CHECK(subscriptions.all());

    const char invalid_unsubscribe[] = R"({"unsubscribe":[{}]})";
    subscriptions.unsubscribe_all();
    CHECK(subscriptions.apply_message(invalid_unsubscribe, strlen(invalid_unsubscribe)) == SubscriptionApplyResult::FallbackToAll);
    CHECK(subscriptions.all());

    return 0;
}
