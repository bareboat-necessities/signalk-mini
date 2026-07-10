#include <assert.h>
#include <string.h>
#include <signalk_mini/subscription.hpp>

using signalk_mini::SignalKSubscriptionSet;
using signalk_mini::SubscriptionApplyResult;
using signalk_mini::SubscriptionPolicy;

int main() {
    SignalKSubscriptionSet<4> subscriptions;
    assert(subscriptions.all());
    assert(subscriptions.matches("navigation.speedThroughWater"));

    const char unsubscribe_all[] = R"({"context":"*","unsubscribe":[{"path":"*"}]})";
    assert(subscriptions.apply_message(unsubscribe_all, strlen(unsubscribe_all)) == SubscriptionApplyResult::Applied);
    assert(subscriptions.empty());

    const char subscribe[] = R"({"context":"vessels.self","subscribe":[{"path":"navigation.speedThroughWater","period":250,"minPeriod":50,"policy":"instant","format":"delta"},{"path":"environment.*.temperature","policy":"fixed"}]})";
    assert(subscriptions.apply_message(subscribe, strlen(subscribe)) == SubscriptionApplyResult::Applied);
    assert(subscriptions.size() == 2);
    assert(subscriptions.matches("navigation.speedThroughWater"));
    assert(!subscriptions.matches("navigation.speedOverGround"));
    assert(subscriptions.matches("environment.inside.temperature"));
    assert(!subscriptions.matches("environment.inside.humidity"));
    assert(subscriptions[0].timing.period_ms == 250);
    assert(subscriptions[0].timing.min_period_ms == 50);
    assert(subscriptions[0].timing.policy == SubscriptionPolicy::Instant);

    const char trailing_wildcard[] = R"({"subscribe":[{"path":"propulsion.port.*"}]})";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(trailing_wildcard, strlen(trailing_wildcard)) == SubscriptionApplyResult::Applied);
    assert(subscriptions.matches("propulsion.port.oilTemperature"));
    assert(subscriptions.matches("propulsion.port.transmission.oilTemperature"));
    assert(!subscriptions.matches("propulsion.starboard.oilTemperature"));

    const char malformed[] = R"({"subscribe":[{"path":)";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(malformed, strlen(malformed)) == SubscriptionApplyResult::FallbackToAll);
    assert(subscriptions.all());

    const char unsupported_context[] = R"({"context":"vessels.123456789","subscribe":[{"path":"navigation.position"}]})";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(unsupported_context, strlen(unsupported_context)) == SubscriptionApplyResult::FallbackToAll);
    assert(subscriptions.all());

    const char unsupported_policy[] = R"({"subscribe":[{"path":"navigation.position","policy":"sometimes"}]})";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(unsupported_policy, strlen(unsupported_policy)) == SubscriptionApplyResult::FallbackToAll);
    assert(subscriptions.all());

    const char over_capacity[] = R"({"subscribe":[{"path":"a"},{"path":"b"},{"path":"c"},{"path":"d"},{"path":"e"}]})";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(over_capacity, strlen(over_capacity)) == SubscriptionApplyResult::FallbackToAll);
    assert(subscriptions.all());

    const char invalid_unsubscribe[] = R"({"unsubscribe":[{}]})";
    subscriptions.unsubscribe_all();
    assert(subscriptions.apply_message(invalid_unsubscribe, strlen(invalid_unsubscribe)) == SubscriptionApplyResult::FallbackToAll);
    assert(subscriptions.all());

    return 0;
}
