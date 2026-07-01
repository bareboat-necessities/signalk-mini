#include <signalk_mini.hpp>

int main() {
    signalk_mini::SignalKMiniConfig cfg;
    signalk_mini::SignalKMiniApp<float> app(cfg);
    if (!app.begin()) return 1;
    app.run_forever();
    return 0;
}
