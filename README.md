# signalk-mini

Lightweight low-latency C++ Signal K mini server for Arduino and Linux.

This stage vendors only source trees from the uploaded modules:

- `modules/async-event-loop/src`
- `modules/nmea0183/src`
- `modules/data-model/src`

The main project code is now split under `src/signalk_mini/` instead of living in one large `signalk_mini.hpp` file.

The mini server base is now in `signalk_mini/server.hpp`. It follows the original MiniSignalKServerExample structure: one event loop, separate Signal K and NMEA0183 TCP listeners, fixed connection registries, backpressure handling, NMEA line input, typed model updates, and Signal K delta publishing.

Project direction:

- strongly typed internal data model, not Signal K as store
- Signal K as an output adapter layer
- `template<typename Real>` through core paths
- unit suffixes preserved in data model field names
- ArduinoJson v7 at the JSON/config/protocol boundary

## Build

```bash
cmake -S . -B build -DSIGNALK_MINI_FETCH_ARDUINOJSON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Default TCP ports:

- Signal K: `20223`
- NMEA0183: `20224`
