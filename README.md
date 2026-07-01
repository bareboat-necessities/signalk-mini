# signalk-mini

Lightweight low-latency C++ Signal K mini server for Arduino and Linux.

This stage vendors the actual uploaded source modules as source copies under stable module names:

- `modules/async-event-loop/src`
- `modules/nmea0183/src`
- `modules/data-model/src`

Only module sources are copied. The imported module tests, examples, docs, workflows, and package metadata are intentionally not copied into `modules/`.

Project direction:

- single-threaded event-loop architecture
- strongly typed internal data model, not Signal K as store
- Signal K as an output adapter layer
- `template<typename Real>` through core paths, with `float` as the default instantiation
- unit suffixes preserved in data model field names
- ArduinoJson v7 at the JSON/config/protocol boundary

Current smoke build verifies that NMEA0183 input updates the typed data model and that the Signal K mapper can map model changes to Signal K paths. It is not the final server implementation yet.

## Build

```bash
cmake -S . -B build -DSIGNALK_MINI_FETCH_ARDUINOJSON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```
