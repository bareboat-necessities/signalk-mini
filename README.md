# signalk-mini

Lightweight low-latency C++ marine data server for Arduino and Linux.

This first pushed stage establishes the planned shape:

- `modules/data-model`: strongly typed internal store, not Signal K JSON
- `modules/nmea0183`: NMEA 0183 checksum/parser/model applier
- `src`: Signal K mini composition layer and mapper
- `linux`: first executable scaffold
- `tests`: realistic NMEA-to-model-to-Signal-K-mapper test

Core rules captured here:

- single-threaded app model; the event-loop thread owns the data model
- all core numeric code is `template<typename Real>`; `float` is only a default instantiation
- unit suffixes are explicit in field names: `_deg`, `_mps`, `_m`
- ArduinoJson v7.4.3 is configured as the JSON dependency for the protocol boundary
- NMEA 0183 writes typed data model fields; Signal K is an output adapter layer

The current stage implements a starter NMEA set: RMC, HDT, MWV, and DPT. The next stage should expand this module to the full planned NMEA 0183 sentence matrix with the same typed model boundary.

## Build

```bash
cmake -S . -B build -DSIGNALK_MINI_FETCH_ARDUINOJSON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```
