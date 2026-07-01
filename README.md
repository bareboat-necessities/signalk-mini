# signalk-mini

Lightweight low-latency C++ marine data server for Arduino and Linux.

This repository is organized around the planned architecture:

- `modules/async-event-loop`: git submodule pointing to the full async event loop project.
- `modules/data-model`: git submodule pointing to the full strongly typed marine/autopilot data model.
- `modules/nmea0183`: git submodule pointing to the full NMEA0183 connector/parser/formatter project.
- `src`: signalk-mini composition layer that owns model store, change queue, NMEA input adapter, and Signal K mapper scaffolding.
- `linux`: first executable scaffold.
- `tests`: signalk-mini integration stage test using realistic NMEA input.

Core project rules:

- The app is single-threaded. The event-loop thread owns and mutates the data model.
- The internal store is a strongly typed `ship_data_model::DataModel<Real>`, not a Signal K JSON tree.
- Signal K is an output adapter layer above the typed model.
- Core code is parameterized as `template<typename Real>`; `float` is only a default instantiation.
- Data model fields should be unit-explicit. The imported model already carries many suffixes such as `_deg`, `_kn`, `_m`, `_hz`, `_a`, and `_s`; the next data-model cleanup stage should normalize new canonical speed fields to `_mps` for signalk-mini output boundaries.
- ArduinoJson v7.4.3 is configured as the JSON dependency for protocol boundary code.
- NMEA0183 writes typed data-model fields and does not know Signal K paths.

The current signalk-mini wrapper uses the imported NMEA0183 connector and imported data model, and verifies RMC/HDT/MWV/DPT flow into the model plus Signal K mapping for the starter paths. The imported NMEA module already contains broader sentence support and module tests; the next stage should add realistic corpus/golden tests for the full sentence matrix.

## Clone

```bash
git clone --recurse-submodules https://github.com/bareboat-necessities/signalk-mini.git
```

If already cloned:

```bash
git submodule update --init --recursive
```

## Build

```bash
cmake -S . -B build -DSIGNALK_MINI_FETCH_ARDUINOJSON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## First stdin prototype

```bash
./build/signalk-mini < fixtures/nmea0183/coastal_gps_wind_depth.nmea
```
