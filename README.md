# signalk-mini

Lightweight low-latency C++ Signal K mini server for Arduino and Linux.

This stage vendors only source trees from the uploaded modules:

- `modules/async-event-loop/src`
- `modules/nmea0183/src`
- `modules/data-model/src`

The main project code is split under `src/signalk_mini/`.

The mini server base is in `signalk_mini/server.hpp`. It has one event loop, one mandatory main Signal K TCP server, configurable connectors, fixed connection registries, backpressure handling, NMEA line input, typed model updates, and Signal K delta publishing.

Project direction:

- strongly typed internal data model, not Signal K as store
- Signal K as an output adapter layer
- `template<typename Real>` through core paths
- unit suffixes preserved in data model field names
- ArduinoJson v7 at the JSON/protocol boundary
- Linux daemon configuration uses libconfig `.conf`, not JSON

## Linux config

Default path:

```text
/etc/signalk-mini/signalk-mini.conf
```

Example:

```text
examples/linux/signalk-mini.conf
```

The config has one mandatory main Signal K server and optional connectors. Each connector combines protocol plus transport, for example `protocol = "nmea0183"` and `transport = "tcp_client"`.

## Build

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Linux build dependencies: libevent, libconfig, pkg-config.

Default TCP ports:

- Signal K: `20223`
- default NMEA0183 TCP client target: `127.0.0.1:10110`
