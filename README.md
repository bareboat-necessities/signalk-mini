# signalk-mini

Lightweight low-latency C++ Signal K mini server for Arduino and Linux.

This stage vendors only source trees from the uploaded modules:

- `modules/async-event-loop/src`
- `modules/nmea0183/src`
- `modules/data-model/src`

The main project code is split under `src/signalk_mini/`.

The mini server base is in `signalk_mini/server.hpp`. It has one event loop, one mandatory main Signal K TCP server, connector configuration, runtime connection registries, backpressure handling, NMEA line input, typed model updates, and Signal K delta publishing.

Project direction:

- strongly typed internal data model, not Signal K as store
- Signal K as an output adapter layer
- `template<typename Real>` through core paths
- unit suffixes preserved in data model field names
- ArduinoJson v7 at the JSON/protocol boundary
- Linux daemon configuration uses libconfig `.conf`, not JSON

## Naming

Connectors are configuration entries. They describe a protocol plus transport, such as `nmea0183` over `tcp_client`.

Connections are runtime objects. One connector can create zero, one, or many runtime connections. For example, one TCP server connector can accept many TCP connections.

Connector config is separated into common connector identity, access flags, protocol-specific settings, and transport-specific settings. For example, NMEA0183 checksum settings belong to the NMEA0183 protocol block, while TCP host/port belong to the TCP transport block.

## Linux config

Default path:

```text
/etc/signalk-mini/signalk-mini.conf
```

Example:

```text
examples/linux/signalk-mini.conf
```

The config has one mandatory main Signal K server and optional connectors. Each connector combines protocol plus transport, for example:

```text
enabled = true;
label = "nmea0183-tcp-client";
protocol = "nmea0183";
transport = "tcp_client";

access: {
  allow_rx = true;
  allow_tx = false;
};

nmea0183: {
  validate_checksum = false;
};

tcp_client: {
  host = "127.0.0.1";
  port = 10110;
};
```

or:

```text
enabled = true;
label = "nmea0183-serial";
protocol = "nmea0183";
transport = "serial";

access: {
  allow_rx = true;
  allow_tx = false;
};

nmea0183: {
  validate_checksum = true;
};

serial: {
  device = "/dev/ttyUSB0";
  baud = 4800;
};
```

For NMEA0183, omitted `validate_checksum` defaults by transport:

- `serial`: `true`
- `tcp_client`, `tcp_server`, `udp`: `false`

Implemented NMEA0183 transports in this stage:

- `tcp_client`
- `tcp_server`
- `serial`

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
