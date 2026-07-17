# signalk-mini

Lightweight low-latency C++ Signal K mini server for Arduino and Linux.

This stage vendors only source trees from the uploaded modules:

- `modules/async-event-loop/src`
- `modules/nmea0183/src`
- `modules/data-model/src`
- `modules/seatalk/src`
- `modules/ubx/src`

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

Connectors are configuration entries. They describe a protocol plus transport, such as `nmea0183` over `udp`.

Connections are runtime objects. One connector can create zero, one, or many runtime connections. For example, one TCP server connector can accept many TCP connections. UDP listeners do not create TCP-style connections; they receive datagrams on a socket.

Connector config is separated into common connector identity, access flags, protocol-specific settings, and transport-specific settings. For example, NMEA0183 checksum settings belong to the NMEA0183 protocol block, while UDP listen host/port belong to the UDP transport block.

Signal K delta source labels are resolved from the connector that produced a model change when possible. Direct sketch-owned input falls back to the publisher source label.

## Defaults

The built-in default connector listens for NMEA0183 UDP datagrams on all local interfaces:

```text
protocol = "nmea0183";
transport = "udp";
listen_host = "0.0.0.0";
listen_port = 10110;
```

Port `10110` is the de facto NMEA0183-over-IP port used for NMEA0183 navigational data over TCP or UDP. Binding to `0.0.0.0:10110` receives packets sent directly to the host and ordinary IPv4 broadcast packets reaching that interface.

## Linux config

Default config discovery when no explicit config is provided:

```text
~/.signalk-mini/signalk-mini.conf
/etc/signalk-mini/signalk-mini.conf
```

If neither file exists, the Linux binary attempts to create a default config at:

```text
~/.signalk-mini/signalk-mini.conf
```

Example:

```text
examples/linux/signalk-mini.conf
```

The config has one mandatory main Signal K server and optional connectors. Each connector combines protocol plus transport, for example:

```text
enabled = true;
label = "nmea0183-udp-10110";
protocol = "nmea0183";
transport = "udp";

access: {
  allow_rx = true;
  allow_tx = false;
};

nmea0183: {
  validate_checksum = false;
};

udp: {
  listen_host = "0.0.0.0";
  listen_port = 10110;
  allow_broadcast = true;
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

- `udp`
- `tcp_client`
- `tcp_server`
- `serial`

## UBX GNSS connectors

Native u-blox UBX input is available with `protocol = "ubx"` over `serial`, `tcp_client`, `tcp_server`, or `udp`. The initial implementation is receive-only and accepts NAV-PVT, NAV-DOP, NAV-SAT, MON-VER, ACK-ACK, and ACK-NAK messages. It applies complete values directly to the typed GNSS model; raw frames are not retained.

```text
enabled = true;
label = "ublox-main";
protocol = "ubx";
transport = "serial";

access: {
  allow_rx = true;
  allow_tx = false;
};

ubx: {
  configure_receiver = false;
};

serial: {
  device = "/dev/ttyACM0";
  baud = 115200;
};
```

TCP client connectors use bounded reconnect backoff by default. The policy can be configured for NMEA0183, SeaTalk 1, or UBX TCP clients:

```text
reconnect: {
  enabled = true;
  initial_delay_ms = 1000;
  maximum_delay_ms = 30000;
};
```

`gpsd` is recognized by the configuration model so its protocol-specific settings can be loaded, but GPSD runtime support is intentionally not enabled in this stage.

## Minimal Signal K subscribe-all

The main Signal K TCP server sends a hello line immediately after a client connects. Deltas are not sent to that client until it sends a minimal subscribe-all line.

This stage intentionally does not implement the full Signal K subscription protocol yet. To receive all deltas, send one line after reading hello:

```text
{"subscribe":"all"}\r\n
```

Also accepted:

```text
subscribe all\r\n
subscribe\r\n
*\r\n
```

After subscribing, the server publishes all model-change deltas for that connection. It also publishes a server clock delta once per second so a client can verify that the connection is alive even when no NMEA0183 or SeaTalk data is arriving.

The clock is stored internally in the typed model as:

```text
model.comm.server.clock_s
```

It is published on the Signal K TCP stream as:

```text
communication.server.clock
```

Example session with `nc`:

```bash
nc 127.0.0.1 20223
```

Expected first line from the server:

```json
{"name":"signalk-mini","version":"0.1.0","self":"vessels.self","roles":["master","main"]}
```

Then type or send:

```text
{"subscribe":"all"}
```

Expected clock delta shape:

```json
{"updates":[{"source":{"label":"signalk-mini"},"values":[{"path":"communication.server.clock","value":1760000000}]}]}
```

One-shot shell example that subscribes and prints deltas:

```bash
printf '{"subscribe":"all"}\r\n' | nc 127.0.0.1 20223
```

Python example that reads hello, subscribes, and prints clock deltas:

```python
import json
import socket

with socket.create_connection(("127.0.0.1", 20223)) as s:
    f = s.makefile("rw", newline="\n")
    hello = json.loads(f.readline())
    print("hello:", hello)

    f.write('{"subscribe":"all"}\r\n')
    f.flush()

    while True:
        msg = json.loads(f.readline())
        for update in msg.get("updates", []):
            for value in update.get("values", []):
                if value.get("path") == "communication.server.clock":
                    print("server clock:", value.get("value"))
```

## Arduino / MCU strategy

On MCU targets, board-specific sketch code owns hardware I/O for now. The core app owns the Signal K TCP server, the typed model, and delta publishing. The sketch polls UART/I2C/pins or other board APIs and feeds the typed protocol facade directly, for example:

```cpp
app.nmea0183().feed_line(line, signalk_mini::SourceId(1), micros(), true);
app.ubx().feed_octets(bytes, length, signalk_mini::SourceId(2), micros());
```

Use `make_sketch_owned_io_config(...)` for this mode. It disables configured core connectors by setting `connectors = nullptr` and `connector_count = 0`, while keeping the Signal K TCP server and publisher enabled.

The AtomS3R sketch uses this mode. Core UDP/TCP/serial connector runtime support is still Linux-first; Arduino WiFi TCP serving is enabled for the main Signal K server, but Arduino UDP listener connectors are not enabled by the core connector abstraction yet.

## Build

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Linux build dependencies: libevent, libconfig, pkg-config.

Default TCP/UDP ports:

- Signal K TCP server: `20223`
- NMEA0183 UDP listener: `0.0.0.0:10110`
