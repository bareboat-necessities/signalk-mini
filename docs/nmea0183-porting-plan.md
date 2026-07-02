# NMEA0183 Parser and Data Model Porting Plan

The current `nmea0183_connector` and `ship_data_model` coverage is intentionally small. It should be replaced incrementally with a broader tokenizer, parser, decoder set, and typed data model, but not by dumping a large external codebase into the tree all at once.

## Candidate upstreams to study

1. gpsd

- Mature C implementation with broad GNSS and AIS handling.
- Useful as a reference for packetization, checksum policy, odd-device handling, AIS handling, and regression-style test data.
- Not a direct copy target because gpsd is a full daemon with its own JSON model, scheduler assumptions, and large platform surface.

2. minmea

- Small C NMEA parser with a compact style that is easier to port into embedded C++.
- Useful for sentence-level parsing patterns and small test fixtures.
- Mostly GNSS focused, so it will not be enough for marine-specific AIS, SeaTalk bridge data, digital selective calling, satellite-message, and instrument sentences.

3. Signal K nmea0183-signalk

- Useful for mapping semantics from NMEA0183 sentences to Signal K paths.
- Good reference for expected Signal K output behavior and edge cases around multiple sources.
- JavaScript/Node implementation, so port concepts and tests rather than code directly.

## Porting strategy

1. Keep raw-line tokenization, checksum validation, sentence decoding, model update, and Signal K mapping as separate layers.
2. Keep NMEA0183, AIS, SeaTalk bridge data, digital selective calling, and satellite-message handling as separate decoder modules.
3. Keep decoded values going into the typed model first; Signal K remains an output adapter.
4. Expand `ship_data_model` alongside decoder support, not after it.
5. Add fixtures before each decoder expansion.
6. Prefer small, table-driven sentence decoders over one large switch statement.
7. Preserve Arduino compatibility: no heap-heavy parser requirements in the hot line path.

## Parser and tokenizer extension hooks already added

The base parser now tags successful sentences with `NmeaSentenceFamily` and exposes optional callback hooks through `NmeaParserHooks`.

Families reserved for later decoder modules:

- `Standard`
- `Ais`
- `SeaTalk`
- `Dsc`
- `Inmarsat`
- `Proprietary`
- `UnknownEncapsulation`

The tokenizer can also split an Inmarsat-C style envelope from an embedded NMEA sentence, such as `/g:.../$CSSM3,...`.

The hooks are intentionally classification-only for now. They do not decode payloads yet.

## Typed data model expansion plan

`ship_data_model` is the internal store. It must be expanded before or together with parser support. Signal K JSON must not become the internal data model.

### Data model principles

- Strongly typed fields with explicit units in names.
- Preserve source and timestamp metadata at the store/change layer.
- Prefer optional/valid fields over sentinel numeric values.
- Keep raw protocol payloads only where decoding is not yet implemented.
- Keep AIS/DSC/satellite communication state separate from own-vessel navigation state.
- Avoid heap requirements in core embedded model paths unless bounded and explicit.

### Proposed model subtrees

1. Own-vessel navigation

- Position: latitude, longitude, altitude, datum where applicable.
- Motion: speed over ground, course over ground, track made good.
- GNSS quality: fix type, satellites used, HDOP/VDOP/PDOP, differential age/station.
- Time/date: UTC time, date, source validity.

2. Heading and attitude

- True heading, magnetic heading, variation, deviation.
- Rate of turn.
- Roll/pitch/yaw where available from instrument sentences.

3. Wind

- Apparent wind angle/speed.
- True wind angle/speed/direction.
- Unit-normalized storage plus original-unit parsing tests.

4. Water/environment

- Depth below transducer/surface/keel offsets.
- Water temperature.
- Speed through water.
- Trip/log distance.

5. AIS targets

- Target table keyed by MMSI.
- Dynamic target data: position, course, speed, heading, rate of turn, navigation status, timestamp.
- Static/voyage data: vessel name, callsign, IMO, ship type, dimensions, destination, ETA, draught.
- AtoN/base-station specific fields.
- Fragment collection should remain outside the final target table until a full AIS message is decoded.

6. SeaTalk bridge data

- Raw bridge command id and payload bytes for unknown commands.
- Decoded known fields as they are implemented: depth, speed, wind, heading, autopilot status, alarms.
- Keep this as bridge/instrument data, not as native SeaTalk bus state yet.

7. Digital selective calling / safety communications

- Received call metadata fields only.
- Message category/type, source/destination identities, position/time if present, acknowledgement/status fields.
- No automatic external action; state update only.

8. Satellite/Inmarsat-style message data

- Envelope metadata: gateway/source name, message id, timestamp when present.
- Embedded NMEA message family and raw embedded sentence.
- Decoded position/status fields only after fixtures exist.

9. System/source metadata

- Source label, protocol, transport, connector id.
- Last-seen timestamps.
- Per-source parse error counters and accepted sentence counters.

## Data model test strategy

Each new model subtree needs tests at three levels:

1. Model-only tests

- Field default invalid state.
- Setting a field marks validity and preserves units.
- Updating one subtree does not mutate unrelated subtrees.

2. Decoder-to-model tests

- Feed one sentence fixture.
- Assert exact model fields and units.
- Assert source/timestamp metadata when routed through `ModelStore`.
- Include malformed and empty-field variants.

3. Model-to-Signal K tests

- Assert mapped Signal K path and converted SI value.
- Assert missing/invalid model fields do not publish bogus values.
- Assert AIS/DSC/satellite data is not flattened into own-vessel paths.

## Decoder module plan

### Phase 1: Parser/tokenizer conformance fixtures

- Valid and invalid checksum cases.
- Missing checksum cases under strict and relaxed policy.
- Empty fields and trailing empty fields.
- Encapsulated `!` sentences.
- Proprietary `$P...` sentences.
- Inmarsat-C envelope plus embedded NMEA sentence.
- SeaTalk bridge sentence shape.
- DSC sentence shape.
- Long sentence rejection.

### Phase 2: Core instrument sentences and model fields

- Position/navigation: RMC, GGA, GLL, VTG, GSA, GSV.
- Heading: HDT, HDM, HDG, ROT.
- Wind: MWV, MWD, VWR.
- Depth: DBT, DPT.
- Water and environment: MTW, VHW, VLW.
- Expand `ship_data_model` for missing fields before accepting each decoder.

### Phase 3: AIS

- Add AIS fragment collector keyed by sequential message id and radio channel.
- Add AIS six-bit payload decoder.
- Start with message types 1, 2, 3, 5, 18, 19, 21, 24.
- Keep AIS targets in a typed AIS model subtree, not directly in Signal K JSON.
- Add fixtures from upstream regression data and synthetic minimal messages.

### Phase 4: SeaTalk bridge hooks

- Support NMEA-style SeaTalk bridge sentences such as `$STALK,...` and known proprietary variants.
- Keep raw datagram fields available until exact command mappings are implemented.
- Decode only well-known, documented bridge records first.
- Add bridge-command tests before mapping those values into common instrument fields.

### Phase 5: Digital selective calling sentence family

- Add dedicated decoder for DSC/DSE-style sentence families.
- Keep decoded fields in a typed communications/safety model subtree.
- Do not interpret or trigger any external action from incoming messages; parse and publish state only.

### Phase 6: Satellite/Inmarsat-style sentence family

- Add vendor/proprietary satellite-message decoder modules after collecting real sample logs.
- Preserve envelope metadata separately from embedded NMEA sentence data.
- Keep this separate from GNSS talkers because talker IDs can overlap with integrated navigation equipment.

### Phase 7: Signal K mapping expansion

- Add Signal K mapper coverage after typed model coverage exists.
- Add integration tests from NMEA fixture line to typed model to Signal K delta.

## Acceptance rule

Each added sentence decoder must include:

- tokenizer/parser fixture when the line shape is new,
- at least one valid sentence fixture,
- at least one malformed/empty-field fixture,
- typed model field assertions,
- source/timestamp/change assertions when routed through `ModelStore`,
- Signal K output assertion when the field has a Signal K path,
- no direct Signal K JSON writes from the decoder.
