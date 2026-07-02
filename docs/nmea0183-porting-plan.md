# NMEA0183 Parser Porting Plan

The current `nmea0183_connector` and `ship_data_model` coverage is intentionally small. It should be replaced incrementally with a broader parser and a larger typed data model, but not by dumping a large external codebase into the tree all at once.

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

1. Keep line parsing and checksum validation independent from sentence decoding.
2. Keep NMEA0183, AIS, SeaTalk bridge data, digital selective calling, and satellite-message handling as separate decoder modules.
3. Keep decoded values going into the typed model first; Signal K remains an output adapter.
4. Add fixtures before each decoder expansion.
5. Prefer small, table-driven sentence decoders over one large switch statement.
6. Preserve Arduino compatibility: no heap-heavy parser requirements in the hot line path.

## Parser extension hooks already added

The base parser now tags successful sentences with `NmeaSentenceFamily` and exposes optional callback hooks through `NmeaParserHooks`.

Families reserved for later decoder modules:

- `Standard`
- `Ais`
- `SeaTalk`
- `Dsc`
- `Inmarsat`
- `Proprietary`
- `UnknownEncapsulation`

The hooks are intentionally classification-only for now. They do not decode payloads yet.

## Decoder module plan

### Phase 1: Parser conformance fixtures

- Valid and invalid checksum cases.
- Missing checksum cases under strict and relaxed policy.
- Empty fields and trailing empty fields.
- Encapsulated `!` sentences.
- Proprietary `$P...` sentences.
- Long sentence rejection.

### Phase 2: Core instrument sentences

- Position/navigation: RMC, GGA, GLL, VTG, GSA, GSV.
- Heading: HDT, HDM, HDG.
- Wind: MWV, MWD, VWR.
- Depth: DBT, DPT.
- Water and environment: MTW, VHW, VLW where model support exists.

### Phase 3: AIS

- Add AIS fragment collector keyed by sequential message id and radio channel.
- Add AIS six-bit payload decoder.
- Start with message types 1, 2, 3, 5, 18, 19, 21, 24.
- Keep AIS targets in a typed AIS model subtree, not directly in Signal K JSON.

### Phase 4: SeaTalk bridge hooks

- Support NMEA-style SeaTalk bridge sentences such as `$STALK,...` and known proprietary variants.
- Keep raw datagram fields available until exact command mappings are implemented.
- Decode only well-known, documented bridge records first.

### Phase 5: Digital selective calling sentence family

- Add dedicated decoder for DSC/DSE-style sentence families.
- Keep decoded fields in a typed communications/safety model subtree.
- Do not interpret or trigger any external action from incoming messages; parse and publish state only.

### Phase 6: Satellite/Inmarsat-style sentence family

- Add vendor/proprietary satellite-message decoder modules after collecting real sample logs.
- Keep this separate from GNSS talkers because talker IDs can overlap with integrated navigation equipment.

### Phase 7: Signal K mapping expansion

- Add Signal K mapper coverage after typed model coverage exists.
- Add integration tests from NMEA fixture line to typed model to Signal K delta.

## Acceptance rule

Each added sentence decoder must include:

- at least one valid sentence fixture,
- at least one malformed/empty-field fixture,
- model update assertions,
- Signal K output assertion when the field has a Signal K path.
