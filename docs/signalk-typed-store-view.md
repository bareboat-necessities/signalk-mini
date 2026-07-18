# Signal K typed-store projection

`signalk-mini` keeps the strongly typed `DataModel<Real>` as its only retained vessel-state store.

The Signal K layer does not retain a second JSON tree and does not cache serialized values as strings. `SignalKTypedModelView<Real>` reads valid typed fields directly from the current model and supplies descriptors to bounded serializers. Object-valued Signal K data, including `navigation.position`, `navigation.attitude`, `navigation.datetime`, and `environment.current`, is composed only while a delta or full-model snapshot is being written.

This design keeps MCU memory use predictable:

- typed values are stored once;
- only compact field-presence and source bookkeeping is retained;
- snapshot output uses caller-provided fixed-capacity buffers;
- reading a snapshot does not consume the live model-change queue;
- no dynamic Signal K DOM is required;
- no serialized-value cache exists alongside the typed model.

When multiple typed fields map to the same Signal K path, the view selects the freshest valid field and uses a deterministic field-order tie break. Configured expiry is applied while walking both self-vessel values and AIS contexts, so a new client does not receive stale targets or stale alternate-unit values.

Object completeness is enforced before projection. In particular, `navigation.position` requires both latitude and longitude; altitude is included only when available. Partial coordinates are not advertised as a current Signal K position.

New TCP or WebSocket clients can receive current values by walking this live view. The values are therefore always generated from the authoritative typed store rather than from stale serialized copies.
