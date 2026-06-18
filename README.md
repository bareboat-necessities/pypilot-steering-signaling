# pypilot-steering-signaling

Actuation-side steering and signaling primitives for the modular pypilot C++ port.

This project owns rudder calibration/runtime and servo command/runtime behavior.  It does not own pilot decision logic, navigation math, GPS parsing, NMEA/SignalK parsing, or global data-model definitions.

## Scope

Owned here:

- rudder sensor calibration
- rudder angle normalization
- rudder range and limit handling
- servo command state
- servo watchdog and timeout behavior
- steering safety gates
- pure protocol/runtime adapters for steering output

Not owned here:

- heading error calculation
- pilot mode selection
- basic/absolute/wind pilot algorithms
- APB/NAV steering math
- GPS, WMM, NMEA, or SignalK parsing
- Linux or Arduino full application orchestration

## Current phases

- Phase 5.0: project scaffold, CMake, Arduino metadata, CI, smoke examples
- Phase 5.1: rudder calibration API and tests

## Dependency direction

Expected runtime flow:

```text
sensors -> data-model -> pilots-logic -> steering-signaling -> servo-protocol/device
```

Keep this project usable from Linux CMake builds and Arduino sketches.
