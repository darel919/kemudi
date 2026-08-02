# `@kemudi/engine-spec`

Private workspace package for JavaScript and TypeScript data shared by the web application and repository tooling.

The package is intended for generated types, telemetry metadata, compatibility-version constants, content-schema helpers, and serializers. Simulation state and Three.js objects remain in their respective runtime packages.

Public field names, telemetry indices, units, enums, and serialized messages are documented in [`spec/README.md`](../../spec/README.md). The package is currently a workspace boundary; its authoritative simulation implementation remains in `crates/kemudi-engine`.
