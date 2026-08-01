# Kemudi.js

Kemudi.js is a web-native vehicle sandbox focused on real-time soft-body deformation, drivable physics, and extensible content. The project is designed to remain playable on low-resource devices while providing a clear path toward multiplayer, user-created maps, vehicles, models, and mods.

> **Status:** Early development. The repository is being built production-first; the architecture and guides describe the intended implementation and acceptance standards. Features are not considered complete until they pass automated tests and production-like browser verification.

## What Kemudi.js aims to provide

- Deformable vehicles powered by Rust and WebAssembly physics.
- A Three.js renderer with `auto`, `low`, `medium`, and `high` graphics presets.
- A Web Worker physics boundary so simulation does not block the UI thread.
- Deterministic controls, collisions, and vehicle behavior across graphics presets.
- Terrain and environment systems designed for culling, LOD, and low-resource operation.
- Multiplayer through validated WebSocket messages, prediction, reconciliation, and reconnect handling.
- Versioned content formats for future user-created maps, vehicles, models, skins, and mods.
- Structured, gated `console.debug` diagnostics without noisy per-frame logging or sensitive payloads.
- Required unit, integration, performance, and production-bundle E2E coverage.

## Architecture at a glance

| Boundary | Responsibility | Reliability/performance rule |
| --- | --- | --- |
| Nuxt/Vue UI | Menus, settings, editor, gameplay screens | Keep UI state typed and avoid work in render loops |
| Three.js renderer | Scene, vehicles, terrain, effects | Apply the effective graphics preset; reuse and dispose resources |
| Physics Worker | Rust/WASM simulation and fixed-timestep stepping | No main-thread physics; bound queues and reuse buffers |
| WebSocket layer | Inputs, snapshots, rooms, reconnects | Validate, cap, compress or delta-encode, and avoid unchanged broadcasts |
| Content pipeline | Vehicle, map, model, and mod loading | Version, validate, size-limit, and fail safely |
| CI/CD | Quality gates, builds, E2E, deployment | Fail closed; deploy immutable artifacts through staging |

See [the architecture guide](docs/architecture.md) for the runtime boundaries and data flow.

## Requirements

- [Bun](https://bun.sh/) for JavaScript dependencies and scripts.
- Node.js 20+ compatibility for tooling and server processes.
- Rust and the `wasm32-unknown-unknown` target for physics development.
- `wasm-pack` for building the WebAssembly package.
- A modern browser with WebGL 2 and WebAssembly support. The application must feature-detect capabilities and fall back safely.

## Local development

```bash
bun install
bun run dev
```

Open the URL printed by Nuxt. Use the browser console and the in-game diagnostics controls when debugging; production diagnostics are disabled by default.

Available scripts currently include:

```bash
bun run dev       # Start the Nuxt development server
bun run build     # Build the production bundle
bun run generate  # Generate a static build when supported by the application
bun run preview   # Serve the production build locally
```

As the test tooling is added, the canonical commands will be exposed through `package.json` scripts and documented in [Testing](docs/testing.md).

## Documentation and guides

- [Getting started](docs/getting-started.md) — install tools, run the app, and understand the development workflow.
- [Architecture](docs/architecture.md) — runtime boundaries, data flow, and lifecycle rules.
- [Graphics and performance](docs/graphics-and-performance.md) — presets, budgets, adaptive quality, and low-resource guidance.
- [Physics](docs/physics.md) — node/beam simulation, fixed timesteps, XPBD, and worker communication.
- [Vehicles and controls](docs/vehicles-and-controls.md) — vehicle definitions, deformation, input, drivetrain, and rendering.
- [Multiplayer](docs/multiplayer.md) — WebSocket protocol expectations, prediction, reconciliation, and recovery.
- [Content and modding](docs/content-and-modding.md) — future maps, vehicles, models, skins, schemas, and safe mod loading.
- [Debugging](docs/debugging.md) — console diagnostics, runtime-boundary verification, and failure triage.
- [Testing](docs/testing.md) — unit, integration, performance, and production E2E testing.
- [CI/CD and releases](docs/ci-cd.md) — required gates, staging promotion, artifacts, and rollback.

## Production quality bar

Every feature is treated as production code from its first implementation. A feature is not complete when it merely compiles: it must have explicit failure behavior, bounded resource usage, tests at the appropriate boundary, and browser evidence from the built production artifact. Pull requests must pass the CI/CD gates before merge.

## Contributing

Keep changes focused, preserve the existing architecture boundaries, and update the relevant guide when behavior or contracts change. Do not commit secrets, generated build output, or unverified performance claims. See the implementation plan in `.hermes/plans/implementation.md` for the current delivery breakdown.

## License

License details will be added before public release.