# Testing

Testing is layered. Fast tests provide feedback during development; production-bundle E2E tests verify that the user-facing runtime actually works.

## Unit tests

Use Vitest for TypeScript/Vue utilities and Rust tests for physics. Cover graphics preset validation, capability detection, persistence fallback, debug gating/throttling, input normalization, schemas, mod validation, and resource disposal.

Physics tests should cover forces, beams, collisions, XPBD stability, fixed timesteps, finite-value recovery, deterministic resets, and malformed definitions.

## Integration tests

Test the worker message contract, WASM initialization, transferable buffers, worker restart, WebSocket validation, room state, rate limiting, prediction/reconciliation, reconnects, and content-loader boundaries.

## Performance tests

Run deterministic WASM microbenchmarks and browser performance checks. Record FPS, frame-time p50/p95, physics-step p50/p95, memory, queue depth, and bandwidth for low-end and desktop reference profiles. Treat regressions as failures when they exceed the checked-in baseline.

## Production E2E with Playwright

E2E tests run against the built production bundle using an isolated browser context and a local production server. Configure traces, screenshots, and video on failure. Capture browser console errors, relevant debug events, failed requests, and UI state as CI artifacts.

Required journeys include:

- Production startup and playable-scene loading.
- Auto/manual graphics presets and persistence.
- Low-resource fallback and unsupported capability messaging.
- Vehicle load, controls, deformation, collision, reset, pause/resume, and visibility changes.
- Multiplayer connect, join/leave, prediction/reconciliation, malformed messages, and reconnect.
- Future map/vehicle/model mod validation, size limits, incompatibility, and safe rejection.

Run a smoke subset on pull requests and the complete suite before release. Do not rely only on tests against the development server.

## Completion standard

A feature is complete only when focused tests, applicable integration tests, production E2E coverage, build validation, and runtime browser verification pass. Known failures must be documented rather than hidden with retries or disabled assertions.
