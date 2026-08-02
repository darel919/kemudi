# Architecture

Kemudi.js separates presentation, simulation, transport, and content so a low graphics preset can reduce cost without changing gameplay rules.

## Runtime boundaries

```text
Nuxt/Vue UI
   ├── graphics/settings state ──> Three.js renderer
   ├── controls/input ───────────> Physics Worker
   └── multiplayer UI ───────────> WebSocket client

Physics Worker ──> Rust/WASM solver ──> transferable state buffers
WebSocket server ──> validated rooms, inputs, snapshots, reconnect state
Content loader ──> versioned vehicle/map/model/mod schemas
```

### UI and state

Vue components render state and collect input. Pinia or composables own state that crosses component boundaries. UI code must not directly mutate solver internals or create network payloads without validation.

### Configurable driving HUD

The driving HUD uses a modular telemetry deck rather than a fixed collection of gauges. The default data display renders the loaded vehicle's engine torque curve against the live engine RPM operating point. Drivers can switch the display to a live trace and select any numeric virtual OBD/CAN signal with a declared range; signals currently published by the session include acceleration, wheel speed, torque output, thermal values, and safety-controller values.

Live trace history is sampled by the presentation layer and remains bounded by the telemetry store history limit. Chart selection and display mode are persisted locally per browser; they do not alter authoritative physics or vehicle configuration. Invalid local preferences fall back to the default engine response display.

### Renderer

Three.js owns visual state only. It consumes the latest simulation snapshot, applies LOD/culling/preset settings, and updates existing typed arrays and attributes where possible. It must dispose geometries, materials, textures, render targets, and renderer resources when a scene is removed.

### Physics Worker

The worker owns the Rust/WASM instance and fixed-timestep simulation. The main thread sends commands and inputs; it does not run physics. Worker queues are bounded. Stale render snapshots may be coalesced, but authoritative simulation inputs must not be silently reordered or duplicated.

### Networking

The client sends validated input messages and receives authoritative snapshots. The server validates message shape, size, rate, and ownership before applying input. Payloads must not contain secrets or unbounded user-controlled data.

### Content

Maps, vehicles, models, and skins are data-driven and versioned. Loading is isolated from rendering and simulation so malformed or incompatible content produces a recoverable error rather than corrupting the active session.

## Lifecycle rules

- Initialize resources once and expose an explicit shutdown/dispose path.
- Handle tab visibility, WebGL context loss, worker failure, and WebSocket disconnects.
- Preserve the root exception and include a user-safe error state.
- Keep queues, caches, histories, particle pools, and content sizes bounded.
- Never treat a successful build as proof that the runtime boundary works.

## Performance ownership

The renderer owns render budgets, the worker owns simulation budgets, and the network layer owns bandwidth/update budgets. Graphics adaptation may change visual quality and non-authoritative update frequency, but it must not change collision correctness or authoritative gameplay rules.
