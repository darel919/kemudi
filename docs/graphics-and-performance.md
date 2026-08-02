# Graphics and performance

Kemudi.js supports four effective graphics modes:

- `auto`: choose a safe starting point from detected device and browser capabilities.
- `low`: first-class low-resource mode for integrated GPUs, mobile devices, and constrained CPUs/RAM.
- `medium`: balanced quality and performance.
- `high`: increased visual quality for capable desktop hardware.

## Preset responsibilities

A preset is a single declarative settings object consumed by the renderer, environment, effects, physics scheduling, and network interpolation. It may control:

- Device pixel ratio and canvas resolution.
- Antialiasing, shadows, lighting, tone mapping, and post-processing.
- Texture resolution, geometry density, vegetation, draw distance, and vehicle LOD.
- Particle and decal budgets.
- Physics rate, solver iterations, interpolation, and snapshot frequency.

The low preset must reduce cost without changing controls, collision rules, or authoritative simulation behavior.

## Auto selection and persistence

Capability detection must be defensive. Unsupported WebGL/WASM features should select a safe fallback and show a clear, non-blocking message. A manual override is persisted locally and can be reset to `auto`. Invalid stored values must be ignored rather than crashing startup.

## Adaptive quality

Adaptive quality reacts to sustained frame-budget violations, not a single slow frame. It should lower expensive visual settings gradually, log one sampled diagnostic event, and recover slowly after stable performance. It must not oscillate rapidly or silently degrade gameplay-critical physics.

The renderer now starts within a bounded physical-pixel budget, adapts the
pixel ratio when measured frame work remains slow, and disables shadows as a
last-resort presentation fallback. The fallback never changes worker physics,
controls, collision behavior, or telemetry.

## Performance budgets

Track at least:

- Render frame-time p50/p95.
- Physics step-time p50/p95.
- Sustained FPS and long-frame count.
- Main-thread CPU time, worker time, GPU memory where available, and JS heap.
- Network bytes per second and snapshot queue depth.

The low-end reference profile targets a sustained 30 FPS minimum. Desktop targets 60 FPS. Measurements must be captured before and after optimization on documented devices; averages alone are insufficient.

## Low-resource implementation rules

- Reuse typed arrays, geometries, materials, and buffers.
- Avoid allocations and object creation in animation and physics hot paths.
- Use frustum/distance culling, instancing, LOD, and pooled particles.
- Coalesce stale visual snapshots while preserving input ordering.
- Dispose GPU and worker resources on scene/session shutdown.
- Keep caches, histories, and queues bounded.
