# Physics

Kemudi.js uses a custom Rust-to-WebAssembly soft-body solver. Vehicles are represented by nodes connected by beams and rendered separately from the simulation.

Current status: the checked-in solver implements node/beam force integration, bounded fixed-step stepping, finite-value guards, XPBD-style compliance/lambda constraints, triangle area preservation, terrain profiles/contact friction, suspension, drivetrain torque, engine thermal/damage state, tire wear, safety hooks, and a fixed-width telemetry buffer.

## Data model

- **Node:** position, velocity, force, mass/inverse mass, and fixed state.
- **Beam:** endpoint node IDs, rest length, stiffness, damping, break strength, broken state, and XPBD lambda.
- **Vehicle:** nodes, beams, triangles/skin data, drivetrain configuration, suspension/wheel state, tires, fuel, safety systems, and engine thermal/damage state.

Bundled vehicle definitions use their first four nodes as suspension mounts and author the vehicle's forward direction along negative Z. The runtime places those mounts at `restLength + tireRadius` above terrain, raycasts suspension compression from the mount height, and renders the tire center at the sampled terrain contact when the wheel can reach it. Suspension mounts are not rigid terrain colliders; their forces transfer into the upper cage through beams, while upper-cage nodes retain collision protection.

## Stepping

The solver uses a fixed timestep with an accumulator. Real elapsed time is clamped, and the number of catch-up substeps is capped to prevent a spiral of death after a stall. The solver guards against non-finite values; reusable output buffers remain a follow-up optimization.

The fixed-step pipeline is:

1. Read controls, update drivetrain/engine/safety state, sample the selected flat, bumpy, or offroad terrain profile, and accumulate grounded suspension/tire forces. Wheel speeds, ABS/TCS, and drivetrain speed use linear m/s; drivetrain wheel state remains angular rad/s.
2. Apply gravity and external forces, then integrate velocities semi-implicitly.
3. Solve compliant beam and triangle constraints for the configured iteration budget so suspension loads travel through the authored cage.
4. Resolve upper-body terrain collisions, apply aerodynamic drag, and emit positions, velocities, and fixed-width telemetry.

## XPBD and stability

Beam compliance and accumulated constraint lambdas provide XPBD-style stability, while bounded triangle area constraints resist uncontrolled angular/shear collapse. Parameters are clamped, invalid states are recovered, and deterministic reset behavior is covered by Rust tests. Solver iteration tuning is still a performance follow-up; graphics presets do not change authoritative controls or collision rules.

## Worker contract

The worker initializes WASM, accepts typed vehicle/runtime configuration and per-tick controls, and returns transferable position, velocity, and telemetry buffers. It reports lifecycle and timing aggregates through gated diagnostics. It reports initialization, runtime, and WASM errors with the root error preserved.

Do not send full node arrays through `console.debug`. Do not allocate a fresh buffer for every frame when a reusable or transferable buffer is possible.

## Testing

Test node and beam construction, rest lengths, force integration, breaking thresholds, collisions, XPBD stability, finite-value recovery, deterministic resets, fixed-timestep behavior, and benchmark budgets. Add property-based tests for extreme deformation and malformed vehicle definitions.
