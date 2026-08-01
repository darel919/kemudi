# Physics

Kemudi.js uses a custom Rust-to-WebAssembly soft-body solver. Vehicles are represented by nodes connected by beams and rendered separately from the simulation.

Current status: the checked-in solver implements node/beam force integration, a bounded fixed-step accumulator, finite-value guards, beam breaking, and a simple ground constraint. XPBD, terrain friction, compact reusable snapshots, and gameplay-system integration remain planned work; the sections below distinguish the target contract from the current prototype.

## Data model

- **Node:** position, velocity, force, mass/inverse mass, and fixed state.
- **Beam:** endpoint node IDs, rest length, stiffness, damping, break strength, and broken state.
- **Vehicle:** nodes, beams, and triangles/skin data. Versioned metadata and drivetrain state are validated/represented in the frontend and standalone Rust modules but are not yet part of the worker's authoritative vehicle simulation.

## Stepping

The solver uses a fixed timestep with an accumulator. Real elapsed time is clamped, and the number of catch-up substeps is capped to prevent a spiral of death after a stall. The solver guards against non-finite values; reusable output buffers remain a follow-up optimization.

The intended pipeline is:

1. Apply gravity and external forces.
2. Integrate velocities semi-implicitly.
3. Solve beam and XPBD constraints for the configured iteration budget.
4. Resolve ground and terrain collisions with friction and restitution.
5. Integrate positions.
6. Emit a compact snapshot for rendering.

## XPBD and stability

XPBD is the planned constraint solver for stable deformation; the current implementation uses bounded beam constraint correction. Parameters must be clamped, invalid states must be recoverable, and deterministic reset behavior must be tested. Solver iterations can vary by performance preset, but gameplay correctness and authoritative collision behavior remain stable.

## Worker contract

The worker initializes WASM, accepts typed commands such as load, step, and force application, and returns transferable position/state buffers. It reports lifecycle and timing aggregates through gated diagnostics. It must report initialization, runtime, and WASM errors with the root error preserved.

Do not send full node arrays through `console.debug`. Do not allocate a fresh buffer for every frame when a reusable or transferable buffer is possible.

## Testing

Test node and beam construction, rest lengths, force integration, breaking thresholds, collisions, XPBD stability, finite-value recovery, deterministic resets, fixed-timestep behavior, and benchmark budgets. Add property-based tests for extreme deformation and malformed vehicle definitions.
