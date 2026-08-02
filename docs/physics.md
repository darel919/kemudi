# Physics

Kemudi.js uses a custom Rust-to-WebAssembly soft-body solver. Vehicles are represented by nodes connected by beams and rendered separately from the simulation.

Current status: the checked-in solver implements node/beam force integration, bounded fixed-step stepping, finite-value guards, XPBD-style compliance/lambda constraints, triangle area preservation, terrain profiles/contact friction, suspension, drivetrain torque, engine thermal/damage state, tire wear, safety hooks, and a fixed-width telemetry buffer.

## Data model

- **Node:** position, velocity, force, mass/inverse mass, and fixed state.
- **Beam:** endpoint node IDs, rest length, stiffness, damping, break strength, broken state, and XPBD lambda.
- **Vehicle:** nodes, beams, triangles/skin data, drivetrain configuration, suspension/wheel state, tires, fuel, safety systems, and engine thermal/damage state.

Bundled vehicle definitions use their first four nodes as suspension mounts and author the vehicle's forward direction along negative Z. The runtime places those mounts at `restLength + tireRadius` above terrain, raycasts suspension compression from the mount height, and renders the tire center at the sampled terrain contact when the wheel can reach it. Suspension mounts are not rigid terrain colliders; their forces transfer into the upper cage through beams. Layouts with an authored upper cage also apply mount-to-body attachment constraints, so a healthy body/roof layer follows the wheel-mount frame instead of folding into a linkage while upper-cage nodes retain collision protection with a small underbody clearance. A direct attachment beam can still break under tensile deformation, after which the normal damage/deformation path is allowed to separate that body point.

Suspension `antiRollBarStiffness` is interpreted as N/m and is applied to physical left/right wheel deflection, not normalized compression. The render assembly keeps each wheel at its authoritative suspension contact, steers the front wheel pivots from telemetry, and spins each tire from its measured rolling speed.

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

## TCM fault simulation

Automatic transmissions include a deterministic TCM fault boundary. Faults are injected through the worker's `set_tcm_fault` command and are applied to virtual sensors and physical actuator behavior rather than directly changing the selected gear. Fault IDs are:

| ID | Fault | Representative behavior |
| ---: | --- | --- |
| 0 | Input speed sensor | stale/zero RPM, ratio plausibility fault, limp mode |
| 1 | Output speed sensor | stale/zero vehicle speed, hunting or delayed shifts |
| 2 | Throttle signal | conservative fallback load and poor shift decisions |
| 3 | Transmission temperature sensor | incorrect thermal protection or hidden overheating |
| 4 | Range sensor | refused/unsafe gear selection and fail-safe state |
| 5 | Shift solenoid | requested shifts remain pending and take longer or fail |
| 6 | Pressure-control solenoid | oscillating line pressure and harsh/soft shifts |
| 7 | Hydraulic pressure loss | clutch slip, derated torque, slow or blocked shifts |
| 8 | Converter lockup | deterministic stuck-on or stuck-off converter behavior |
| 9 | Communication loss | latched diagnostic state, torque reduction, fail-safe gear |
| 10 | TCM power loss | latched diagnostic state, torque reduction, fail-safe gear |
| 11 | Adaptation memory | corrupted learned pressure and shift-map corrections |

The optional intermittent flag produces bounded deterministic dropouts. A seed can be supplied for replayable stuck-on/stuck-off and intermittent behavior. Fault state and representative diagnostic codes are exposed in the appended telemetry signals: fault mask, diagnostic code, virtual input/output sensor values, sensor age, shift latency, torque reduction, and fail-safe gear.

For example, a scenario can inject intermittent hydraulic pressure loss and later clear the diagnostic state:

```ts
physics.setTcmFault(7, true, true, 42)
// ...run the scenario...
physics.clearTcmFaults()
```

Do not send full node arrays through `console.debug`. Do not allocate a fresh buffer for every frame when a reusable or transferable buffer is possible.

## Testing

Test node and beam construction, rest lengths, force integration, breaking thresholds, collisions, XPBD stability, finite-value recovery, deterministic resets, fixed-timestep behavior, and benchmark budgets. Add property-based tests for extreme deformation and malformed vehicle definitions.
