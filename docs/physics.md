# Physics

Kemudi.js uses a custom Rust-to-WebAssembly soft-body solver. Vehicles are represented by nodes connected by beams and rendered separately from the simulation.

Current status: the checked-in solver implements node/beam force integration, bounded fixed-step stepping, finite-value guards, XPBD-style compliance/lambda constraints, triangle area preservation, terrain profiles/contact friction, suspension, drivetrain torque, engine thermal/damage state, tire wear, safety hooks, and a fixed-width telemetry buffer.

## Data model

- **Node:** position, velocity, force, mass/inverse mass, and fixed state.
- **Beam:** endpoint node IDs, rest length, axial stiffness in N/m, tensile failure load in N, damping, broken state, and XPBD lambda.
- **Vehicle:** nodes, beams, triangles/skin data, drivetrain configuration, suspension/wheel state, tires, fuel, safety systems, and engine thermal/damage state.

Bundled vehicle definitions use their first four nodes as suspension mounts and author the vehicle's forward direction along negative Z. The runtime derives wheelbase and track width from those authored mounts, places them at `restLength + tireRadius` above terrain, raycasts suspension compression from the mount height, and renders the tire center at the sampled terrain contact when the wheel can reach it. Suspension mounts are not rigid terrain colliders; their forces transfer into the upper cage through beams. Layouts with an authored upper cage express mount-to-body offsets in the rotating 3D chassis basis, so a healthy body/roof layer follows chassis yaw, pitch, and roll instead of being held to a world-vertical offset or folding into a linkage. Upper-cage nodes retain collision protection with a small underbody clearance. A direct attachment beam can still break under tensile deformation, after which the normal damage/deformation path is allowed to separate that body point.

Chassis-frame beams must be materially stiffer than the suspension springs they support. The premium vehicle uses MN/m-scale frame members so normal launch and cornering loads produce millimetre-scale elastic movement; wheel hardpoints may separate visibly only after the corresponding structure reaches its configured tensile failure load.

Suspension `antiRollBarStiffness` is interpreted as N/m and is applied to physical left/right wheel deflection, not normalized compression. Damper velocity is derived from the change in terrain-to-mount suspension length, so moving over a changing terrain height contributes to compression/rebound; the derived shaft rate is bounded so discrete height-field and rut samples cannot inject an unphysical launch impulse. The render assembly keeps each wheel at its authoritative suspension contact, steers the front wheel pivots from telemetry, and spins each tire from its measured rolling speed.

Steering input produces a bounded virtual center-wheel angle. The front wheel angles use the common-turn-center Ackermann equations, so the inner wheel turns farther and the requested center angle lies between the two physical wheel angles. All four rendered wheels inherit the chassis yaw, pitch, and roll; only front-left and front-right receive the additional Ackermann steering rotations, while both rear wheels remain at zero steer relative to the chassis. Authored body meshes whose horizontal scale materially disagrees with the node cage are aligned to the authoritative chassis footprint before skin weights are computed. Rendering first maps every body vertex through the rigid 3D transform of the four suspension mounts, then adds weighted node deformation relative to that frame. This preserves body-to-wheel alignment through yaw, pitch, and roll without suppressing local crash deformation.

## Stepping

The solver uses a fixed timestep with an accumulator. Real elapsed time is clamped, and the number of catch-up substeps is capped to prevent a spiral of death after a stall. The solver guards against non-finite values; reusable output buffers remain a follow-up optimization.

The fixed-step pipeline is:

1. Read controls, update drivetrain/engine/safety state, sample the selected flat, bumpy, or offroad terrain profile, and accumulate grounded suspension/tire forces. Wheel speeds and ABS/TCS use linear m/s; drivetrain wheel state remains angular rad/s. Chassis-node velocity is the only authoritative vehicle linear velocity.
2. Apply gravity and external forces, then integrate velocities semi-implicitly.
3. Solve compliant beam and triangle constraints for the configured iteration budget so suspension loads travel through the authored cage.
4. Resolve upper-body terrain collisions, apply aerodynamic drag, and emit positions, velocities, and fixed-width telemetry.

Driven-wheel torque crosses into the chassis exactly once through the grounded tire force. While the requested torque fits inside the available traction, static contact constrains the driven shaft to the same-step ground-relative rolling speed; this keeps automatic-transmission crawl from under-rotating the driven tires as the chassis accelerates. Once the contact patch saturates, only the transmitted reaction torque is returned to the shaft and the untransmitted torque remains as physical wheelspin. With zero throttle, static contact likewise synchronizes the driven wheel instead of retaining an independently integrated vehicle velocity. Engine and service braking are dissipative: their torque is bounded by the angular momentum available in the fixed step, and contact force is bounded so it can stop but not reverse the chassis. Service-brake torque is not included in `last_drive_torque`, because braking is applied separately at the grounded wheel contacts.

Differential output always conserves the input torque. Open mode keeps an equal left/right torque split. Locked mode reacts to both available left/right grip and output-speed mismatch, transferring torque toward the wheel that can carry it and toward the slower output. Limited-slip mode blends the open split toward the available-grip split using the configured bias.

Reportable settling impacts remain separate from material-yield events. A sufficiently severe collision distributes its impulse over connected beam load paths; beams above yield acquire a bounded persistent rest-length change and beams above strength break. Because node masses remain on the deformed authoritative cage, crash-induced mass-distribution and alignment effects come from the changed geometry rather than a parallel cosmetic damage offset. Tire pressure uses absolute temperature relative to the authored nominal pressure at 20 C, and gross-slip residual grip falls farther on low-friction surfaces such as ice than on dry asphalt.

## XPBD and stability

Beam compliance and accumulated constraint lambdas provide XPBD-style stability. Triangle area constraints use the derived area gradients and inverse-mass-weighted XPBD corrections rather than centroid scaling, so fixed and differently weighted vertices respond consistently. Parameters are clamped, invalid states are recovered, and deterministic reset behavior is covered by Rust tests. Solver iteration tuning is still a performance follow-up; graphics presets do not change authoritative controls or collision rules.

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
