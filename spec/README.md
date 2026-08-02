# Kemudi engine contract

Version: `0.1`

This is the public contract shared by Kemudi engine implementations. The Rust engine in `crates/kemudi-engine` is the current reference implementation. Browser, native, and Roblox integrations may expose different host APIs, but the simulation data described here uses the same conventions.

## Coordinate system

- Right-handed 3D coordinates.
- `Y` is up.
- Bundled vehicles point forward along negative `Z`.
- Terrain road points use `[X, Z]` coordinates.
- Angles are radians unless a field says otherwise.

The first four vehicle nodes are suspension mounts in this order: front-left, front-right, rear-left, rear-right. Wheelbase and track width are calculated from their rest positions.

## Units

| Quantity | Unit |
| --- | --- |
| Length, position, radius, width, depth | metres (`m`) |
| Time | seconds (`s`) |
| Mass | kilograms (`kg`) |
| Linear velocity | metres per second (`m/s`) |
| Angular velocity | radians per second (`rad/s`) |
| Force | newtons (`N`) |
| Torque | newton-metres (`N·m`) |
| Spring and beam stiffness | newtons per metre (`N/m`) |
| Temperature | degrees Celsius |
| Engine speed | revolutions per minute (`rpm`) |

Friction, grip, moisture, roughness, and compactness are dimensionless values. The browser telemetry includes speed in `m/s` and `km/h`; drivetrain wheel speed is angular, while tire contact speed is linear.

## World lifecycle

A host creates a `PhysicsWorld`, adds the vehicle and collision data, applies runtime configuration, submits controls, advances the world, and reads positions, velocities, and telemetry. The host owns the worker or runtime instance and releases it when the simulation session ends.

The current Rust defaults are:

- gravity: `-9.81 m/s²`;
- fixed step: `1/60 s`;
- maximum catch-up steps per public update: `8`.

Elapsed time is accumulated and simulated in fixed substeps. The catch-up limit prevents a stalled host from creating an unbounded backlog.

## Fixed-step order

Each fixed substep currently performs these operations in order:

1. Update vehicle systems and accumulate suspension, tire, drivetrain, safety, and terrain forces.
2. Apply gravity and external forces.
3. Integrate velocities and predicted positions.
4. Solve beam and triangle constraints.
5. Resolve terrain and static geometry collisions.
6. Apply aerodynamic drag.
7. Update telemetry.

Driven-wheel reaction is applied through the grounded tire force. Torque that exceeds available contact grip becomes wheel spin rather than a second chassis force.

## Inputs

Controls are normalized as follows:

```text
steering:  -1..1
throttle:   0..1
brake:      0..1
clutch:     0..1
handbrake:  boolean
gearUp:     edge-triggered boolean
gearDown:   edge-triggered boolean
engineOn:   boolean
```

The Rust-facing names are `gear_up` and `gear_down`. Inputs are validated before they reach the simulation.

## Vehicle data

A node contains an ID, position, velocity, mass, inverse mass, fixed state, collision flag, and accumulated force. A beam connects two node IDs and contains stiffness, damping, tensile strength, yield strength, plasticity, rest length, break state, and solver state.

Beam behavior has elastic, yielding, and broken states. A triangle constraint preserves authored area using inverse-mass weighting. Scalar beams do not model bending or buckling.

## Terrain and collision

Terrain samples contain height, normal, friction, roughness, moisture, compactness, rut depth, and a surface preset. Positive rut depth lowers the sampled height. Static collision data currently supports axis-aligned boxes, spheres, and horizontal boundaries.

The Rust implementation bounds restitution, friction, dimensions, and normalized surface values. Invalid numeric values are replaced by documented safe defaults before entering the simulation.

## Vehicle systems

The current engine includes torque curves, manual and automatic transmissions, open/locked/limited-slip differentials, suspension, wheel inertia, tire compounds, tire pressure and temperature, wear, ABS, traction control, VSC/ESC, fuel, engine thermal state, damage, TCM diagnostics, and ADAS telemetry.

Tire forces combine longitudinal and lateral demand and are limited by available contact grip.

## Telemetry

Telemetry is a fixed-width array of 86 `f64` values. Stable indices are declared in `crates/kemudi-engine/src/types.rs`. They cover speed, engine and transmission state, controls, thermal state, fuel, damage, suspension, terrain, tires, safety systems, TCM diagnostics, and ADAS state.

Existing indices retain their meanings. New values require an appended index or a new contract version.

## Compatibility

Compatibility fixtures use the same validated input, initial state, fixed timestep, and substep limit for each implementation. Floating-point results may differ within the tolerance recorded by a fixture; signs, bounds, event order, and state transitions are part of the observable result.

See:

- [Physics behavior](../docs/physics.md)
- [Fixture format](../fixtures/engine/README.md)
- [Roblox port status](../ports/kemudi-blox/README.md)
