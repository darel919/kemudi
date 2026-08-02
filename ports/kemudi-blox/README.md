# Kemudi Engine — Roblox / Luau Port

Independent Luau implementation of the [Kemudi engine contract](../../spec/README.md) for Roblox.

## Status

**Core physics and vehicle systems implemented; Roblox runtime adapter is wired as a prototype and still requires Studio boundary validation.**

| Component | Status |
|-----------|--------|
| Contract types and telemetry indices | Done |
| Math utilities | Done |
| PhysicsWorld: nodes, beams, triangles | Done |
| XPBD constraint solver (30 iterations) | Done |
| Gravity, force integration, drag | Done |
| Terrain collision | Done |
| Static geometry collision (boxes, spheres, boundaries) | Done |
| Beam material state (yield, break, plasticity) | Done |
| Telemetry buffer | Done |
| Fixture runner | Done |
| Engine fixtures | 16 fixtures / 53 assertions passing |
| Suspension (raycast wheel, Ackermann, fuel) | Done |
| Drivetrain (engine, transmission, differential) | Done |
| Tires (Pacejka, thermal, wear, damage) | Done |
| Safety systems (ABS, TCS, VSC/ESC, ADAS) | Done |
| TCM (Transmission Control Module) | Vehicle-authored calibration; Studio validation pending |
| Drive modes (Normal/Eco/Sport/Track/Snow) | Done |
| Vehicle→World pipeline wiring | Done |
| Fuel mass coupling and live weight change | Done |
| Per-wheel road-surface friction lookup | Done |
| Rotating chassis-frame body attachments | Done |
| Production vehicle modules under Lune | Done |
| Roblox server runtime | Bootstrap and authoritative heartbeat wired |
| Client presentation and replication | Snapshot/telemetry path wired; tabbed OBD dashboard; Studio validation pending |
| Chassis-frame orientation presentation | Shared server/client frame helper; Studio validation pending |

The current implementation is a deterministic soft-body foundation inspired by the node/beam model used by Rigs of Rods and BeamNG-style vehicle rigs. It is not yet a claim of feature parity with either engine; full rigid-body angular integration and network reconciliation still need deeper validation in Studio. Map collision import is implemented for anchored, collidable GroundZero parts, with the final Play-mode contact behavior still requiring Studio validation.

Automatic transmission behavior is vehicle data, not a universal code preset. The server validates the authored `VehicleSpec/Transmission` content, forwards its transmission and optional `VehicleSpec/Transmission/TCM` calibration into the physics world, and leaves the TCM disabled when an asset does not provide calibration. See [`docs/vehicles.md`](docs/vehicles.md) for the folder layout, attributes, units, validation rules, and current Studio boundary.

When the local player occupies the authored driver seat, the client HUD shows a persistent speed/gear summary and a tabbed OBD-II dashboard. The dashboard pages are **DRIVE**, **ENGINE**, **TRANS**, **CHASSIS**, **TIRES**, and **SAFETY**; together they expose the complete 86-value telemetry contract, including per-wheel temperatures, wear, suspension compression/load, TCM sensor values, fault codes, and ADAS state. Pages scroll independently so detailed diagnostics do not replace the primary driving readout.

## Compatibility

The `single-node-gravity` fixture passes with exact matches:

- `nodes[0].positionM[1]` = `9.9972750000` (tolerance `1e-6`)
- `nodes[0].velocityMps[1]` = `-0.1635000000` (tolerance `1e-6`)

## Running tests

Requires [Lune](https://lune.land/) and [Rokit](https://github.com/rojo-rbx/rokit):

```bash
cd ports/kemudi-blox
lune run test/fixture_runner.luau       # 16 engine fixtures / 53 assertions
lune run test/gravity_fixture_test.luau # compatibility smoke test
lune run test/vehicle_unit_test.luau    # formula tests
lune run test/module_compat_test.luau   # production vehicle modules
lune run test/physics_integration_test.luau # physics integration regression
lune run test/chassis_test.luau         # orientation frame regression
lune run test/tcm_test.luau             # vehicle-authored TCM behavior
lune run test/ground_zero_grid_test.luau # deterministic streamed map grid
lune run test/client_hud_visibility_test.luau # HUD only while occupying the driver seat
lune run test/static_collision_regression_test.luau # oriented wall collision
```

Build the Roblox place with Rojo:

```bash
rojo build Default.project.json -o kemudi-blox.rbxl
```

For live synchronization into Roblox Studio, install the Studio plugin once from the project directory:

```bash
cd ports/kemudi-blox
rojo plugin install
rojo serve Default.project.json
```

Then open the Rojo plugin in Studio and connect to `localhost:34872`. This syncs the repository bootstraps and engine modules into `ReplicatedStorage`, `ServerScriptService`, and `StarterPlayerScripts`; keep the Studio-authored vehicle under `ServerStorage/KemudiVehicles` and save the `.rbxlx` after synchronization.

## Roblox project

`Default.project.json` defines the Rojo-compatible project used to build the Roblox place.

See [`docs/vehicles.md`](docs/vehicles.md) for the vehicle asset import, physics-rig, mass, and runtime integration contract.

During Play mode, approach the spawned vehicle and use the **Enter** proximity prompt. Automatic transmissions start in **P**. While seated, **LeftShift** advances the selector one detent (`P → R → N → D → M/S`) and **LeftControl** reverses it (`M/S → D → N → R → P`). Entering or leaving Park requires the vehicle to be stopped with the brake held. Moving `N → D` or `N → R` without the brake remains operable but creates a transmission damage-risk event; `N → R` is more severe because it engages the opposite direction. In **M/S**, **E** and **Q** request manual up/down shifts through the TCM; an unsafe request is rejected, produces a double beep, and shows a timed orange warning such as `UNSAFE DOWNSHIFT // OVER-REV PROTECTION`. The selector and transmission remain server-authoritative.

Transmission abuse is modeled separately from engine damage. Excessive clutch shock, thermal load, input torque, or mechanically forced over-revving accumulates irreversible wear. Damage derates drive torque; complete failure drops the transmission to neutral, enters TCM fault state with diagnostic code `722`, and rejects further shift commands. The TRANS OBD tab exposes the temperature, damage, warning, torque reduction, and fault diagnostics.

### GroundZero streamed tuning range

GroundZero uses a server-owned procedural chunk streamer rather than one
unbounded Baseplate. The server keeps a bounded window of deterministic
512-stud asphalt chunks around each active player or vehicle, creates at most
two chunks per Heartbeat, and unloads chunks outside the configured radius.
Roblox `StreamingEnabled` remains enabled for replication, but it does not
generate world geometry by itself.

The authored `Workspace/Maps/GroundZero/CarSpawn` part is the vehicle spawn
marker. The first implementation provides an infinite-looking flat tuning
surface; ramps, skidpads, suspension bumps, and other test facilities should
be added as anchored, collidable BaseParts under `GroundZero`. At vehicle
spawn, those authored parts are imported into the custom solver as oriented
static boxes. `Baseplate`, `CarSpawn`, and generated `ActiveChunks` floors are
excluded because the solver owns the analytic ground and spawn surface.
Non-anchored map parts are ignored and produce a server warning; anchor a wall
before testing it. Wheel/suspension nodes are excluded from analytic terrain
collision but remain eligible for these authored static boxes, so a four-node
vehicle can collide with a wall. Static impacts now update damage, impact
severity, deformation depth, and damage zone telemetry; the contact impulse is
also included in beam tensile stress so sufficiently hard impacts can break
beams. Imported static map walls use zero restitution, so they absorb normal
velocity rather than behaving like elastic barriers.

The grid helpers are covered by `test/ground_zero_grid_test.luau`. The real
chunk lifecycle still requires Roblox Studio Play-mode validation because it
depends on `Workspace`, `Players`, `RunService`, and Instance replication.

### Module structure

```
src/
  init.luau           Package entry point
  contract/           Engine contract types and constants
  math/               Vector3 and numeric utilities
  physics/            World, XPBD solver, collision
  vehicle/            Suspension, tires, drivetrain, safety
  runtime/            Server heartbeat, client input, snapshots

test/
  ...                 Lune fixture and production-module tests
```

Core modules are pure Luau with no Roblox API dependencies. The runtime adapter layer (`runtime/`) is intentionally Roblox-specific and handles RunService, Part↔Node mapping, and RemoteEvents.

## Engine contract

Version `0.1` — see [spec/README.md](../../spec/README.md) and [docs/physics.md](../../docs/physics.md).
