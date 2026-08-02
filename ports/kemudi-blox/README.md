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
| TCM (Transmission Control Module) | Simplified |
| Drive modes (Normal/Eco/Sport/Track/Snow) | Done |
| Vehicle→World pipeline wiring | Done |
| Fuel mass coupling and live weight change | Done |
| Per-wheel road-surface friction lookup | Done |
| Rotating chassis-frame body attachments | Done |
| Production vehicle modules under Lune | Done |
| Roblox server runtime | Bootstrap and authoritative heartbeat wired |
| Client presentation and replication | Snapshot/telemetry path wired; Studio validation pending |
| Chassis-frame orientation presentation | Shared server/client frame helper; Studio validation pending |

The current implementation is a deterministic soft-body foundation inspired by the node/beam model used by Rigs of Rods and BeamNG-style vehicle rigs. It is not yet a claim of feature parity with either engine; full rigid-body angular integration, map collision import, and network reconciliation still need deeper validation in Studio.

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
lune run test/chassis_test.luau         # orientation frame regression
```

Build the Roblox place with Rojo:

```bash
rojo build Default.project.json -o kemudi-blox.rbxl
```

## Roblox project

`Default.project.json` defines the Rojo-compatible project used to build the Roblox place.

See [`docs/vehicles.md`](docs/vehicles.md) for the vehicle asset import, physics-rig, mass, and runtime integration contract.

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
