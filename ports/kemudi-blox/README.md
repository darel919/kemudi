# Kemudi Engine — Roblox / Luau Port

Independent Luau implementation of the [Kemudi engine contract](../../spec/README.md) for Roblox.

## Status

**Core physics and vehicle systems implemented; Roblox runtime adapter is scaffolded.**

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
| Production vehicle modules under Lune | Done |
| Roblox server runtime | Scaffolded |
| Client presentation and replication | Scaffolded |

The current implementation is a deterministic soft-body foundation inspired by the node/beam model used by Rigs of Rods and BeamNG-style vehicle rigs. It is not yet a claim of feature parity with either engine; tire contact, rigid-body orientation, suspension raycast integration, and network reconciliation still need deeper validation in Studio.

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
```

Build the Roblox place with Rojo:

```bash
rojo build Default.project.json -o kemudi-blox.rbxl
```

## Roblox project

`Default.project.json` defines the Rojo-compatible project used to build the Roblox place.

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
