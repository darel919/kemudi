# Kemudi — Soft-body vehicle sandbox

Soft-body vehicle physics engine for Roblox, implemented in Luau. Deterministic node/beam simulation with XPBD constraint solving, Pacejka tire model, drivetrain, suspension, safety systems, and real-time telemetry.

This repository is a port/experiment from [kemudi-rust](https://github.com/darel919/kemudi-rust) — exploring how far Roblox can be pushed for realistic vehicle physics.

> **Status:** Core physics and vehicle systems implemented. Roblox runtime adapter is wired as a prototype and still requires Studio boundary validation.

## Quick start

Requires [Rokit](https://github.com/rojo-rbx/rokit) and [Lune](https://lune-org.github.io/docs/):

```bash
# Install tools
rokit install

# Run tests
lune run test/fixture_runner.luau
lune run test/vehicle_unit_test.luau

# Build the Roblox place
rojo build Default.project.json -o kemudi-blox.rbxl
```

For live sync into Studio:

```bash
rojo plugin install
rojo serve Default.project.json
```

Then open the Rojo plugin in Studio and connect to `localhost:34872`.

## What it does

- Deterministic soft-body physics (nodes, beams, triangles) with XPBD solver
- Pacejka tire model with thermal, wear, and damage
- Drivetrain: engine, manual/automatic transmission, differential, TCM
- Suspension: raycast wheels, Ackermann steering, fuel coupling
- Safety: ABS, traction control, VSC/ESC, ADAS
- Real-time OBD-II telemetry (86-value contract)
- GroundZero streamed map with procedural chunk loading
- Vehicle configurator with parametric presets
- Static geometry collision (oriented boxes, walls)

## Project structure

```
src/
  init.luau           Package entry point
  contract/           Engine contract types and constants
  math/               Vector3 and numeric utilities
  physics/            World, XPBD solver, collision, aerodynamics
  vehicle/            Suspension, tires, drivetrain, safety, brakes, turbo
  runtime/            Server heartbeat, client input, snapshots, chassis
  configurator/       Vehicle configurator, manifest, UI, server sync

test/
  ...                 Lune fixture and production-module tests

tools/
  ...                 Rojo setup helpers for vehicle rigs

fixtures/
  ...                 Engine compatibility fixtures (shared contract)

spec/
  ...                 Language-neutral engine contract

place/
  server.server.luau  Roblox server bootstrap
  client.client.luau  Roblox client bootstrap
  kemudi-blox.rbxlx   Rojo place file
```

## Tests

All core modules are pure Luau with no Roblox API dependencies. Tests run under Lune:

```bash
lune run test/fixture_runner.luau              # 16 engine fixtures / 53 assertions
lune run test/gravity_fixture_test.luau         # compatibility smoke test
lune run test/vehicle_unit_test.luau            # formula tests
lune run test/module_compat_test.luau           # production vehicle modules
lune run test/physics_integration_test.luau     # physics integration regression
lune run test/chassis_test.luau                 # orientation frame regression
lune run test/tcm_test.luau                     # vehicle-authored TCM behavior
lune run test/ground_zero_grid_test.luau        # deterministic streamed map grid
lune run test/client_hud_visibility_test.luau   # HUD only while occupying driver seat
lune run test/static_collision_regression_test.luau  # oriented wall collision
lune run test/drivetrain_tcm_regression_test.luau    # TCM drivetrain regression
lune run test/brakes_test.luau                  # brake system
lune run test/bushings_test.luau                # bushing dynamics
lune run test/configurator_test.luau            # vehicle configurator
lune run test/contact_model_test.luau           # contact model
lune run test/engine_thermal_test.luau          # engine thermal
lune run test/suspension_kinematics_test.luau   # suspension kinematics
lune run test/tire_force_curve_test.luau        # tire force curves
lune run test/turbo_smoke_test.luau             # turbo system
lune run test/terrain_deform_test.luau          # terrain deformation
lune run test/angular_validation_test.luau      # angular integration
lune run test/stop_drift_regression_test.luau   # drift regression
lune run test/wheel_dynamics_smoke_test.luau    # wheel dynamics
lune run test/phase1_physics_test.luau          # phase 1 physics
lune run test/constraints_module_load_test.luau # constraints module
```

## Engine contract

Version `0.1` — see [spec/README.md](spec/README.md) for units, axes, stepping, inputs, telemetry, and compatibility rules.

Engine fixtures are in [fixtures/](fixtures/engine/). The current reference fixture set covers gravity, beam springs, collisions, constraints, drag, structural integrity, and multi-body scenarios.

## Roblox project

`Default.project.json` defines the Rojo-compatible project. The runtime adapter layer (`src/runtime/`) handles RunService, Part-to-Node mapping, and RemoteEvents.

During Play mode, approach the spawned vehicle and use the **Enter** proximity prompt. **LeftShift** advances the gear selector, **LeftControl** reverses it. In **M/S** mode, **E** and **Q** request manual shifts.

See [docs/vehicles.md](docs/vehicles.md) for the vehicle asset import, physics-rig, mass, and runtime integration contract.

## Map building

See [docs/maps.md](docs/maps.md) for the complete map-building workflow.

## License

License details will be added before public release.
