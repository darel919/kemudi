# Vehicle Import for Roblox

This guide defines how Kemudi vehicles are authored, imported, and spawned in Roblox.

The vehicle is a **content asset**. The Luau runtime provides the generic loader and physics simulation; it must not contain the geometry, node layout, vehicle-specific mass, or default-car tuning.

## Status

The Roblox runtime now resolves an allow-listed model from `ServerStorage/KemudiVehicles`, validates it with `src/runtime/vehicle_asset_loader.luau`, derives mass from its proxies, and forwards its authored drivetrain configuration into the physics world. Studio Play-mode validation of the complete spawn, TCM, and presentation path is still pending.

## Design goals

- Keep vehicle geometry outside the Luau source tree.
- Keep vehicle-specific physics data outside the Luau source tree.
- Derive mass from imported physics content instead of hardcoding a default vehicle mass.
- Keep the physics engine generic so the same loader can spawn user-created vehicles.
- Keep the server authoritative for vehicle simulation.
- Separate visible render geometry from solver-controlled physics geometry.
- Validate imported content before it enters the physics world.

## Asset location

For the first implementation, keep the vehicle model in the Roblox place under `ServerStorage`:

```text
ServerStorage/
└── KemudiVehicles/
    └── ExampleVehicle/
```

The server clones the model when a player requests it. Keeping the source model in `ServerStorage` gives the server ownership of the authoritative vehicle and avoids making the first prototype depend on runtime asset downloads or asset permissions.

A published Roblox Model or Package can be used later for distribution. Runtime loading by asset ID is a later concern and must handle ownership, availability, validation, and failure behavior explicitly.

## Model structure

A vehicle asset should have separate render and physics content:

```text
ExampleVehicle (Model)
├── Render (Model)
│   ├── Body
│   ├── Hood
│   ├── Glass
│   ├── Wheel_FL
│   ├── Wheel_FR
│   ├── Wheel_RL
│   └── Wheel_RR
│
├── PhysicsRig (Folder)
│   ├── Node_FL
│   ├── Node_FR
│   ├── Node_RL
│   ├── Node_RR
│   ├── Chassis_FL
│   ├── Chassis_FR
│   ├── Chassis_RL
│   └── Chassis_RR
│
├── Wheels (Folder)
│   ├── FL
│   ├── FR
│   ├── RL
│   └── RR
│
├── VehicleSpec (Folder)
│   ├── DriverSeat (Seat)
│   ├── Beam_000
│   ├── Beam_001
│   ├── Engine
│   │   └── TorqueCurve/1..N
│   ├── Suspension/FL, FR, RL, RR
│   ├── Transmission
│   │   ├── GearRatios
│   │   └── TCM (required for automatic transmissions)
│   └── Damage
```

The exact Instance names may evolve, but the asset must expose a versioned, validated contract. The loader must not depend on the default car's hardcoded child order.

## Import workflow

### 1. Create the render model

Create the body and wheel meshes in Blender or another modeling tool. Use a simple blockout for the first vehicle. Establish a consistent unit scale and orientation before importing into Studio.

The imported render model belongs under `Render` and is presentation-only. It must not be used as the soft-body node graph.

### 2. Import the render model into Studio

Use Roblox Studio's model importer to bring the render geometry into the place. Organize the result under the vehicle Model and separate the body, glass, wheels, and other visual components.

Render geometry should normally be configured so that it does not participate in Roblox's built-in rigid-body simulation:

- `CanCollide = false`.
- `CanTouch = false` where touch events are not required.
- `CanQuery = false` where spatial queries are not required.
- `Massless = true`.

Kemudi's server-side solver remains authoritative. Do not combine it with an independent Roblox chassis controller for the same vehicle.

### Studio setup utility

The first imported asset can be prepared with the one-time Studio Command Bar utility at `tools/setup_vehicle_asset.luau`:

1. Save the place before running the utility so the operation can be undone safely.
2. Select exactly one imported vehicle Model under `ServerStorage/KemudiVehicles`.
3. Copy the utility contents from macOS Terminal or an editor. If using Terminal, run `pbcopy` in Terminal—not in Studio:

   ```bash
   cd /Users/darelisme/Documents/Development/kemudi.js
   pbcopy < ports/kemudi-blox/tools/setup_vehicle_asset.luau
   ```

   Then open Studio's Command Bar, replace its contents with the Lua text (it should begin with `--!strict`), and run it once.
4. Save the place again as the source `.rbxlx` file.

The utility moves imported render parts into `Render`, creates `PhysicsRig`, `Wheels`, and `VehicleSpec`, derives wheel/node positions from the selected model, and marks render parts as anchored, non-colliding, non-queryable, non-touching, and massless. It does not set `KemudiMassKg`, choose a vehicle mass, or start the runtime.

The downloaded FBX was authored in centimetres while the simulation contract uses metres. The utility applies the generic Roblox scale conversion of `1/28` stud per source centimetre (`1 stud ≈ 0.28 m`) and refuses to run when the model already appears normalized, preventing accidental double-scaling. Verify the resulting bounds visually before continuing.

The generated `PhysicsRig/MassProxies` folder is intentionally marked `RequiresAuthoring`. Mass proxies and beam/tuning content must be authored and validated before the runtime loader accepts the vehicle.

### Mass-proxy setup utility

After the first utility succeeds and the `.rbxlx` is saved, use `tools/setup_vehicle_mass_proxies.luau` to create the initial asset-side proxy volumes:

1. Select the prepared vehicle Model under `ServerStorage/KemudiVehicles`.
2. In macOS Terminal, copy the utility contents:

   ```bash
   cd /Users/darelisme/Documents/Development/kemudi.js
   pbcopy < ports/kemudi-blox/tools/setup_vehicle_mass_proxies.luau
   ```

3. Return to Studio, clear the Command Bar, paste the Lua contents, and run it. The Command Bar must contain Lua beginning with `--!strict`; do not paste the `pbcopy` command itself.
4. Inspect `PhysicsRig/MassProxies`, then save the `.rbxlx` again.

The utility creates one bounded proxy Part for each node (`FL`, `FR`, `RL`, and `RR`) using the imported body geometry and chassis-marker positions. Roblox derives the proxy mass from the Part's physical geometry and material. The utility refuses to overwrite an existing generated or authored proxy set and does not write `KemudiMassKg` or any vehicle mass number.

### Beam-content setup utility

After the mass proxies exist, use `tools/setup_vehicle_beams.luau` to create the initial declarative structural graph:

1. Select the prepared vehicle Model under `ServerStorage/KemudiVehicles`.
2. In macOS Terminal, copy the utility contents:

   ```bash
   cd /Users/darelisme/Documents/Development/kemudi.js
   pbcopy < ports/kemudi-blox/tools/setup_vehicle_beams.luau
   ```

3. Return to Studio, clear the Command Bar, paste the Lua contents, and run it. The Command Bar must contain Lua beginning with `--!strict`.
4. Inspect `VehicleSpec/Beams`, then save the `.rbxlx` again.

The utility creates an initial six-beam four-node frame and stores `NodeA`, `NodeB`, `Stiffness`, `Damping`, and `Strength` as asset attributes. These are starting content values for later vehicle-specific authoring; the server does not contain this topology or tuning.

### Engine, transmission, and suspension authoring utility

After the beam content exists, use `tools/setup_vehicle_dynamics.luau` to write
an initial pickup calibration into the selected asset:

```bash
cd /Users/darelisme/Documents/Development/kemudi.js
pbcopy < ports/kemudi-blox/tools/setup_vehicle_dynamics.luau
```

Paste the contents into Studio's Command Bar with the prepared vehicle selected
under `ServerStorage/KemudiVehicles`, then save the `.rbxlx` source. The utility
creates asset-owned `VehicleSpec/Engine`, `VehicleSpec/Transmission`,
`VehicleSpec/Transmission/TCM`, and four-wheel `VehicleSpec/Suspension` data.
It refuses to overwrite existing folders.

The initial pickup calibration models a naturally aspirated V8-like torque
curve and a ten-forward-speed automatic transmission with torque-converter
lockup, throttle-dependent shift schedules, kickdown, thermal limp behavior,
and distinct front/rear spring and damper values. These are starting values for
the selected asset; tune them per vehicle in Studio. Runtime code
does not provide a default engine, transmission, TCM, or suspension setup.

### Driver-seat setup

After the beam content exists, use `tools/setup_vehicle_driver_seat.luau` to author the driver's entry point:

1. Select the prepared vehicle Model under `ServerStorage/KemudiVehicles`.
2. In macOS Terminal, copy the utility contents:

   ```bash
   cd /Users/darelisme/Documents/Development/kemudi.js
   pbcopy < ports/kemudi-blox/tools/setup_vehicle_driver_seat.luau
   ```

3. Return to Studio, clear the Command Bar, paste the Lua contents, and run it. The Command Bar must contain Lua beginning with `--!strict`.
4. Inspect `VehicleSpec/DriverSeat`, then save the `.rbxlx` again.

The utility derives the seat position from the authored wheel and body geometry. It does not add vehicle mass or physics tuning. The runtime validates the `Seat`, creates a server-owned `ProximityPrompt`, and moves the authored seat with the simulated chassis. The player must trigger **Enter** before controls are accepted; leaving the seat stops vehicle input.

### 3. Author the physics rig

Create invisible solver content inside `PhysicsRig`. The physics rig should contain the node markers, mass proxies, wheel anchors, and any collision proxies required by the adapter.

A node can use a marker and a separate volume:

```text
PhysicsRig/
└── Node_FL/
    ├── Marker
    └── MassProxy
```

- `Marker` identifies the node position and semantic role.
- `MassProxy` provides physical volume for mass derivation.
- The render mesh does not contribute to node mass.

The solver-controlled parts may be invisible and non-colliding because Kemudi handles the authoritative simulation. The adapter is responsible for mapping solver positions back onto the render model.

### 4. Author wheels and suspension anchors

Each wheel should expose, at minimum:

- Wheel center.
- Suspension mount or ray origin.
- Steering axis for front wheels.
- Wheel radius.
- Wheel visual object.
- Driven-wheel designation.

These values belong to the imported vehicle asset or its associated vehicle definition, not to the generic runtime script.

### 5. Author the structural graph

Beam definitions should be part of the vehicle asset content. Each beam needs references to its endpoint nodes and its material parameters:

```text
Beam_000
  NodeA = "Node_FL"
  NodeB = "Chassis_FL"
  Stiffness = <content value>
  Damping = <content value>
  Strength = <content value>
```

The representation may be Roblox Attributes, structured child Instances, or a serialized and versioned definition object. The choice should favor validation and source-control usability. The important rule is that `server.luau` must not contain vehicle-specific beam lists.

### 6. Author vehicle tuning

Vehicle-specific tuning belongs in `VehicleSpec` or an associated content definition:

- Drivetrain layout.
- Optional `VehicleSpec.DriveMode` attribute (`Normal`, `Eco`, `Comfort`, `Sport`, `Track`, or `Snow`); it selects the authored TCM shift, kickdown, and converter-lockup strategy at spawn.
- Engine and transmission values.
- Suspension parameters.
- Tire compound selection.
- Brake parameters.
- Damage and beam material data.
- Aerodynamic parameters.
- Optional fuel and thermal parameters.
- Automatic-transmission TCM calibration, when the vehicle uses an automatic gearbox:
  - `VehicleSpec/Transmission` attributes: `Mode` (`Manual` or `Automatic`), `FinalDrive`, `ReverseRatio`, `ClutchEngagement`, and `ShiftDelay`.
  - `VehicleSpec/Transmission/GearRatios/1..N` numbered `NumberValue` children, in forward-gear order.
  - Optional `VehicleSpec/Transmission/TCM` folder with `Enabled` and authored `UpshiftRPM/1..N`, `DownshiftRPM/1..N`, `ThrottleShiftFactor`, `KickdownThreshold`, `LockupSpeedThreshold`, `LockupThrottleThreshold`, `ThermalLimit`, `LimpGear`, and `MinShiftInterval` attributes/children.

All TCM speeds are in km/h, engine schedules are in RPM, temperatures are in °C, shift timing is in seconds, and throttle thresholds are normalized `[0, 1]` values. Schedule entries are mapped to the corresponding forward gear. The loader requires finite non-negative calibration values and rejects malformed numbered schedules before spawning.

Scalar tuning fields may be authored as Instance attributes or as same-named `NumberValue` children. New authoring utilities use attributes; the loader accepts both forms so existing Studio-authored assets remain compatible.

The TCM folder is not a generic vehicle preset. Its schedules, converter lockup point, thermal limit, limp gear, and shift timing belong to the specific transmission. An automatic asset without a TCM folder is rejected rather than silently receiving generic shift behavior. Engine and suspension folders are likewise required for a spawnable vehicle.

The loader converts this content into the existing `PhysicsWorld` input structures. It does not provide a hidden default car configuration when required values are absent.

## Mass policy

Mass must not be hardcoded in the default vehicle spawn code.

The preferred initial policy is to derive mass from imported physics proxy geometry:

1. Each dynamic node has one or more mass proxy Parts.
2. Proxy volume and physical density determine the proxy mass.
3. The loader reads the proxy mass at asset-load time.
4. The loader passes the resulting node masses to `PhysicsWorld:addNode`.
5. The sum of node masses becomes the vehicle mass used by vehicle systems.

The visible render mesh should be massless so it does not double-count the vehicle.

If a later vehicle requires an exact measured mass rather than geometry-derived mass, that value may be authored as vehicle content in the asset definition. It must still remain outside the Luau implementation and must be validated against the node mass distribution.

The loader rejects missing or invalid mass proxies rather than silently falling back to a vehicle-specific number. Generic numerical safety checks may protect the solver from invalid input, but they must not conceal an invalid imported asset.

## Server runtime flow

The runtime flow is:

```text
Client requests an allow-listed vehicle identifier
        ↓
Server selects an allowed vehicle asset
        ↓
Server clones the Model from ServerStorage
        ↓
Vehicle asset loader validates the Model
        ↓
Loader discovers nodes, beams, wheels, and tuning
        ↓
Loader derives node masses from physics proxies
        ↓
PhysicsWorld receives the normalized vehicle definition
        ↓
RunService.Heartbeat advances the authoritative solver
        ↓
Render objects follow the solver state
        ↓
Validated snapshots are replicated to clients
```

The server must validate the vehicle selection. A client should send a vehicle identifier or request, not arbitrary beam stiffness, mass, engine, or collision values.

The runtime API is conceptually similar to:

```text
spawnVehicle(player, vehicleId)
```

The server does not construct a default vehicle with calls such as:

```text
makePart("BodyFL", ..., 200)
makePart("BodyFR", ..., 200)
```

## Loader responsibilities

A Roblox-specific loader is implemented under the runtime adapter layer:

```text
src/runtime/vehicle_asset_loader.luau
```

It:

1. Accept a vehicle `Model`.
2. Validate the asset schema version.
3. Validate required folders, markers, wheels, and definitions.
4. Reject duplicate node identifiers.
5. Reject missing beam endpoints.
6. Reject non-finite or out-of-range numeric values.
7. Enforce bounded node, beam, and child counts.
8. Discover node positions and mass proxies.
9. Derive node masses without vehicle-specific code constants.
10. Discover wheel and suspension anchors.
11. Read vehicle tuning and optional TCM calibration from content.
12. Validate the authored driver seat.
13. Produce normalized input for `PhysicsWorld` and `Runtime:spawn`.
14. Return actionable errors that identify the asset and invalid field.

The loader must not execute arbitrary code from an imported vehicle. Vehicle definitions should be declarative data.

## Rojo and source control

The existing `Default.project.json` maps the engine source tree into Roblox. Vehicle assets should not be generated by `server.luau` and should not be hidden inside the generic engine modules.

A Studio-authored place or asset is a legitimate content artifact. If the project later chooses Rojo as the complete source of the DataModel, the asset must be represented in a reproducible Rojo-compatible form. Until then, keep the Studio-authored vehicle model in the place's `ServerStorage/KemudiVehicles` hierarchy and keep the loader contract documented here.

The generated `.rbxl` output from `rojo build` should be treated as a build artifact unless the place is intentionally maintained as a Studio-authored source artifact.

## Validation checklist

A vehicle is ready for runtime integration when:

- The render model imports into Studio without missing meshes or invalid scale.
- The model has a unique vehicle identifier and schema version.
- All required nodes and wheel anchors exist.
- An authored `VehicleSpec/DriverSeat` exists and is massless, anchored, and non-colliding.
- Node identifiers are unique and stable.
- Every beam endpoint resolves to a node.
- Mass is derived from physics content or explicitly declared content, never from spawn-code constants.
- Render geometry is not contributing duplicate mass or collision.
- The vehicle can be cloned from `ServerStorage` on the server.
- The loader rejects malformed content before creating a physics world.
- The solver receives the expected node and beam counts.
- The vehicle spawns at a marker with a valid transform.
- Server Heartbeat advances the simulation.
- Render parts follow the solver state.
- Client input is validated through server-owned remotes.
- The player must occupy the authored driver seat before controls are accepted.
- The client cannot authoritatively change vehicle mass, beams, tuning, or node positions.
- Studio Play mode shows no runtime or loader errors.

## Current implementation and remaining validation

The runtime adapter now resolves an authored template, validates it with `vehicle_asset_loader.luau`, derives proxy mass, reads beam and wheel content, reads the vehicle's transmission/TCM configuration, and forwards the normalized definition through `Runtime.new` and `Runtime:spawn`.

Remaining validation is deliberately runtime-specific:

1. Run the generated place in Roblox Studio Play mode.
2. Spawn a model containing an authored `Transmission` and `TCM` folder.
3. Confirm malformed schedules reject the vehicle with an actionable server warning.
4. Confirm the automatic gearbox shifts according to that model's maps rather than a generic schedule.
5. Confirm converter lockup, torque reduction, thermal limp behavior, telemetry, and client presentation. Limp mode holds the authored fail-safe gear as a floor, but may upshift above it when output-shaft RPM would exceed the rev-limiter safety threshold.
6. Add a Studio smoke test covering the complete server/client boundary.
