# Vehicles and controls

Vehicles combine a versioned data definition, a soft-body physics representation, a renderable deformable mesh, and gameplay systems such as controls and drivetrain simulation.

## Vehicle definitions

Vehicle files should contain a schema version, metadata, nodes, beams, visual/skin references, wheel/suspension data, and drivetrain parameters. Loaders must validate structure, numeric ranges, referenced IDs, asset paths, and compatibility before creating runtime objects.

Invalid definitions must produce a user-visible recoverable error. They must not leave partial physics or GPU resources behind.

## Deformable rendering

The renderer precomputes vertex-to-node weights and caches them. Each frame updates only the required position attributes using reusable arrays. Every bundled vehicle has a visible deformable body shell skinned to its authoritative node/beam cage; body metadata controls its material and damage profile. Distant vehicles use a lower-cost LOD path. Geometries and materials are shared where safe and disposed when the vehicle is removed.

Bundled body assets live in `public/models/` and use the following GLTF contract: one mesh, Y-up coordinates, no external dependencies, and an origin aligned to the center of the vehicle's physics cage. The sample assets can be regenerated with `bun scripts/generate-sample-vehicle-models.mjs`. Wheels remain part of the runtime physics/rendering layer, so body GLBs should contain only the deformable body shell.

## Controls and ignition

The input layer normalizes keyboard and gamepad state into a typed command. `E` advances ignition from off to accessory and then to running; a newly entered level always starts off. Steering, throttle, braking, clutch, handbrake, and gear-edge commands are passed to the Rust worker, not converted into direct JavaScript position changes. Input handling is independent of render FPS and releases stuck inputs on blur, visibility changes, or disconnect.

`Q` requests a downshift and `R` requests an upshift when the selected gearbox is manual. `T` toggles manual/automatic on vehicles that expose both modes. Manual mode uses sequential gear changes and the clutch input; automatic mode owns gear selection through the TCM, including torque-converter coupling, lockup, shift delay, kickdown, and the vehicle's configured shift schedule. Vehicles with only one transmission mode keep that mode.

## Drivetrain

The drivetrain models RPM, torque, manual/automatic gearbox engagement, differential distribution, wheel/suspension behavior, engine thermal/damage derating, fuel, and tire state inside the authoritative worker. The driving HUD reads its speedometer and dismissable OBD panel from the returned telemetry buffer. The speedometer and OBD panel are hidden for the interior camera. The premium sportscar exposes both manual sequential and real automatic transmission setup in the drive menu.

## Presets and correctness

Graphics presets may reduce mesh detail, effects, update frequency, or interpolation work. They must not change controls, collision semantics, vehicle ownership, or authoritative drivetrain rules. Test the same vehicle behavior at every preset.

## Future vehicle and model mods

Vehicle/model content is intentionally versioned so future user-created parts, complete vehicles, replacement meshes, and skins can be added without changing the physics API. Content registration should use stable interfaces rather than renderer-specific imports.
