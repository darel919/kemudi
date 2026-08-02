# Vehicles and controls

Vehicles combine a versioned data definition, a soft-body physics representation, a renderable deformable mesh, and gameplay systems such as controls and drivetrain simulation.

## Vehicle definitions

Vehicle files should contain a schema version, metadata, nodes, beams, visual/skin references, wheel/suspension data, and drivetrain parameters. Beam stiffness is authored in N/m and tensile strength in N; chassis members should be calibrated against vehicle mass, suspension spring rates, and expected tire loads rather than treated as unitless tuning values. Loaders must validate structure, numeric ranges, referenced IDs, asset paths, and compatibility before creating runtime objects.

Invalid definitions must produce a user-visible recoverable error. They must not leave partial physics or GPU resources behind.

## Deformable rendering

The renderer precomputes vertex-to-node weights and caches them. Each frame maps the body through the suspension-mount chassis frame and adds only residual weighted deformation, using reusable arrays. A rigid chassis rotation therefore rotates the shell and wheel footprint together instead of approximating rotation as independent node translations. Every bundled vehicle has a visible deformable body shell skinned to its authoritative node/beam cage; body metadata controls its material and damage profile. Distant vehicles use a lower-cost LOD path without replacing the shell with a non-rotating centroid proxy. Wheels are grouped with the same vehicle assembly, follow suspension contact, steer with the front axle, and spin from authoritative wheel-speed telemetry. Geometries and materials are shared where safe and disposed when the vehicle is removed.

Bundled body assets live in `app/kemudi.js/public/models/` and use the following GLTF contract: one mesh, Y-up coordinates, no external dependencies, and an origin aligned to the center of the vehicle's physics cage. The sample assets can be regenerated with `bun run --cwd app/kemudi.js scripts/generate-sample-vehicle-models.mjs`. Wheels remain part of the runtime physics/rendering layer, so body GLBs should contain only the deformable body shell.

## Controls and ignition

The input layer normalizes keyboard and gamepad state into a typed command. `E` advances ignition from off to accessory and then to running; a newly entered level always starts off. Steering, throttle, braking, clutch, handbrake, and gear-edge commands are passed to the Rust worker, not converted into direct JavaScript position changes. Input handling is independent of render FPS and releases stuck inputs on blur, visibility changes, or disconnect.

Analog gamepad triggers reject non-finite values, clamp to 0–1, and apply a small dead zone. Vehicle wheelbase and track width come from the first four authored suspension mounts; the solver and renderer convert the center steering command into Ackermann-correct front-left/front-right angles. Rear wheels inherit vehicle orientation for rendering but receive no steering rotation.

`Q` requests a downshift and `R` requests an upshift when the selected gearbox is manual. `T` toggles manual/automatic on vehicles that expose both modes. Manual mode uses sequential gear changes and the clutch input; automatic mode owns gear selection through the TCM, including torque-converter coupling, lockup, shift delay, kickdown, and the vehicle's configured shift schedule. Vehicles with only one transmission mode keep that mode.

Automatic shift scheduling is load-aware: light throttle permits earlier upshifts while high requested load holds the current gear for acceleration. The TCM also consumes the selected drive-mode strategy through the physics worker. Eco, Comfort, and Snow lower upshift points; Sport and Track hold gears longer and adjust kickdown sensitivity. These modifiers change scheduling only—the physical transmission, converter, tire grip, and engine limits remain authoritative.

When a vehicle has forward-collision warning or automatic emergency braking installed, ADAS considers static map collision shapes and boundaries as well as remote vehicles. Closing relative speed is signed as target speed minus ego speed, so a stationary wall is a closing target while the vehicle moves toward it. The system remains bounded by perception range, time-to-collision, tire grip, brake capacity, and sensor faults.

## Drivetrain

The drivetrain models RPM, torque, manual/automatic gearbox engagement, differential distribution, wheel/suspension behavior, engine thermal/damage derating, fuel, and tire state inside the authoritative worker. The driving HUD reads its speedometer and dismissable OBD panel from the returned telemetry buffer. The speedometer and OBD panel are hidden for the interior camera. The premium sportscar exposes both manual sequential and real automatic transmission setup in the drive menu.

Automatic transmission faults are causal and replayable. Sensor failures corrupt the TCM's observed signals with bounded stale-value fallbacks; solenoid and hydraulic faults change line pressure, shift latency, converter coupling, and torque delivery; communication and power faults enter a fail-safe state. The resulting erratic shifts, delayed engagement, high RPM, slip, overheating, torque derate, or stuck 2nd/3rd gear are produced by the drivetrain update. Clearing a diagnostic flag does not undo physical wear or heat accumulated while the fault was active.

## Presets and correctness

Graphics presets may reduce mesh detail, effects, update frequency, or interpolation work. They must not change controls, collision semantics, vehicle ownership, or authoritative drivetrain rules. Test the same vehicle behavior at every preset.

## Future vehicle and model mods

Vehicle/model content is intentionally versioned so future user-created parts, complete vehicles, replacement meshes, and skins can be added without changing the physics API. Content registration should use stable interfaces rather than renderer-specific imports.
