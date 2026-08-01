# Vehicles and controls

Vehicles combine a versioned data definition, a soft-body physics representation, a renderable deformable mesh, and gameplay systems such as controls and drivetrain simulation.

## Vehicle definitions

Vehicle files should contain a schema version, metadata, nodes, beams, visual/skin references, wheel/suspension data, and drivetrain parameters. Loaders must validate structure, numeric ranges, referenced IDs, asset paths, and compatibility before creating runtime objects.

Invalid definitions must produce a user-visible recoverable error. They must not leave partial physics or GPU resources behind.

## Deformable rendering

The renderer precomputes vertex-to-node weights and caches them. Each frame updates only the required position attributes using reusable arrays. Distant vehicles use a lower-cost LOD path. Geometries and materials are shared where safe and disposed when the vehicle is removed.

## Controls

The input layer normalizes keyboard and gamepad state into a typed command. Steering, throttle, braking, handbrake, and future force-feedback signals are passed to gameplay/physics through a stable interface. Input handling must be independent of render FPS and must release stuck inputs on blur, visibility changes, or disconnect.

## Drivetrain

The drivetrain models RPM, torque, gearbox engagement, differential distribution, and wheel/suspension behavior. It belongs to gameplay/physics, not the renderer, and should be testable without a browser.

## Presets and correctness

Graphics presets may reduce mesh detail, effects, update frequency, or interpolation work. They must not change controls, collision semantics, vehicle ownership, or authoritative drivetrain rules. Test the same vehicle behavior at every preset.

## Future vehicle and model mods

Vehicle/model content is intentionally versioned so future user-created parts, complete vehicles, replacement meshes, and skins can be added without changing the physics API. Content registration should use stable interfaces rather than renderer-specific imports.
