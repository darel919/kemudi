/* tslint:disable */
/* eslint-disable */

export class PhysicsWorld {
    free(): void;
    [Symbol.dispose](): void;
    add_beam(id: number, node_a: number, node_b: number, stiffness: number, damping: number, strength: number): void;
    /**
     * Add a horizontal boundary at the supplied point's Y coordinate.
     * Extra normal components are retained in the ABI for future oriented
     * boundaries and older JS callers can safely pass zeroes.
     */
    add_boundary(x: number, y: number, z: number, _nx: number, _ny: number, _nz: number, restitution: number): void;
    /**
     * Add an axis-aligned static box. The extent parameters are half sizes.
     */
    add_collision_box(x: number, y: number, z: number, half_x: number, half_y: number, half_z: number, restitution: number, friction: number, _surface_id: number): void;
    add_collision_sphere(x: number, y: number, z: number, radius: number, restitution: number, friction: number): void;
    add_node(id: number, x: number, y: number, z: number, mass: number, fixed: boolean): void;
    add_road_surface(points: Float64Array, width: number, surface_id: number, friction: number, roughness: number, moisture: number, compactness: number): void;
    add_triangle(a: number, b: number, c: number): void;
    add_vehicle(_x: number, _y: number, _z: number): number;
    apply_force(node_id: number, fx: number, fy: number, fz: number): void;
    clear_tcm_faults(): void;
    configure_runtime(idle_rpm: number, redline_rpm: number, limiter_rpm: number, throttle_response: number, engine_braking: number, torque_rpms: Float64Array, torque_values: Float64Array, gear_ratios: Float64Array, final_drive: number, reverse_ratio: number, transmission_mode: number, shift_delay: number, differential_mode: number, differential_bias: number, wheel_spring_rates: Float64Array, wheel_dampings: Float64Array, wheel_rebound_dampings: Float64Array, wheel_rest_lengths: Float64Array, wheel_travels: Float64Array, wheel_radii: Float64Array, anti_roll_bar_stiffness: number, bump_stop_rate: number, tire_compounds: Uint8Array, tire_pressures: Float64Array, fuel_capacity: number, fuel_consumption: number, idle_consumption: number, abs_enabled: boolean, traction_control_enabled: boolean, vsc_enabled: boolean, adas_forward_collision_warning: boolean, adas_automatic_emergency_braking: boolean): void;
    /**
     * Configure the driven wheel-set inertia from authored unsprung masses.
     * The current drivetrain contract is rear-driven, so the rear pair is
     * used for the shaft inertia.
     */
    configure_wheel_inertia(unsprung_masses: Float64Array): void;
    get_beam_count(): number;
    get_node_count(): number;
    get_positions_flat(): Float64Array;
    get_telemetry_flat(): Float64Array;
    get_time(): number;
    get_velocities_flat(): Float64Array;
    constructor();
    set_adas_target(distance: number, relative_speed: number): void;
    /**
     * Apply the vehicle's configured automatic shift points to the TCM.
     * Missing/invalid values retain the safe built-in schedule.
     */
    set_automatic_shift_schedule(upshift_rpm: number, downshift_rpm: number): void;
    /**
     * Override the optional material parameters for an authored beam after
     * its endpoints and strength have been installed. Missing/invalid
     * values are ignored by the worker, while values reaching WASM are
     * clamped to safe material ranges.
     */
    set_beam_material(beam_id: number, yield_strength: number, plasticity: number): void;
    set_controls(steering: number, throttle: number, brake: number, clutch: number, handbrake: boolean, gear_up: boolean, gear_down: boolean, engine_on: boolean): void;
    /**
     * Apply the driver's drive-mode strategy to the automatic TCM.
     * Mode values are stable across the worker boundary: 0 normal, 1 eco,
     * 2 comfort, 3 sport, 4 track, and 5 snow.
     */
    set_drive_mode(mode: number): void;
    set_drivetrain_layout(layout: number): void;
    /**
     * Set base ground friction from map terrain layer data.
     * Called by the worker after loading the vehicle with map config.
     */
    set_ground_friction(friction: number): void;
    /**
     * Apply the authored collision flag after node IDs have been mapped to
     * their runtime indices. The first four nodes are always suspension
     * mounts, so they remain raycast contacts rather than rigid terrain
     * colliders.
     */
    set_node_collision(node_id: number, collision: boolean): void;
    /**
     * Set surface properties from map terrain config.
     */
    set_surface_properties(roughness: number, moisture: number, compactness: number, preset_index: number): void;
    /**
     * Inject or clear a deterministic TCM fault. Fault IDs are stable across
     * the worker boundary; see `TCMFaultKind` for the mapping.
     */
    set_tcm_fault(fault_id: number, active: boolean): void;
    set_tcm_fault_intermittent(fault_id: number, intermittent: boolean): void;
    set_tcm_fault_seed(seed: bigint): void;
    /**
     * Install the exact terrain sample grid used by the renderer. This keeps
     * wheel contact and visible terrain on the same height field, including
     * seeded procedural maps and decoded image heightmaps.
     */
    set_terrain_heightmap(samples: Float64Array, width: number, depth: number, segments: number): void;
    /**
     * 0 = flat asphalt, 1 = bumpy asphalt, 2 = offroad dirt.
     */
    set_terrain_profile(profile: number): void;
    /**
     * Change the driver-selectable gearbox mode without rebuilding the
     * vehicle. Mode 0 is manual sequential; mode 1 is torque-converter
     * automatic. A mode change cancels an in-progress shift so the new mode
     * starts from one authoritative gear state.
     */
    set_transmission_mode(mode: number): void;
    step(dt: number): void;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_physicsworld_free: (a: number, b: number) => void;
    readonly physicsworld_add_beam: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_boundary: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
    readonly physicsworld_add_collision_box: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => void;
    readonly physicsworld_add_collision_sphere: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_node: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_road_surface: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => void;
    readonly physicsworld_add_triangle: (a: number, b: number, c: number, d: number) => void;
    readonly physicsworld_add_vehicle: (a: number, b: number, c: number, d: number) => number;
    readonly physicsworld_apply_force: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly physicsworld_clear_tcm_faults: (a: number) => void;
    readonly physicsworld_configure_runtime: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number, o: number, p: number, q: number, r: number, s: number, t: number, u: number, v: number, w: number, x: number, y: number, z: number, a1: number, b1: number, c1: number, d1: number, e1: number, f1: number, g1: number, h1: number, i1: number, j1: number, k1: number, l1: number, m1: number, n1: number, o1: number, p1: number, q1: number, r1: number) => void;
    readonly physicsworld_configure_wheel_inertia: (a: number, b: number, c: number) => void;
    readonly physicsworld_get_beam_count: (a: number) => number;
    readonly physicsworld_get_node_count: (a: number) => number;
    readonly physicsworld_get_positions_flat: (a: number) => [number, number];
    readonly physicsworld_get_telemetry_flat: (a: number) => [number, number];
    readonly physicsworld_get_velocities_flat: (a: number) => [number, number];
    readonly physicsworld_new: () => number;
    readonly physicsworld_set_adas_target: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_automatic_shift_schedule: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_beam_material: (a: number, b: number, c: number, d: number) => void;
    readonly physicsworld_set_controls: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => void;
    readonly physicsworld_set_drive_mode: (a: number, b: number) => void;
    readonly physicsworld_set_drivetrain_layout: (a: number, b: number) => void;
    readonly physicsworld_set_ground_friction: (a: number, b: number) => void;
    readonly physicsworld_set_node_collision: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_surface_properties: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly physicsworld_set_tcm_fault: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_tcm_fault_intermittent: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_tcm_fault_seed: (a: number, b: bigint) => void;
    readonly physicsworld_set_terrain_heightmap: (a: number, b: number, c: number, d: number, e: number, f: number) => void;
    readonly physicsworld_set_terrain_profile: (a: number, b: number) => void;
    readonly physicsworld_set_transmission_mode: (a: number, b: number) => void;
    readonly physicsworld_step: (a: number, b: number) => void;
    readonly physicsworld_get_time: (a: number) => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
