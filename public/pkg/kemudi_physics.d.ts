/* tslint:disable */
/* eslint-disable */

export class PhysicsWorld {
    free(): void;
    [Symbol.dispose](): void;
    add_beam(id: number, node_a: number, node_b: number, stiffness: number, damping: number, strength: number): void;
    add_node(id: number, x: number, y: number, z: number, mass: number, fixed: boolean): void;
    add_triangle(a: number, b: number, c: number): void;
    add_vehicle(_x: number, _y: number, _z: number): number;
    apply_force(node_id: number, fx: number, fy: number, fz: number): void;
    configure_runtime(idle_rpm: number, redline_rpm: number, limiter_rpm: number, throttle_response: number, engine_braking: number, torque_rpms: Float64Array, torque_values: Float64Array, gear_ratios: Float64Array, final_drive: number, reverse_ratio: number, transmission_mode: number, shift_delay: number, differential_mode: number, differential_bias: number, wheel_spring_rates: Float64Array, wheel_dampings: Float64Array, wheel_rebound_dampings: Float64Array, wheel_rest_lengths: Float64Array, wheel_travels: Float64Array, wheel_radii: Float64Array, tire_compounds: Uint8Array, tire_pressures: Float64Array, fuel_capacity: number, fuel_consumption: number, idle_consumption: number, abs_enabled: boolean, traction_control_enabled: boolean, vsc_enabled: boolean, adas_forward_collision_warning: boolean, adas_automatic_emergency_braking: boolean): void;
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
    set_controls(steering: number, throttle: number, brake: number, clutch: number, handbrake: boolean, gear_up: boolean, gear_down: boolean, engine_on: boolean): void;
    /**
     * Apply the authored collision flag after node IDs have been mapped to
     * their runtime indices. The first four nodes are always suspension
     * mounts, so they remain raycast contacts rather than rigid terrain
     * colliders.
     */
    set_node_collision(node_id: number, collision: boolean): void;
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
    readonly physicsworld_add_node: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_triangle: (a: number, b: number, c: number, d: number) => void;
    readonly physicsworld_add_vehicle: (a: number, b: number, c: number, d: number) => number;
    readonly physicsworld_apply_force: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly physicsworld_configure_runtime: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number, o: number, p: number, q: number, r: number, s: number, t: number, u: number, v: number, w: number, x: number, y: number, z: number, a1: number, b1: number, c1: number, d1: number, e1: number, f1: number, g1: number, h1: number, i1: number, j1: number, k1: number, l1: number, m1: number, n1: number, o1: number, p1: number) => void;
    readonly physicsworld_get_beam_count: (a: number) => number;
    readonly physicsworld_get_node_count: (a: number) => number;
    readonly physicsworld_get_positions_flat: (a: number) => [number, number];
    readonly physicsworld_get_telemetry_flat: (a: number) => [number, number];
    readonly physicsworld_get_velocities_flat: (a: number) => [number, number];
    readonly physicsworld_new: () => number;
    readonly physicsworld_set_adas_target: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_automatic_shift_schedule: (a: number, b: number, c: number) => void;
    readonly physicsworld_set_controls: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => void;
    readonly physicsworld_set_node_collision: (a: number, b: number, c: number) => void;
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
