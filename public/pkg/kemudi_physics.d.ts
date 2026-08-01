/* tslint:disable */
/* eslint-disable */

export class PhysicsWorld {
    free(): void;
    [Symbol.dispose](): void;
    apply_force(index: number, fx: number, fy: number, fz: number): void;
    get_positions(): Float64Array;
    load_vehicle(x: number, y: number, z: number): number;
    constructor();
    step(dt: number): void;
}

export class VehicleState {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    vx: number;
    vy: number;
    vz: number;
    x: number;
    y: number;
    z: number;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_get_vehiclestate_vx: (a: number) => number;
    readonly __wbg_get_vehiclestate_vy: (a: number) => number;
    readonly __wbg_get_vehiclestate_vz: (a: number) => number;
    readonly __wbg_get_vehiclestate_x: (a: number) => number;
    readonly __wbg_get_vehiclestate_y: (a: number) => number;
    readonly __wbg_get_vehiclestate_z: (a: number) => number;
    readonly __wbg_physicsworld_free: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_vx: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_vy: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_vz: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_x: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_y: (a: number, b: number) => void;
    readonly __wbg_set_vehiclestate_z: (a: number, b: number) => void;
    readonly __wbg_vehiclestate_free: (a: number, b: number) => void;
    readonly physicsworld_apply_force: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly physicsworld_get_positions: (a: number) => [number, number];
    readonly physicsworld_load_vehicle: (a: number, b: number, c: number, d: number) => number;
    readonly physicsworld_new: () => number;
    readonly physicsworld_step: (a: number, b: number) => void;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
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
