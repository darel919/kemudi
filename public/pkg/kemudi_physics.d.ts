/* tslint:disable */
/* eslint-disable */

export class PhysicsWorld {
    free(): void;
    [Symbol.dispose](): void;
    add_beam(id: number, node_a: number, node_b: number, stiffness: number, damping: number, strength: number): void;
    add_node(id: number, x: number, y: number, z: number, mass: number, fixed: boolean): void;
    add_vehicle(_x: number, _y: number, _z: number): number;
    apply_force(node_id: number, fx: number, fy: number, fz: number): void;
    get_beam_count(): number;
    get_node_count(): number;
    get_positions_flat(): Float64Array;
    get_time(): number;
    get_velocities_flat(): Float64Array;
    constructor();
    step(dt: number): void;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_physicsworld_free: (a: number, b: number) => void;
    readonly physicsworld_add_beam: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_node: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_add_vehicle: (a: number, b: number, c: number, d: number) => number;
    readonly physicsworld_apply_force: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly physicsworld_get_beam_count: (a: number) => number;
    readonly physicsworld_get_node_count: (a: number) => number;
    readonly physicsworld_get_positions_flat: (a: number) => [number, number];
    readonly physicsworld_get_velocities_flat: (a: number) => [number, number];
    readonly physicsworld_new: () => number;
    readonly physicsworld_step: (a: number, b: number) => void;
    readonly physicsworld_get_time: (a: number) => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
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
