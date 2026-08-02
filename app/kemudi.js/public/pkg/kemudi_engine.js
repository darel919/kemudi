/* @ts-self-types="./kemudi_engine.d.ts" */

export class PhysicsWorld {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        PhysicsWorldFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_physicsworld_free(ptr, 0);
    }
    /**
     * @param {number} id
     * @param {number} node_a
     * @param {number} node_b
     * @param {number} stiffness
     * @param {number} damping
     * @param {number} strength
     */
    add_beam(id, node_a, node_b, stiffness, damping, strength) {
        wasm.physicsworld_add_beam(this.__wbg_ptr, id, node_a, node_b, stiffness, damping, strength);
    }
    /**
     * Add a horizontal boundary at the supplied point's Y coordinate.
     * Extra normal components are retained in the ABI for future oriented
     * boundaries and older JS callers can safely pass zeroes.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} _nx
     * @param {number} _ny
     * @param {number} _nz
     * @param {number} restitution
     */
    add_boundary(x, y, z, _nx, _ny, _nz, restitution) {
        wasm.physicsworld_add_boundary(this.__wbg_ptr, x, y, z, _nx, _ny, _nz, restitution);
    }
    /**
     * Add an axis-aligned static box. The extent parameters are half sizes.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} half_x
     * @param {number} half_y
     * @param {number} half_z
     * @param {number} restitution
     * @param {number} friction
     * @param {number} _surface_id
     */
    add_collision_box(x, y, z, half_x, half_y, half_z, restitution, friction, _surface_id) {
        wasm.physicsworld_add_collision_box(this.__wbg_ptr, x, y, z, half_x, half_y, half_z, restitution, friction, _surface_id);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} radius
     * @param {number} restitution
     * @param {number} friction
     */
    add_collision_sphere(x, y, z, radius, restitution, friction) {
        wasm.physicsworld_add_collision_sphere(this.__wbg_ptr, x, y, z, radius, restitution, friction);
    }
    /**
     * @param {number} id
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} mass
     * @param {boolean} fixed
     */
    add_node(id, x, y, z, mass, fixed) {
        wasm.physicsworld_add_node(this.__wbg_ptr, id, x, y, z, mass, fixed);
    }
    /**
     * @param {Float64Array} points
     * @param {number} width
     * @param {number} surface_id
     * @param {number} friction
     * @param {number} roughness
     * @param {number} moisture
     * @param {number} compactness
     */
    add_road_surface(points, width, surface_id, friction, roughness, moisture, compactness) {
        const ptr0 = passArrayF64ToWasm0(points, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        wasm.physicsworld_add_road_surface(this.__wbg_ptr, ptr0, len0, width, surface_id, friction, roughness, moisture, compactness);
    }
    /**
     * @param {number} a
     * @param {number} b
     * @param {number} c
     */
    add_triangle(a, b, c) {
        wasm.physicsworld_add_triangle(this.__wbg_ptr, a, b, c);
    }
    /**
     * @param {number} _x
     * @param {number} _y
     * @param {number} _z
     * @returns {number}
     */
    add_vehicle(_x, _y, _z) {
        const ret = wasm.physicsworld_add_vehicle(this.__wbg_ptr, _x, _y, _z);
        return ret >>> 0;
    }
    /**
     * @param {number} node_id
     * @param {number} fx
     * @param {number} fy
     * @param {number} fz
     */
    apply_force(node_id, fx, fy, fz) {
        wasm.physicsworld_apply_force(this.__wbg_ptr, node_id, fx, fy, fz);
    }
    clear_tcm_faults() {
        wasm.physicsworld_clear_tcm_faults(this.__wbg_ptr);
    }
    /**
     * @param {number} idle_rpm
     * @param {number} redline_rpm
     * @param {number} limiter_rpm
     * @param {number} throttle_response
     * @param {number} engine_braking
     * @param {Float64Array} torque_rpms
     * @param {Float64Array} torque_values
     * @param {Float64Array} gear_ratios
     * @param {number} final_drive
     * @param {number} reverse_ratio
     * @param {number} transmission_mode
     * @param {number} shift_delay
     * @param {number} differential_mode
     * @param {number} differential_bias
     * @param {Float64Array} wheel_spring_rates
     * @param {Float64Array} wheel_dampings
     * @param {Float64Array} wheel_rebound_dampings
     * @param {Float64Array} wheel_rest_lengths
     * @param {Float64Array} wheel_travels
     * @param {Float64Array} wheel_radii
     * @param {number} anti_roll_bar_stiffness
     * @param {number} bump_stop_rate
     * @param {Uint8Array} tire_compounds
     * @param {Float64Array} tire_pressures
     * @param {number} fuel_capacity
     * @param {number} fuel_consumption
     * @param {number} idle_consumption
     * @param {boolean} abs_enabled
     * @param {boolean} traction_control_enabled
     * @param {boolean} vsc_enabled
     * @param {boolean} adas_forward_collision_warning
     * @param {boolean} adas_automatic_emergency_braking
     */
    configure_runtime(idle_rpm, redline_rpm, limiter_rpm, throttle_response, engine_braking, torque_rpms, torque_values, gear_ratios, final_drive, reverse_ratio, transmission_mode, shift_delay, differential_mode, differential_bias, wheel_spring_rates, wheel_dampings, wheel_rebound_dampings, wheel_rest_lengths, wheel_travels, wheel_radii, anti_roll_bar_stiffness, bump_stop_rate, tire_compounds, tire_pressures, fuel_capacity, fuel_consumption, idle_consumption, abs_enabled, traction_control_enabled, vsc_enabled, adas_forward_collision_warning, adas_automatic_emergency_braking) {
        const ptr0 = passArrayF64ToWasm0(torque_rpms, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(torque_values, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(gear_ratios, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ptr3 = passArrayF64ToWasm0(wheel_spring_rates, wasm.__wbindgen_malloc);
        const len3 = WASM_VECTOR_LEN;
        const ptr4 = passArrayF64ToWasm0(wheel_dampings, wasm.__wbindgen_malloc);
        const len4 = WASM_VECTOR_LEN;
        const ptr5 = passArrayF64ToWasm0(wheel_rebound_dampings, wasm.__wbindgen_malloc);
        const len5 = WASM_VECTOR_LEN;
        const ptr6 = passArrayF64ToWasm0(wheel_rest_lengths, wasm.__wbindgen_malloc);
        const len6 = WASM_VECTOR_LEN;
        const ptr7 = passArrayF64ToWasm0(wheel_travels, wasm.__wbindgen_malloc);
        const len7 = WASM_VECTOR_LEN;
        const ptr8 = passArrayF64ToWasm0(wheel_radii, wasm.__wbindgen_malloc);
        const len8 = WASM_VECTOR_LEN;
        const ptr9 = passArray8ToWasm0(tire_compounds, wasm.__wbindgen_malloc);
        const len9 = WASM_VECTOR_LEN;
        const ptr10 = passArrayF64ToWasm0(tire_pressures, wasm.__wbindgen_malloc);
        const len10 = WASM_VECTOR_LEN;
        wasm.physicsworld_configure_runtime(this.__wbg_ptr, idle_rpm, redline_rpm, limiter_rpm, throttle_response, engine_braking, ptr0, len0, ptr1, len1, ptr2, len2, final_drive, reverse_ratio, transmission_mode, shift_delay, differential_mode, differential_bias, ptr3, len3, ptr4, len4, ptr5, len5, ptr6, len6, ptr7, len7, ptr8, len8, anti_roll_bar_stiffness, bump_stop_rate, ptr9, len9, ptr10, len10, fuel_capacity, fuel_consumption, idle_consumption, abs_enabled, traction_control_enabled, vsc_enabled, adas_forward_collision_warning, adas_automatic_emergency_braking);
    }
    /**
     * Configure the driven wheel-set inertia from authored unsprung masses.
     * The current drivetrain contract is rear-driven, so the rear pair is
     * used for the shaft inertia.
     * @param {Float64Array} unsprung_masses
     */
    configure_wheel_inertia(unsprung_masses) {
        const ptr0 = passArrayF64ToWasm0(unsprung_masses, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        wasm.physicsworld_configure_wheel_inertia(this.__wbg_ptr, ptr0, len0);
    }
    /**
     * @returns {number}
     */
    get_beam_count() {
        const ret = wasm.physicsworld_get_beam_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    get_node_count() {
        const ret = wasm.physicsworld_get_node_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {Float64Array}
     */
    get_positions_flat() {
        const ret = wasm.physicsworld_get_positions_flat(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {Float64Array}
     */
    get_telemetry_flat() {
        const ret = wasm.physicsworld_get_telemetry_flat(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {number}
     */
    get_time() {
        const ret = wasm.physicsworld_get_time(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Float64Array}
     */
    get_velocities_flat() {
        const ret = wasm.physicsworld_get_velocities_flat(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    constructor() {
        const ret = wasm.physicsworld_new();
        this.__wbg_ptr = ret;
        PhysicsWorldFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} distance
     * @param {number} relative_speed
     */
    set_adas_target(distance, relative_speed) {
        wasm.physicsworld_set_adas_target(this.__wbg_ptr, distance, relative_speed);
    }
    /**
     * Apply the vehicle's configured automatic shift points to the TCM.
     * Missing/invalid values retain the safe built-in schedule.
     * @param {number} upshift_rpm
     * @param {number} downshift_rpm
     */
    set_automatic_shift_schedule(upshift_rpm, downshift_rpm) {
        wasm.physicsworld_set_automatic_shift_schedule(this.__wbg_ptr, upshift_rpm, downshift_rpm);
    }
    /**
     * Override the optional material parameters for an authored beam after
     * its endpoints and strength have been installed. Missing/invalid
     * values are ignored by the worker, while values reaching WASM are
     * clamped to safe material ranges.
     * @param {number} beam_id
     * @param {number} yield_strength
     * @param {number} plasticity
     */
    set_beam_material(beam_id, yield_strength, plasticity) {
        wasm.physicsworld_set_beam_material(this.__wbg_ptr, beam_id, yield_strength, plasticity);
    }
    /**
     * @param {number} steering
     * @param {number} throttle
     * @param {number} brake
     * @param {number} clutch
     * @param {boolean} handbrake
     * @param {boolean} gear_up
     * @param {boolean} gear_down
     * @param {boolean} engine_on
     */
    set_controls(steering, throttle, brake, clutch, handbrake, gear_up, gear_down, engine_on) {
        wasm.physicsworld_set_controls(this.__wbg_ptr, steering, throttle, brake, clutch, handbrake, gear_up, gear_down, engine_on);
    }
    /**
     * Apply the driver's drive-mode strategy to the automatic TCM.
     * Mode values are stable across the worker boundary: 0 normal, 1 eco,
     * 2 comfort, 3 sport, 4 track, and 5 snow.
     * @param {number} mode
     */
    set_drive_mode(mode) {
        wasm.physicsworld_set_drive_mode(this.__wbg_ptr, mode);
    }
    /**
     * @param {number} layout
     */
    set_drivetrain_layout(layout) {
        wasm.physicsworld_set_drivetrain_layout(this.__wbg_ptr, layout);
    }
    /**
     * Set base ground friction from map terrain layer data.
     * Called by the worker after loading the vehicle with map config.
     * @param {number} friction
     */
    set_ground_friction(friction) {
        wasm.physicsworld_set_ground_friction(this.__wbg_ptr, friction);
    }
    /**
     * Apply the authored collision flag after node IDs have been mapped to
     * their runtime indices. The first four nodes are always suspension
     * mounts, so they remain raycast contacts rather than rigid terrain
     * colliders.
     * @param {number} node_id
     * @param {boolean} collision
     */
    set_node_collision(node_id, collision) {
        wasm.physicsworld_set_node_collision(this.__wbg_ptr, node_id, collision);
    }
    /**
     * Set surface properties from map terrain config.
     * @param {number} roughness
     * @param {number} moisture
     * @param {number} compactness
     * @param {number} preset_index
     */
    set_surface_properties(roughness, moisture, compactness, preset_index) {
        wasm.physicsworld_set_surface_properties(this.__wbg_ptr, roughness, moisture, compactness, preset_index);
    }
    /**
     * Inject or clear a deterministic TCM fault. Fault IDs are stable across
     * the worker boundary; see `TCMFaultKind` for the mapping.
     * @param {number} fault_id
     * @param {boolean} active
     */
    set_tcm_fault(fault_id, active) {
        wasm.physicsworld_set_tcm_fault(this.__wbg_ptr, fault_id, active);
    }
    /**
     * @param {number} fault_id
     * @param {boolean} intermittent
     */
    set_tcm_fault_intermittent(fault_id, intermittent) {
        wasm.physicsworld_set_tcm_fault_intermittent(this.__wbg_ptr, fault_id, intermittent);
    }
    /**
     * @param {bigint} seed
     */
    set_tcm_fault_seed(seed) {
        wasm.physicsworld_set_tcm_fault_seed(this.__wbg_ptr, seed);
    }
    /**
     * Install the exact terrain sample grid used by the renderer. This keeps
     * wheel contact and visible terrain on the same height field, including
     * seeded procedural maps and decoded image heightmaps.
     * @param {Float64Array} samples
     * @param {number} width
     * @param {number} depth
     * @param {number} segments
     */
    set_terrain_heightmap(samples, width, depth, segments) {
        const ptr0 = passArrayF64ToWasm0(samples, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        wasm.physicsworld_set_terrain_heightmap(this.__wbg_ptr, ptr0, len0, width, depth, segments);
    }
    /**
     * 0 = flat asphalt, 1 = bumpy asphalt, 2 = offroad dirt.
     * @param {number} profile
     */
    set_terrain_profile(profile) {
        wasm.physicsworld_set_terrain_profile(this.__wbg_ptr, profile);
    }
    /**
     * Change the driver-selectable gearbox mode without rebuilding the
     * vehicle. Mode 0 is manual sequential; mode 1 is torque-converter
     * automatic. A mode change cancels an in-progress shift so the new mode
     * starts from one authoritative gear state.
     * @param {number} mode
     */
    set_transmission_mode(mode) {
        wasm.physicsworld_set_transmission_mode(this.__wbg_ptr, mode);
    }
    /**
     * @param {number} dt
     */
    step(dt) {
        wasm.physicsworld_step(this.__wbg_ptr, dt);
    }
}
if (Symbol.dispose) PhysicsWorld.prototype[Symbol.dispose] = PhysicsWorld.prototype.free;
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_throw_344f42d3211c4765: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./kemudi_engine_bg.js": import0,
    };
}

const PhysicsWorldFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_physicsworld_free(ptr, 1));

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedFloat64ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('kemudi_engine_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
