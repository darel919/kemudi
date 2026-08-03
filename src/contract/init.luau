--[[
	Contract types and constants matching spec/README.md version 0.1.

	All simulation data uses these conventions:
	  - Right-handed 3D, Y up, forward = -Z.
	  - SI units: metres, seconds, kilograms, newtons, rpm, Celsius.
	  - Angles in radians unless noted.
--]]
local Contract = {}

Contract.VERSION = "0.1"
Contract.SCHEMA_VERSION = 1

-- Defaults from the Rust reference implementation
Contract.DEFAULT_GRAVITY = -9.81 -- m/s^2
Contract.DEFAULT_FIXED_DT = 1 / 240 -- seconds
Contract.DEFAULT_MAX_SUBSTEPS = 16

-- Terrain grid
Contract.TERRAIN_GRID_SIZE = 64
Contract.TERRAIN_EXTENT = 400.0

-- Telemetry array length and stable indices (must match types.rs T_* constants)
Contract.TELEMETRY_LEN = 86

Contract.T = {
	SPEED_MPS            = 0,
	SPEED_KMH            = 1,
	RPM                  = 2,
	GEAR                 = 3,
	THROTTLE             = 4,
	BRAKE                = 5,
	STEERING             = 6,
	CLUTCH               = 7,
	COOLANT              = 8,
	OIL_TEMP             = 9,
	OIL_PRESSURE         = 10,
	TRANS_TEMP           = 11,
	FUEL                 = 12,
	DERATE               = 13,
	DAMAGE               = 14,
	SUSPENSION_LOAD      = 15,
	GRIP                 = 16,
	TERRAIN_HEIGHT       = 17,
	WHEEL_SPEED          = 18,
	DRIVE_TORQUE         = 19,
	ENGINE_RUNNING       = 20,
	SHIFT_PHASE          = 21,
	ABS_ACTIVE           = 22,
	TC_ACTIVE            = 23,
	VSC_ACTIVE           = 24,
	BROKEN_BEAMS         = 25,
	ENGINE_STAGE         = 26,
	TIRE_WEAR            = 27,
	TIRE_TEMP            = 28,
	CONTACTS             = 29,
	SIM_TIME             = 30,
	FUEL_PERCENT         = 31,
	TCM_STATE            = 32,
	TCM_LINE_PRESSURE    = 33,
	TCM_LOCKUP           = 34,
	REQUESTED_GEAR       = 35,
	SLIP_RATIO           = 36,
	YAW_ERROR            = 37,
	LUGGING_STRESS       = 38,
	OVERREV_STRESS       = 39,
	RUT_DEPTH            = 40,
	INPUT_SHAFT_RPM      = 41,
	OUTPUT_SHAFT_RPM     = 42,
	CONVERTER_COUPLING   = 43,
	WHEEL_SPEED_FL       = 44,
	WHEEL_SPEED_FR       = 45,
	WHEEL_SPEED_RL       = 46,
	WHEEL_SPEED_RR       = 47,
	SUSP_COMPRESSION_FL  = 48,
	SUSP_COMPRESSION_FR  = 49,
	SUSP_COMPRESSION_RL  = 50,
	SUSP_COMPRESSION_RR  = 51,
	SUSP_LOAD_FL         = 52,
	SUSP_LOAD_FR         = 53,
	SUSP_LOAD_RL         = 54,
	SUSP_LOAD_RR         = 55,
	TIRE_TEMP_FL         = 56,
	TIRE_TEMP_FR         = 57,
	TIRE_TEMP_RL         = 58,
	TIRE_TEMP_RR         = 59,
	TIRE_WEAR_FL         = 60,
	TIRE_WEAR_FR         = 61,
	TIRE_WEAR_RL         = 62,
	TIRE_WEAR_RR         = 63,
	IMPACT_SEVERITY      = 64,
	DEFORMATION_DEPTH    = 65,
	BROKEN_PARTS         = 66,
	DAMAGE_ZONE          = 67,
	ENGINE_WARNING       = 68,
	ADAS_FCW             = 69,
	ADAS_AEB             = 70,
	ADAS_CONFIDENCE      = 71,
	TCM_FAULT_MASK       = 72,
	TCM_DIAGNOSTIC_CODE  = 73,
	TCM_SENSOR_INPUT_RPM = 74,
	TCM_SENSOR_SPEED_KMH = 75,
	TCM_SENSOR_AGE       = 76,
	TCM_SHIFT_LATENCY    = 77,
	TCM_TORQUE_REDUCTION = 78,
	TCM_FAIL_SAFE_GEAR   = 79,
	ADAS_TARGET_DETECTED     = 80,
	ADAS_TARGET_DISTANCE     = 81,
	ADAS_TIME_TO_COLLISION   = 82,
	ADAS_RELATIVE_SPEED      = 83,
	ADAS_BRAKE_COMMAND       = 84,
	ADAS_SENSOR_FAULTS       = 85,
}

-- Controls input ranges
Contract.CONTROLS = {
	STEERING_MIN  = -1,
	STEERING_MAX  =  1,
	THROTTLE_MIN  =  0,
	THROTTLE_MAX  =  1,
	BRAKE_MIN     =  0,
	BRAKE_MAX     =  1,
	CLUTCH_MIN    =  0,
	CLUTCH_MAX    =  1,
}

return Contract
