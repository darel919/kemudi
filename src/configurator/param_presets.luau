--[[
	Parameter presets — torque curve packs, gear ratio packs, tire compounds.
	Used by the configurator when users select preset values instead of
	editing raw numbers.

	Pure Luau, no Roblox API dependencies.
--]]

local Presets = {}

-- ============================================================
-- Torque curve presets
-- ============================================================

--- Scale a torque curve by a factor (sport = 1.15x, aggressive = 1.30x).
--- Keeps the same RPM points, scales torque values.
local function scaleTorqueCurve(curve, factor)
	if not curve then return nil end
	local scaled = {}
	for i, pt in curve do
		scaled[i] = { pt[1], pt[2] * factor }
	end
	return scaled
end

--- Shift a torque curve's peak to a higher RPM by stretching the low end.
--- Creates a "peakier" power band.
local function sharpenTorqueCurve(curve, sharpenFactor)
	if not curve then return nil end
	-- sharpenFactor > 1 makes the curve peakier around the middle
	local count = #curve
	if count < 2 then return curve end
	local mid = math.floor(count / 2)
	local scaled = {}
	for i, pt in curve do
		local distFromPeak = math.abs(i - mid) / mid
		local factor = 1 - (1 - sharpenFactor) * distFromPeak * distFromPeak
		scaled[i] = { pt[1], pt[2] * math.max(factor, 0.5) }
	end
	return scaled
end

--- Get a torque curve preset by name.
--- @param name string  — "stock", "sport", "aggressive"
--- @param baseCurve table|nil  — the original torque curve from the asset
--- @return table|nil  — scaled torque curve, or nil if not applicable
function Presets.getTorqueCurve(name, baseCurve)
	if not baseCurve then return nil end
	if name == "stock" then return baseCurve end
	if name == "sport" then
		return scaleTorqueCurve(baseCurve, 1.12)
	end
	if name == "aggressive" then
		local scaled = scaleTorqueCurve(baseCurve, 1.25)
		return sharpenTorqueCurve(scaled, 1.15)
	end
	return baseCurve
end

-- ============================================================
-- Gear ratio presets
-- ============================================================

--- Shift all gear ratios by a factor. Short = higher ratios (acceleration),
--- Tall = lower ratios (top speed).
local function shiftRatios(ratios, factor)
	if not ratios then return nil end
	local shifted = {}
	for i, r in ratios do
		shifted[i] = r * factor
	end
	return shifted
end

--- Get a gear ratio preset.
--- @param name string  — "short", "stock", "tall"
--- @param baseRatios table|nil  — original gear ratios
--- @param baseFinalDrive number|nil  — original final drive
--- @return table|nil, number|nil  — gear_ratios, final_drive
function Presets.getGearPreset(name, baseRatios, baseFinalDrive)
	if not baseRatios then return nil, baseFinalDrive end
	if name == "stock" then return baseRatios, baseFinalDrive end
	if name == "short" then
		-- Shorter ratios for better acceleration, compensate with taller final drive
		return shiftRatios(baseRatios, 1.15), assert(baseFinalDrive, "vehicle asset is missing final drive") * 0.90
	end
	if name == "tall" then
		-- Taller ratios for higher top speed
		return shiftRatios(baseRatios, 0.88), assert(baseFinalDrive, "vehicle asset is missing final drive") * 1.12
	end
	return baseRatios, baseFinalDrive
end

-- ============================================================
-- Tire compound presets
-- ============================================================

--- Grip and wear multipliers per compound.
--- Applied as modifiers to the base Pacejka parameters.
Presets.TIRE_COMPOUNDS = {
	economy = {
		grip_factor = 0.85,
		wear_rate = 0.7,
		rolling_resistance = 1.2,
	},
	all_season = {
		grip_factor = 1.0,
		wear_rate = 1.0,
		rolling_resistance = 1.0,
	},
	performance = {
		grip_factor = 1.15,
		wear_rate = 1.4,
		rolling_resistance = 0.9,
	},
	offroad = {
		grip_factor = 0.90,
		wear_rate = 0.8,
		rolling_resistance = 1.3,
	},
}

--- Get tire compound modifiers.
--- @param name string  — compound name
--- @return table|nil  — { grip_factor, wear_rate, rolling_resistance }
function Presets.getTireCompound(name)
	return Presets.TIRE_COMPOUNDS[name]
end

-- ============================================================
-- Default values (for "reset to stock")
-- ============================================================

--- Store original values at spawn time for "reset to stock" feature.
--- @param world table  — the physics World instance
--- @param manifest table  — capability manifest
--- @return table  — snapshot of defaults
function Presets.captureDefaults(world, manifest)
	local ManifestMod = if script then require(script.Parent.manifest) else require("./manifest")
	return ManifestMod.snapshot(world, manifest)
end

return Presets
