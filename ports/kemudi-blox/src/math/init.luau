--[[
	Math utilities for the Kemudi physics engine.
	Pure Luau — no Roblox API dependencies.
--]]

local Math = {}

function Math.finite_or_zero(v: number): number
	if v ~= v then return 0 end -- NaN
	if v == math.huge or v == -math.huge then return 0 end
	return v
end

function Math.sane_or(v: number, fallback: number): number
	if v ~= v then return fallback end
	if v == math.huge or v == -math.huge then return fallback end
	return v
end

function Math.clamp(v: number, lo: number, hi: number): number
	return math.min(math.max(v, lo), hi)
end

function Math.sign(v: number): number
	if v > 0 then return 1 end
	if v < 0 then return -1 end
	return 0
end

function Math.dot3(a: {number}, b: {number}): number
	return a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
end

function Math.cross3(a: {number}, b: {number}): {number}
	return {
		a[2]*b[3] - a[3]*b[2],
		a[3]*b[1] - a[1]*b[3],
		a[1]*b[2] - a[2]*b[1],
	}
end

function Math.length3(v: {number}): number
	return math.sqrt(v[1]*v[1] + v[2]*v[2] + v[3]*v[3])
end

function Math.normalize3(v: {number}): {number}?
	local len = Math.length3(v)
	if len > 1e-8 then
		return { v[1]/len, v[2]/len, v[3]/len }
	end
	return nil
end

function Math.distance3(a: {number}, b: {number}): number
	local dx, dy, dz = b[1]-a[1], b[2]-a[2], b[3]-a[3]
	return math.sqrt(dx*dx + dy*dy + dz*dz)
end

function Math.sub3(a: {number}, b: {number}): {number}
	return { a[1]-b[1], a[2]-b[2], a[3]-b[3] }
end

function Math.add3(a: {number}, b: {number}): {number}
	return { a[1]+b[1], a[2]+b[2], a[3]+b[3] }
end

function Math.scale3(v: {number}, s: number): {number}
	return { v[1]*s, v[2]*s, v[3]*s }
end

function Math.triangleArea(
	ax: number, ay: number, az: number,
	bx: number, by: number, bz: number,
	cx: number, cy: number, cz: number
): number
	local abx, aby, abz = bx-ax, by-ay, bz-az
	local acx, acy, acz = cx-ax, cy-ay, cz-az
	local cr = Math.cross3({abx,aby,abz},{acx,acy,acz})
	return 0.5 * Math.length3(cr)
end

function Math.classifyDamageZone(
	x: number, y: number, z: number,
	centerX: number, centerZ: number
): number
	local dx = math.abs(x - centerX)
	local dz = z - centerZ
	if y > 1.2 then return 4
	elseif y < 0.15 then return 5
	elseif dz < -0.35 then return 1
	elseif dz > 0.35 then return 2
	elseif dx > 0.55 then return 3
	else return 0 end
end

function Math.terrainHeightForProfile(profile: number, x: number, z: number): number
	if profile == 2 then
		local broad = math.sin(x * 0.008) * 0.6 + math.cos(z * 0.006) * 0.5
		local ridges = math.sin((x - z) * 0.025) * 0.35 + math.cos((x + z) * 0.018) * 0.25
		local rocks = math.sin(x * 0.08) * math.cos(z * 0.06) * 0.18
			+ math.sin((x * 2.1 + z * 0.7) * 0.05) * 0.12
		local rough = math.sin(x * 0.35 + z * 0.28) * 0.06
			+ math.cos(x * 0.42 - z * 0.31) * 0.04
		return (broad + ridges + rocks + rough) * 14.0 * 0.12
	end
	return 0.0
end

return Math
