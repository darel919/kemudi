--[[

	Client HUD — tachometer gauge, OBD-II telemetry panel, transmission warnings,
	formatters, and updateHud. Extracted from client.luau for maintainability.
	ModuleScript required by the client LocalScript.

]]

local function makePanel(parent, name, size, position, anchorPoint)
	local panel = Instance.new("Frame")
	panel.Name = name
	panel.Size = size
	panel.Position = position
	panel.AnchorPoint = anchorPoint or Vector2.zero
	panel.BackgroundColor3 = Color3.fromRGB(10, 16, 24)
	panel.BackgroundTransparency = 0.18
	panel.BorderSizePixel = 0
	panel.Parent = parent
	local stroke = Instance.new("UIStroke")
	stroke.Color = Color3.fromRGB(80, 170, 220)
	stroke.Transparency = 0.35
	stroke.Parent = panel
	return panel
end

local function makeLabel(parent, name, position, size, text, textSize)
	local label = Instance.new("TextLabel")
	label.Name = name
	label.Position = position
	label.Size = size
	label.BackgroundTransparency = 1
	label.TextColor3 = Color3.fromRGB(235, 245, 255)
	label.Font = Enum.Font.Code
	label.TextSize = textSize or 14
	label.TextXAlignment = Enum.TextXAlignment.Left
	label.TextYAlignment = Enum.TextYAlignment.Top
	label.Text = text
	label.Parent = parent
	return label
end

-- ============================================================
-- Tachometer gauge
-- ponytail: ScreenGui dots for the arc; upgrade to CanvasGroup
-- when cross-platform Roblox coverage improves.
-- ============================================================
local TACHO_W = 220
local TACHO_H = 220
local TACHO_CX = TACHO_W / 2
local TACHO_CY = 88
local TACHO_R = 76
local TACHO_ARC_R = TACHO_R + 2
local NEEDLE_POINTER_LEN = 70   -- hub -> tip (long side, reads the dial)
local NEEDLE_TAIL_LEN = 16      -- hub -> butt (short counterweight side)
local NEEDLE_TOTAL_LEN = NEEDLE_POINTER_LEN + NEEDLE_TAIL_LEN
-- Roblox rotates GuiObjects about their own center regardless of AnchorPoint
-- (an engine limitation, not something fixable via AnchorPoint alone). So the
-- needle stays centered on itself, and instead we recompute its Position each
-- update so its center orbits the hub at a fixed radius -- that's what makes
-- an off-center (long tip / short tail) needle pivot correctly at the hub.
local NEEDLE_CENTER_OFFSET = (NEEDLE_POINTER_LEN - NEEDLE_TAIL_LEN) / 2

local tacho_max_rpm = 8000
local tacho_redline_rpm = 6000
local tacho_spec_read = false

-- State set during init, used by updateHud and warning functions
local dashboard, tacho_arc_dots, tacho_needle, tacho_rpm_text
local gear_label, speed_label, status_label, input_label
local transmission_warning_label, obd_status_label
local obd_panel
local transmission_warning_token = 0
local local_player = nil
local SoundService = nil

local function playTransmissionRejectAlert()
	-- ponytail: requires "KemudiTransmissionRejectBeep" Sound in SoundService
	local sound = SoundService:FindFirstChild("KemudiTransmissionRejectBeep")
	if not sound then return end
	SoundService:PlayLocalSound(sound)
	task.delay(0.12, function()
		local s = SoundService:FindFirstChild("KemudiTransmissionRejectBeep")
		if s then SoundService:PlayLocalSound(s) end
	end)
end

local transmission_warning_messages = {
	projected_overrev = "UNSAFE DOWNSHIFT // OVER-REV PROTECTION",
	vehicle_moving = "SHIFT REJECTED // VEHICLE MOVING",
	brake_interlock = "SHIFT REJECTED // HOLD BRAKE",
	shift_in_progress = "SHIFT REJECTED // SHIFT IN PROGRESS",
	transmission_failed = "TRANSMISSION FAILED // NEUTRAL",
	gear_limit = "SHIFT REJECTED // GEAR LIMIT",
	neutral_engagement_damage = "NEUTRAL ENGAGEMENT // DAMAGE RISK",
}

local function showTransmissionRejectWarning(reason, code, kind)
	if kind ~= "warning" then playTransmissionRejectAlert() end
	transmission_warning_token += 1
	local token = transmission_warning_token
	local message = transmission_warning_messages[reason] or string.format("SHIFT REJECTED // CODE %s", tostring(code or 0))
	transmission_warning_label.Text = "TRANS WARNING // " .. message
	transmission_warning_label.Visible = true
	task.delay(2.5, function()
		if token == transmission_warning_token and transmission_warning_label.Parent then
			transmission_warning_label.Visible = false
		end
	end)
end

local function recolorArcDots(redline, maxRpm)
	for _, d in tacho_arc_dots do
		local rpm_at = d.frac * maxRpm
		d.frame.BackgroundColor3 = if rpm_at >= redline then Color3.fromRGB(220, 55, 55)
			elseif rpm_at >= redline * 0.8 then Color3.fromRGB(220, 180, 50)
			else Color3.fromRGB(75, 175, 100)
	end
end

local function readTachoSpec()
	if tacho_spec_read then return end
	local model = workspace:FindFirstChild("Vehicle_" .. local_player.Name)
	if not model then return end
	local spec = model:FindFirstChild("VehicleSpec")
	if not spec then return end
	local engine = spec:FindFirstChild("Engine")
	if not engine then return end
	local redline = engine:GetAttribute("RedlineRPM")
	local revlimiter = engine:GetAttribute("RevLimiterRPM")
	if type(redline) == "number" and redline > 0 then
		tacho_redline_rpm = redline
	end
	if type(revlimiter) == "number" and revlimiter > 0 then
		tacho_max_rpm = math.max(revlimiter + 500, tacho_redline_rpm + 1000)
	end
	tacho_spec_read = true
	recolorArcDots(tacho_redline_rpm, tacho_max_rpm)
end

local function updateNeedleTransform(angleDeg)
	local angleRad = math.rad(angleDeg)
	local centerX = TACHO_CX + NEEDLE_CENTER_OFFSET * math.sin(angleRad)
	local centerY = TACHO_CY - NEEDLE_CENTER_OFFSET * math.cos(angleRad)
	tacho_needle.Position = UDim2.fromOffset(centerX, centerY)
	tacho_needle.Rotation = angleDeg
end

-- ============================================================
-- OBD-II telemetry formatters
-- ============================================================

local function telemetryValue(data, index)
	local value = data[index + 1]
	return if type(value) == "number" and value == value then value else 0
end

local function gearText(value)
	local gear = math.floor(value + 0.5)
	if gear == -2 then return "P" end
	if gear < 0 then return "R" end
	if gear == 0 then return "N" end
	return tostring(gear)
end

local function percentText(value)
	return string.format("%3d%%", math.clamp(math.floor(value * 100 + 0.5), 0, 100))
end

local function signedPercentText(value)
	return string.format("%+4d%%", math.floor(value * 100 + (if value >= 0 then 0.5 else -0.5)))
end

local function booleanText(value)
	return if value > 0.5 then "ON" else "OFF"
end

local function stageText(value)
	local stage = math.floor(value + 0.5)
	return if stage == 1 then "ACCESSORY" elseif stage == 2 then "RUNNING" else "OFF"
end

local function shiftPhaseText(value)
	local phase = math.floor(value + 0.5)
	return ({ "IDLE", "DISENGAGING", "NEUTRAL", "ENGAGING" })[phase + 1] or tostring(phase)
end

local function tcmStateText(value)
	return ({ "NORMAL", "SPORT", "WINTER", "MANUAL", "LIMP", "FAULT", "OFF" })[math.floor(value + 0.5) + 1] or "UNKNOWN"
end

local function selectorText(data, Contract)
	local gear = math.floor(telemetryValue(data, Contract.T.GEAR) + 0.5)
	if gear == -2 then return "P" end
	if gear == -1 then return "R" end
	if gear == 0 then return "N" end
	local tcmState = telemetryValue(data, Contract.T.TCM_STATE)
	return if math.floor(tcmState + 0.5) == 3 then "M/S" else "D"
end

local function formatMetric(metric, value, data, Contract)
	local formatted = if metric.format == "percent" then percentText(value)
		elseif metric.format == "signed_percent" then signedPercentText(value)
		elseif metric.format == "bool" then booleanText(value)
		elseif metric.format == "gear" then gearText(value)
		elseif metric.format == "selector" then selectorText(data or {}, Contract)
		elseif metric.format == "stage" then stageText(value)
		elseif metric.format == "phase" then shiftPhaseText(value)
		elseif metric.format == "tcm_state" then tcmStateText(value)
		elseif metric.format == "integer" then tostring(math.floor(value + 0.5))
		elseif metric.format == "rpm" then string.format("%d rpm", math.max(0, math.floor(value + 0.5)))
		elseif metric.format == "celsius" then string.format("%.1f C", value)
		elseif metric.format == "kmh" then string.format("%.1f km/h", value)
		elseif metric.format == "mps" then string.format("%.2f m/s", value)
		elseif metric.format == "radians" then string.format("%.2f rad/s", value)
		elseif metric.format == "seconds" then string.format("%.2f s", value)
		elseif metric.format == "nm" then string.format("%.1f Nm", value)
		elseif metric.format == "hp" then string.format("%.1f hp", value)
		else string.format("%.3f", value)
	return string.format("%-24s %s", metric.label, formatted)
end

-- ============================================================
-- OBD tab definitions
-- ============================================================

local function getTabs(Contract)
	return {
		{
			name = "DRIVE",
			metrics = {
				{ label = "Speed", index = Contract.T.SPEED_KMH, format = "kmh" },
				{ label = "Speed SI", index = Contract.T.SPEED_MPS, format = "mps" },
				{ label = "Engine RPM", index = Contract.T.RPM, format = "rpm" },
				{ label = "Current gear", index = Contract.T.GEAR, format = "gear" },
				{ label = "Selector", index = Contract.T.GEAR, format = "selector" },
				{ label = "Throttle", index = Contract.T.THROTTLE, format = "percent" },
				{ label = "Brake", index = Contract.T.BRAKE, format = "percent" },
				{ label = "Steering", index = Contract.T.STEERING, format = "signed_percent" },
				{ label = "Clutch", index = Contract.T.CLUTCH, format = "percent" },
				{ label = "Wheel speed", index = Contract.T.WHEEL_SPEED, format = "radians" },
				{ label = "Drive torque", index = Contract.T.DRIVE_TORQUE },
				{ label = "Engine running", index = Contract.T.ENGINE_RUNNING, format = "bool" },
				{ label = "Engine stage", index = Contract.T.ENGINE_STAGE, format = "stage" },
				{ label = "Shift phase", index = Contract.T.SHIFT_PHASE, format = "phase" },
				{ label = "Simulation time", index = Contract.T.SIM_TIME, format = "seconds" },
				{ label = "Terrain height", index = Contract.T.TERRAIN_HEIGHT },
				{ label = "Wheel contacts", index = Contract.T.CONTACTS, format = "integer" },
			},
		},
		{
			name = "ENGINE",
			metrics = {
				{ label = "Coolant", index = Contract.T.COOLANT, format = "celsius" },
				{ label = "Oil temperature", index = Contract.T.OIL_TEMP, format = "celsius" },
				{ label = "Oil pressure", index = Contract.T.OIL_PRESSURE },
				{ label = "Fuel level", index = Contract.T.FUEL },
				{ label = "Fuel remaining", index = Contract.T.FUEL_PERCENT, format = "percent" },
				{ label = "Power derate", index = Contract.T.DERATE, format = "percent" },
				{ label = "Engine damage", index = Contract.T.DAMAGE, format = "percent" },
				{ label = "Engine warning", index = Contract.T.ENGINE_WARNING, format = "bool" },
				{ label = "Lugging stress", index = Contract.T.LUGGING_STRESS, format = "percent" },
				{ label = "Overrev stress", index = Contract.T.OVERREV_STRESS, format = "percent" },
				{ label = "Impact severity", index = Contract.T.IMPACT_SEVERITY, format = "percent" },
				{ label = "Engine torque", index = Contract.T.ENGINE_TORQUE, format = "nm" },
				{ label = "Available torque", index = Contract.T.AVAILABLE_TORQUE, format = "nm" },
				{ label = "Engine power", index = Contract.T.ENGINE_POWER_HP, format = "hp" },
				{ label = "Available power", index = Contract.T.AVAILABLE_POWER_PCT, format = "percent" },
			},
		},
		{
			name = "TRANS",
			metrics = {
				{ label = "TCM state", index = Contract.T.TCM_STATE, format = "tcm_state" },
				{ label = "Line pressure", index = Contract.T.TCM_LINE_PRESSURE },
				{ label = "Lockup", index = Contract.T.TCM_LOCKUP, format = "bool" },
				{ label = "Requested gear", index = Contract.T.REQUESTED_GEAR, format = "gear" },
				{ label = "Input shaft", index = Contract.T.INPUT_SHAFT_RPM, format = "rpm" },
				{ label = "Output shaft", index = Contract.T.OUTPUT_SHAFT_RPM, format = "rpm" },
				{ label = "Converter coupling", index = Contract.T.CONVERTER_COUPLING, format = "percent" },
				{ label = "Transmission temp", index = Contract.T.TRANS_TEMP, format = "celsius" },
				{ label = "Transmission damage", index = Contract.T.DAMAGE, format = "percent" },
				{ label = "Sensor input RPM", index = Contract.T.TCM_SENSOR_INPUT_RPM, format = "rpm" },
				{ label = "Sensor speed", index = Contract.T.TCM_SENSOR_SPEED_KMH, format = "kmh" },
				{ label = "Sensor age", index = Contract.T.TCM_SENSOR_AGE, format = "seconds" },
				{ label = "Shift latency", index = Contract.T.TCM_SHIFT_LATENCY, format = "seconds" },
				{ label = "Torque reduction", index = Contract.T.TCM_TORQUE_REDUCTION, format = "percent" },
				{ label = "Fail-safe gear", index = Contract.T.TCM_FAIL_SAFE_GEAR, format = "gear" },
				{ label = "Fault mask", index = Contract.T.TCM_FAULT_MASK, format = "integer" },
				{ label = "Diagnostic code", index = Contract.T.TCM_DIAGNOSTIC_CODE, format = "integer" },
			},
		},
		{
			name = "CHASSIS",
			metrics = {
				{ label = "Suspension load", index = Contract.T.SUSPENSION_LOAD },
				{ label = "Average grip", index = Contract.T.GRIP, format = "percent" },
				{ label = "Slip ratio", index = Contract.T.SLIP_RATIO, format = "percent" },
				{ label = "Yaw error", index = Contract.T.YAW_ERROR },
				{ label = "Rut depth", index = Contract.T.RUT_DEPTH },
				{ label = "Broken beams", index = Contract.T.BROKEN_BEAMS, format = "integer" },
				{ label = "Deformation depth", index = Contract.T.DEFORMATION_DEPTH },
				{ label = "Broken parts", index = Contract.T.BROKEN_PARTS, format = "integer" },
				{ label = "Damage zone", index = Contract.T.DAMAGE_ZONE, format = "integer" },
				{ label = "Wheel speed FL", index = Contract.T.WHEEL_SPEED_FL, format = "radians" },
				{ label = "Wheel speed FR", index = Contract.T.WHEEL_SPEED_FR, format = "radians" },
				{ label = "Wheel speed RL", index = Contract.T.WHEEL_SPEED_RL, format = "radians" },
				{ label = "Wheel speed RR", index = Contract.T.WHEEL_SPEED_RR, format = "radians" },
				{ label = "Suspension FL", index = Contract.T.SUSP_COMPRESSION_FL },
				{ label = "Suspension FR", index = Contract.T.SUSP_COMPRESSION_FR },
				{ label = "Suspension RL", index = Contract.T.SUSP_COMPRESSION_RL },
				{ label = "Suspension RR", index = Contract.T.SUSP_COMPRESSION_RR },
				{ label = "Load FL", index = Contract.T.SUSP_LOAD_FL },
				{ label = "Load FR", index = Contract.T.SUSP_LOAD_FR },
				{ label = "Load RL", index = Contract.T.SUSP_LOAD_RL },
				{ label = "Load RR", index = Contract.T.SUSP_LOAD_RR },
			},
		},
		{
			name = "TIRES",
			metrics = {
				{ label = "Tire temperature", index = Contract.T.TIRE_TEMP, format = "celsius" },
				{ label = "Tire wear", index = Contract.T.TIRE_WEAR, format = "percent" },
				{ label = "Temp FL", index = Contract.T.TIRE_TEMP_FL, format = "celsius" },
				{ label = "Temp FR", index = Contract.T.TIRE_TEMP_FR, format = "celsius" },
				{ label = "Temp RL", index = Contract.T.TIRE_TEMP_RL, format = "celsius" },
				{ label = "Temp RR", index = Contract.T.TIRE_TEMP_RR, format = "celsius" },
				{ label = "Wear FL", index = Contract.T.TIRE_WEAR_FL, format = "percent" },
				{ label = "Wear FR", index = Contract.T.TIRE_WEAR_FR, format = "percent" },
				{ label = "Wear RL", index = Contract.T.TIRE_WEAR_RL, format = "percent" },
				{ label = "Wear RR", index = Contract.T.TIRE_WEAR_RR, format = "percent" },
			},
		},
		{
			name = "SAFETY",
			metrics = {
				{ label = "ABS", index = Contract.T.ABS_ACTIVE, format = "bool" },
				{ label = "Traction control", index = Contract.T.TC_ACTIVE, format = "bool" },
				{ label = "Stability control", index = Contract.T.VSC_ACTIVE, format = "bool" },
				{ label = "FCW", index = Contract.T.ADAS_FCW, format = "bool" },
				{ label = "AEB", index = Contract.T.ADAS_AEB, format = "bool" },
				{ label = "ADAS confidence", index = Contract.T.ADAS_CONFIDENCE, format = "percent" },
				{ label = "Target detected", index = Contract.T.ADAS_TARGET_DETECTED, format = "bool" },
				{ label = "Target distance", index = Contract.T.ADAS_TARGET_DISTANCE },
				{ label = "Time to collision", index = Contract.T.ADAS_TIME_TO_COLLISION },
				{ label = "Relative speed", index = Contract.T.ADAS_RELATIVE_SPEED },
				{ label = "AEB brake command", index = Contract.T.ADAS_BRAKE_COMMAND, format = "percent" },
				{ label = "ADAS sensor faults", index = Contract.T.ADAS_SENSOR_FAULTS, format = "integer" },
			},
		},
	}
end

-- Module state — set during init
local tab_pages = {}
local page_rows = {}
local Contract = nil

local function setActiveTab(name)
	for tabName, page in tab_pages do
		page.Visible = tabName == name
	end
end

-- ============================================================
-- updateHud — called from telemetry event handler
-- ============================================================

local function updateHud(data)
	local speed = telemetryValue(data, Contract.T.SPEED_KMH)
	local rpm = telemetryValue(data, Contract.T.RPM)
	local gear = telemetryValue(data, Contract.T.GEAR)
	local throttle = telemetryValue(data, Contract.T.THROTTLE)
	local brake = telemetryValue(data, Contract.T.BRAKE)
	local steering = telemetryValue(data, Contract.T.STEERING)
	local engineStage = math.floor(telemetryValue(data, Contract.T.ENGINE_STAGE) + 0.5)

	-- Read vehicle spec for redline (once per spawn)
	readTachoSpec()

	-- Tachometer needle
	local clampedRpm = math.max(0, math.min(rpm, tacho_max_rpm))
	local needleAngle = -135 + (clampedRpm / tacho_max_rpm) * 270
	updateNeedleTransform(needleAngle)

	-- Digital RPM readout
	tacho_rpm_text.Text = tostring(math.floor(rpm + 0.5))
	tacho_rpm_text.TextColor3 = if rpm >= tacho_redline_rpm then Color3.fromRGB(255, 80, 80) else Color3.fromRGB(235, 245, 255)

	-- Needle highlights at redline
	tacho_needle.BackgroundColor3 = if rpm >= tacho_redline_rpm then Color3.fromRGB(255, 70, 70) else Color3.fromRGB(230, 65, 65)

	-- Info labels
	speed_label.Text = string.format("%03d km/h", math.max(0, math.floor(speed + 0.5)))
	gear_label.Text = gearText(gear)
	local driving = false
	local engineText = if engineStage == 1 then "ACC" elseif engineStage == 2 then "ON" else "OFF"
	status_label.Text = if driving then "ENG " .. engineText .. (if engineStage == 2 then " // READY" else " // F") else "EXITED"
	input_label.Text = string.format("T%02d B%02d S%+03d", math.floor(throttle * 100 + 0.5), math.floor(brake * 100 + 0.5), math.floor(steering * 100 + (if steering >= 0 then 0.5 else -0.5)))

	obd_status_label.Text = if telemetryValue(data, Contract.T.ENGINE_WARNING) > 0 or telemetryValue(data, Contract.T.TCM_FAULT_MASK) > 0
		then "LINK // WARNING"
		else "LINK CONNECTED"

	for tabName, rows in page_rows do
		for _, row in rows do
			row.label.Text = formatMetric(row.metric, telemetryValue(data, row.metric.index), data, Contract)
		end
	end
end

-- ============================================================
-- init — creates all GUI elements, called once from client.luau
-- ============================================================

local function init(params)
	local_player = params.localPlayer
	SoundService = params.SoundService
	Contract = params.Contract
	local playerGui = params.playerGui
	local TransmissionAlertEvent = params.TransmissionAlertEvent

	-- Create HUD ScreenGui
	local hud_gui = playerGui:FindFirstChild("KemudiHUD")
	if not hud_gui then
		local created = Instance.new("ScreenGui")
		created.Name = "KemudiHUD"
		created.ResetOnSpawn = false
		created.IgnoreGuiInset = true
		created.DisplayOrder = 20
		created.Parent = playerGui
		hud_gui = created
	end
	-- Keep the complete HUD hidden until the local player occupies the authored
	-- driver seat. The state transition below is the single visibility boundary.
	hud_gui.Enabled = false

	-- Dashboard panel (tachometer housing)
	dashboard = makePanel(
		hud_gui,
		"Dashboard",
		UDim2.fromOffset(TACHO_W, TACHO_H),
		UDim2.new(1, -22, 1, -22),
		Vector2.new(1, 1)
	)

	-- Colored arc (270° sweep: -135° to +135° from vertical)
	tacho_arc_dots = {}
	local ARC_SEGMENTS = 80
	for i = 0, ARC_SEGMENTS do
		local frac = i / ARC_SEGMENTS
		local angle_rad = math.rad(-135 + frac * 270)
		local ax = TACHO_CX + TACHO_ARC_R * math.sin(angle_rad)
		local ay = TACHO_CY - TACHO_ARC_R * math.cos(angle_rad)
		local dot = Instance.new("Frame")
		dot.Name = "Arc_" .. i
		dot.Size = UDim2.fromOffset(3, 3)
		dot.AnchorPoint = Vector2.new(0.5, 0.5)
		dot.Position = UDim2.fromOffset(ax, ay)
		dot.BackgroundColor3 = Color3.fromRGB(80, 170, 220)
		dot.BorderSizePixel = 0
		Instance.new("UICorner", dot).CornerRadius = UDim.new(1, 0)
		dot.Parent = dashboard
		tacho_arc_dots[i + 1] = { frame = dot, frac = frac }
	end
	recolorArcDots(tacho_redline_rpm, tacho_max_rpm)

	-- Major tick labels (0-8 x1000 RPM, 270° sweep)
	local LABEL_R = TACHO_R - 20
	for i = 0, 8 do
		local frac = i / 8
		local angle_rad = math.rad(-135 + frac * 270)
		local lx = TACHO_CX + LABEL_R * math.sin(angle_rad)
		local ly = TACHO_CY - LABEL_R * math.cos(angle_rad)
		local is_red = i >= math.ceil(tacho_redline_rpm / 1000)
		local lbl = Instance.new("TextLabel")
		lbl.Size = UDim2.fromOffset(22, 14)
		lbl.Position = UDim2.fromOffset(lx - 11, ly - 7)
		lbl.BackgroundTransparency = 1
		lbl.Text = tostring(i)
		lbl.TextColor3 = if is_red then Color3.fromRGB(255, 80, 80) else Color3.fromRGB(170, 195, 210)
		lbl.Font = Enum.Font.Code
		lbl.TextSize = 11
		lbl.TextXAlignment = Enum.TextXAlignment.Center
		lbl.Parent = dashboard
	end

	-- Minor tick marks (every 500 RPM, 270° sweep)
	for i = 0, 16 do
		local frac = i / 16
		local angle_deg = -135 + frac * 270
		local angle_rad = math.rad(angle_deg)
		local is_major = (i % 2 == 0)
		local tick_len = if is_major then 12 else 7
		local tick_w = if is_major then 2 else 1
		local tx = TACHO_CX + (TACHO_R - 3) * math.sin(angle_rad)
		local ty = TACHO_CY - (TACHO_R - 3) * math.cos(angle_rad)
		local tick = Instance.new("Frame")
		tick.Size = UDim2.fromOffset(tick_w, tick_len)
		tick.AnchorPoint = Vector2.new(0.5, 0)
		tick.Position = UDim2.fromOffset(tx, ty)
		tick.Rotation = angle_deg
		tick.BackgroundColor3 = if is_major then Color3.fromRGB(180, 200, 215) else Color3.fromRGB(100, 120, 135)
		tick.BorderSizePixel = 0
		tick.Parent = dashboard
	end

	-- Needle
	tacho_needle = Instance.new("Frame")
	tacho_needle.Name = "Needle"
	tacho_needle.Size = UDim2.fromOffset(3, NEEDLE_TOTAL_LEN)
	tacho_needle.AnchorPoint = Vector2.new(0.5, 0.5)
	tacho_needle.BackgroundColor3 = Color3.fromRGB(230, 65, 65)
	tacho_needle.BorderSizePixel = 0
	tacho_needle.ZIndex = 5
	tacho_needle.Parent = dashboard
	Instance.new("UICorner", tacho_needle).CornerRadius = UDim.new(0, 1)
	local needle_glow = Instance.new("UIStroke", tacho_needle)
	needle_glow.Color = Color3.fromRGB(255, 100, 60)
	needle_glow.Transparency = 0.5
	needle_glow.Thickness = 1
	updateNeedleTransform(-135) -- idle position, matches the 0 RPM tick

	-- Center hub
	local hub = Instance.new("Frame")
	hub.Size = UDim2.fromOffset(14, 14)
	hub.AnchorPoint = Vector2.new(0.5, 0.5)
	hub.Position = UDim2.fromOffset(TACHO_CX, TACHO_CY)
	hub.BackgroundColor3 = Color3.fromRGB(180, 195, 210)
	hub.BorderSizePixel = 0
	hub.ZIndex = 6
	hub.Parent = dashboard
	Instance.new("UICorner", hub).CornerRadius = UDim.new(1, 0)

	-- Digital RPM readout
	tacho_rpm_text = Instance.new("TextLabel")
	tacho_rpm_text.Name = "RPMText"
	tacho_rpm_text.Size = UDim2.fromOffset(80, 22)
	tacho_rpm_text.Position = UDim2.fromOffset(TACHO_CX - 40, TACHO_CY + 8)
	tacho_rpm_text.BackgroundTransparency = 1
	tacho_rpm_text.Text = "0"
	tacho_rpm_text.TextColor3 = Color3.fromRGB(235, 245, 255)
	tacho_rpm_text.Font = Enum.Font.Code
	tacho_rpm_text.TextSize = 20
	tacho_rpm_text.TextXAlignment = Enum.TextXAlignment.Center
	tacho_rpm_text.ZIndex = 5
	tacho_rpm_text.Parent = dashboard

	-- RPM unit label
	local rpm_unit = Instance.new("TextLabel")
	rpm_unit.Size = UDim2.fromOffset(40, 12)
	rpm_unit.Position = UDim2.fromOffset(TACHO_CX - 20, TACHO_CY + 28)
	rpm_unit.BackgroundTransparency = 1
	rpm_unit.Text = "RPM"
	rpm_unit.TextColor3 = Color3.fromRGB(120, 145, 165)
	rpm_unit.Font = Enum.Font.Code
	rpm_unit.TextSize = 9
	rpm_unit.TextXAlignment = Enum.TextXAlignment.Center
	rpm_unit.ZIndex = 5
	rpm_unit.Parent = dashboard

	-- Below-gauge info
	gear_label = makeLabel(dashboard, "Gear", UDim2.fromOffset(TACHO_CX - 30, 155), UDim2.fromOffset(60, 20), "N", 18)
	gear_label.TextXAlignment = Enum.TextXAlignment.Center
	speed_label = makeLabel(dashboard, "Speed", UDim2.fromOffset(10, 175), UDim2.fromOffset(100, 18), "000 km/h", 14)
	status_label = makeLabel(dashboard, "Status", UDim2.fromOffset(10, 193), UDim2.fromOffset(200, 16), "ENGINE OFF", 11)
	input_label = makeLabel(dashboard, "Inputs", UDim2.fromOffset(110, 175), UDim2.fromOffset(100, 18), "", 11)

	transmission_warning_label = makeLabel(dashboard, "TransmissionWarning", UDim2.fromOffset(10, 160), UDim2.fromOffset(200, 30), "", 10)
	transmission_warning_label.TextColor3 = Color3.fromRGB(255, 190, 80)
	transmission_warning_label.TextWrapped = true
	transmission_warning_label.Visible = false

	TransmissionAlertEvent.OnClientEvent:Connect(showTransmissionRejectWarning)

	-- OBD-II panel
	obd_panel = makePanel(
		hud_gui,
		"OBD",
		UDim2.fromOffset(430, 430),
		UDim2.fromOffset(22, 22)
	)
	makeLabel(obd_panel, "Title", UDim2.fromOffset(12, 8), UDim2.fromOffset(400, 20), "OBD-II // FULL TELEMETRY", 15)
	obd_status_label = makeLabel(obd_panel, "LinkStatus", UDim2.fromOffset(290, 8), UDim2.fromOffset(125, 20), "LINK CONNECTED", 11)
	obd_status_label.TextXAlignment = Enum.TextXAlignment.Right

	local tab_bar = Instance.new("Frame")
	tab_bar.Name = "Tabs"
	tab_bar.Position = UDim2.fromOffset(10, 34)
	tab_bar.Size = UDim2.new(1, -20, 0, 28)
	tab_bar.BackgroundTransparency = 1
	tab_bar.Parent = obd_panel

	local content = Instance.new("Frame")
	content.Name = "Content"
	content.Position = UDim2.fromOffset(10, 68)
	content.Size = UDim2.new(1, -20, 1, -78)
	content.BackgroundTransparency = 1
	content.Parent = obd_panel

	local tabs = getTabs(Contract)
	local tab_buttons = {}
	local active_tab = tabs[1].name

	for tabIndex, tab in tabs do
		local button = Instance.new("TextButton")
		button.Name = tab.name
		button.Size = UDim2.new(1 / #tabs, -3, 1, 0)
		button.Position = UDim2.new((tabIndex - 1) / #tabs, 0, 0, 0)
		button.BackgroundColor3 = Color3.fromRGB(18, 30, 42)
		button.BorderSizePixel = 0
		button.AutoButtonColor = false
		button.Font = Enum.Font.Code
		button.TextSize = 11
		button.Text = tab.name
		button.Parent = tab_bar
		button.Activated:Connect(function() setActiveTab(tab.name) end)
		tab_buttons[tab.name] = button

		local page = Instance.new("ScrollingFrame")
		page.Name = tab.name .. "Page"
		page.Size = UDim2.fromScale(1, 1)
		page.BackgroundTransparency = 1
		page.BorderSizePixel = 0
		page.ScrollBarThickness = 5
		page.ScrollBarImageColor3 = Color3.fromRGB(80, 170, 220)
		page.CanvasSize = UDim2.fromOffset(0, #tab.metrics * 22 + 8)
		page.ScrollingDirection = Enum.ScrollingDirection.Y
		page.Parent = content
		local layout = Instance.new("UIListLayout")
		layout.Padding = UDim.new(0, 2)
		layout.SortOrder = Enum.SortOrder.LayoutOrder
		layout.Parent = page

		tab_pages[tab.name] = page
		page_rows[tab.name] = {}
		for metricIndex, metric in tab.metrics do
			local row = makeLabel(page, metric.label, UDim2.fromOffset(8, 0), UDim2.new(1, -16, 0, 20), metric.label .. "  --", 13)
			row.LayoutOrder = metricIndex
			row.TextYAlignment = Enum.TextYAlignment.Center
			table.insert(page_rows[tab.name], { label = row, metric = metric })
		end
	end
	setActiveTab(active_tab)

	-- Return controls for the parent LocalScript
	return {
		hud_gui = hud_gui,
		dashboard = dashboard,
		obd_panel = obd_panel,
		transmission_warning_label = transmission_warning_label,
		transmission_warning_token = function() return transmission_warning_token end,
		resetSpec = function()
			tacho_spec_read = false
		end,
		incrementWarningToken = function()
			transmission_warning_token += 1
		end,
	}
end

return {
	init = init,
	updateHud = updateHud,
}