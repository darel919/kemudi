--!strict
-- Roblox vehicle content loader.
-- Validates declarative Studio-authored assets and returns normalized runtime data.

local Loader = {}

local NODE_IDS = { "FL", "FR", "RL", "RR" }
local MAX_RENDER_PARTS = 128
local MAX_BEAMS = 256
local MAX_WHEELS = 4
local MAX_MASS_PROXIES = 64

local function describe(model: Model, field: string, message: string): string
	return string.format("vehicle asset %s invalid at %s: %s", model:GetFullName(), field, message)
end

local function fail(model: Model, field: string, message: string)
	error(describe(model, field, message), 2)
end

local function finiteNumber(value: any): boolean
	return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
end

local function requireFolder(model: Model, parent: Instance, name: string, field: string): Folder
	local child = parent:FindFirstChild(name)
	if not child or not child:IsA("Folder") then
		fail(model, field, "missing Folder " .. name)
	end
	return child :: Folder
end

local function requirePart(model: Model, parent: Instance, name: string, field: string): BasePart
	local child = parent:FindFirstChild(name)
	if not child or not child:IsA("BasePart") then
		fail(model, field, "missing BasePart " .. name)
	end
	return child :: BasePart
end

local function requireDriverSeat(model: Model, spec: Folder): Seat
	local child = spec:FindFirstChild("DriverSeat")
	if not child or not child:IsA("Seat") then
		fail(model, "VehicleSpec.DriverSeat", "missing Seat DriverSeat; author the driver entry point before spawning")
	end
	local seat = child :: Seat
	if not seat.Anchored or seat.CanCollide or not seat.Massless then
		fail(model, "VehicleSpec.DriverSeat", "DriverSeat must be anchored, massless, and non-colliding")
	end
	if seat:GetAttribute("KemudiDriverSeat") ~= true then
		fail(model, "VehicleSpec.DriverSeat", "missing KemudiDriverSeat=true")
	end
	return seat
end

local function requiredNumberAttribute(model: Model, instance: Instance, name: string, field: string, minimum: number): number
	local value = instance:GetAttribute(name)
	if value == nil then
		local child = instance:FindFirstChild(name)
		if child and child:IsA("NumberValue") then
			value = child.Value
		end
	end
	if not finiteNumber(value) or value < minimum then
		fail(model, field .. "." .. name, "expected a finite number >= " .. tostring(minimum))
	end
	return value :: number
end

local function requireStringAttribute(model: Model, instance: Instance, name: string, field: string): string
	local value = instance:GetAttribute(name)
	if value == nil then
		local child = instance:FindFirstChild(name)
		if child and child:IsA("StringValue") then value = child.Value end
	end
	if type(value) ~= "string" or value == "" then
		fail(model, field .. "." .. name, "expected a non-empty string")
	end
	return value :: string
end

local function loadSimulation(model: Model, spec: Folder)
	local root = requireFolder(model, spec, "Simulation", "VehicleSpec.Simulation")
	local function folder(name: string): Folder
		return requireFolder(model, root, name, "VehicleSpec.Simulation." .. name)
	end
	local function number(parent: Instance, name: string, field: string, minimum: number): number
		return requiredNumberAttribute(model, parent, name, "VehicleSpec.Simulation." .. field, minimum)
	end

	local aeroFolder = folder("Aero")
	local aero = {
		air_density = number(aeroFolder, "AirDensity", "Aero", 0), frontal_area = number(aeroFolder, "FrontalArea", "Aero", 0),
		cd = number(aeroFolder, "Cd", "Aero", 0), cl = number(aeroFolder, "Cl", "Aero", -100), cs = number(aeroFolder, "Cs", "Aero", -100),
		cm = number(aeroFolder, "Cm", "Aero", -100), wing_cl = number(aeroFolder, "WingCl", "Aero", -100), wing_area = number(aeroFolder, "WingArea", "Aero", 0),
		node_drag = number(aeroFolder, "NodeDrag", "Aero", 0), ground_ref_height = number(aeroFolder, "GroundRefHeight", "Aero", 0),
		pitch_brake_coeff = number(aeroFolder, "PitchBrakeCoeff", "Aero", -100), pitch_power_coeff = number(aeroFolder, "PitchPowerCoeff", "Aero", -100),
		ground_effect_min = number(aeroFolder, "GroundEffectMin", "Aero", 0.001), ground_effect_max = number(aeroFolder, "GroundEffectMax", "Aero", 1),
	}

	local brakeFolder = folder("Brakes")
	local brakes = {}
	for _, pair in ipairs({
		{ "BiasFront", "bias_front" }, { "DiscMass", "disc_mass" }, { "DiscSpecificHeat", "disc_specific_heat" }, { "DiscArea", "disc_area" },
		{ "ConvectiveCoeff", "convective_coeff" }, { "PadFrictionNominal", "pad_friction_nominal" }, { "FluidBoilTemp", "fluid_boil_temp" },
		{ "FadeOnsetTemp", "fade_onset_temp" }, { "FadeCriticalTemp", "fade_critical_temp" }, { "MaxDiscTemp", "max_disc_temp" },
 { "BrakeTorqueMax", "brake_torque_max" }, { "ContactPatchRadius", "contact_patch_radius" },
 { "AirflowSpeedReference", "airflow_speed_reference" }, { "FluidFadeEndTemp", "fluid_fade_end_temp" },
	}) do brakes[pair[2]] = number(brakeFolder, pair[1], "Brakes", 0) end

	local turboFolder = folder("Turbo")
	local turbo = {
		type = requireStringAttribute(model, turboFolder, "Type", "Turbo"),
		torque_per_psi = number(turboFolder, "TorquePerPsi", "Turbo", 0),
		reference_charge_temp = number(turboFolder, "ReferenceChargeTemp", "Turbo", -273.15),
		sc_torque_per_psi = number(turboFolder, "ScTorquePerPsi", "Turbo", 0),
	}
	local wheelFolder = folder("WheelDynamics")
	local wheelDynamics = { wheels = {} }
	for index, id in ipairs(NODE_IDS) do
		local wf = requireFolder(model, wheelFolder, id, "VehicleSpec.Simulation.WheelDynamics." .. id)
		wheelDynamics.wheels[index] = { wheel_mass = number(wf, "WheelMass", "WheelDynamics." .. id, 0), tire_radius = number(wf, "TireRadius", "WheelDynamics." .. id, 0) }
	end

	local kinematicsFolder = folder("SuspensionKinematics")
	local suspensionKinematics = {}
	for _, pair in ipairs({
		{ "CamberGain", "camber_gain" }, { "CamberAtRest", "camber_at_rest" }, { "ToeGain", "toe_gain" }, { "ToeAtRest", "toe_at_rest" },
		{ "RollCenterHeight", "roll_center_height" }, { "RollCenterMigration", "roll_center_migration" }, { "AntiDivePct", "anti_dive_pct" },
		{ "AntiSquatPct", "anti_squat_pct" }, { "KingpinInclination", "kingpin_inclination" }, { "ScrubRadius", "scrub_radius" },
		{ "CasterAngle", "caster_angle" }, { "MaxCompression", "max_compression" }, { "MaxExtension", "max_extension" },
	}) do suspensionKinematics[pair[2]] = number(kinematicsFolder, pair[1], "SuspensionKinematics", -10000) end

	local breathingFolder = folder("EngineBreathing")
	local veFolder = requireFolder(model, breathingFolder, "VECurve", "EngineBreathing.VECurve")
	local veCurve = {}
	local index = 1
	while true do
		local point = veFolder:FindFirstChild(tostring(index))
		if not point then break end
		if not point:IsA("Folder") then fail(model, "VehicleSpec.Simulation.EngineBreathing.VECurve", "points must be Folders") end
		veCurve[index] = { requiredNumberAttribute(model, point, "RPM", "EngineBreathing.VECurve." .. index, 0), requiredNumberAttribute(model, point, "VE", "EngineBreathing.VECurve." .. index, 0) }
		index += 1
	end
	if #veCurve < 2 then fail(model, "VehicleSpec.Simulation.EngineBreathing.VECurve", "at least two points are required") end
	local breathing = { ve_curve = veCurve }
	for _, pair in ipairs({
		{ "ExhaustBackpressureCoeff", "exhaust_backpressure_coeff", 0 }, { "IntakeTempOffset", "intake_temp_offset", -273.15 },
		{ "IntercoolerEffectiveness", "intercooler_effectiveness", 0 }, { "BaseAirDensity", "base_air_density", 0 }, { "AltitudeDecayPerKm", "altitude_decay_per_km", 0 },
		{ "AlternatorCoeff", "alternator_coeff", 0 }, { "AlternatorMaxTorque", "alternator_max_torque", 0 }, { "AcCompressorTorque", "ac_compressor_torque", 0 },
		{ "ExhaustPressurePenalty", "exhaust_pressure_penalty", 0 },
	}) do breathing[pair[2]] = number(breathingFolder, pair[1], "EngineBreathing", pair[3]) end
	breathing.ac_enabled = breathingFolder:GetAttribute("ACEnabled") == true

	local angularFolder = folder("Angular")
	local angular = { inertia = { number(angularFolder, "InertiaX", "Angular", 0), number(angularFolder, "InertiaY", "Angular", 0), number(angularFolder, "InertiaZ", "Angular", 0) }, shape_matching_stiffness = number(angularFolder, "ShapeMatchingStiffness", "Angular", 0), angular_damping = number(angularFolder, "AngularDamping", "Angular", 0) }
	local solverFolder = folder("Solver")
	local solver = {
		volume_compliance = number(solverFolder, "VolumeCompliance", "Solver", 0),
		body_attachment_softness = number(solverFolder, "BodyAttachmentSoftness", "Solver", 0),
		iterations = math.floor(number(solverFolder, "Iterations", "Solver", 1)),
		max_iterations = math.floor(number(solverFolder, "MaxIterations", "Solver", 1)),
		velocity_damping = number(solverFolder, "VelocityDamping", "Solver", 0),
	}
	local environmentFolder = folder("Environment")
	local environment = { ambient_temperature = number(environmentFolder, "AmbientTemperature", "Environment", -273.15) }

	local contactFolder = folder("Contact")
	local contact = {}
	for _, pair in ipairs({
		{ "ContactStiffness", "contact_stiffness" }, { "ContactDamping", "contact_damping" }, { "BodyContactStiffness", "body_contact_stiffness" },
		{ "BodyContactDamping", "body_contact_damping" }, { "TerrainCompliance", "terrain_compliance" }, { "FrictionCoefficient", "friction_coefficient" },
		{ "RollingResistanceCoeff", "rolling_resistance_coeff" }, { "TireContactPatchArea", "tire_contact_patch_area" },
	}) do contact[pair[2]] = number(contactFolder, pair[1], "Contact", 0) end
	local terrainFolder = folder("Terrain")
	local terrain = {}
	for _, pair in ipairs({
		{ "SurfaceHardness", "surface_hardness" }, { "RutRecoveryRate", "rut_recovery_rate" }, { "MaxRutDepth", "max_rut_depth" },
		{ "SpreadFactor", "spread_factor" }, { "RecoveryDepthDamping", "recovery_depth_damping" }, { "ResistanceMax", "resistance_max" },
	}) do terrain[pair[2]] = number(terrainFolder, pair[1], "Terrain", 0) end
	local collisionFolder = folder("Collision")
	local collision = {
		cell_size = number(collisionFolder, "CellSize", "Collision", 0),
		grid_extent = number(collisionFolder, "GridExtent", "Collision", 0),
		contact_radius = number(collisionFolder, "ContactRadius", "Collision", 0),
	}

	local bushingsFolder = folder("Bushings")
	local bushings = {}
	for _, item in ipairs(bushingsFolder:GetChildren()) do
		if not item:IsA("Folder") then fail(model, "VehicleSpec.Simulation.Bushings", "bushing definitions must be Folders") end
		local field = "Bushings." .. item.Name
		table.insert(bushings, { node_a = math.floor(number(item, "NodeA", field, 0)), node_b = math.floor(number(item, "NodeB", field, 0)), stiffness = number(item, "Stiffness", field, 0), damping = number(item, "Damping", field, 0), deadzone = number(item, "Deadzone", field, 0) })
	end
	return { aero = aero, brakes = brakes, turbo = turbo, wheel_dynamics = wheelDynamics, suspension_kinematics = suspensionKinematics, engine_breathing = breathing, angular = angular, solver = solver, environment = environment, contact = contact, terrain = terrain, collision = collision, bushings = bushings }
end

local function loadWheels(model: Model, wheelsFolder: Folder, render: Folder)
	local wheels = {}
	for _, id in ipairs(NODE_IDS) do
		local wheel = wheelsFolder:FindFirstChild(id)
		if not wheel or not wheel:IsA("Folder") then
			fail(model, "Wheels", "missing wheel Folder " .. id)
		end
		local visual = wheel:FindFirstChild("Visual")
		if not visual or not visual:IsA("ObjectValue") or not visual.Value or not visual.Value:IsDescendantOf(render) then
			fail(model, "Wheels." .. id .. ".Visual", "must reference a render BasePart")
		end
		local radius = requiredNumberAttribute(model, wheel, "RadiusStuds", "Wheels." .. id, 1e-4)
		wheels[id] = {
			id = id,
			visual = visual.Value :: BasePart,
			radiusStuds = radius,
			steered = wheel:GetAttribute("Steered") == true,
		}
	end
	return wheels
end

local function loadBeams(model: Model, spec: Folder, nodeIndexByName: {[string]: number})
	local beamsFolder = spec:FindFirstChild("Beams")
	if not beamsFolder or not beamsFolder:IsA("Folder") then
		fail(model, "VehicleSpec.Beams", "missing Folder Beams; author beam content before spawning")
	end

	local beams = {}
	for _, instance in ipairs(beamsFolder:GetChildren()) do
		if not instance:IsA("Folder") then
			fail(model, "VehicleSpec.Beams." .. instance.Name, "beam definition must be a Folder")
		end
		if #beams >= MAX_BEAMS then
			fail(model, "VehicleSpec.Beams", "beam count exceeds " .. tostring(MAX_BEAMS))
		end
		local field = "VehicleSpec.Beams." .. instance.Name
		local nodeAName = requireStringAttribute(model, instance, "NodeA", field)
		local nodeBName = requireStringAttribute(model, instance, "NodeB", field)
		local nodeA = nodeIndexByName[nodeAName]
		local nodeB = nodeIndexByName[nodeBName]
		if nodeA == nil or nodeB == nil or nodeA == nodeB then
			fail(model, field, "NodeA and NodeB must reference two distinct solver nodes")
		end
		local stiffness = requiredNumberAttribute(model, instance, "Stiffness", field, 1)
		local damping = requiredNumberAttribute(model, instance, "Damping", field, 0)
		local strength = requiredNumberAttribute(model, instance, "Strength", field, 1)
		table.insert(beams, {
			nodeA = nodeA :: number,
			nodeB = nodeB :: number,
			stiffness = stiffness,
			damping = damping,
			strength = strength,
		})
	end
	if #beams == 0 then
		fail(model, "VehicleSpec.Beams", "at least one beam is required")
	end
	return beams
end

local function loadNumberSchedule(model: Model, parent: Instance, name: string, field: string): {number}
	local folder = parent:FindFirstChild(name)
	if not folder or not folder:IsA("Folder") then
		fail(model, field, "missing Folder " .. name)
	end
	local values = {}
	local index = 1
	while true do
		local value = folder:FindFirstChild(tostring(index))
		if not value then break end
		if not value:IsA("NumberValue") or not finiteNumber(value.Value) then
			fail(model, field .. "." .. tostring(index), "expected a finite NumberValue")
		end
		table.insert(values, value.Value)
		index += 1
	end
	if #values == 0 then fail(model, field, "must contain at least one numbered value") end
	return values
end

local function loadEngine(model: Model, spec: Folder)
	local folder = spec:FindFirstChild("Engine")
	if not folder or not folder:IsA("Folder") then
		fail(model, "VehicleSpec.Engine", "missing Folder Engine; every vehicle must author engine parameters")
	end
	local curveFolder = folder:FindFirstChild("TorqueCurve")
	if not curveFolder or not curveFolder:IsA("Folder") then
		fail(model, "VehicleSpec.Engine.TorqueCurve", "missing Folder TorqueCurve")
	end
	local torqueCurve = {}
	local previousRpm = -math.huge
	local index = 1
	while true do
		local point = curveFolder:FindFirstChild(tostring(index))
		if not point then break end
		if not point:IsA("Folder") then fail(model, "VehicleSpec.Engine.TorqueCurve." .. tostring(index), "expected a Folder") end
		local rpm = requiredNumberAttribute(model, point, "RPM", "VehicleSpec.Engine.TorqueCurve." .. tostring(index), 0)
		local torque = requiredNumberAttribute(model, point, "Torque", "VehicleSpec.Engine.TorqueCurve." .. tostring(index), 0)
		if rpm <= previousRpm then fail(model, "VehicleSpec.Engine.TorqueCurve", "RPM points must be strictly increasing") end
		torqueCurve[index] = { rpm, torque }
		previousRpm = rpm
		index += 1
	end
	if #torqueCurve < 2 then fail(model, "VehicleSpec.Engine.TorqueCurve", "at least two points are required") end
	return {
		idle_rpm = requiredNumberAttribute(model, folder, "IdleRPM", "VehicleSpec.Engine", 0),
		redline_rpm = requiredNumberAttribute(model, folder, "RedlineRPM", "VehicleSpec.Engine", 1),
		rev_limiter_rpm = requiredNumberAttribute(model, folder, "RevLimiterRPM", "VehicleSpec.Engine", 1),
		throttle_response = requiredNumberAttribute(model, folder, "ThrottleResponse", "VehicleSpec.Engine", 0),
		engine_braking = requiredNumberAttribute(model, folder, "EngineBraking", "VehicleSpec.Engine", 0),
		torque_curve = torqueCurve,
	}
end

local function loadDrivetrain(model: Model, spec: Folder)
	local folder = spec:FindFirstChild("Transmission")
	if not folder then
		fail(model, "VehicleSpec.Transmission", "missing Folder Transmission; every vehicle must author its transmission")
	end
	if not folder:IsA("Folder") then fail(model, "VehicleSpec.Transmission", "must be a Folder") end
	local mode = folder:GetAttribute("Mode")
	if mode ~= "Manual" and mode ~= "Automatic" then
		fail(model, "VehicleSpec.Transmission.Mode", "expected authored Mode Manual or Automatic")
	end
	local driveMode = spec:GetAttribute("DriveMode") or "Normal"
	local validDriveModes = { Normal = true, Eco = true, Comfort = true, Sport = true, Track = true, Snow = true }
	if type(driveMode) ~= "string" or not validDriveModes[driveMode] then
		fail(model, "VehicleSpec.DriveMode", "expected Normal, Eco, Comfort, Sport, Track, or Snow")
	end
	local reverseRatio = folder:GetAttribute("ReverseRatio")
	if not finiteNumber(reverseRatio) or math.abs(reverseRatio) < 0.01 then
		fail(model, "VehicleSpec.Transmission.ReverseRatio", "expected an authored non-zero ratio")
	end
	local transmission = {
		mode = mode,
		gear_ratios = loadNumberSchedule(model, folder, "GearRatios", "VehicleSpec.Transmission.GearRatios"),
		final_drive = requiredNumberAttribute(model, folder, "FinalDrive", "VehicleSpec.Transmission", 0.01),
		reverse_ratio = reverseRatio,
		clutch_engagement = requiredNumberAttribute(model, folder, "ClutchEngagement", "VehicleSpec.Transmission", 0),
	}
	local shiftDelay = requiredNumberAttribute(model, folder, "ShiftDelay", "VehicleSpec.Transmission", 0)
	local tcmFolder = folder:FindFirstChild("TCM")
	local tcm = nil
	if tcmFolder then
		if not tcmFolder:IsA("Folder") then fail(model, "VehicleSpec.Transmission.TCM", "must be a Folder") end
		tcm = {
			enabled = tcmFolder:GetAttribute("Enabled") == true,
			upshift_rpm = loadNumberSchedule(model, tcmFolder, "UpshiftRPM", "VehicleSpec.Transmission.TCM.UpshiftRPM"),
			downshift_rpm = loadNumberSchedule(model, tcmFolder, "DownshiftRPM", "VehicleSpec.Transmission.TCM.DownshiftRPM"),
			throttle_shift_factor = requiredNumberAttribute(model, tcmFolder, "ThrottleShiftFactor", "VehicleSpec.Transmission.TCM", 0),
			kickdown_threshold = requiredNumberAttribute(model, tcmFolder, "KickdownThreshold", "VehicleSpec.Transmission.TCM", 0),
			lockup_speed_threshold = requiredNumberAttribute(model, tcmFolder, "LockupSpeedThreshold", "VehicleSpec.Transmission.TCM", 0),
			lockup_throttle_threshold = requiredNumberAttribute(model, tcmFolder, "LockupThrottleThreshold", "VehicleSpec.Transmission.TCM", 0),
			thermal_limit = requiredNumberAttribute(model, tcmFolder, "ThermalLimit", "VehicleSpec.Transmission.TCM", 0),
			limp_gear = math.floor(requiredNumberAttribute(model, tcmFolder, "LimpGear", "VehicleSpec.Transmission.TCM", 1)),
			min_shift_interval = requiredNumberAttribute(model, tcmFolder, "MinShiftInterval", "VehicleSpec.Transmission.TCM", 0),
		}
	elseif mode == "Automatic" then
		fail(model, "VehicleSpec.Transmission.TCM", "automatic transmissions must author a TCM folder")
	end
	return {
		transmission = transmission,
		shift_duration = shiftDelay,
		tcm = tcm,
		drive_mode = driveMode,
	}
end

local function loadSuspension(model: Model, spec: Folder, wheels, studsPerMeter: number)
	local folder = spec:FindFirstChild("Suspension")
	if not folder or not folder:IsA("Folder") then
		fail(model, "VehicleSpec.Suspension", "missing Folder Suspension; every vehicle must author suspension parameters")
	end
	local suspension = {
		wheels = {},
		anti_roll_bar_stiffness = requiredNumberAttribute(model, folder, "AntiRollBarStiffness", "VehicleSpec.Suspension", 0),
		bump_stop_rate = requiredNumberAttribute(model, folder, "BumpStopRate", "VehicleSpec.Suspension", 0),
	}
	for _, id in ipairs(NODE_IDS) do
		local wheelFolder = folder:FindFirstChild(id)
		if not wheelFolder or not wheelFolder:IsA("Folder") then
			fail(model, "VehicleSpec.Suspension", "missing wheel Folder " .. id)
		end
		local field = "VehicleSpec.Suspension." .. id
		local wheel = wheels[id]
		local authoredRestLength = requiredNumberAttribute(model, wheelFolder, "RestLength", field, 0.01)
		local geometryRestLength = nil
		local physicsRig = model:FindFirstChild("PhysicsRig")
		local nodePart = physicsRig and physicsRig:FindFirstChild("Node_" .. id)
		local mountPart = physicsRig and physicsRig:FindFirstChild("Chassis_" .. id)
		if nodePart and nodePart:IsA("BasePart") and mountPart and mountPart:IsA("BasePart") then
			local separation = mountPart.Position - nodePart.Position
			local measuredLength = separation.Magnitude / studsPerMeter
			if measuredLength >= 0.01 and measuredLength == measuredLength then
				geometryRestLength = measuredLength
			end
		end
		suspension.wheels[#suspension.wheels + 1] = {
			-- Imported wheel geometry is authoritative for the neutral ride height.
			-- Keep the authored value for assets without separate chassis markers.
			rest_length = geometryRestLength or authoredRestLength,
			travel = requiredNumberAttribute(model, wheelFolder, "Travel", field, 0.01),
			spring_rate = requiredNumberAttribute(model, wheelFolder, "SpringRate", field, 0),
			damping = requiredNumberAttribute(model, wheelFolder, "Damping", field, 0),
			rebound_damping = requiredNumberAttribute(model, wheelFolder, "ReboundDamping", field, 0),
			unsprung_mass = requiredNumberAttribute(model, wheelFolder, "UnsprungMass", field, 0),
			tire_radius = wheel.radiusStuds / studsPerMeter,
		}
	end
	return suspension
end

function Loader.load(model: Model)
	if not model or not model:IsA("Model") then
		error("vehicle asset loader expected a Model", 2)
	end

	local version = model:GetAttribute("KemudiVehicleAssetVersion")
	if version ~= 1 then
		fail(model, "KemudiVehicleAssetVersion", "expected schema version 1")
	end

	local render = requireFolder(model, model, "Render", "Render")
	local physicsRig = requireFolder(model, model, "PhysicsRig", "PhysicsRig")
	local wheelsFolder = requireFolder(model, model, "Wheels", "Wheels")
	local spec = requireFolder(model, model, "VehicleSpec", "VehicleSpec")
	local driverSeat = requireDriverSeat(model, spec)
	local massProxies = requireFolder(model, physicsRig, "MassProxies", "PhysicsRig.MassProxies")

	local studsPerMeter = requiredNumberAttribute(model, spec, "StudsPerMeter", "VehicleSpec", 1e-4)
	local renderParts = {}
	for _, instance in ipairs(render:GetDescendants()) do
		if instance:IsA("BasePart") then
			if #renderParts >= MAX_RENDER_PARTS then
				fail(model, "Render", "render part count exceeds " .. tostring(MAX_RENDER_PARTS))
			end
			if not instance.Anchored or instance.CanCollide or instance.CanTouch or instance.CanQuery or not instance.Massless then
				fail(model, "Render." .. instance.Name, "render BaseParts must be anchored, massless, and non-colliding")
			end
			table.insert(renderParts, instance)
		end
	end
	if #renderParts == 0 then
		fail(model, "Render", "no render BaseParts found")
	end

	local nodeDefs = {}
	local nodeIndexByName = {}
	local totalMass = 0
	for index, id in ipairs(NODE_IDS) do
		local nodeName = "Node_" .. id
		local node = requirePart(model, physicsRig, nodeName, "PhysicsRig")
		local mountPart = physicsRig:FindFirstChild("Chassis_" .. id)
		if mountPart and not mountPart:IsA("BasePart") then
			fail(model, "PhysicsRig.Chassis_" .. id, "optional suspension mount must be a BasePart")
		end
		local proxy = requirePart(model, massProxies, "MassProxy_" .. id, "PhysicsRig.MassProxies")
		if proxy:GetAttribute("KemudiMassProxy") ~= true then
			fail(model, "PhysicsRig.MassProxies." .. proxy.Name, "missing KemudiMassProxy=true")
		end
		if proxy.Massless or proxy.CanCollide or proxy.CanTouch or proxy.CanQuery then
			fail(model, "PhysicsRig.MassProxies." .. proxy.Name, "mass proxy must be non-massless and non-colliding")
		end
		local mass = proxy:GetMass()
		if not finiteNumber(mass) or mass <= 0 then
			fail(model, "PhysicsRig.MassProxies." .. proxy.Name, "derived mass must be positive")
		end
		nodeIndexByName[nodeName] = index - 1
		table.insert(nodeDefs, {
			part = node,
			mountPart = mountPart,
			mass = mass,
			fixed = node:GetAttribute("KemudiFixed") == true,
		})
		totalMass += mass
	end

	if #massProxies:GetChildren() > MAX_MASS_PROXIES then
		fail(model, "PhysicsRig.MassProxies", "mass proxy count exceeds " .. tostring(MAX_MASS_PROXIES))
	end

	-- PhysicsRig parts are solver proxies, not a second visible chassis. Keeping
	-- them visible makes deformable node markers appear detached from the rigid
	-- render body as speed and beam forces increase.
	for _, instance in ipairs(physicsRig:GetDescendants()) do
		if instance:IsA("BasePart") then
			instance.Transparency = 1
			instance.CanCollide = false
			instance.CanTouch = false
			instance.CanQuery = false
			instance.CastShadow = false
		end
	end

	local wheels = loadWheels(model, wheelsFolder, render)
	local beams = loadBeams(model, spec, nodeIndexByName)
	local engineConfig = loadEngine(model, spec)
	local drivetrainConfig = loadDrivetrain(model, spec)
	drivetrainConfig.engine = engineConfig
	local suspensionConfig = loadSuspension(model, spec, wheels, studsPerMeter)
	local simulationConfig = loadSimulation(model, spec)
	local driveLayout = spec:GetAttribute("DriveLayout") or "RWD"
	if type(driveLayout) ~= "string" then
		fail(model, "VehicleSpec.DriveLayout", "expected a string such as RWD, FWD, or AWD")
	end

	return {
		model = model,
		nodeDefs = nodeDefs,
		beamDefs = beams,
		renderParts = renderParts,
		wheels = wheels,
		driverSeat = driverSeat,
		rootPart = nodeDefs[1].mountPart or nodeDefs[1].part,
		vehicleMass = totalMass,
		studsPerMeter = studsPerMeter,
		driveLayout = driveLayout,
		drivetrainConfig = drivetrainConfig,
		suspensionConfig = suspensionConfig,
		simulationConfig = simulationConfig,
	}
end

return Loader
