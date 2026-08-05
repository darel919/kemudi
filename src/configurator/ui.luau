--[[
	Configurator UI — Roblox ScreenGui for real-time vehicle configuration.
	Matches the existing KemudiHUD style: dark panels, Code font, blue accents.

	Goes in StarterPlayerScripts alongside client.luau.
	Pure Roblox API — no physics dependencies.
--]]

local Players = game:GetService("Players")
local UserInputService = game:GetService("UserInputService")
local ReplicatedStorage = game:GetService("ReplicatedStorage")

local ConfigChangeEvent = ReplicatedStorage:WaitForChild("KemudiConfigChange")
local ConfigSyncEvent = ReplicatedStorage:WaitForChild("KemudiConfigSync")
local ConfigManifestEvent = ReplicatedStorage:WaitForChild("KemudiConfigManifest")

local UI = {}

-- ============================================================
-- Style constants (matches KemudiHUD)
-- ============================================================

local COLORS = {
	bg = Color3.fromRGB(10, 16, 24),
	bg_active = Color3.fromRGB(35, 115, 155),
	bg_hover = Color3.fromRGB(25, 50, 70),
	stroke = Color3.fromRGB(80, 170, 220),
	stroke_dim = Color3.fromRGB(50, 100, 140),
	text = Color3.fromRGB(235, 245, 255),
	text_dim = Color3.fromRGB(140, 170, 190),
	text_accent = Color3.fromRGB(100, 200, 255),
	text_warn = Color3.fromRGB(255, 190, 80),
	text_error = Color3.fromRGB(255, 100, 100),
	slider_fill = Color3.fromRGB(60, 160, 220),
	slider_bg = Color3.fromRGB(20, 30, 42),
	toggle_on = Color3.fromRGB(60, 180, 120),
	toggle_off = Color3.fromRGB(80, 80, 90),
	reset_btn = Color3.fromRGB(120, 50, 50),
}

local FONT = Enum.Font.Code
local PANEL_WIDTH = 380
local PANEL_HEIGHT = 520
local TOGGLE_KEY = Enum.KeyCode.Z  -- Z to open/close configurator

-- ============================================================
-- Helpers
-- ============================================================

local local_player = Players.LocalPlayer

local function makeCorner(parent, radius)
	local c = Instance.new("UICorner")
	c.CornerRadius = UDim.new(0, radius or 6)
	c.Parent = parent
	return c
end

local function makeStroke(parent, color, transparency)
	local s = Instance.new("UIStroke")
	s.Color = color or COLORS.stroke
	s.Transparency = transparency or 0.35
	s.Thickness = 1
	s.Parent = parent
	return s
end

local function makeText(parent, name, props)
	local label = Instance.new("TextLabel")
	label.Name = name
	label.BackgroundTransparency = 1
	label.Font = FONT
	label.TextColor3 = COLORS.text
	label.TextXAlignment = Enum.TextXAlignment.Left
	for k, v in props do label[k] = v end
	label.Parent = parent
	return label
end

local function makeButton(parent, name, props)
	local btn = Instance.new("TextButton")
	btn.Name = name
	btn.Font = FONT
	btn.TextColor3 = COLORS.text
	btn.BackgroundColor3 = COLORS.bg
	btn.AutoButtonColor = true
	makeCorner(btn, 4)
	makeStroke(btn, COLORS.stroke_dim, 0.5)
	for k, v in props do btn[k] = v end
	btn.Parent = parent
	return btn
end

-- ============================================================
-- Slider widget
-- ============================================================

local function createSlider(parent, label, min, max, step, current, onChange)
	local frame = Instance.new("Frame")
	frame.Size = UDim2.new(1, 0, 0, 44)
	frame.BackgroundTransparency = 1
	frame.Parent = parent

	makeText(frame, "Label", {
		Position = UDim2.fromOffset(0, 0),
		Size = UDim2.new(0.55, 0, 0, 16),
		Text = label,
		TextColor3 = COLORS.text_dim,
		TextSize = 11,
	})

	local valueLabel = makeText(frame, "Value", {
		Position = UDim2.new(0.55, 0, 0, 0),
		Size = UDim2.new(0.45, 0, 0, 16),
		Text = string.format("%.2f", current),
		TextXAlignment = Enum.TextXAlignment.Right,
		TextColor3 = COLORS.text_accent,
		TextSize = 11,
	})

	-- Slider track
	local track = Instance.new("Frame")
	track.Name = "Track"
	track.Position = UDim2.fromOffset(0, 22)
	track.Size = UDim2.new(1, 0, 0, 14)
	track.BackgroundColor3 = COLORS.slider_bg
	track.BorderSizePixel = 0
	track.Parent = frame
	makeCorner(track, 3)
	makeStroke(track, COLORS.stroke_dim, 0.7)

	-- Fill bar
	local fill = Instance.new("Frame")
	fill.Name = "Fill"
	local fraction = if max > min then (current - min) / (max - min) else 0
	fill.Size = UDim2.new(math.clamp(fraction, 0, 1), 0, 1, 0)
	fill.BackgroundColor3 = COLORS.slider_fill
	fill.BorderSizePixel = 0
	fill.Parent = track
	makeCorner(fill, 3)

	-- Knob
	local knob = Instance.new("Frame")
	knob.Name = "Knob"
	knob.Size = UDim2.fromOffset(12, 12)
	knob.Position = UDim2.new(math.clamp(fraction, 0, 1), -6, 0.5, -6)
	knob.BackgroundColor3 = COLORS.text
	knob.BorderSizePixel = 0
	knob.ZIndex = 3
	knob.Parent = track
	makeCorner(knob, 6)

	-- Interaction
	local dragging = false

	local function updateFromPosition(x)
		local absPos = track.AbsolutePosition.X
		local absSize = track.AbsoluteSize.X
		local t = math.clamp((x - absPos) / absSize, 0, 1)
		local raw = min + t * (max - min)
		-- Snap to step
		local snapped = math.floor(raw / step + 0.5) * step
		snapped = math.clamp(snapped, min, max)
		if snapped ~= current then
			current = snapped
			valueLabel.Text = string.format("%.2f", current)
			fill.Size = UDim2.new(t, 0, 1, 0)
			knob.Position = UDim2.new(t, -6, 0.5, -6)
			onChange(current)
		end
	end

	track.InputBegan:Connect(function(input)
		if input.UserInputType == Enum.UserInputType.MouseButton1
			or input.UserInputType == Enum.UserInputType.Touch then
			dragging = true
			updateFromPosition(input.Position.X)
		end
	end)

	UserInputService.InputChanged:Connect(function(input)
		if dragging and (input.UserInputType == Enum.UserInputType.MouseMovement
			or input.UserInputType == Enum.UserInputType.Touch) then
			updateFromPosition(input.Position.X)
		end
	end)

	UserInputService.InputEnded:Connect(function(input)
		if dragging and (input.UserInputType == Enum.UserInputType.MouseButton1
			or input.UserInputType == Enum.UserInputType.Touch) then
			dragging = false
		end
	end)

	track.InputBegan:Connect(function(input)
		if input.UserInputType == Enum.UserInputType.MouseButton1 then
			dragging = true
		end
	end)

	return frame
end

-- ============================================================
-- Toggle widget
-- ============================================================

local function createToggle(parent, label, current, onChange)
	local frame = Instance.new("Frame")
	frame.Size = UDim2.new(1, 0, 0, 30)
	frame.BackgroundTransparency = 1
	frame.Parent = parent

	makeText(frame, "Label", {
		Position = UDim2.fromOffset(0, 0),
		Size = UDim2.new(0.7, 0, 1, 0),
		Text = label,
		TextColor3 = COLORS.text_dim,
		TextSize = 11,
	})

	local btn = Instance.new("TextButton")
	btn.Name = "Toggle"
	btn.Size = UDim2.fromOffset(50, 22)
	btn.Position = UDim2.new(1, -55, 0.5, -11)
	btn.BackgroundColor3 = if current then COLORS.toggle_on else COLORS.toggle_off
	btn.Text = if current then "ON" else "OFF"
	btn.TextColor3 = COLORS.text
	btn.Font = FONT
	btn.TextSize = 10
	btn.BorderSizePixel = 0
	btn.Parent = frame
	makeCorner(btn, 4)

	btn.MouseButton1Click:Connect(function()
		current = not current
		btn.BackgroundColor3 = if current then COLORS.toggle_on else COLORS.toggle_off
		btn.Text = if current then "ON" else "OFF"
		onChange(current)
	end)

	return frame
end

-- ============================================================
-- Dropdown widget
-- ============================================================

local function createDropdown(parent, label, options, current, onChange)
	local frame = Instance.new("Frame")
	frame.Size = UDim2.new(1, 0, 0, 34)
	frame.BackgroundTransparency = 1
	frame.Parent = parent

	makeText(frame, "Label", {
		Position = UDim2.fromOffset(0, 2),
		Size = UDim2.new(0.4, 0, 0, 16),
		Text = label,
		TextColor3 = COLORS.text_dim,
		TextSize = 11,
	})

	local btn = makeButton(frame, "Value", {
		Position = UDim2.new(0.42, 0, 0, 2),
		Size = UDim2.new(0.56, 0, 0, 22),
		Text = string.upper(current or options[1] or ""),
		TextColor3 = COLORS.text_accent,
		TextSize = 10,
	})

	local optionIndex = 1
	for i, opt in options do
		if opt == current then optionIndex = i break end
	end

	btn.MouseButton1Click:Connect(function()
		optionIndex = (optionIndex % #options) + 1
		btn.Text = string.upper(options[optionIndex])
		onChange(options[optionIndex])
	end)

	return frame
end

-- ============================================================
-- Snapshot value resolution
-- ============================================================

local function resolveCurrentValue(snapshot, categoryKey, paramName, wheelId)
	if not snapshot then return nil end
	-- Per-wheel: check "suspension_FL" etc.
	if wheelId then
		local perWheel = snapshot[categoryKey .. "_" .. wheelId]
		if perWheel and perWheel[paramName] ~= nil then
			return perWheel[paramName]
		end
	end
	-- Shared: check "engine", "suspension", etc.
	local catData = snapshot[categoryKey]
	if catData and catData[paramName] ~= nil then
		return catData[paramName]
	end
	return nil
end

-- ============================================================
-- Category builder
-- ============================================================

local function buildCategoryUI(scrollFrame, category, categoryKey, manifest, configurator, snapshot)
	-- Section header
	local header = makeText(scrollFrame, "Header_" .. categoryKey, {
		Size = UDim2.new(1, 0, 0, 24),
		Text = "// " .. string.upper(categoryKey),
		TextColor3 = COLORS.text_accent,
		TextSize = 13,
		LayoutOrder = #scrollFrame:GetChildren() * 10,
	})
	local sep = Instance.new("Frame")
	sep.Size = UDim2.new(1, 0, 0, 1)
	sep.BackgroundColor3 = COLORS.stroke_dim
	sep.BorderSizePixel = 0
	sep.LayoutOrder = #scrollFrame:GetChildren() * 10 + 1
	sep.Parent = scrollFrame

	local function fireChange(param, value, wheelId)
		ConfigChangeEvent:FireServer({
			action = "change",
			category = categoryKey,
			param = param,
			value = value,
			wheelId = wheelId,
		})
	end

	-- Per-wheel sections
	if category.per_wheel and category.per_wheel_params then
		for _, wheelId in { "FL", "FR", "RL", "RR" } do
			local wheelParams = category.per_wheel_params[wheelId]
			if wheelParams then
				-- Wheel header
				local wh = makeText(scrollFrame, "Wheel_" .. wheelId, {
					Size = UDim2.new(1, 0, 0, 18),
					Text = "  " .. wheelId,
					TextColor3 = COLORS.text,
					TextSize = 11,
					LayoutOrder = #scrollFrame:GetChildren() * 10,
				})
				for paramName, paramDef in wheelParams do
					if paramDef.min ~= nil then
						local current = resolveCurrentValue(snapshot, categoryKey, paramName, wheelId) or paramDef.default or paramDef.min
						createSlider(scrollFrame, paramName, paramDef.min, paramDef.max, paramDef.step, current, function(val)
							fireChange(paramName, val, wheelId)
						end)
					end
				end
			end
		end
	end

	-- Shared params (non-per-wheel)
	if category.params then
		for paramName, paramDef in category.params do
			if paramDef.min ~= nil then
				-- Numeric slider
				local current = resolveCurrentValue(snapshot, categoryKey, paramName, nil) or paramDef.default or paramDef.min
				createSlider(scrollFrame, paramName, paramDef.min, paramDef.max, paramDef.step, current, function(val)
					fireChange(paramName, val, nil)
				end)
			elseif paramDef.options then
				-- Dropdown or toggle
				if #paramDef.options == 2 and type(paramDef.options[1]) == "boolean" then
					-- Boolean toggle
					local current = resolveCurrentValue(snapshot, categoryKey, paramName, nil)
					createToggle(scrollFrame, paramName, if current ~= nil then current else paramDef.default == true, function(val)
						fireChange(paramName, val, nil)
					end)
				else
					-- Dropdown
					local current = resolveCurrentValue(snapshot, categoryKey, paramName, nil) or paramDef.default
					createDropdown(scrollFrame, paramName, paramDef.options, current, function(val)
						fireChange(paramName, val, nil)
					end)
				end
			end
		end
	end

	-- Shared suspension params
	if category.shared_params then
		for paramName, paramDef in category.shared_params do
			if paramDef.min ~= nil then
				local current = resolveCurrentValue(snapshot, categoryKey, paramName, nil) or paramDef.default or paramDef.min
				createSlider(scrollFrame, paramName, paramDef.min, paramDef.max, paramDef.step, current, function(val)
					fireChange(paramName, val, nil)
				end)
			end
		end
	end

	-- Reset category button
	local resetBtn = makeButton(scrollFrame, "Reset_" .. categoryKey, {
		Size = UDim2.new(1, 0, 0, 22),
		BackgroundColor3 = COLORS.reset_btn,
		Text = "RESET " .. string.upper(categoryKey),
		TextColor3 = COLORS.text,
		TextSize = 10,
		LayoutOrder = #scrollFrame:GetChildren() * 10,
	})
	resetBtn.MouseButton1Click:Connect(function()
		ConfigChangeEvent:FireServer({
			action = "reset_category",
			category = categoryKey,
		})
	end)
end

-- ============================================================
-- Main panel
-- ============================================================

local function createConfiguratorPanel()
	local player_gui = local_player:WaitForChild("PlayerGui")

	-- Create the ScreenGui
	local gui = Instance.new("ScreenGui")
	gui.Name = "KemudiConfigurator"
	gui.ResetOnSpawn = false
	gui.IgnoreGuiInset = true
	gui.DisplayOrder = 30  -- Above the HUD (20)
	gui.Enabled = false
	gui.Parent = player_gui

	-- Main panel
	local panel = Instance.new("Frame")
	panel.Name = "Panel"
	panel.Size = UDim2.fromOffset(PANEL_WIDTH, PANEL_HEIGHT)
	panel.Position = UDim2.new(0.5, -PANEL_WIDTH / 2, 0.5, -PANEL_HEIGHT / 2)
	panel.BackgroundColor3 = COLORS.bg
	panel.BackgroundTransparency = 0.05
	panel.BorderSizePixel = 0
	panel.Parent = gui
	makeCorner(panel, 8)
	makeStroke(panel, COLORS.stroke, 0.25)

	-- Title bar
	local titleBar = Instance.new("Frame")
	titleBar.Name = "TitleBar"
	titleBar.Size = UDim2.new(1, 0, 0, 36)
	titleBar.BackgroundColor3 = COLORS.bg_active
	titleBar.BackgroundTransparency = 0.3
	titleBar.BorderSizePixel = 0
	titleBar.Parent = panel
	makeCorner(titleBar, 8)

	makeText(titleBar, "Title", {
		Position = UDim2.fromOffset(12, 8),
		Size = UDim2.new(0.7, 0, 0, 20),
		Text = "VEHICLE CONFIGURATOR",
		TextSize = 14,
		TextColor3 = COLORS.text,
	})

	-- Close button
	local closeBtn = makeButton(titleBar, "Close", {
		Position = UDim2.new(1, -30, 0, 8),
		Size = UDim2.fromOffset(20, 20),
		Text = "X",
		TextSize = 12,
		BackgroundColor3 = COLORS.reset_btn,
	})
	closeBtn.MouseButton1Click:Connect(function()
		gui.Enabled = false
	end)

	-- Reset All button
	local resetAllBtn = makeButton(titleBar, "ResetAll", {
		Position = UDim2.new(1, -100, 0, 8),
		Size = UDim2.fromOffset(64, 20),
		Text = "RESET ALL",
		TextSize = 9,
		BackgroundColor3 = COLORS.reset_btn,
	})
	resetAllBtn.MouseButton1Click:Connect(function()
		ConfigChangeEvent:FireServer({ action = "reset_all" })
	end)

	-- Scrollable content
	local scroll = Instance.new("ScrollingFrame")
	scroll.Name = "Content"
	scroll.Position = UDim2.fromOffset(10, 44)
	scroll.Size = UDim2.new(1, -20, 1, -52)
	scroll.BackgroundTransparency = 1
	scroll.BorderSizePixel = 0
	scroll.ScrollBarThickness = 4
	scroll.ScrollBarImageColor3 = COLORS.stroke
	scroll.CanvasSize = UDim2.new(0, 0, 0, 0)  -- Auto-size via UIListLayout
	scroll.Parent = panel

	local layout = Instance.new("UIListLayout")
	layout.Padding = UDim.new(0, 4)
	layout.SortOrder = Enum.SortOrder.LayoutOrder
	layout.Parent = scroll

	-- Auto-resize canvas
	layout:GetPropertyChangedSignal("AbsoluteContentSize"):Connect(function()
		scroll.CanvasSize = UDim2.new(0, 0, 0, layout.AbsoluteContentSize.Y + 8)
	end)

	-- Status bar
	local statusBar = Instance.new("Frame")
	statusBar.Name = "StatusBar"
	statusBar.Size = UDim2.new(1, 0, 0, 20)
	statusBar.Position = UDim2.new(0, 0, 1, -20)
	statusBar.BackgroundColor3 = COLORS.bg
	statusBar.BackgroundTransparency = 0.5
	statusBar.BorderSizePixel = 0
	statusBar.Parent = panel

	local statusLabel = makeText(statusBar, "Status", {
		Position = UDim2.fromOffset(10, 2),
		Size = UDim2.new(1, -20, 0, 16),
		Text = "NO VEHICLE LOADED",
		TextSize = 9,
		TextColor3 = COLORS.text_dim,
	})

	return gui, scroll, statusLabel
end

-- ============================================================
-- Public API
-- ============================================================

local activeGui = nil
local activeScroll = nil
local activeStatus = nil
local currentManifest = nil
local last_snapshot = nil  -- last received snapshot for rebuild on changed events

--- Initialize the configurator UI.
--- Call once from the client script.
function UI.init()
	activeGui, activeScroll, activeStatus = createConfiguratorPanel()

	-- Toggle with Z key
	UserInputService.InputBegan:Connect(function(input, processed)
		if processed then return end
		if input.KeyCode == TOGGLE_KEY then
			UI.toggle()
		end
	end)

	-- Receive manifest from server
	ConfigManifestEvent.OnClientEvent:Connect(function(manifest)
		currentManifest = manifest
		rebuildUI(manifest)
		activeStatus.Text = "CONFIGURATOR READY // PRESS Z TO OPEN"
		activeStatus.TextColor3 = COLORS.text_accent
	end)

	-- Receive config changes from server
	ConfigSyncEvent.OnClientEvent:Connect(function(data)
		if data.action == "error" then
			activeStatus.Text = "ERROR // " .. (data.error or "unknown")
			activeStatus.TextColor3 = COLORS.text_error
			task.delay(3, function()
				if activeStatus then
					activeStatus.Text = "CONFIGURATOR ACTIVE"
					activeStatus.TextColor3 = COLORS.text_dim
				end
			end)
		elseif data.action == "full_sync" then
			-- Full snapshot: update all displayed values
			-- (For now, rebuild the UI. Future: update in-place.)
			if currentManifest and data.snapshot then
				applySnapshotToUI(data.snapshot)
			end
		elseif data.action == "changed" then
			-- Update local snapshot copy and rebuild UI
			if last_snapshot then
				local catKey = data.category
				if data.wheelId then catKey = data.category .. "_" .. data.wheelId end
				if not last_snapshot[catKey] then last_snapshot[catKey] = {} end
				last_snapshot[catKey][data.param] = data.value
			end
			applySnapshotToUI(nil)  -- rebuild with updated local snapshot
		end
	end)
end

--- Toggle the configurator panel visibility.
--- Called from the HUD button or the Z key binding.
function UI.toggle()
	if activeGui then
		activeGui.Enabled = not activeGui.Enabled
		if activeGui.Enabled and currentManifest then
			ConfigChangeEvent:FireServer({ action = "request_snapshot" })
		end
	end
end

--- Rebuild the scroll content from a manifest, optionally with snapshot values.
function rebuildUI(manifest, snapshot)
	-- Clear existing content
	for _, child in activeScroll:GetChildren() do
		if not child:IsA("UIListLayout") then
			child:Destroy()
		end
	end

	-- Build each category
	for categoryKey, category in manifest.categories do
		buildCategoryUI(activeScroll, category, categoryKey, manifest, nil, snapshot)
	end
end

--- Apply a snapshot to update displayed values.
--- Rebuilds the UI with correct current values from the server.
function applySnapshotToUI(snapshot)
	if snapshot then last_snapshot = snapshot end
	if currentManifest and last_snapshot then
		rebuildUI(currentManifest, last_snapshot)
	end
end

return UI
