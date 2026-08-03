-- Main menu for selecting a server-validated map before vehicle spawn.
local MainMenu = {}

local function addCorner(instance, radius)
	local corner = Instance.new("UICorner")
	corner.CornerRadius = UDim.new(0, radius)
	corner.Parent = instance
end

local function addStroke(instance, color, transparency)
	local stroke = Instance.new("UIStroke")
	stroke.Color = color
	stroke.Transparency = transparency or 0
	stroke.Thickness = 1
	stroke.Parent = instance
end

local function makeLabel(parent, name, text, size, position, textSize)
	local label = Instance.new("TextLabel")
	label.Name = name
	label.Text = text
	label.Size = size
	label.Position = position
	label.BackgroundTransparency = 1
	label.TextColor3 = Color3.fromRGB(235, 245, 255)
	label.Font = Enum.Font.Code
	label.TextSize = textSize
	label.TextXAlignment = Enum.TextXAlignment.Left
	label.TextYAlignment = Enum.TextYAlignment.Center
	label.Parent = parent
	return label
end

function MainMenu.init(config)
	assert(type(config) == "table", "MainMenu.init requires a config table")
	assert(config.playerGui, "MainMenu.init requires playerGui")
	assert(config.mapListFunction, "MainMenu.init requires mapListFunction")
	assert(type(config.onStart) == "function", "MainMenu.init requires onStart")

	local existing = config.playerGui:FindFirstChild("KemudiMainMenu")
	if existing then existing:Destroy() end

	local gui = Instance.new("ScreenGui")
	gui.Name = "KemudiMainMenu"
	gui.ResetOnSpawn = false
	gui.IgnoreGuiInset = true
	gui.DisplayOrder = 100
	gui.Parent = config.playerGui

	local backdrop = Instance.new("Frame")
	backdrop.Name = "Backdrop"
	backdrop.Size = UDim2.fromScale(1, 1)
	backdrop.BackgroundColor3 = Color3.fromRGB(5, 9, 15)
	backdrop.BackgroundTransparency = 0.08
	backdrop.BorderSizePixel = 0
	backdrop.Parent = gui

	local panel = Instance.new("Frame")
	panel.Name = "Panel"
	panel.Size = UDim2.fromOffset(560, 520)
	panel.Position = UDim2.fromScale(0.5, 0.5)
	panel.AnchorPoint = Vector2.new(0.5, 0.5)
	panel.BackgroundColor3 = Color3.fromRGB(12, 20, 31)
	panel.BorderSizePixel = 0
	panel.Parent = backdrop
	addCorner(panel, 12)
	addStroke(panel, Color3.fromRGB(80, 170, 220), 0.2)

	makeLabel(panel, "Title", "KEMUDI", UDim2.fromOffset(500, 54), UDim2.fromOffset(30, 24), 36)
	local subtitle = makeLabel(panel, "Subtitle", "SOFT-BODY DRIVING SIMULATOR", UDim2.fromOffset(500, 24), UDim2.fromOffset(32, 76), 13)
	subtitle.TextColor3 = Color3.fromRGB(130, 190, 220)
	makeLabel(panel, "MapHeading", "SELECT MAP", UDim2.fromOffset(500, 24), UDim2.fromOffset(32, 123), 15)

	local status = makeLabel(panel, "Status", "Loading maps...", UDim2.fromOffset(500, 24), UDim2.fromOffset(32, 150), 13)
	status.TextColor3 = Color3.fromRGB(170, 190, 205)

	local list = Instance.new("ScrollingFrame")
	list.Name = "MapList"
	list.Size = UDim2.fromOffset(496, 190)
	list.Position = UDim2.fromOffset(32, 182)
	list.BackgroundColor3 = Color3.fromRGB(8, 14, 22)
	list.BackgroundTransparency = 0.2
	list.BorderSizePixel = 0
	list.ScrollBarThickness = 6
	list.CanvasSize = UDim2.fromOffset(0, 0)
	list.Parent = panel
	addCorner(list, 8)

	local layout = Instance.new("UIListLayout")
	layout.Padding = UDim.new(0, 8)
	layout.SortOrder = Enum.SortOrder.LayoutOrder
	layout.Parent = list

	local selectedLabel = makeLabel(panel, "Selected", "SELECTED: NONE", UDim2.fromOffset(300, 24), UDim2.fromOffset(32, 388), 13)
	selectedLabel.TextColor3 = Color3.fromRGB(130, 220, 170)

	local startButton = Instance.new("TextButton")
	startButton.Name = "StartButton"
	startButton.Text = "START DRIVING"
	startButton.Size = UDim2.fromOffset(210, 48)
	startButton.Position = UDim2.fromOffset(318, 430)
	startButton.BackgroundColor3 = Color3.fromRGB(34, 145, 103)
	startButton.TextColor3 = Color3.fromRGB(245, 255, 250)
	startButton.Font = Enum.Font.Code
	startButton.TextSize = 16
	startButton.AutoButtonColor = true
	startButton.Active = false
	startButton.Parent = panel
	addCorner(startButton, 8)

	local selectedMap = nil
	local selectedButton = nil

	local function selectMap(mapInfo, button)
		selectedMap = mapInfo
		if selectedButton then
			selectedButton.BackgroundColor3 = Color3.fromRGB(18, 31, 45)
		end
		selectedButton = button
		selectedButton.BackgroundColor3 = Color3.fromRGB(35, 91, 116)
		selectedLabel.Text = "SELECTED: " .. mapInfo.name
		startButton.Active = true
		startButton.BackgroundColor3 = Color3.fromRGB(34, 145, 103)
	end

	local function addMapButton(mapInfo, index)
		local button = Instance.new("TextButton")
		button.Name = "Map_" .. tostring(index)
		button.LayoutOrder = index
		button.Size = UDim2.new(1, -16, 0, 50)
		button.BackgroundColor3 = Color3.fromRGB(18, 31, 45)
		button.BorderSizePixel = 0
		button.Text = ""
		button.AutoButtonColor = true
		button.Parent = list
		addCorner(button, 6)

		local title = makeLabel(button, "Name", mapInfo.name, UDim2.new(1, -24, 0, 23), UDim2.fromOffset(12, 5), 16)
		title.TextColor3 = Color3.fromRGB(235, 245, 255)
		local dimensions = string.format(
			"%.1f m x %.1f m  //  %d x %d samples",
			mapInfo.widthMeters or 0,
			mapInfo.depthMeters or 0,
			mapInfo.samplesX or 0,
			mapInfo.samplesZ or 0
		)
		local detail = makeLabel(button, "Details", dimensions, UDim2.new(1, -24, 0, 18), UDim2.fromOffset(12, 28), 11)
		detail.TextColor3 = Color3.fromRGB(145, 175, 195)

		button.Activated:Connect(function()
			selectMap(mapInfo, button)
		end)
		return button
	end

	local function renderMaps(mapInfos)
		for _, child in ipairs(list:GetChildren()) do
			if child:IsA("TextButton") then child:Destroy() end
		end
		selectedMap = nil
		selectedButton = nil
		if type(mapInfos) ~= "table" or #mapInfos == 0 then
			status.Text = "No generated maps installed. GroundZero is available."
			local fallbackInfo = {
				name = "GroundZero",
				widthMeters = 0,
				depthMeters = 0,
				samplesX = 0,
				samplesZ = 0,
			}
			local button = addMapButton(fallbackInfo, 1)
			selectMap(fallbackInfo, button)
			return
		end

		status.Text = tostring(#mapInfos) .. " map(s) available"
		for index, mapInfo in ipairs(mapInfos) do
			if type(mapInfo) == "table" and type(mapInfo.name) == "string" then
				local button = addMapButton(mapInfo, index)
				if not selectedMap then selectMap(mapInfo, button) end
			end
		end
		list.CanvasSize = UDim2.fromOffset(0, layout.AbsoluteContentSize.Y + 12)
	end

	startButton.Activated:Connect(function()
		if not selectedMap then return end
		startButton.Active = false
		startButton.Text = "LOADING MAP..."
		status.Text = "Server is loading " .. selectedMap.name .. "..."
		gui.Enabled = false
		config.onStart(selectedMap.name)
	end)

	local ok, mapInfos = pcall(function()
		return config.mapListFunction:InvokeServer()
	end)
	if ok then
		renderMaps(mapInfos)
	else
		status.Text = "Unable to load maps from the server."
		warn("[KemudiMainMenu] map list request failed: " .. tostring(mapInfos))
	end

	return gui
end

return MainMenu
