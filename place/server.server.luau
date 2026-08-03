-- ServerScriptService bootstrap for the Rojo-built place.
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local engine = ReplicatedStorage:WaitForChild("KemudiEngine")
local ok, err = pcall(function()
	require(engine:WaitForChild("runtime"):WaitForChild("server"))
end)
if not ok then
	warn("[KemudiServer] bootstrap failed: " .. tostring(err))
	error(err, 0)
end
