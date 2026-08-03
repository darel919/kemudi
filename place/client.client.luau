-- StarterPlayerScripts bootstrap for the Rojo-built place.
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local engine = ReplicatedStorage:WaitForChild("KemudiEngine")
local ok, err = pcall(function()
	require(engine:WaitForChild("runtime"):WaitForChild("client"))
end)
if not ok then
	warn("[KemudiClient] bootstrap failed: " .. tostring(err))
	error(err, 0)
end
