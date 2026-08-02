-- StarterPlayerScripts bootstrap for the Rojo-built place.
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local engine = ReplicatedStorage:WaitForChild("KemudiEngine")
require(engine:WaitForChild("runtime"):WaitForChild("client"))
