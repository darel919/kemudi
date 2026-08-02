local Constraints = require("../src/physics/constraints")
local Math = require("../src/math")

local World = {}
Constraints.install(World, Math)

assert(type(World._solveXPBD) == "function", "constraints installer must attach _solveXPBD")
assert(type(World._solveVolumePreservation) == "function", "constraints installer must attach _solveVolumePreservation")
assert(type(World._solveBodyAttachmentConstraints) == "function", "constraints installer must attach body attachment solver")

print("constraints module load test passed")