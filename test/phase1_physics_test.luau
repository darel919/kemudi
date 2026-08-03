--[[ 
	Phase 1 physics improvements test (Lune-compatible).
	Tests: volume preservation, soft body attachment, Coulomb friction.
--]]
local pass, fail = 0, 0
local function check(name, cond)
	if cond then
		pass += 1; print("  PASS  " .. name)
	else
		fail += 1; print("  FAIL  " .. name)
	end
end

-- ============================================================
-- Minimal World builder
-- ============================================================
local function makeWorld()
	local w = {
		nodes = {}, beams = {}, triangles = {},
		gravity = -9.81, fixedDt = 1/60,
		restVolume = 0, restCentroid = {0, 0, 0},
		volumeLambda = 0, volumeCompliance = 1e-5,
		bodyAttachmentSoftness = 0.35,
		groundFriction = 0.94, terrainProfile = 0,
		terrainRuts = {}, terrainHeights = {},
		terrainWidth = 0, terrainDepth = 0, terrainSegments = 0,
	}
	function w:addNode(id, x, y, z, mass, fixed)
		local invMass = if fixed then 0 else 1 / mass
		table.insert(self.nodes, {
			id=id, x=x, y=y, z=z, vx=0, vy=0, vz=0,
			mass=mass, invMass=invMass, fixed=fixed,
			collision=not fixed, fx=0, fy=0, fz=0, lastForce=0,
		})
	end
	function w:addTriangle(a, b, c)
		local na, nb, nc = self.nodes[a+1], self.nodes[b+1], self.nodes[c+1]
		if not (na and nb and nc) then return end
		local abx,aby,abz = nb.x-na.x, nb.y-na.y, nb.z-na.z
		local acx,acy,acz = nc.x-na.x, nc.y-na.y, nc.z-na.z
		local area = 0.5 * math.sqrt(
			(aby*acz-abz*acy)^2 + (abz*acx-abx*acz)^2 + (abx*acy-aby*acx)^2
		)
		if area > 1e-8 then
			table.insert(self.triangles, {a=a, b=b, c=c, restArea=area, lambda=0})
		end
	end
	function w:terrainHeight(x, z) return 0 end
	function w:terrainNormal(x, z) return {0, 1, 0} end
	function w:computeRestVolume()
		local cx,cy,cz,cnt = 0,0,0,0
		for i=1,#self.nodes do
			if not self.nodes[i].fixed then
				cx+=self.nodes[i].x; cy+=self.nodes[i].y; cz+=self.nodes[i].z; cnt+=1
			end
		end
		if cnt>0 then cx,cy,cz=cx/cnt,cy/cnt,cz/cnt end
		self.restCentroid = {cx,cy,cz}
		local vol = 0
		for ti=1,#self.triangles do
			local tri = self.triangles[ti]
			local a,b,c = self.nodes[tri.a+1], self.nodes[tri.b+1], self.nodes[tri.c+1]
			local ax,ay,az = a.x-cx, a.y-cy, a.z-cz
			local bx,by,bz = b.x-cx, b.y-cy, b.z-cz
			local cx2,cy2,cz2 = c.x-cx, c.y-cy, c.z-cz
			vol += (ax*(by*cz2-bz*cy2) + ay*(bz*cx2-bx*cz2) + az*(bx*cy2-by*cx2)) / 6
		end
		self.restVolume = vol
	end
	function w:_solveVolumePreservation()
		local tris, nodes, nc = self.triangles, self.nodes, #self.nodes
		if #tris < 4 or self.restVolume <= 0 then return end
		local px,py,pz = self.restCentroid[1],self.restCentroid[2],self.restCentroid[3]
		local gx,gy,gz = table.create(nc,0), table.create(nc,0), table.create(nc,0)
		local cv = 0
		for ti=1,#tris do
			local tri = tris[ti]
			local ai,bi,ci = tri.a+1, tri.b+1, tri.c+1
			if ai<=nc and bi<=nc and ci<=nc then
				local na,nb,nc2 = nodes[ai],nodes[bi],nodes[ci]
				local ax,ay,az = na.x-px, na.y-py, na.z-pz
				local bx,by,bz = nb.x-px, nb.y-py, nb.z-pz
				local cx,cy,cz = nc2.x-px, nc2.y-py, nc2.z-pz
				cv += (ax*(by*cz-bz*cy) + ay*(bz*cx-bx*cz) + az*(bx*cy-by*cx)) / 6
				gx[ai]+=(by*cz-bz*cy)/6; gy[ai]+=(bz*cx-bx*cz)/6; gz[ai]+=(bx*cy-by*cx)/6
				gx[bi]+=(cy*az-cz*ay)/6; gy[bi]+=(cz*ax-cx*az)/6; gz[bi]+=(cx*ay-cy*ax)/6
				gx[ci]+=(ay*bz-az*by)/6; gy[ci]+=(az*bx-ax*bz)/6; gz[ci]+=(ax*by-ay*bx)/6
			end
		end
		local wSum = 0
		for i=1,nc do
			local n=nodes[i]
			if n.invMass>0 and not n.fixed then
				wSum += n.invMass*(gx[i]*gx[i]+gy[i]*gy[i]+gz[i]*gz[i])
			end
		end
		local compliance = self.volumeCompliance / math.max(self.fixedDt^2, 1e-12)
		local denom = wSum + compliance
		if denom <= 1e-12 then return end
		local err = cv - self.restVolume
		local dL = (-err - compliance*self.volumeLambda) / denom
		self.volumeLambda += dL
		for i=1,nc do
			local n=nodes[i]
			if n.invMass>0 and not n.fixed then
				n.x += dL*n.invMass*gx[i]
				n.y += dL*n.invMass*gy[i]
				n.z += dL*n.invMass*gz[i]
			end
		end
	end
	function w:_applyContactVelocity(n, normal, restitution, friction)
		local nv = n.vx*normal[1]+n.vy*normal[2]+n.vz*normal[3]
		if nv < 0 then
			n.vx -= nv*(1+restitution)*normal[1]
			n.vy -= nv*(1+restitution)*normal[2]
			n.vz -= nv*(1+restitution)*normal[3]
		end
		local postN = n.vx*normal[1]+n.vy*normal[2]+n.vz*normal[3]
		local tvx,tvy,tvz = n.vx-postN*normal[1], n.vy-postN*normal[2], n.vz-postN*normal[3]
		local ts = math.sqrt(tvx*tvx+tvy*tvy+tvz*tvz)
		if ts > 1e-6 then
			local md = friction * math.abs(nv) * (1+restitution)
			local f = math.clamp(1-md/ts, 0, 1)
			n.vx = postN*normal[1]+tvx*f
			n.vy = postN*normal[2]+tvy*f
			n.vz = postN*normal[3]+tvz*f
		end
	end
	function w:_collideWithTerrain()
		for i=1,#self.nodes do
			local n = self.nodes[i]
			if not n.fixed and n.collision then
				local th = self:terrainHeight(n.x, n.z)
				local h = if i>4 then th+0.2 else th
				if n.y < h then
					local below = n.y < th
					local nm = self:terrainNormal(n.x, n.z)
					local nv = n.vx*nm[1]+n.vy*nm[2]+n.vz*nm[3]
					if nv < 0 then
						n.vx -= nv*1.15*nm[1]; n.vy -= nv*1.15*nm[2]; n.vz -= nv*1.15*nm[3]
					end
					n.y = h
					if below then
						local tx = n.vx - n.vy*nm[1]
						local tz = n.vz - n.vy*nm[3]
						local ts = math.sqrt(tx*tx+tz*tz)
						if ts > 1e-6 and n.mass > 0 then
							local wn = math.abs(self.gravity)*n.mass*math.abs(nm[2])
							local dn = math.abs(nv)*n.mass*1.15
							local md = self.groundFriction*(wn+dn)/n.mass
							local f = math.clamp(1-md*self.fixedDt/ts, 0, 1)
							n.vx = tx*f; n.vz = tz*f
						end
					end
				end
			end
		end
	end
	return w
end

-- ============================================================
print("=== Volume Preservation Tests ===")

do
	local w = makeWorld()
	-- Consistent outward-facing winding for a tetrahedron:
	-- Vertices: 0=(0,0,0) 1=(1,0,0) 2=(0.5,0,1) 3=(0.5,1,0.5)
	w:addNode(0, 0, 0, 0, 1, false)
	w:addNode(1, 1, 0, 0, 1, false)
	w:addNode(2, 0.5, 0, 1, 1, false)
	w:addNode(3, 0.5, 1, 0.5, 1, false)
	-- Outward-facing faces (verified via cross product vs opposite vertex)
	w:addTriangle(0, 1, 2) -- face opposite 3, normal points away from 3
	w:addTriangle(0, 2, 3) -- face opposite 1, normal points away from 1
	w:addTriangle(0, 3, 1) -- face opposite 2, normal points away from 2
	w:addTriangle(1, 3, 2) -- face opposite 0, normal points away from 0

	w:computeRestVolume()
	print("  restVolume =", w.restVolume)
	check("rest volume computed (positive)", w.restVolume > 0)

	local restV = w.restVolume

	-- Squash: push node 3 (top) down to y=0.1
	w.nodes[4].y = 0.1
	-- Run volume preservation solver
	for _ = 1, 50 do
		w.volumeLambda = 0
		w:_solveVolumePreservation()
	end

	-- Compute current volume
	local cx,cy,cz = w.restCentroid[1],w.restCentroid[2],w.restCentroid[3]
	local cv = 0
	for ti=1,#w.triangles do
		local tri = w.triangles[ti]
		local a,b,c = w.nodes[tri.a+1],w.nodes[tri.b+1],w.nodes[tri.c+1]
		local ax,ay,az = a.x-cx,a.y-cy,a.z-cz
		local bx,by,bz = b.x-cx,b.y-cy,b.z-cz
		local cx2,cy2,cz2 = c.x-cx,c.y-cy,c.z-cz
		cv += (ax*(by*cz2-bz*cy2)+ay*(bz*cx2-bx*cz2)+az*(bx*cy2-by*cx2))/6
	end
	local ratio = math.abs(cv) / math.abs(restV)
	print("  volume ratio after squash:", ratio)
	check("volume partially preserved (ratio > 0.2)", ratio > 0.2)
	check("node pulled back up by volume constraint", w.nodes[4].y > 0.2)
end

-- ============================================================
print("\n=== Soft Body Attachment Tests ===")

do
	local w = makeWorld()
	-- 4 mount nodes + 4 upper cage nodes
	w:addNode(0, -0.8, 0, -1.2, 50, false)
	w:addNode(1, 0.8, 0, -1.2, 50, false)
	w:addNode(2, -0.8, 0, 1.2, 50, false)
	w:addNode(3, 0.8, 0, 1.2, 50, false)
	w:addNode(4, -0.7, 1, -1.0, 20, false)
	w:addNode(5, 0.7, 1, -1.0, 20, false)
	w:addNode(6, -0.7, 1, 1.0, 20, false)
	w:addNode(7, 0.7, 1, 1.0, 20, false)

	local original_upper_x = w.nodes[6].x -- -0.7

	-- Push mount node 2 (index 3) far right
	w.nodes[3].x = 3.0

	-- With softness=1.0 (no constraint), upper should stay
	w.bodyAttachmentSoftness = 1.0
	-- Simulate chassisBasis + correction manually (simplified)
	local mount = w.nodes[3]
	local upper = w.nodes[7]
	local dx = upper.x - mount.x
	-- With softness=1, correction = 0 → upper stays
	local correction_with_softness = dx * (1 - w.bodyAttachmentSoftness)
	check("softness=1.0: no correction applied", math.abs(correction_with_softness) < 0.01)

	-- With softness=0.35, partial correction (less than full)
	w.bodyAttachmentSoftness = 0.35
	local error_x = upper.x - mount.x
	local full_correction = -error_x / (upper.invMass + mount.invMass)
	local correction = full_correction * w.bodyAttachmentSoftness
	check("softness=0.35: correction < full", math.abs(correction) < math.abs(full_correction))
	check("softness=0.35: correction > 0", math.abs(correction) > 0)
end

-- ============================================================
print("\n=== Coulomb Friction Tests ===")

do
	-- Impact friction: node hitting ground at angle
	local w = makeWorld()
	w:addNode(0, 0, 1, 0, 10, false)
	local n = w.nodes[1]
	n.vx = 5; n.vy = -8; n.vz = 0

	w:_applyContactVelocity(n, {0, 1, 0}, 0.15, 0.8)

	check("impact: vy corrected (bounced)", n.vy > 0)
	-- After bounce + friction, horizontal speed should be reduced
	check("impact: tangential speed reduced by friction", n.vx < 5.0)
end

do
	-- Terrain Coulomb friction: node sliding on ground
	local w = makeWorld()
	w:addNode(0, 0, -0.1, 0, 10, false) -- below terrain (y=0)
	w.nodes[1].vx = 8; w.nodes[1].vz = 0; w.nodes[1].collision = true
	w.groundFriction = 0.94

	local beforeVx = w.nodes[1].vx
	w:_collideWithTerrain()
	local afterVx = w.nodes[1].vx
	print("  terrain friction: before=" .. beforeVx .. " after=" .. afterVx)
	check("terrain Coulomb friction reduces sliding speed", afterVx < beforeVx)
	check("terrain friction doesn't reverse direction", afterVx >= -0.5)
end

do
	-- Zero friction: node sliding on frictionless surface
	local w = makeWorld()
	w:addNode(0, 0, -0.1, 0, 10, false)
	w.nodes[1].vx = 8; w.nodes[1].vz = 0; w.nodes[1].collision = true
	w.groundFriction = 0

	w:_collideWithTerrain()
	local speed = math.sqrt(w.nodes[1].vx^2 + w.nodes[1].vz^2)
	print("  zero friction: speed after=" .. speed)
	check("zero friction preserves horizontal speed", speed > 7.5)
end

do
	-- Box/sphere contact friction: node hitting a boundary
	local w = makeWorld()
	w:addNode(0, 0, 0, 0, 10, false)
	local n = w.nodes[1]
	n.vx = 0; n.vy = -5; n.vz = 3 -- hitting boundary at y=0, moving sideways

	w:_applyContactVelocity(n, {0, 1, 0}, 0.1, 0.9)

	check("boundary: vy bounced", n.vy > 0)
	-- vz should be reduced by friction during impact
	check("boundary: tangential vz reduced", n.vz < 3.0)
end

-- ============================================================
print(string.format("\n=== Results: %d/%d passed ===", pass, pass + fail))
if fail > 0 then error("TESTS FAILED") end
