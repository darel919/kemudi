# Kemudi map building

`tools/build_map.py` converts a georeferenced DEM GeoTIFF into a Kemudi map
package. The package is the shared content boundary for the Rust/WASM engine,
three.js renderer, and Luau/Roblox adapter.

The builder does **not** download DEM files. Obtain a GeoTIFF separately, then
pass its path to the builder.

## Requirements

Install GDAL once on macOS:

```bash
brew install gdal
```

The input must be a georeferenced elevation GeoTIFF. The builder reads the
source coordinate system, reprojects the DEM into a local metric CRS, and
resamples it into a square heightmap.

## Build a map

From the repository root:

```bash
python3 tools/build_map.py \
  assets/maps/output_hh.tif \
  --name hh \
  --out assets/maps/hh \
  --target-crs auto \
  --segments 256 \
  --force
```

For another DEM, change the source, name, and output directory:

```bash
python3 tools/build_map.py \
  /path/to/my-dem.tif \
  --name mountain-pass \
  --out assets/maps/mountain-pass \
  --target-crs auto \
  --segments 256 \
  --force
```

`--target-crs auto` selects the appropriate UTM zone from the DEM's geographic
coordinates. Pass an explicit CRS when automatic selection is unsuitable, for
example:

```bash
--target-crs EPSG:32647
```

`--segments 256` creates a `257 × 257` sample grid. Increase it for more terrain
detail or reduce it for a small prototype. Valid values are 8 through 2048.

`--force` allows the builder to replace files in an existing output directory.
The source DEM itself is never overwritten.

## Generated package

A successful build creates:

```text
assets/maps/<name>/
├── manifest.json          # map package entry point
├── terrain.tif            # projected/resampled DEM
├── heightmap.f32le        # row-major little-endian Float32 heights
├── heightmap.json         # same heightmap, convenient for Luau prototypes
├── terrain.obj            # simple terrain render mesh
├── surfaces.json          # starter terrain material
├── collision.json         # four map-edge boundary planes
├── ATTRIBUTION.md         # source/license record to complete
├── README.md              # package-local summary
└── source/
    └── <original-dem>    # preserved input copy

assets/maps/roblox/<name>/
├── Map.luau               # Roblox ModuleScript entry point
└── heightmap_###.luau     # generated chunks required by Map.luau
```

The generated local coordinate system is:

- `X`: east
- `Y`: elevation/up
- `Z`: south
- physics/render origin: center of the map
- source CRS origin: recorded separately in `manifest.json`

`manifest.json` contains the projected CRS, map dimensions in meters, elevation
range, file names, spawn position, and conversion information.

## What to do after building

### Rust/WASM and three.js

Use `manifest.json` to discover the map dimensions and load
`heightmap.f32le` as a little-endian `Float32Array` for terrain queries. Use
`terrain.obj` as the initial render mesh, or generate a three.js mesh from the
same heightmap.

The binary heightmap is the runtime-oriented format. `heightmap.json` is useful
for early debugging and Roblox content import, but is larger and slower to
parse.

### Luau/Roblox

Roblox cannot read a repository JSON file or parse a GeoTIFF by itself. The
builder therefore generates a Roblox-native `ModuleScript` as well:

```text
assets/maps/roblox/Duri/Map.luau
assets/maps/roblox/Duri/heightmap_000.luau
assets/maps/roblox/Duri/heightmap_001.luau
...
```

The `Default.project.json` Rojo project already maps `assets/maps/roblox` to:

```text
ServerStorage.KemudiMaps
```

#### Import into Studio

1. Run the map builder from the repository root.
2. Start Rojo:

   ```bash
   rojo serve Default.project.json
   ```

3. Open the place in Roblox Studio.
4. Use the Rojo Studio plugin to connect to the running Rojo server.
5. In Explorer, expand:

   ```text
   ServerStorage
   └── KemudiMaps
       └── Duri
           ├── Map
           └── heightmap_000, heightmap_001, ...
   ```

There is no JSON upload and no GeoTIFF upload. Rojo turns the generated `.luau`
files into Roblox `ModuleScript` instances.

#### Read the map from Luau

A server script can require the map entry point:

```lua
local ServerStorage = game:GetService("ServerStorage")
local map = require(ServerStorage.KemudiMaps.Duri.Map)

local heightmap = map.heightmap
world.terrainHeights = heightmap.heights
world.terrainSegments = heightmap.samplesX - 1
world.terrainWidth = heightmap.widthMeters
world.terrainDepth = heightmap.depthMeters
```

The current Kemudi server loads all valid map modules into a server-owned registry,
returns only safe map metadata to the client, and validates the selected map name
again when the player presses **START DRIVING**. The selected map’s heightmap is
then passed into that player’s vehicle runtime.

#### Main menu flow

After Rojo syncs the project, the client shows `KemudiMainMenu` in `PlayerGui`:

1. The menu calls the server’s `KemudiMapList` `RemoteFunction`.
2. The server returns map names, dimensions, and sample counts—not heightmap data.
3. The player selects a map and presses **START DRIVING**.
4. The client sends only the selected map name through `KemudiSpawn`.
5. The server looks up that name in its registry, converts the map’s authored
   spawn point from meters into Roblox studs, moves the character there, and
   spawns the vehicle with the corresponding map data.

When at least one valid generated map is installed, the server disables the
procedural GroundZero fallback before the play session begins. This prevents
flat GroundZero chunks and authored GroundZero colliders from covering the
selected generated map. GroundZero remains available only as the fallback when
no generated map package is installed.

The custom Kemudi physics solver uses the selected map heightmap for terrain
contact, while the imported/generated map geometry supplies the visible terrain.

The character’s `CharacterAdded` path remains bound to the selected map spawn,
so a later respawn does not return the player to the GroundZero `SpawnLocation`.

The client cannot upload a map, choose an uninstalled map, or provide physics
configuration. Add another generated map by placing its `Map.luau` folder under
`assets/maps/roblox`; it will appear in the menu after restarting/reconnecting
Rojo and restarting the play session.

The generated `heightmap.json` remains useful for inspection and external tools,
but it is not the Roblox runtime import. Use the generated `Map.luau` module.

#### Import the visible terrain

`Map.luau` supplies the physics heightmap; it does not create visible Roblox
geometry. To see the terrain in Studio:

1. In Roblox Studio, use **File → 3D Importer**.
2. Select `assets/maps/duri/terrain.obj` (or the `<name>` output directory you used).
3. In `Workspace`, create or use `Maps/<name>`—for Duri, `Workspace.Maps.Duri`—and
   put the imported terrain Model or MeshPart inside that folder.
4. Place the imported geometry at the map origin.
5. Anchor it and set `CanCollide = false` for the initial prototype.

The server checks for visible BaseParts under `Workspace.Maps.<name>`. If none are
present, it keeps GroundZero as a temporary visual fallback instead of leaving the
player and vehicle in empty space. Once the Duri geometry is placed there, restart
Play mode and the server will disable GroundZero automatically.

The custom Kemudi physics solver uses the heightmap for terrain contact. Do not
try to parse `terrain.tif`, `heightmap.f32le`, or GeoTIFF data from the Roblox
server. A future streaming adapter can replace the single OBJ with tiled terrain
geometry.

### Runtime map integration (Roblox)

When the game starts, the server registry validates every installed map. If a
valid generated map exists, the server **disables the procedural GroundZero
fallback** so flat chunks do not cover the selected terrain.

When a player selects a map and presses **START DRIVING**:

1. The server looks up the map, loads its heightmap into the vehicle physics
   runtime, and reads the vehicle asset's `StudsPerMeter` value.
2. The imported visual terrain under `Workspace.Maps.<name>` is **automatically
   scaled** from meters to Roblox studs using that same `StudsPerMeter`. The
   scale is applied to the Model/MeshPart's size and position, and its pivot is
   preserved.
3. After scaling, the server **forces the terrain appearance** to solid opaque
   slate material with shadows enabled (`Transparency = 0`, `Material =
   Enum.Material.Slate`, `CastShadow = true`). This ensures the OBJ renders
   correctly even if the import produced transparent or single-sided geometry.
4. The player character is moved to the map's authored spawn point (converted
   to studs) and a `CharacterAdded` handler keeps future respawns on the same
   map.
5. The vehicle is spawned at the identical map position with the same
   orientation.

The server logs the scaling:

```text
[KemudiEngine] scaled Duri visual geometry by 3.5714 studs per meter
```

If `Workspace.Maps.<name>` has no visible BaseParts when the server starts, it
keeps GroundZero as a temporary visual fallback instead of leaving the player in
empty space. Place the imported terrain there and restart Play mode to activate
the generated map visuals.

The `Map.luau` module supplies only the physics heightmap; it does not create
visible Roblox geometry. The OBJ import and `Workspace.Maps.<name>` placement
remain a manual Studio step.

### Roads, scenery, and props

A DEM supplies elevation only. It does not contain roads, road materials,
buildings, trees, guardrails, or other scenery. Add those as separate authored
map content after the terrain package is generated. The starter
`surfaces.json` and `collision.json` provide the initial physics-side boundary
and material contracts.

## Attribution and licensing

Before redistributing a generated map:

1. Open the OpenTopography dataset page used to obtain the DEM.
2. Record the underlying provider, dataset name, source URL or DOI, license,
   and download date in `assets/maps/<name>/ATTRIBUTION.md`.
3. Preserve any required attribution or license text alongside the map.

The GeoTIFF's coordinate metadata does not by itself determine the dataset's
redistribution license. The generated `ATTRIBUTION.md` intentionally contains a
TODO until the actual source terms are recorded.

## Validation

The builder validates the output dimensions and writes all related artifacts in
one operation. A quick manual check is:

```bash
gdalinfo -stats assets/maps/<name>/terrain.tif
```

The reported raster size should be `segments + 1` by `segments + 1`, and the
minimum/maximum elevation should be plausible for the selected area.
