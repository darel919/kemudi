#!/usr/bin/env python3
"""Build a Kemudi map package from a DEM GeoTIFF.

The input remains the source artifact. The generated package contains:

- terrain.tif: projected/resampled DEM for inspection
- heightmap.f32le: row-major little-endian Float32 terrain heights
- heightmap.json: serialized heightmap for inspection/external tools
- assets/maps/roblox/<name>/: Roblox ModuleScript map import split into chunks
- terrain.obj: simple renderable terrain mesh for three.js/Blender/Roblox 3D Importer
- manifest.json: dimensions, coordinate conversion, spawn, and data paths
- surfaces.json: starter surface material definition
- collision.json: map boundary colliders
- ATTRIBUTION.md: source/license record to complete before distribution

Requires GDAL command-line tools: gdalinfo, gdalwarp, and gdal_translate.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import struct
import subprocess
import sys
import tempfile
from collections import deque
from pathlib import Path
from typing import Any


class MapBuildError(RuntimeError):
    pass


def run(command: list[str], *, capture: bool = True) -> str:
    try:
        result = subprocess.run(
            command,
            check=True,
            text=True,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise MapBuildError(
            f"Missing required command '{command[0]}'. Install GDAL with: brew install gdal"
        ) from exc
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or "").strip()
        raise MapBuildError(
            f"Command failed ({exc.returncode}): {' '.join(command)}\n{detail}"
        ) from exc
    return result.stdout if capture else ""


def gdal_json(path: Path) -> dict[str, Any]:
    raw = run(["gdalinfo", "-json", str(path)])
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise MapBuildError(f"gdalinfo returned invalid JSON for {path}") from exc


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def infer_utm_crs(info: dict[str, Any]) -> str:
    corners = info.get("cornerCoordinates", {})
    center = corners.get("center")
    if not isinstance(center, list) or len(center) < 2 or not all(finite(v) for v in center[:2]):
        raise MapBuildError("Cannot infer UTM zone; pass --target-crs explicitly")
    lon, lat = float(center[0]), float(center[1])
    if not (-180 <= lon <= 180 and -90 <= lat <= 90):
        raise MapBuildError("--target-crs auto requires a geographic input DEM")
    zone = int(math.floor((lon + 180) / 6) + 1)
    zone = max(1, min(zone, 60))
    return f"EPSG:{32600 + zone if lat >= 0 else 32700 + zone}"


def parse_xyz(path: Path, samples_x: int, samples_z: int) -> list[float]:
    values: list[float] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            fields = line.split()
            if len(fields) < 3:
                continue
            try:
                value = float(fields[2])
            except ValueError as exc:
                raise MapBuildError(f"Invalid elevation at XYZ line {line_number}") from exc
            values.append(value)
    expected = samples_x * samples_z
    if len(values) != expected:
        raise MapBuildError(
            f"Expected {expected} resampled elevations, received {len(values)} from GDAL"
        )
    return values


def fill_nodata(values: list[float], samples_x: int, samples_z: int, nodata: float) -> tuple[list[float], int]:
    valid = [
        index
        for index, value in enumerate(values)
        if math.isfinite(value) and not math.isclose(value, nodata, rel_tol=0.0, abs_tol=1e-6)
    ]
    if not valid:
        raise MapBuildError("The DEM contains no valid elevation samples")
    missing = [
        index
        for index, value in enumerate(values)
        if not math.isfinite(value) or math.isclose(value, nodata, rel_tol=0.0, abs_tol=1e-6)
    ]
    if not missing:
        return values, 0

    # Propagate the nearest valid value through invalid regions. This keeps
    # coastlines/holes finite without inventing a large artificial pit.
    nearest = [-1] * len(values)
    distance = [-1] * len(values)
    queue: deque[int] = deque()
    for index in valid:
        nearest[index] = index
        distance[index] = 0
        queue.append(index)
    while queue:
        current = queue.popleft()
        row, col = divmod(current, samples_x)
        for nr, nc in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if 0 <= nr < samples_z and 0 <= nc < samples_x:
                neighbor = nr * samples_x + nc
                if distance[neighbor] == -1:
                    distance[neighbor] = distance[current] + 1
                    nearest[neighbor] = nearest[current]
                    queue.append(neighbor)
    for index in missing:
        values[index] = values[nearest[index]]
    return values, len(missing)


def projected_bounds(info: dict[str, Any]) -> tuple[float, float, float, float]:
    corners = info.get("cornerCoordinates", {})
    upper_left = corners.get("upperLeft")
    lower_right = corners.get("lowerRight")
    if not (
        isinstance(upper_left, list)
        and isinstance(lower_right, list)
        and len(upper_left) >= 2
        and len(lower_right) >= 2
        and all(finite(v) for v in upper_left[:2] + lower_right[:2])
    ):
        raise MapBuildError("Projected DEM has no usable corner coordinates")
    west, north = float(upper_left[0]), float(upper_left[1])
    east, south = float(lower_right[0]), float(lower_right[1])
    if east <= west or north <= south:
        raise MapBuildError(f"Invalid projected DEM bounds: {upper_left} to {lower_right}")
    return west, south, east, north


def write_obj(path: Path, values: list[float], samples_x: int, samples_z: int, width: float, depth: float) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Kemudi DEM terrain mesh\n")
        for row in range(samples_z):
            z = depth * (row / (samples_z - 1) - 0.5)
            for col in range(samples_x):
                x = width * (col / (samples_x - 1) - 0.5)
                y = values[row * samples_x + col]
                handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for row in range(samples_z - 1):
            for col in range(samples_x - 1):
                a = row * samples_x + col + 1
                b = a + 1
                c = a + samples_x + 1
                d = a + samples_x
                handle.write(f"f {a} {b} {c} {d}\n")


def write_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, allow_nan=False)
        handle.write("\n")


def lua_number(value: float) -> str:
    text = format(float(value), ".9g")
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return text


def write_luau_array(path: Path, values: list[float]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("-- Generated by tools/build_map.py; do not edit.\nreturn {\n")
        for value in values:
            handle.write(f"    {lua_number(value)},\n")
        handle.write("}\n")


def write_roblox_module(
    root: Path,
    name: str,
    values: list[float],
    samples: int,
    width: float,
    depth: float,
    spawn: dict[str, float],
    boundaries: list[dict[str, list[float]]],
    target_crs: str,
    force: bool,
) -> Path:
    map_dir = root / name
    if map_dir.exists() and any(map_dir.iterdir()) and not force:
        raise MapBuildError(f"Roblox map directory is not empty; pass --force: {map_dir}")
    if map_dir.exists() and force:
        shutil.rmtree(map_dir)
    map_dir.mkdir(parents=True, exist_ok=True)

    chunk_size = 2048
    chunk_names = []
    for start in range(0, len(values), chunk_size):
        chunk_name = f"heightmap_{start // chunk_size:03d}"
        chunk_names.append(chunk_name)
        write_luau_array(map_dir / f"{chunk_name}.luau", values[start : start + chunk_size])

    with (map_dir / "Map.luau").open("w", encoding="utf-8") as handle:
        handle.write("-- Generated by tools/build_map.py; do not edit.\n")
        handle.write("local heights = table.create(%d)\n" % len(values))
        for chunk_name in chunk_names:
            handle.write(f"for _, value in require(script.Parent.{chunk_name}) do\n")
            handle.write("    table.insert(heights, value)\n")
            handle.write("end\n")
        handle.write("\nreturn {\n")
        handle.write("    schemaVersion = 1,\n")
        handle.write(f"    name = {json.dumps(name)},\n")
        handle.write(f"    projectedCrs = {json.dumps(target_crs)},\n")
        handle.write("    coordinateSystem = { localAxes = \"X=east, Y=up, Z=south\" },\n")
        handle.write("    heightmap = {\n")
        handle.write(f"        samplesX = {samples},\n        samplesZ = {samples},\n")
        handle.write(f"        widthMeters = {lua_number(width)},\n        depthMeters = {lua_number(depth)},\n")
        handle.write("        heights = heights,\n    },\n")
        handle.write("    spawn = {\n")
        handle.write(f"        x = {lua_number(spawn['x'])}, y = {lua_number(spawn['y'])}, z = {lua_number(spawn['z'])},\n")
        handle.write(f"        headingRadians = {lua_number(spawn['headingRadians'])},\n    }},\n")
        handle.write("    boundaries = {\n")
        for boundary in boundaries:
            point = ", ".join(lua_number(v) for v in boundary["point"])
            normal = ", ".join(lua_number(v) for v in boundary["normal"])
            handle.write(f"        {{ point = {{ {point} }}, normal = {{ {normal} }} }},\n")
        handle.write("    },\n")
        handle.write("    surfaces = {\n")
        handle.write("        { id = \"default\", friction = 0.94, roughness = 0.65, moisture = 0.0, compactness = 1.0 },\n")
        handle.write("    },\n")
        handle.write("}\n")
    return map_dir


def build(args: argparse.Namespace) -> Path:
    source = args.source.resolve()
    if not source.is_file():
        raise MapBuildError(f"DEM not found: {source}")
    output = args.output.resolve()
    if output == source or source.is_relative_to(output):
        raise MapBuildError("Output directory must not contain the source DEM")
    if output.exists() and any(output.iterdir()) and not args.force:
        raise MapBuildError(f"Output directory is not empty; pass --force: {output}")
    output.mkdir(parents=True, exist_ok=True)

    source_info = gdal_json(source)
    target_crs = args.target_crs
    if target_crs == "auto":
        target_crs = infer_utm_crs(source_info)

    samples = args.segments + 1
    projected_copy = output / "terrain.tif"
    with tempfile.TemporaryDirectory(prefix="kemudi-map-") as temporary:
        projected = Path(temporary) / "terrain_projected.tif"
        xyz = Path(temporary) / "heightmap.xyz"
        run([
            "gdalwarp",
            "-overwrite",
            "-t_srs",
            target_crs,
            "-r",
            "bilinear",
            "-ts",
            str(samples),
            str(samples),
            "-dstnodata",
            str(args.nodata),
            "-of",
            "GTiff",
            "-co",
            "TILED=YES",
            "-co",
            "COMPRESS=LZW",
            str(source),
            str(projected),
        ], capture=False)
        projected_info = gdal_json(projected)
        run(["gdal_translate", "-of", "XYZ", str(projected), str(xyz)], capture=False)
        values = parse_xyz(xyz, samples, samples)
        shutil.copy2(projected, projected_copy)

    values, filled_count = fill_nodata(values, samples, samples, args.nodata)
    west, south, east, north = projected_bounds(projected_info)
    width = east - west
    depth = north - south
    minimum = min(values)
    maximum = max(values)
    center_height = values[(samples // 2) * samples + samples // 2]

    source_dir = output / "source"
    source_dir.mkdir(exist_ok=True)
    shutil.copy2(source, source_dir / source.name)

    with (output / "heightmap.f32le").open("wb") as handle:
        handle.write(struct.pack("<" + "f" * len(values), *values))

    heightmap = {
        "schemaVersion": 1,
        "samplesX": samples,
        "samplesZ": samples,
        "widthMeters": width,
        "depthMeters": depth,
        "heights": values,
    }
    write_json(output / "heightmap.json", heightmap)
    write_obj(output / "terrain.obj", values, samples, samples, width, depth)

    write_json(output / "surfaces.json", {
        "schemaVersion": 1,
        "surfaces": [{
            "id": "default",
            "name": "Default terrain",
            "friction": 0.94,
            "roughness": 0.65,
            "moisture": 0.0,
            "compactness": 1.0,
        }],
    })
    boundaries = [
        {"point": [-width * 0.5, 0.0, 0.0], "normal": [1.0, 0.0, 0.0]},
        {"point": [width * 0.5, 0.0, 0.0], "normal": [-1.0, 0.0, 0.0]},
        {"point": [0.0, 0.0, -depth * 0.5], "normal": [0.0, 0.0, 1.0]},
        {"point": [0.0, 0.0, depth * 0.5], "normal": [0.0, 0.0, -1.0]},
    ]
    write_json(output / "collision.json", {
        "schemaVersion": 1,
        "boundaries": boundaries,
        "boxes": [],
        "spheres": [],
    })
    spawn = {
        "x": 0.0,
        "y": center_height + 2.0,
        "z": 0.0,
        "headingRadians": 0.0,
    }
    roblox_root = (args.roblox_out or (output.parent / "roblox")).resolve()
    roblox_map_dir = write_roblox_module(
        roblox_root,
        args.name,
        values,
        samples,
        width,
        depth,
        spawn,
        boundaries,
        target_crs,
        args.force,
    )
    write_json(output / "manifest.json", {
        "schemaVersion": 1,
        "contractVersion": "0.1",
        "name": args.name,
        "type": "heightmap-terrain",
        "coordinateSystem": {
            "sourceCrs": "EPSG:4326",
            "projectedCrs": target_crs,
            "localAxes": "X=east, Y=up, Z=south",
            "origin": [west, north],
        },
        "dimensions": {
            "samplesX": samples,
            "samplesZ": samples,
            "widthMeters": width,
            "depthMeters": depth,
            "minElevationMeters": minimum,
            "maxElevationMeters": maximum,
        },
        "files": {
            "heightmapBinary": "heightmap.f32le",
            "heightmapJson": "heightmap.json",
            "renderMesh": "terrain.obj",
            "surfaces": "surfaces.json",
            "collision": "collision.json",
            "robloxModule": f"{roblox_map_dir.relative_to(output.parent.parent)}/Map.luau" if output.parent.parent in roblox_map_dir.parents else str(roblox_map_dir / "Map.luau"),
        },
        "spawn": spawn,
        "conversion": {
            "resampling": "bilinear",
            "nodataValue": args.nodata,
            "filledNodataSamples": filled_count,
        },
        "source": {
            "file": f"source/{source.name}",
            "license": "TODO: record the DEM provider and license before distribution",
        },
    })
    (output / "ATTRIBUTION.md").write_text(
        "# Map attribution\n\n"
        f"Map: {args.name}\n"
        f"Source file: {source.name}\n\n"
        "Complete this file with the OpenTopography dataset/provider, source URL or DOI,\n"
        "license, and download date before redistributing the map.\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        f"# {args.name}\n\n"
        "Generated by `tools/build_map.py` from a DEM GeoTIFF.\n\n"
        "- `manifest.json` is the map entry point.\n"
        "- `heightmap.f32le` is the runtime heightmap for Rust/WASM.\n"
        "- `heightmap.json` is convenient for inspecting the map data.\n"
        "- `terrain.obj` is a simple render mesh for three.js/Blender/Roblox 3D Importer.\n"
        "- `assets/maps/roblox/<name>/Map.luau` is the Roblox ModuleScript import.\n"
        "- `surfaces.json` and `collision.json` are starter map physics data.\n\n"
        "The DEM alone is terrain; authored roads, props, and scenery can be added\n"
        "to this package later.\n",
        encoding="utf-8",
    )
    print(f"Built map: {output}")
    print(f"  grid: {samples} x {samples}")
    print(f"  size: {width:.1f}m x {depth:.1f}m")
    print(f"  elevation: {minimum:.2f}m .. {maximum:.2f}m")
    print(f"  target CRS: {target_crs}")
    print(f"  filled NoData samples: {filled_count}")
    print(f"  Roblox ModuleScript: {roblox_map_dir / 'Map.luau'}")
    return output


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("source", type=Path, help="Input DEM GeoTIFF")
    result.add_argument("--name", required=True, help="Map identifier")
    result.add_argument("--out", dest="output", type=Path, required=True, help="Output map directory")
    result.add_argument("--roblox-out", type=Path, help="Roblox ModuleScript root (default: sibling roblox directory)")
    result.add_argument("--segments", type=int, default=256, help="Terrain segments per axis (default: 256)")
    result.add_argument("--target-crs", default="auto", help="Projected CRS, e.g. EPSG:32647, or auto (default)")
    result.add_argument("--nodata", type=float, default=-9999.0, help="NoData marker used during conversion")
    result.add_argument("--force", action="store_true", help="Replace files in a non-empty output directory")
    return result


def main() -> int:
    args = parser().parse_args()
    if args.segments < 8 or args.segments > 2048:
        print("error: --segments must be between 8 and 2048", file=sys.stderr)
        return 2
    try:
        build(args)
    except MapBuildError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
