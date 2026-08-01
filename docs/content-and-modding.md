# Content and modding

Maps, vehicles, models, and skins are planned as addable content rather than hard-coded application features. The content pipeline must be versioned from the beginning so future mods do not require unsafe runtime coupling.

## Planned package structure

A mod package may eventually contain:

```text
mod.json
maps/
vehicles/
models/
skins/
```

The exact archive format and hosting mechanism are still open, but the manifest should declare a schema version, mod ID, compatibility range, content types, entry points, dependencies, and checksums.

## Validation and safety

The loader must enforce:

- Manifest and content schema validation.
- Supported version and compatibility checks.
- Maximum archive, file, and decompressed sizes.
- Safe relative paths with path traversal rejected.
- Allowed content types and asset references.
- Checksums or integrity metadata where content is downloaded.
- Bounded parsing and clear cleanup on failure.

A bad mod should be rejected without crashing the active session or leaving partial GPU/worker resources.

## Maps

Map definitions should be versioned and reference heightmaps, terrain materials, collision data, and optional environment assets through stable IDs. Runtime loading should support validation before activation and a safe fallback map if activation fails.

## Vehicles and models

Vehicle definitions contain physics data and references to visual models. A model must never be trusted to provide gameplay behavior directly. Physics definitions, visual assets, and scripts (if scripts are ever introduced) must remain separate, validated capabilities.

## Compatibility

When a schema changes, provide explicit migration or a clear unsupported-version error. Never guess how to interpret unknown fields that affect physics or security. Add tests for valid, invalid, oversized, incompatible, and partially missing packages.
