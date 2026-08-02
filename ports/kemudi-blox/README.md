# Roblox port

The Roblox/Luau port has not been implemented yet. This directory is reserved for an independent implementation of the public engine contract.

The contract is documented in [`spec/README.md`](../../spec/README.md). The current Rust behavior is described in [`docs/physics.md`](../../docs/physics.md), and the compatibility data format is documented in [`fixtures/engine/README.md`](../../fixtures/engine/README.md).

The port will use Roblox-specific runtime and replication code; it will not load Rust, WASM, Nuxt, or browser-worker code.
