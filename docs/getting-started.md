# Getting started

Kemudi.js is currently in early development. This guide covers the repository workflow and the production-oriented expectations for adding a feature.

## Prerequisites

Install:

- Bun for JavaScript dependencies and scripts.
- Node.js 20+ for Nuxt and server tooling.
- Rust with the `wasm32-unknown-unknown` target.
- `wasm-pack` for the physics package.
- A WebGL 2 and WebAssembly-capable browser.

Verify the toolchain before making changes:

```bash
bun --version
node --version
rustc --version
rustup target list --installed
wasm-pack --version
```

## Install and run

```bash
bun install
bun run dev
```

Use the URL printed by Nuxt. The browser console should remain free of unexpected errors. Development diagnostics use scoped `console.debug` events; production diagnostics are disabled by default.

## Build like production

```bash
bun run build
bun run preview
```

Always test important browser behavior against the built bundle as well as the development server. Differences in bundling, asset paths, feature detection, and runtime configuration can hide production-only failures.

## Development workflow

1. Read the relevant architecture guide and implementation plan.
2. Locate existing symbols and neighboring tests before editing.
3. Define the acceptance criteria and failure behavior.
4. Write or update tests before implementation where practical.
5. Keep physics, rendering, networking, and content loading behind their existing boundaries.
6. Run focused checks, then the full applicable CI commands.
7. Verify the exact runtime boundary in a browser using the built artifact.
8. Update documentation when a public contract, setting, schema, or command changes.

## Common checks

The project will expose all canonical commands through `package.json`. The intended baseline is:

```bash
bun run lint
bun run typecheck
bun run test
bun run test:integration
bun run test:e2e
bun run build
```

If a command is not yet present, do not silently invent a replacement in CI. Add the script and document its behavior first.
