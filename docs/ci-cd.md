# CI/CD and releases

Kemudi.js uses a fail-closed pipeline. A pull request cannot merge when required quality, test, build, security, E2E, or performance checks fail.

## Pipeline stages

1. Validate Bun/Node/Rust versions, lockfile integrity, repository hygiene, and generated-file expectations.
2. Run formatting, ESLint, TypeScript/Vue checks, Rust formatting, and Clippy.
3. Run Vitest with coverage thresholds, Rust tests, server integration tests, and schema/mod-loader tests.
4. Build the Nuxt production bundle and WASM artifacts using the deployment commands.
5. Run dependency/license audits and secret scanning without exposing credentials.
6. Start the production bundle and run Playwright smoke tests on pull requests.
7. Run the complete browser E2E matrix on protected-branch merges and release candidates.
8. Run deterministic performance checks and compare frame-time/memory reports with the baseline.
9. Create immutable artifacts with build metadata and provenance/SBOM information.
10. Deploy to staging, run smoke checks, then promote to production only after approval and passing gates.

The workflow should live in `.github/workflows/ci.yml`; release, security, and scheduled performance workflows may be split out but must reuse the same scripts and production build path.

## Release safety

- Protect release branches and require reviewed pull requests.
- Keep deployment identifiers, test reports, traces, build metadata, and artifact checksums.
- Use environment-based configuration; never commit credentials or production secrets.
- Maintain a documented rollback procedure and test it before public release.
- Do not publish artifacts from a failed, dirty, or unverified build.

## Local preflight

Before opening a pull request, run the relevant local checks:

```bash
bun run lint
bun run typecheck
bun run test
bun run test:integration
bun run test:e2e
bun run build
```

If a script is not yet implemented, add it to `package.json` as part of the corresponding feature rather than weakening the CI contract.
