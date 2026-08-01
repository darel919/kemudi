# Debugging

Kemudi.js uses structured, gated `console.debug` diagnostics to make runtime failures diagnosable without introducing production noise or hot-path overhead.

## Logging rules

- Use stable scope/event names such as `graphics:preset-selected` or `physics-worker:initialized`.
- Gate diagnostics behind development mode and an explicit diagnostics setting.
- Sample or throttle repeated events.
- Log lifecycle transitions and aggregate timings, not every node or frame.
- Redact credentials, tokens, URLs with secrets, and full network payloads.
- Preserve and surface the root exception; do not replace it with a generic message.

Useful events include capability detection, preset selection, renderer initialization, worker start/failure, frame-budget warnings, WebSocket state changes, and content-load failures.

## Runtime-boundary verification

When debugging a bug:

1. Reproduce it in the browser using the actual UI action.
2. Record the visible UI state and the exact failed control or transition.
3. Inspect the root console exception and relevant scoped debug events.
4. Inspect failed network requests and worker messages.
5. Rebuild the production artifact if the issue may be bundling or configuration related.
6. Repeat the same UI action against the rebuilt artifact.
7. Verify that the fix works and that no new console errors or resource leaks appear.

A passing unit test or production build does not prove that the browser runtime boundary is healthy.

## Common failure areas

### Renderer startup

Check WebGL 2 support, context creation, canvas sizing, device pixel ratio, asset paths, and whether the selected preset is applied before renderer creation.

### Physics worker

Check WASM initialization, worker error events, message ordering, transferable buffers, finite-value guards, and shutdown/restart handling. Avoid calling the solver from the main thread as a workaround.

### Performance

Capture p50/p95 frame time, physics step time, memory, queue depth, and network rate. Determine whether the bottleneck is UI, renderer, worker, or transport before changing settings.

### Multiplayer

Check connection state, protocol version, sequence ordering, validation failures, stale snapshots, reconnect behavior, and server-side rate limits without printing payload contents.
