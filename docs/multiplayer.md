# Multiplayer

The multiplayer design uses a server-authoritative WebSocket session with local prediction for responsive controls and reconciliation for correctness.

## Message rules

Every message needs a type, protocol version, bounded size, timestamp or sequence, and validated fields. The server validates ownership, ranges, rate limits, and room membership. Payloads must not be logged in full.

Prefer compact input messages and delta/snapshot updates. Do not broadcast unchanged state. All queues and prediction histories must be bounded.

## Client flow

1. Connect and negotiate protocol capabilities.
2. Join a room and receive the initial authoritative state.
3. Apply local input immediately to the local prediction.
4. Send timestamped/sequence-numbered input to the server.
5. Reconcile when an authoritative state differs from the prediction.
6. Interpolate remote vehicles between snapshots.
7. Drop stale snapshots safely and recover from disconnects.

Correction should be smooth for small errors and explicit for large errors. The implementation must avoid unbounded history growth and must not apply a snapshot twice.

## Reliability

Test connect, join/leave, reconnect, timeout, malformed messages, oversized messages, rate limiting, ownership changes, stale snapshots, and server failure. UI should show connection state without freezing the simulation or leaking internal payloads.

## Performance

Snapshot frequency, interpolation buffers, and bandwidth budgets may be preset-aware. Authoritative simulation remains independent of visual quality. Collect sampled `console.debug` events for connection transitions and aggregate queue/latency warnings only when diagnostics are enabled.
