# Engine compatibility fixtures

Fixtures are small JSON scenarios used to compare engine implementations. They describe initial state, controls, elapsed time, and observable assertions without depending on Rust, Luau, browser, or Roblox APIs.

## Format

Each fixture contains:

- `schemaVersion`: fixture format version;
- `contractVersion`: version from [`spec/README.md`](../../spec/README.md);
- `name`: stable fixture name;
- `initial`: world and vehicle state;
- `steps`: elapsed time and controls in order;
- `assertions`: expected observable results.

The units are the ones defined by the engine contract. Numeric comparisons carry an explicit absolute tolerance when exact equality is not appropriate.

The initial assertion operators are:

| Operator | Meaning |
| --- | --- |
| `equals` | Exact value comparison |
| `approximatelyEquals` | Numeric comparison within `absoluteTolerance` |
| `lessThan` / `greaterThan` | Numeric bound |
| `finite` | Value is neither NaN nor infinite |
| `arrayLength` | Fixed-width array check |

## Available fixture

[`single-node-gravity.json`](./single-node-gravity.json) starts one five-kilogram node at `[1, 10, 3]` with zero velocity and advances it for one `1/60 s` step. It records the expected downward velocity and position using the default `-9.81 m/s²` gravity.

The fixture runner is not connected to CI yet. The Rust unit test suite currently covers the reference behavior; a Luau runner will consume the same JSON files when the Roblox port is implemented.
