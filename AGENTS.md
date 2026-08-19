# LinkageSimulator Agent Guidance

## Repository Purpose

`LinkageSimulator` is a static browser-based planar linkage simulator intended for designing and evaluating servo-driven rotational linkage systems that interact with a simplified index-finger model.

The application is built with HTML, CSS, and TypeScript and is deployed as a static GitHub Pages site.

Keep the implementation lightweight, deterministic, and understandable. Do not introduce a backend or general-purpose physics engine unless explicitly requested.

## Project Structure

Maintain the existing separation of concerns:

```text
src/
├── css/
│   └── main.css
└── ts/
    ├── main.ts
    ├── geometry/
    ├── model/
    ├── rendering/
    ├── simulation/
    └── ui/
```

Responsibilities:

* `geometry/`: pure 2-D geometry and mathematical utilities.
* `model/`: data structures describing links, joints, hand geometry, contactors, and application state.
* `simulation/`: constraint solving, servo actuation, kinematic updates, and mechanism validity.
* `rendering/`: Canvas rendering and coordinate transforms only.
* `ui/`: selection, inspectors, controls, pointer interactions, and radial/context menus.
* `main.ts`: application initialization and the top-level animation loop.

Do not put simulation mathematics into rendering or UI code.

Geometry and simulation modules must remain independent of DOM and Canvas APIs.

## Coordinate and Simulation Conventions

Use a meaningful world-coordinate system, preferably millimeter-like units.

Keep world coordinates separate from screen/canvas coordinates.

Transforms between world and screen space belong in the rendering/viewport layer.

Prefer deterministic planar kinematics and analytical geometry over rigid-body dynamics.

Use explicit geometry result states for invalid, unreachable, tangent, degenerate, or ambiguous configurations. Do not allow unsatisfied constraints to propagate as `NaN` values.

When a constraint has multiple valid geometric solutions, preserve mechanism branch continuity where practical rather than arbitrarily switching solutions between frames.

Simulation state updates must remain separate from rendering.

The main loop should conceptually remain:

```ts
if (simulationEnabled) {
    simulation.step(state, dt);
}

renderer.render(state);
```

## Hand Model

The hand is a simplified planar reference model, not a full biomechanical simulation.

Only the index finger is expected to articulate unless requirements change.

Keep anthropometric dimensions and joint-ROM assumptions centralized and easy to modify.

Do not scatter hand dimensions or physiological constants throughout rendering or simulation code.

Finger contactors must distinguish between:

* the rigid attachment point on a linkage component; and
* the corresponding contact location on the finger.

## TypeScript

Use strict TypeScript and explicit types for mechanism state and geometry.

Favor small, focused modules and readable data structures over excessive abstraction.

Avoid `any` unless interaction with an external API makes it unavoidable.

Do not suppress TypeScript errors merely to make the build pass.

The production build must continue to perform type checking before Vite bundling.

## Dependencies

Keep dependencies minimal.

The core simulator should preferably have no browser runtime dependencies unless a dependency clearly reduces complexity or improves correctness.

Development dependencies such as TypeScript, Vite, and Vitest are appropriate.

Before adding a new npm dependency, verify that the required functionality is not already simple enough to implement locally.

Do not commit `node_modules/`.

Do commit `package-lock.json`.

Use:

```bash
npm ci
```

for reproducible CI installs when a lockfile is present.

## Vite and GitHub Pages

This repository is deployed as a GitHub Pages **project site** under the repository path:

```text
/LinkageSimulator/
```

Keep the Vite base path explicit:

```ts
import { defineConfig } from 'vite';

export default defineConfig({
  base: '/LinkageSimulator/',

  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
});
```

Do not replace the explicit Pages base with `./` unless there is a deliberate change in deployment strategy.

The expected Pages URL structure is:

```text
https://<username>.github.io/LinkageSimulator/
```

The production build output is:

```text
dist/
```

`dist/` is generated output and should not be committed.

## GitHub Pages Deployment

GitHub Pages is deployed using GitHub Actions, not directly from a branch.

Keep the workflow under:

```text
.github/workflows/deploy.yml
```

The expected deployment sequence is:

```text
checkout
→ setup Node
→ npm ci
→ npm run build
→ upload ./dist
→ deploy GitHub Pages artifact
```

Do not introduce a `gh-pages` branch or commit compiled site output unless explicitly requested.

The GitHub repository setting should remain:

```text
Settings
→ Pages
→ Build and deployment
→ Source
→ GitHub Actions
```

## Build and Tests

Before considering implementation work complete, run:

```bash
npm run build
npm test
```

when tests are configured.

At minimum, geometry and constraint-solving code should have deterministic tests for important edge cases.

High-value tests include:

* vector operations;
* rotations and transforms;
* circle-circle intersections;
* tangent circles;
* non-intersecting circles;
* coincident/degenerate geometry;
* reachable and unreachable linkage configurations;
* mechanism branch continuity;
* joint ROM enforcement.

Do not weaken or remove tests simply to accommodate an implementation change.

## UI and Rendering

The simulator is primarily desktop-oriented but should remain usable at different browser sizes.

Keep pointer hit-testing conceptually separate from drawing.

Components such as links, joints, servos, and contactors should have stable IDs independent of their visual representation.

Selection and inspector state should refer to model IDs rather than Canvas objects.

The radial context menu should remain context-sensitive and extensible rather than hard-coded around one demonstration mechanism.

## Default Mechanism

A default mechanism may be used to demonstrate application behavior, but do not encode assumptions about that topology into the general model or geometry APIs.

The architecture must continue to support arbitrary collections of links, revolute joints, fixed joints, servo joints, and contactors.

Prefer extending general mechanism primitives rather than adding special cases for the initial demonstrator.

## State and Serialization

Keep mechanism state sufficiently data-driven that import/export of mechanism definitions can be added later.

Avoid storing essential mechanism state only in closures, DOM properties, or Canvas objects.

Where practical, use plain serializable TypeScript interfaces for persistent mechanism configuration.

Transient simulation/cache state may remain separate from persistent configuration.

## Numerical Robustness

Use named tolerances for floating-point geometric comparisons.

Do not rely on exact floating-point equality for positions, angles, intersections, or constraint closure.

Centralize tolerances where practical.

A geometrically impossible mechanism should be reported or visualized as invalid without crashing the animation loop.

## Repository Hygiene

Do not commit:

* `node_modules/`
* `dist/`
* coverage output
* temporary files
* editor-specific state
* generated local caches

Do not make unrelated formatting or architectural changes while implementing a focused task.

Preserve existing public interfaces unless changing them clearly improves the design and all call sites/tests are updated.

## Documentation

Keep `README.md` synchronized with significant user-visible or architectural changes.

Document non-obvious linkage mathematics and constraint-solving decisions close to the implementation.

Comments should explain why a mechanical or geometric operation is performed, not restate straightforward code.

## Completion Criteria

For repository changes, do not stop at writing code.

Before reporting completion:

1. inspect the affected implementation;
2. run the relevant tests;
3. run `npm run build`;
4. resolve TypeScript/build errors;
5. verify generated output is not accidentally tracked;
6. summarize files changed, validation performed, and any remaining limitations.

Prefer a working, validated incremental implementation over speculative large-scale refactors.
