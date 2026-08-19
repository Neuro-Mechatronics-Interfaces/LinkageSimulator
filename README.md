# Linkage Simulator

Linkage Simulator is a static TypeScript application for sketching and exploring planar rotational mechanisms intended to move an index finger. This repository currently contains the first functional prototype: a deterministic four-bar demonstrator, a simplified left-hand radial profile, interactive component inspection, and basic editing.

The project is an engineering visualization tool, not a clinical or biomechanical model.

## Current prototype

- Animates a dorsally mounted servo, proximal to the D2 metacarpal, through configurable angular limits.
- Solves a crank/coupler/rocker four-bar analytically with both ground pivots on the proximal side.
- Preserves assembly-branch continuity by selecting the solution nearest the last valid output point.
- Carries the floating four-bar output into a three-segment dorsal chain: structural anchor, middle-phalanx driver, and ungrounded distal driver.
- Simultaneously solves independent middle- and distal-phalanx contactors, each with a dorsal rectangular pad and flexor-side ring.
- Treats a joint at its range limit as a local active constraint: the limited joint stays fixed while any remaining proximal or distal degrees of freedom continue solving.
- Exposes each contactor's total ring width, clamps it to the rendered digit width, and reports the remaining radial offset affordance per side.
- Places the ground plane on the dorsal hand surface and derives the raised servo position from an editable servo-ground offset.
- Exposes dorsal base-rail length and servo-relative angle in the inspector; the rail visibly anchors the proximal rocker.
- Exposes hand scale from 72% to 132% of the centralized default proportions.
- Selects links, joints, the servo, and contactors with independent world-space hit testing.
- Edits link name, length, and width plus joint and servo ROM values.
- Adds jointed links only from an underlying link or servo attachment, plus ground joints and contactors, through the radial menu.
- Repositions servos and contactors interactively after choosing **Reposition** from the radial menu.
- Inverse-drives the mechanism by dragging the right-end handle of any rectangular link.
- Supports wheel zoom, middle-button or Shift-drag pan, reset, pause, and live left-joint-centred construction circles for every rectangular link.
- Keeps impossible mechanism geometry finite and displays an explicit solver error.
- Rejects candidate poses where a rectangular link enters a finger capsule, crosses the ground plane or base rail, or passes to the hand-side of the rail within its span; rejection restores the last valid configuration.
- Reports gravity-only posture-holding moments at the MCP, PIP, and DIP using size-scaled cylindrical phalanx mass estimates.
- Exports the complete current state, units, contactor geometry, active joint limits, and moment estimates as versioned JSON.

## Architecture

The layers are intentionally small and one-directional. Geometry and simulation contain no DOM or Canvas dependencies.

![Linkage Simulator module dependency diagram](docs/architecture.svg)

```text
src/ts/
├── geometry/    Reusable vectors, transforms, distances, and circle intersections
├── model/       Typed state, defaults, contactor constraints, and export schema
├── simulation/  Kinematic constraint solving and gravity-only joint statics
├── rendering/   Canvas renderer and world/screen camera transform
├── ui/          Hit testing, inspector, pointer controls, radial menu, and JSON download
└── main.ts      Composition, controls, summaries, and requestAnimationFrame loop
```

The editable diagram source is [`docs/architecture.dot`](docs/architecture.dot). The implementation-derived equations and assumptions are recorded in [`manuscript/main.tex`](manuscript/main.tex).

## Local setup

Node.js 24 or newer is required by the current package configuration.

```bash
npm install
npm run dev
```

Run validation with:

```bash
npm test
npm run build
```

The production site is emitted to `dist/`. Vite uses the explicit `/LinkageSimulator/` project-site base path expected at `https://<username>.github.io/LinkageSimulator/`. The included `.github/workflows/deploy.yml` workflow builds and publishes `main`; GitHub Pages must be configured to use GitHub Actions in the repository settings.

## Controls

- **Play / Pause** sweeps or freezes the servo.
- **Servo angle** directly poses the paused mechanism.
- **Hand size** scales centralized palm and phalanx dimensions.
- **Export JSON** downloads a detached, versioned snapshot of the current mechanism, hand, contactors, solver state, units, and moment estimates.
- **Reset** recreates the default mechanism and viewport.
- **Construction geometry** shows every link's current distal-joint circle locus about its left revolute joint.
- **Click** selects a component; click empty canvas to clear selection.
- **Drag a link’s right-end handle** to drive the servo-constrained mechanism; an imported unconstrained link rotates about its left end.
- **Right-click** opens context-sensitive actions. For a servo or contactor, choose **Reposition**, move the pointer, and left-click to place it.
- **Select a contactor** to edit ring width. Its minimum follows the digit's current rendered width; at that minimum the ring has zero offset affordance.
- **Select the servo** to edit its ground offset, or **select the dorsal base rail** to edit rail length and angle relative to the servo.
- **Mouse wheel** zooms about the cursor; **middle-drag** or **Shift-drag** pans.
- **Delete / Backspace** removes a selected link, joint, or contactor when an input is not focused.

## Important limitations

- The model accepts arbitrary link and joint collections, but the active mechanism solver recognizes the default four-bar and its three-link contactor chain by stable IDs.
- Added left-end revolute attachments follow their parent, but added joints are not yet incorporated into a general closed-loop constraint solve.
- The two default contacts are solved simultaneously with projected planar numerical IK and heuristic DIP coupling; joint limits are handled locally, but additional arbitrary contactors do not expand that coupled solve.
- Hard geometry exclusions still reject an entire candidate pose: partial mobility applies to angular joint limits, not to permitted interpenetration.
- The moment readout is a quasi-static gravity estimate, not a musculoskeletal force solution. It excludes linkage weight, contact forces, tendon forces, friction, and joint reactions.
- Joint stiffness and rest angle exist as zero-default data fields for extension, but are not yet configurable in the inspector.
- Right-end handles provide inverse posing, but arbitrary joint dragging and full closed-loop solving for user-added topology are not implemented.
- Anthropometric values are broad visualization defaults, not values inferred from height or validated clinical measurements.

## Roadmap

1. Build an arbitrary-topology constraint graph with degree-of-freedom and connectivity checks.
2. Add stronger analytic/numerical constraint solving and mechanism validity/singularity visualization.
3. Generalize the simultaneous two-contactor solve to arbitrary editable contact sets.
4. Add direct manipulation, snapping, and complete topology editing.
5. Add anthropometric presets and documented ranges.
6. Add versioned JSON import and migration; versioned JSON export is implemented.

## License

See [`LICENSE`](LICENSE).
