# Linkage Designer

A browser-based, CAD-like prototype for **inverse design of linkages and 3-D
origami mechanisms**: build a mechanism from links, joints and construction
geometry, see its degrees of freedom live, and then drag the *editing points*
along an output path — the solver changes the link geometry so the mechanism
reaches what you drew.

![Four-bar in simulation mode](docs/images/simulation.png)

- `docs/ORIGINAL_SPEC.md` — the brief this prototype implements, verbatim.
- `docs/SPEC.md` — detailed specification of every implemented feature, with
  deviations from the brief called out.
- `docs/DESIGN_DECISIONS.md` — assumptions, implementation decisions and the
  research / textbook references behind them.

## Running

```bash
npm install
npm run dev        # http://localhost:5173
npm test           # solver / kinematics / synthesis unit tests (vitest)
npm run build      # type-check + production build into dist/
node scripts/smoke.mjs            # headless-browser smoke test of the built app (needs Playwright's Chromium)
node scripts/smoke-features.mjs   # same for the sketch / edit / pattern tools, model tree and the origami example
```

Node 22 is assumed (`.nvmrc`). The built app is static and can be served from
any path (GitHub Pages workflow included in `.github/workflows/ci.yml`).

## Quick tour

1. **Construction mode.** Pick *Link* (`2`), click two points. Click an
   existing link end to join the new link to it with a pin (revolute). Or type
   `3<45` in the coordinate box for a 3-unit link at 45°. *Sketch polygon*
   (`S`) draws free polygons (snap onto existing vertices to join them;
   shared edges become creases) and *Extrude* in Properties makes prisms.
   *Edit points* (`E`) snaps vertices onto other geometry. `Ctrl+C`/`Ctrl+V`,
   *Mirror* and *Pattern* duplicate links. Pick *Ground* (`G`) and click a
   link to fix it. The DOF chip and the model tree update after every edit.
2. Select a vertex and tick **Show output path & design space** in the
   Properties panel (the examples already do this).
3. **Simulation mode.** The output path appears with green editing points and
   the magenta design space. Drag an editing point: it turns red, the mechanism
   adapts, and *Design DOF left* drops. Lock points or constrain them to a
   datum in Properties. Toggle *Soft assumptions* to hide the preview until the
   design is fully specified.
4. **Preview mode.** Play the motion through the driver's range.
5. *File → Save* writes a `.linkage.json` with the solidified design.

See `docs/SPEC.md` §1 for all shortcuts; `?` in the top bar shows quick help.

## Project layout

```
src/core/        solver and mechanism model (no DOM / three.js)
  types.ts         data model
  constraints.ts   residuals + Jacobians (distance, coincidence, plane, line, angle, dihedral, screw, …)
  model.ts         creation/editing of links, joints, datums, drivers
  compile.ts       model → solver system (point merging, frozen coordinates, poses)
  solver.ts        Levenberg–Marquardt, dense Cholesky, pivoted QR (rank / null space)
  kinematics.ts    forward solve, mobility, driver sweeps, output surfaces
  synthesis.ts     stacked multi-pose inverse design, design DOF, design space
  examples.ts      built-in example mechanisms
src/viewport/    three.js scene, renderer and interactive tools
src/ui/          strings (all UI text), settings + colour maths, panels, colour picker, icons
tests/           vitest unit tests
examples/        example mechanism files
docs/            specification and design notes
```

## Status

Prototype. Mechanisms with up to a few hundred solver variables are
interactive; see `docs/DESIGN_DECISIONS.md` §7 for known limitations.
