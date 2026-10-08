// Headless smoke test of the v0.2 tools (sketch/extrude, edit, mirror, pattern, model tree).
// Usage: npm run build && node scripts/smoke-features.mjs   (CHROMIUM_PATH / SHOTS_DIR optional)
import { chromium } from 'playwright';
import { spawn } from 'node:child_process';

const OUT = process.env.SHOTS_DIR ?? 'test-results';
const PORT = process.env.PORT ?? '4190';
const server = spawn('npx', ['vite', 'preview', '--port', PORT, '--strictPort'], { cwd: process.cwd(), stdio: 'pipe' });
await new Promise((r) => setTimeout(r, 2500));
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || undefined, args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 860 } });
const errors = [];
page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));
page.on('console', (m) => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
await page.goto(`http://localhost:${PORT}/`);
await page.waitForTimeout(2000);
const W = async (p) => page.evaluate((p) => window.linkageDesigner.app.viewport.worldToScreen(p), p);
const clickWorld = async (p) => { const s = await W(p); await page.mouse.move(s.x, s.y); await page.waitForTimeout(40); await page.mouse.down(); await page.mouse.up(); await page.waitForTimeout(150); };
const state = () => page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; return { links: Object.values(m.links).map((l) => [l.name, l.kind, l.pointIds.length, !!l.hidden]), joints: Object.values(m.joints).filter((j) => j.a.kind !== 'body').map((j) => j.type), dof: app.sim.mobility?.dof, violation: app.sim.violation, tool: app.tool, status: app.status }; });

await page.evaluate(() => { const { app } = window.linkageDesigner; app.newModel(); app.viewport.setView('top'); app.viewport.fit([[-3, -3, 0], [6, 5, 0]]); });
await page.waitForTimeout(500);
// 1. Sketch a pentagon-ish polygon, close by clicking first vertex
await page.click('button[title^="Sketch polygon"]');
for (const p of [[0, 0, 0], [2, 0, 0], [2.5, 1.5, 0], [1, 2.5, 0], [-0.5, 1.5, 0]]) await clickWorld(p);
await clickWorld([0, 0, 0]);
console.log('after sketch', JSON.stringify(await state()));
// 2. Extrude via properties
const polyId = await page.evaluate(() => Object.values(window.linkageDesigner.app.model.links)[0]?.id);
await page.evaluate((id) => window.linkageDesigner.app.select({ type: 'link', id }), polyId);
await page.waitForTimeout(200);
await page.fill('.props-panel input[type="number"][step="0.1"]', '0.8');
await page.click('button[title^="Turns this polygon into a prism"]');
await page.waitForTimeout(400);
console.log('after extrude', JSON.stringify(await state()));
// 3. Mirror the prism across the RIGHT plane (x=0): click link body then the plane (pick via evaluate as plane may be hidden behind geometry)
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('iso'); app.viewport.fit([[-4, -3, 0], [6, 5, 1]]); });
await page.waitForTimeout(300);
await page.click('button[title^="Mirror"]');
await clickWorld([1.2, 1, 0.8]); // top face of prism
await page.waitForTimeout(200);
const planeHit = await page.evaluate(() => { const { app } = window.linkageDesigner; return app.viewport.worldToScreen([0, 5.5, 0.2]); });
await page.mouse.click(planeHit.x, planeHit.y);
await page.waitForTimeout(400);
console.log('after mirror', JSON.stringify(await state()));
// 4. Linear pattern of a bar (3 copies)
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('top'); app.viewport.fit([[-4, -4, 0], [8, 8, 0]]); });
await page.waitForTimeout(300);
await page.click('button[title^="Link —"]');
await clickWorld([4, -2, 0]); await clickWorld([5, -2, 0]);
await page.click('button[title^="Pattern"]');
await page.selectOption('.tool-options select', 'linear');
await page.fill('.tool-options input[type="number"]', '3');
await clickWorld([4.5, -2, 0]); // the bar
await clickWorld([4, -3, 0]); await clickWorld([4, -3.6, 0]);
await page.waitForTimeout(300);
console.log('after linear pattern', JSON.stringify(await state()));
// 5. Polar pattern around ORIGIN of the first copy: pick link then datum point via evaluate
await page.selectOption('.tool-options select', 'polar');
await page.fill('.tool-options input[type="number"]', '2');
await clickWorld([4.5, -2.6, 0]);
const originScreen = await W([0, 0, 0]);
await page.mouse.click(originScreen.x, originScreen.y);
await page.waitForTimeout(300);
console.log('after polar pattern', JSON.stringify(await state()));
// 6. Edit tool: move a bar end onto another bar's end → joint
await page.click('button[title^="Edit points"]');
const bars = await page.evaluate(() => Object.values(window.linkageDesigner.app.model.links).filter((l) => l.kind === 'bar').map((l) => ({ id: l.id, a: window.linkageDesigner.app.model.points[l.pointIds[0]].pos, b: window.linkageDesigner.app.model.points[l.pointIds[1]].pos })));
await clickWorld(bars[1].b); // source: end of second bar
await clickWorld(bars[0].a); // destination: start of first bar (snap → joint)
await page.waitForTimeout(400);
const afterEdit = await state();
console.log('after edit', JSON.stringify(afterEdit));
// 7. Model tree: count items, hide a link via eye, select via click
const tree = await page.evaluate(() => ({ groups: [...document.querySelectorAll('.tree-group__summary')].map((e) => e.textContent), items: document.querySelectorAll('.tree-item').length }));
console.log('tree', JSON.stringify(tree));
await page.click('.tree-eye');
await page.waitForTimeout(200);
await page.click('.tree-list .tree-item:nth-child(2)');
await page.waitForTimeout(200);
console.log('after tree ops', JSON.stringify(await page.evaluate(() => { const { app } = window.linkageDesigner; return { hidden: Object.values(app.model.links).filter((l) => l.hidden).map((l) => l.name), selection: app.selection }; })));
// 8. Ctrl+C / Ctrl+V
await page.keyboard.press('Control+c');
await page.keyboard.press('Control+v');
await page.waitForTimeout(200);
console.log('after paste', JSON.stringify((await state()).links.length));
await page.screenshot({ path: `${OUT}/08-features.png` });

// 9. Thin lone bar in the DEFAULT view: a drag started 4 px off the centreline must still grab the bar
// (the bar is placed away from the ORIGIN datum, which would otherwise snap and pin its first end)
await page.evaluate(() => { const { app } = window.linkageDesigner; app.newModel(); });
await page.waitForTimeout(300);
await page.click('button[title^="Link —"]');
await clickWorld([0.4, 0.7, 0]); await clickWorld([2.4, 0.7, 0]);
await page.click('button[title^="Select —"]');
const mid = await W([1.4, 0.7, 0]);
const before9 = await page.evaluate(() => Object.values(window.linkageDesigner.app.model.points).map((p) => [...p.pos]));
await page.mouse.move(mid.x, mid.y + 4);
await page.mouse.down();
for (let i = 1; i <= 8; i++) { await page.mouse.move(mid.x + i * 8, mid.y + 4 - i * 4); await page.waitForTimeout(40); }
await page.mouse.up();
await page.waitForTimeout(300);
const after9 = await page.evaluate(() => Object.values(window.linkageDesigner.app.model.points).map((p) => [...p.pos]));
const moved9 = Math.hypot(...[0, 1, 2].map((i) => after9[0][i] - before9[0][i]));
console.log('thin bar drag moved', moved9.toFixed(3), moved9 > 0.3 ? 'OK' : 'FAIL');

// 10. Origami: load the Miura example, delete panel 4, re-sketch it by snapping onto the existing vertices → 2 creases, DOF 1
await page.evaluate(() => { const { app } = window.linkageDesigner; app.loadExample('miuraVertex'); });
await page.waitForTimeout(800);
const panel4 = await page.evaluate(() => { const { app } = window.linkageDesigner; const l = Object.values(app.model.links).find((x) => x.name === 'Panel 4'); return { id: l.id, pts: l.pointIds.map((id) => [...app.model.points[id].pos]) }; });
await page.evaluate((id) => { const { tools } = window.linkageDesigner; tools.deletePick({ type: 'link', id }); }, panel4.id);
await page.waitForTimeout(300);
const dofWithout = await page.evaluate(() => window.linkageDesigner.app.sim.mobility?.dof);
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('iso'); app.viewport.fit([[-3, -3, -2], [4, 4, 2]]); });
await page.waitForTimeout(300);
await page.click('button[title^="Sketch polygon"]');
for (const p of panel4.pts) await clickWorld(p);
await page.keyboard.press('Enter');
await page.waitForTimeout(500);
const after10 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; return { links: Object.keys(m.links).length, creases: Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge').length, pins: Object.values(m.joints).filter((j) => j.a.kind === 'vertex').length, dof: app.sim.mobility?.dof, violation: app.sim.violation, status: app.status }; });
const corner = await page.evaluate((orig) => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links).find((x) => x.kind === 'polygon' && !x.name.startsWith('Panel')); if (!l) return null; const far = m.points[l.pointIds[2]].pos; return Math.hypot(far[0] - orig[2][0], far[1] - orig[2][1], far[2] - orig[2][2]); }, panel4.pts);
console.log('fourth panel sketched', JSON.stringify({ dofWithout, ...after10, farCornerError: corner }), after10.creases === 4 && after10.dof === 1 && corner !== null && corner < 1e-3 ? 'OK' : 'FAIL');

// 11. Miura example folds as a vertex: all four creases move over the sweep
await page.evaluate(() => { const { app } = window.linkageDesigner; app.loadExample('miuraVertex'); app.setMode('simulation'); });
await page.waitForTimeout(2000);
const miura = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const sw = app.sim.sweep; const pts = Object.values(m.links).filter((l) => !l.ground).map((l) => l.pointIds[2]); const moved = pts.map((id) => { const a = sw.poses.map((p) => p.positions.get(id)); let mx = 0; for (const q of a) mx = Math.max(mx, Math.hypot(q[0] - a[0][0], q[1] - a[0][1], q[2] - a[0][2])); return +mx.toFixed(2); }); return { range: sw.range.map((v) => +v.toFixed(1)), crank: sw.isCrank, farCornerTravel: moved, dof: app.sim.mobility?.dof }; });
console.log('miura', JSON.stringify(miura), miura.farCornerTravel.every((d) => d > 1) ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/09-miura.png` });

// 12. The model tree is not rebuilt on pointer moves (same DOM nodes before/after)
await page.evaluate(() => window.linkageDesigner.app.setMode('construction'));
await page.waitForTimeout(300);
const same = await page.evaluate(async () => { const first = document.querySelector('.tree-item'); const canvas = document.querySelector('.viewport-canvas'); const r = canvas.getBoundingClientRect(); for (let i = 0; i < 5; i++) canvas.dispatchEvent(new PointerEvent('pointermove', { clientX: r.left + 300 + i * 20, clientY: r.top + 300, bubbles: true })); await new Promise((res) => setTimeout(res, 100)); return first === document.querySelector('.tree-item'); });
console.log('tree stable across pointer moves', same ? 'OK' : 'FAIL');

// 13. Escape during a drag restores the pre-drag model and leaves the undo history alone
await page.evaluate(() => { const { app } = window.linkageDesigner; app.newModel(); app.viewport.setView('top'); app.viewport.fit([[-1, -1, 0], [5, 3, 0]]); });
await page.waitForTimeout(300);
await page.click('button[title^="Link —"]');
await clickWorld([0, 0, 0]); await clickWorld([2, 0, 0]);
await clickWorld([3, 1, 0]); await clickWorld([4, 1, 0]);
await page.click('button[title^="Select —"]');
const pre13 = await page.evaluate(() => ({ undo: window.linkageDesigner.app.undoStack.length, json: JSON.stringify(window.linkageDesigner.app.model) }));
const grab = await W([1, 0, 0]);
await page.mouse.move(grab.x, grab.y);
await page.mouse.down();
for (let i = 1; i <= 6; i++) { await page.mouse.move(grab.x + i * 10, grab.y - i * 6); await page.waitForTimeout(40); }
await page.keyboard.press('Escape');
await page.mouse.up();
await page.waitForTimeout(300);
const post13 = await page.evaluate(() => ({ undo: window.linkageDesigner.app.undoStack.length, json: JSON.stringify(window.linkageDesigner.app.model), links: Object.keys(window.linkageDesigner.app.model.links).length }));
console.log('escape mid-drag', JSON.stringify({ undoBefore: pre13.undo, undoAfter: post13.undo, links: post13.links, restored: pre13.json === post13.json }), pre13.undo === post13.undo && pre13.json === post13.json ? 'OK' : 'FAIL');

// 14. Double-click closes a sketch without adding a sliver vertex
await page.click('button[title^="Sketch polygon"]');
for (const p of [[0, 2, 0], [2, 2, 0], [2, 3.5, 0]]) await clickWorld(p);
const last = await W([0.5, 3.5, 0]);
await page.mouse.move(last.x, last.y);
await page.mouse.dblclick(last.x + 1, last.y + 1);
await page.waitForTimeout(300);
const sketched = await page.evaluate(() => { const l = Object.values(window.linkageDesigner.app.model.links).find((x) => x.kind === 'polygon'); return l ? l.pointIds.length : 0; });
console.log('double-click close vertices', sketched, sketched === 4 ? 'OK' : 'FAIL');

// 15. Tree rename: Enter commits and closes the editor, Escape cancels
await page.click('button[title^="Select —"]');
await page.dblclick('.tree-list .tree-item__label');
await page.keyboard.type('Rocker');
await page.keyboard.press('Enter');
await page.waitForTimeout(200);
const rename1 = await page.evaluate(() => ({ inputs: document.querySelectorAll('.tree-item__rename').length, name: Object.values(window.linkageDesigner.app.model.links)[0].name }));
await page.dblclick('.tree-list .tree-item__label');
await page.keyboard.type('Zed');
await page.keyboard.press('Escape');
await page.waitForTimeout(200);
const rename2 = await page.evaluate(() => ({ inputs: document.querySelectorAll('.tree-item__rename').length, name: Object.values(window.linkageDesigner.app.model.links)[0].name }));
console.log('tree rename', JSON.stringify({ rename1, rename2 }), rename1.inputs === 0 && rename1.name === 'Rocker' && rename2.inputs === 0 && rename2.name === 'Rocker' ? 'OK' : 'FAIL');

// 16. ORIGIN datum is pickable in the top view (end-on Z axis must not win) — on an empty model
const originPick = await page.evaluate(async () => { const { app } = window.linkageDesigner; const saved = app.saveToJson(); app.newModel(); app.viewport.setView('top'); await new Promise((r) => setTimeout(r, 150)); const s = app.viewport.worldToScreen([0, 0, 0]); const p = app.viewport.pick(s.x, s.y); app.loadFromJson(saved, 'restore'); await new Promise((r) => setTimeout(r, 150)); return p && p.id; });
console.log('origin pick in top view', originPick, originPick === 'point_origin' ? 'OK' : 'FAIL');

// 17. Mirror acts on the tree selection
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('top'); app.select({ type: 'link', id: Object.values(app.model.links)[0].id }); });
await page.waitForTimeout(300);
const linksBefore17 = await page.evaluate(() => Object.keys(window.linkageDesigner.app.model.links).length);
await page.click('button[title^="Mirror"]');
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('iso'); app.viewport.fit([[-4, -4, -1], [6, 6, 1]]); });
await page.waitForTimeout(300);
const frontHit = await W([2, 0, 0.5]);
await page.mouse.click(frontHit.x, frontHit.y);
await page.waitForTimeout(300);
const linksAfter17 = await page.evaluate(() => Object.keys(window.linkageDesigner.app.model.links).length);
console.log('mirror from tree selection', linksBefore17, '->', linksAfter17, linksAfter17 === linksBefore17 + 1 ? 'OK' : 'FAIL');

// ---------------------------------------------------------------------------------------------------------------
// Construction workflow (docs/CONSTRUCTION_PLAN.md §5, Phase 0 and Phase 1)
// ---------------------------------------------------------------------------------------------------------------
// shared helpers: the status bar is the contract of every tool, so each step waits for the text it expects
const waitStatus = async (re, ms = 8000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { const s = await page.evaluate(() => window.linkageDesigner.app.status); if (re.test(s)) return s; await page.waitForTimeout(50); } return 'TIMEOUT: ' + (await page.evaluate(() => window.linkageDesigner.app.status)); };
const typeCoord = async (text) => { await page.fill('.coord__input', text); await page.press('.coord__input', 'Enter'); await page.waitForTimeout(150); };
// newModel re-frames the camera on the next animation frame (loadModel → zoomToFit), so the view is set after a wait
const freshTopView = async (box) => { await page.evaluate(() => window.linkageDesigner.app.newModel()); await page.waitForTimeout(400); await page.evaluate((box) => { const { app } = window.linkageDesigner; app.viewport.setView('top'); app.viewport.fit(box); }, box); await page.waitForTimeout(300); };
/** midpoint of the edge of link #i whose end points are nearest to p and q (world coordinates) */
const edgeMid = (i, p, q) => page.evaluate(({ i, p, q }) => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links)[i]; const near = (x) => l.pointIds.reduce((b, id) => (Math.hypot(...m.points[id].pos.map((v, k) => v - x[k])) < Math.hypot(...m.points[b].pos.map((v, k) => v - x[k])) ? id : b), l.pointIds[0]); const a = m.points[near(p)].pos, b = m.points[near(q)].pos; return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2]; }, { i, p, q });
const workflowState = () => page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; return { creases: Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2).length, joints: Object.values(m.joints).filter((j) => j.a.kind !== 'body').length, bodyJoints: Object.values(m.joints).filter((j) => j.a.kind === 'body').length, drivers: m.drivers.length, dof: app.sim.mobility?.dof, violation: app.sim.violation, undo: app.undoStack.length, status: app.status, chip: document.querySelector('.chip')?.textContent ?? '', hints: [...document.querySelectorAll('.mode-panel .hint.warn')].map((e) => e.textContent), foldBtn: !!document.querySelector('button[title^="Drives one crease"]'), creaseLabels: [...document.querySelectorAll('.tree-item__label')].map((e) => e.textContent).filter((t) => /^Crease [MV]? ?\d+°/.test(t)), json: JSON.stringify(m) }; });

// 18. The reported workflow: four regular triangles (Polygon tool, 3 sides, typed coordinates, each placed 1.2 units
//     outside its final place) joined edge to edge with the Joint tool. Joints 1–3 are exact creases that pull the
//     triangles into a fan; the fourth would close a vertex whose sector angles sum to 240° and must be refused with
//     the sector-angle message, moving nothing and recording no undo entry.
await freshTopView([[-4, -4, 0], [4, 4, 0]]);
const SIDE = Math.sqrt(3);
const O18 = [0, 0, 0];
const P18 = (k) => [SIDE * Math.cos((Math.PI / 3) * k), SIDE * Math.sin((Math.PI / 3) * k), 0];
const centroid18 = (k) => [(P18(k)[0] + P18(k + 1)[0]) / 3, (P18(k)[1] + P18(k + 1)[1]) / 3, 0];
const offset18 = (k) => { if (k === 0) return [0, 0, 0]; const c = centroid18(k); const l = Math.hypot(c[0], c[1]); return [(1.2 * c[0]) / l, (1.2 * c[1]) / l, 0]; };
const plus = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
await page.click('button[title^="Polygon —"]');
await page.fill('.tool-options input[type="number"]', '3');
await page.press('.tool-options input[type="number"]', 'Tab');
for (let k = 0; k < 4; k++) {
  const c = plus(centroid18(k), offset18(k)), v = plus(P18(k + 1), offset18(k));
  await typeCoord(`${c[0]},${c[1]}`);
  await typeCoord(`${v[0]},${v[1]}`);
}
await page.click('button[title^="Ground —"]');
await clickWorld(centroid18(0));
await waitStatus(/Ground/);
await page.click('button[title^="Joint —"]');
const joined18 = [];
for (let k = 0; k < 3; k++) {
  await clickWorld(await edgeMid(k, O18, P18(k + 1)));
  await waitStatus(/Now pick/);
  await clickWorld(await edgeMid(k + 1, plus(O18, offset18(k + 1)), plus(P18(k + 1), offset18(k + 1))));
  joined18.push((await waitStatus(/Joint created|Nothing was moved|different link/)).slice(0, 30));
}
const pre18 = await workflowState();
await clickWorld(await edgeMid(3, O18, P18(4)));
await waitStatus(/Now pick/);
await clickWorld(await edgeMid(0, O18, P18(0)));
const refused18 = await waitStatus(/Joint created|Nothing was moved|different link/);
await page.waitForTimeout(300);
const post18 = await workflowState();
const ok18 = joined18.every((s) => s === 'Joint created') && pre18.creases === 3 && /add up to 240\.0°, not 360°/.test(refused18) && /Nothing was moved/.test(refused18) && pre18.json === post18.json && pre18.undo === post18.undo && post18.creases === 3 && !/violated/i.test(post18.chip);
console.log('four regular triangles: 3 creases, 4th refused', JSON.stringify({ joined: joined18, creasesBefore: pre18.creases, refused: refused18.slice(0, 90), modelUnchanged: pre18.json === post18.json, undo: [pre18.undo, post18.undo], chip: post18.chip }), ok18 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/10-four-triangles-refused.png` });

// 19. A developable vertex (sectors 60°, 60°, 120°, 120°) sketched flat with the Sketch tool, snapping the shared
//     vertices so the shared edges become creases; Ground → DOF 0 with the locked-creases and flat-vertex hints and the
//     Fold button; Fold → DOF 1, 2-D constraints gone, dihedrals (170°, 160°, 170°, 160°) with a 3:1 mountain/valley
//     split, one fold driver kept; the crease is pickable and has the crease Properties; Preview moves every panel.
await freshTopView([[-3, -3, 0], [3, 3, 0]]);
const R19 = (deg) => [2 * Math.cos((Math.PI / 180) * deg), 2 * Math.sin((Math.PI / 180) * deg), 0];
await page.click('button[title^="Sketch polygon"]');
for (const pts of [[O18, R19(0), R19(60)], [O18, R19(60), R19(120)], [O18, R19(120), R19(240)], [O18, R19(240), R19(0)]]) {
  for (const p of pts) await clickWorld(p);
  await page.keyboard.press('Enter');
  await page.waitForTimeout(400);
}
await page.click('button[title^="Ground —"]');
await clickWorld([1, 0.5, 0]);
await waitStatus(/Ground/);
await page.waitForTimeout(300);
const flat19 = await workflowState();
const okFlat19 = flat19.creases === 4 && flat19.bodyJoints === 4 && flat19.dof === 0 && flat19.hints.some((h) => /cannot fold/.test(h)) && flat19.hints.some((h) => /flat/.test(h)) && flat19.foldBtn && flat19.creaseLabels.every((t) => /180°/.test(t));
console.log('developable vertex sketched flat', JSON.stringify({ creases: flat19.creases, bodyJoints: flat19.bodyJoints, dof: flat19.dof, hints: flat19.hints.length, foldBtn: flat19.foldBtn, labels: flat19.creaseLabels }), okFlat19 ? 'OK' : 'FAIL');
await page.click('button[title^="Drives one crease"]');
const folded19 = await waitStatus(/pre-folded|Nothing to fold|already folded|No folded state/);
await page.waitForTimeout(300);
const post19 = await workflowState();
const angles19 = post19.creaseLabels.map((t) => t.match(/Crease ([MV]) (\d+)°/)).map((x) => x && { mv: x[1], deg: +x[2] });
const pattern19 = angles19.length === 4 && angles19.every(Boolean) && angles19.map((a) => a.deg).sort((a, b) => a - b).join() === '160,160,170,170' && [1, 3].includes(angles19.filter((a) => a.mv === 'M').length);
const okFold19 = /pre-folded/.test(folded19) && post19.dof === 1 && post19.bodyJoints === 0 && post19.drivers === 1 && post19.undo === flat19.undo + 1 && pattern19 && !post19.foldBtn;
console.log('fold', JSON.stringify({ status: folded19.slice(0, 80), dof: post19.dof, bodyJoints: post19.bodyJoints, drivers: post19.drivers, labels: post19.creaseLabels }), okFold19 ? 'OK' : 'FAIL');
await page.click('button[title^="Select —"]');
const creaseMid19 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const j = Object.values(m.joints).find((x) => x.type === 'revolute' && x.a.kind === 'edge'); const [a, b] = j.a.pointIds.map((id) => m.points[id].pos); return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2]; });
await clickWorld(creaseMid19);
await page.waitForTimeout(300);
const props19 = await page.evaluate(() => ({ selection: window.linkageDesigner.app.selection?.type, subtitle: document.querySelector('.props-panel .panel__subtitle')?.textContent, rows: [...document.querySelectorAll('.props-panel .prop-row')].map((r) => r.textContent.trim()), buttons: [...document.querySelectorAll('.props-panel button')].map((b) => b.textContent.trim()) }));
const okProps19 = props19.selection === 'joint' && props19.subtitle === 'Crease' && props19.rows.some((r) => /Fold angle/.test(r)) && props19.rows.some((r) => /Mountain \/ valley/.test(r)) && props19.rows.some((r) => /Target fold angle/.test(r)) && props19.buttons.includes('Fold to target') && props19.buttons.includes('Drive this crease');
console.log('crease picked, crease properties', JSON.stringify({ selection: props19.selection, subtitle: props19.subtitle, angleRow: props19.rows.find((r) => /Fold angle/.test(r)), buttons: props19.buttons }), okProps19 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/11-developable-folded.png` });
await page.click('button.mode-tab:has-text("Preview")');
await page.waitForTimeout(2500);
const travel19 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const sw = app.sim.sweep; if (!sw) return null; const ground = Object.values(m.links).find((l) => l.ground); const gp = ground.pointIds.map((id) => m.points[id].pos); const farFromGround = (id) => Math.min(...gp.map((g) => Math.hypot(...g.map((v, k) => v - m.points[id].pos[k])))); return Object.values(m.links).filter((l) => !l.ground).map((l) => { const id = l.pointIds.reduce((b, x) => (farFromGround(x) > farFromGround(b) ? x : b), l.pointIds[0]); const a = sw.poses.map((p) => p.positions.get(id)); let mx = 0; for (const q of a) mx = Math.max(mx, Math.hypot(...q.map((v, k) => v - a[0][k]))); return +mx.toFixed(2); }); });
console.log('preview after fold moves every panel', JSON.stringify(travel19), travel19 && travel19.length === 3 && travel19.every((d) => d > 1) ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/12-developable-preview.png` });
await page.click('button.mode-tab:has-text("Construction")');
await page.waitForTimeout(300);
console.log('errors:', errors.length ? errors.join('\n') : 'none');
await browser.close();
server.kill();
process.exit(0);
