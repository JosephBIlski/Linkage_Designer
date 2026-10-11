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
await page.click('button[title^="Panel"]');
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
await page.click('button[title^="Panel"]');
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
await page.click('button[title^="Panel"]');
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
/** the Panel tool's sector-angle labels currently shown in the viewport (text and CSS class, plan 2b) */
const sectorLabels = () => page.evaluate(() => [...document.querySelectorAll('.viewport-labels [class*="label--sector"]')].map((e) => ({ text: e.textContent, cls: e.className })));
const workflowState = () => page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; return { creases: Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2).length, joints: Object.values(m.joints).filter((j) => j.a.kind !== 'body').length, bodyJoints: Object.values(m.joints).filter((j) => j.a.kind === 'body').length, drivers: m.drivers.length, dof: app.sim.mobility?.dof, violation: app.sim.violation, undo: app.undoStack.length, status: app.status, chip: document.querySelector('.chip')?.textContent ?? '', hints: [...document.querySelectorAll('.mode-panel .hint.warn')].map((e) => e.textContent), foldBtn: !!document.querySelector('button[title^="Drives one crease"]'), creasePattern: document.querySelector('.mode-panel .crease-pattern')?.textContent ?? '', creaseLabels: [...document.querySelectorAll('.tree-item__label')].map((e) => e.textContent).filter((t) => /^Crease [MV]? ?\d+°/.test(t)), json: JSON.stringify(m) }; });

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

// 18b. The same workflow with the triangles placed by mouse clicks instead of typed coordinates: the clicked
//      circumcircle point is never exactly on the circle, so the shared edges differ by screen-pixel noise and the
//      first three joints are merged creases whose panels adapt by that noise; the fourth must still be refused with
//      the sector-angle message (a released solve must not collapse the fourth triangle to close the loop), the model
//      byte-identical, and the whole diagnosis readable: as the status bar's tooltip and in the Mechanism panel.
await freshTopView([[-4, -4, 0], [4, 4, 0]]);
await page.click('button[title^="Polygon —"]');
await page.fill('.tool-options input[type="number"]', '3');
await page.press('.tool-options input[type="number"]', 'Tab');
for (let k = 0; k < 4; k++) {
  await clickWorld(plus(centroid18(k), offset18(k)));
  await clickWorld(plus(P18(k + 1), offset18(k)));
}
await page.click('button[title^="Ground —"]');
await clickWorld(centroid18(0));
await waitStatus(/Ground/);
await page.click('button[title^="Joint —"]');
const joined18b = [];
for (let k = 0; k < 3; k++) {
  await clickWorld(await edgeMid(k, O18, P18(k + 1)));
  await waitStatus(/Now pick/);
  await clickWorld(await edgeMid(k + 1, plus(O18, offset18(k + 1)), plus(P18(k + 1), offset18(k + 1))));
  joined18b.push((await waitStatus(/Joint created|Nothing was moved|different link/)).slice(0, 30));
}
const pre18b = await workflowState();
const angles18b = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; return Object.values(m.links).map((l) => l.pointIds.map((id, i) => { const v = m.points[id].pos, a = m.points[l.pointIds[(i + 2) % 3]].pos, b = m.points[l.pointIds[(i + 1) % 3]].pos; const u = [a[0] - v[0], a[1] - v[1]], w = [b[0] - v[0], b[1] - v[1]]; return (Math.acos((u[0] * w[0] + u[1] * w[1]) / Math.hypot(...u) / Math.hypot(...w)) * 180) / Math.PI; })); });
await clickWorld(await edgeMid(3, O18, P18(4)));
await waitStatus(/Now pick/);
await clickWorld(await edgeMid(0, O18, P18(0)));
const refused18b = await waitStatus(/Joint created|Nothing was moved|different link/);
await page.waitForTimeout(300);
const post18b = await workflowState();
const readable18b = await page.evaluate(() => ({ title: document.querySelector('.status-text')?.title ?? '', notice: document.querySelector('.mode-panel .notice .hint')?.textContent ?? '', status: window.linkageDesigner.app.status }));
const intact18b = angles18b.every((tri) => tri.every((a) => Math.abs(a - 60) < 0.5));
const ok18b = joined18b.every((s) => s === 'Joint created') && pre18b.creases === 3 && intact18b && /add up to 240\.0°, not 360°/.test(refused18b) && /Nothing was moved/.test(refused18b) && pre18b.json === post18b.json && pre18b.undo === post18b.undo && post18b.creases === 3 && !/violated/i.test(post18b.chip) && readable18b.title === readable18b.status && readable18b.notice === readable18b.status;
console.log('four regular triangles placed by mouse: 3 creases, 4th refused, diagnosis readable', JSON.stringify({ joined: joined18b, creasesBefore: pre18b.creases, trianglesIntact: intact18b, refused: refused18b.slice(0, 90), modelUnchanged: pre18b.json === post18b.json, undo: [pre18b.undo, post18b.undo], chip: post18b.chip, tooltip: readable18b.title === readable18b.status, notice: readable18b.notice === readable18b.status }), ok18b ? 'OK' : 'FAIL');
// dismissing the notice removes it from the Mechanism panel
await page.click('.mode-panel .notice button');
await page.waitForTimeout(200);
const dismissed18b = await page.evaluate(() => !document.querySelector('.mode-panel .notice'));
console.log('notice dismissed', JSON.stringify({ dismissed: dismissed18b }), dismissed18b ? 'OK' : 'FAIL');

// 18c. A refused Properties edit leaves no stale widget behind: on the four-bar example the crank's Length is typed
//      as 100 (impossible in the closed loop), which is refused; the input must then show the model's length again,
//      and a following feasible edit (1.2) must reach the model (previously the panel kept the refused value and its
//      widgets wrote into the discarded model).
await page.evaluate(() => { const { app } = window.linkageDesigner; app.loadExample('fourBar'); });
await page.waitForTimeout(600);
const crank18c = () => page.evaluate(() => { const { app } = window.linkageDesigner; const l = Object.values(app.model.links).find((x) => x.name === 'Crank'); const r = l.rigidity.find((c) => c.kind === 'dist' && !c.fixed); return { id: l.id, length: r.length, undo: app.undoStack.length, status: app.status }; });
const before18c = await crank18c();
await page.evaluate((id) => window.linkageDesigner.app.select({ type: 'link', id }), before18c.id);
await page.waitForTimeout(300);
const lengthInput = '.props-panel label.prop-row:has-text("Length") input';
await page.fill(lengthInput, '100');
await page.press(lengthInput, 'Enter');
await page.waitForTimeout(500);
const refused18c = await crank18c();
const shown18c = await page.inputValue(lengthInput);
await page.fill(lengthInput, '1.2');
await page.press(lengthInput, 'Enter');
await page.waitForTimeout(500);
const applied18c = await crank18c();
const ok18c = refused18c.length === before18c.length && refused18c.undo === before18c.undo && /not applied/.test(refused18c.status) && Math.abs(Number(shown18c) - before18c.length) < 1e-3 && Math.abs(applied18c.length - 1.2) < 1e-9 && applied18c.undo === before18c.undo + 1;
console.log('refused Length edit: input shows the model value, next edit applies', JSON.stringify({ lengthBefore: +before18c.length.toFixed(4), afterRefusal: +refused18c.length.toFixed(4), shown: shown18c, status: refused18c.status.slice(0, 60), afterEdit: +applied18c.length.toFixed(4), undo: [before18c.undo, refused18c.undo, applied18c.undo] }), ok18c ? 'OK' : 'FAIL');

// 18d. Polygon tool joins (plan 2b): three regular triangles drawn with the Polygon tool by typed coordinates so that
//      each new triangle's first vertex lands on O and its second on the previous triangle's corner (the typed
//      circumcircle point is the first vertex; the next one is 120° round the centre). Two coincident vertices on one
//      existing edge become a crease at once, with "Joint created (1)" in the status bar, and the model stays
//      consistent. With the Panel tool a fourth triangle continuing the fan (O, P3, P4) shows the running sum 240°
//      at the centre in the neutral colour: the ring is still open (in a plane the fourth regular triangle cannot
//      close it; a closed ring that is not 360° only arises in 3-D, covered by tests/sector.test.ts).
await freshTopView([[-4, -4, 0], [4, 4, 0]]);
await page.click('button[title^="Polygon —"]');
await page.fill('.tool-options input[type="number"]', '3');
await page.press('.tool-options input[type="number"]', 'Tab');
const joined18d = [];
for (let k = 0; k < 3; k++) {
  const c = centroid18(k), v = k === 0 ? P18(1) : O18;
  await page.evaluate(() => window.linkageDesigner.app.setStatus(''));
  await typeCoord(`${c[0]},${c[1]}`);
  await typeCoord(`${v[0]},${v[1]}`);
  joined18d.push(k === 0 ? await page.evaluate(() => window.linkageDesigner.app.status) : await waitStatus(/Joint created/));
  await page.waitForTimeout(150);
}
const poly18d = await workflowState();
await page.click('button[title^="Panel"]');
await clickWorld(O18);
await clickWorld(P18(3));
const open18d = await W(P18(4));
await page.mouse.move(open18d.x, open18d.y);
await page.waitForTimeout(250);
const labels18d = await sectorLabels();
const running18d = labels18d.find((l) => /label--sector$/.test(l.cls) && /^240\.0° \(\+60\.0°\)/.test(l.text));
await clickWorld(P18(4));
await page.keyboard.press('Enter');
await waitStatus(/Joint created \(1\)/);
const post18d = await workflowState();
const links18d = await page.evaluate(() => Object.keys(window.linkageDesigner.app.model.links).length);
const ok18d = joined18d[1] === 'Joint created (1)' && joined18d[2] === 'Joint created (1)' && poly18d.creases === 2 && poly18d.joints === 2 && poly18d.violation < 1e-8 && !!running18d && links18d === 4 && post18d.creases === 3 && post18d.violation < 1e-8;
console.log('polygon tool joins coincident vertices into creases; running sector sum on the next panel', JSON.stringify({ joined: joined18d, creasesAfterPolygons: poly18d.creases, joints: poly18d.joints, violation: poly18d.violation, labels: labels18d, links: links18d, creasesAfterPanel: post18d.creases }), ok18d ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/10b-polygon-joins.png` });

// 19. A developable vertex (sectors 60°, 60°, 120°, 120°) sketched flat with the Panel (sketch) tool, snapping the shared
//     vertices so the shared edges become creases; Ground → DOF 0 with the locked-creases and flat-vertex hints and the
//     Fold button; Fold → DOF 1, 2-D constraints gone, dihedrals (170°, 160°, 170°, 160°) with a 3:1 mountain/valley
//     split, one fold driver kept; the crease is pickable and has the crease Properties; Preview moves every panel.
await freshTopView([[-3, -3, 0], [3, 3, 0]]);
const R19 = (deg) => [2 * Math.cos((Math.PI / 180) * deg), 2 * Math.sin((Math.PI / 180) * deg), 0];
await page.click('button[title^="Panel"]');
// Panel tool (plan 2b): while the fourth panel is drawn and the cursor rests on its closing vertex, the sector label at
// the centre must read 360° (the three panels' 240° plus the 120° this panel adds) in green: the ring closes flat
let labels19 = [];
const panels19 = [[O18, R19(0), R19(60)], [O18, R19(60), R19(120)], [O18, R19(120), R19(240)], [O18, R19(240), R19(0)]];
for (let k = 0; k < panels19.length; k++) {
  for (let i = 0; i < panels19[k].length; i++) {
    if (k === 3 && i === 2) {
      const s = await W(panels19[k][i]);
      await page.mouse.move(s.x, s.y);
      await page.waitForTimeout(250);
      labels19 = await sectorLabels();
    }
    await clickWorld(panels19[k][i]);
  }
  await page.keyboard.press('Enter');
  await page.waitForTimeout(400);
}
const label19 = labels19.find((l) => /label--sector-ok/.test(l.cls) && /^360\.0°/.test(l.text));
console.log('sector label while closing the fourth panel', JSON.stringify(labels19), label19 ? 'OK' : 'FAIL');
await page.click('button[title^="Ground —"]');
await clickWorld([1, 0.5, 0]);
await waitStatus(/Ground/);
await page.waitForTimeout(300);
const flat19 = await workflowState();
// Crease-pattern validator (plan 2d): the Mechanism panel lists the interior vertex as developable (360.0° ✓) with
// Kawasaki 0.0° ✓ and, while flat, no mountain / valley assignment
const okPattern19 = /Crease pattern/.test(flat19.creasePattern) && /4 panels/.test(flat19.creasePattern) && /360\.0° developable ✓/.test(flat19.creasePattern) && /Kawasaki 0\.0° ✓/.test(flat19.creasePattern) && /M\/V not assigned/.test(flat19.creasePattern) && !/✗/.test(flat19.creasePattern);
const okFlat19 = flat19.creases === 4 && flat19.bodyJoints === 4 && flat19.dof === 0 && flat19.hints.some((h) => /cannot fold/.test(h)) && flat19.hints.some((h) => /flat/.test(h)) && flat19.foldBtn && flat19.creaseLabels.every((t) => /180°/.test(t)) && !!label19 && okPattern19;
console.log('developable vertex sketched flat', JSON.stringify({ creases: flat19.creases, bodyJoints: flat19.bodyJoints, dof: flat19.dof, hints: flat19.hints.length, foldBtn: flat19.foldBtn, labels: flat19.creaseLabels, sectorLabel: label19, creasePattern: flat19.creasePattern }), okFlat19 ? 'OK' : 'FAIL');
await page.click('button[title^="Drives one crease"]');
const folded19 = await waitStatus(/pre-folded|Nothing to fold|already folded|No folded state/);
await page.waitForTimeout(300);
const post19 = await workflowState();
const angles19 = post19.creaseLabels.map((t) => t.match(/Crease ([MV]) (\d+)°/)).map((x) => x && { mv: x[1], deg: +x[2] });
const pattern19 = angles19.length === 4 && angles19.every(Boolean) && angles19.map((a) => a.deg).sort((a, b) => a - b).join() === '160,160,170,170' && [1, 3].includes(angles19.filter((a) => a.mv === 'M').length);
// after the fold the validator reports the Maekawa 3:1 split as a pass
const okFold19 = /pre-folded/.test(folded19) && post19.dof === 1 && post19.bodyJoints === 0 && post19.drivers === 1 && post19.undo === flat19.undo + 1 && pattern19 && !post19.foldBtn && /M\/V [13]:[13] ✓/.test(post19.creasePattern) && !/✗/.test(post19.creasePattern);
console.log('fold', JSON.stringify({ status: folded19.slice(0, 80), dof: post19.dof, bodyJoints: post19.bodyJoints, drivers: post19.drivers, labels: post19.creaseLabels, creasePattern: post19.creasePattern }), okFold19 ? 'OK' : 'FAIL');
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

// 20. Query pick (plan 2a): two triangles with exactly coincident edges (the second is sketched 3 units lower by typed
//     coordinates, which never snap, and carried rigidly onto the first). With the Joint tool a left-click on the
//     shared edge picks the edge of Polygon 1 (the first drawn); a right-click at the same spot starts the query cycle
//     on the coincident edge of Polygon 2 ("2 of N"); a left-click then uses it and the two edges become one crease.
//     A 60 px right-drag must still orbit the camera and must not start a cycle.
await freshTopView([[-1, -4.5, 0], [3, 3, 0]]);
// Enter in the (still focused, now empty) coordinate box closes the panel, as the Panel hint promises a typing user
const closeSketch = async () => { await page.press('.coord__input', 'Enter'); await page.waitForTimeout(300); };
await page.click('button[title^="Panel"]');
for (const c of ['0.5,0.5', '2.5,0.5', '1.5,2']) await typeCoord(c);
await closeSketch();
// same base edge 3 units lower, apex below it: after the +3 translation the triangles share (0.5,0.5)-(2.5,0.5) exactly
for (const c of ['0.5,-2.5', '2.5,-2.5', '1.5,-4']) await typeCoord(c);
await closeSketch();
await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links)[1]; app.beginChange(); for (const id of [...l.pointIds, ...l.helperIds]) m.points[id].pos[1] += 3; app.endChange(); });
await page.waitForTimeout(300);
await page.click('button[title^="Select —"]');
await page.waitForTimeout(100);
const pre20 = await workflowState();
await page.click('button[title^="Joint —"]');
const shared20 = [1.5, 0.5, 0];
await clickWorld(shared20);
const first20 = await waitStatus(/Now pick/);
const s20 = await W(shared20);
await page.mouse.click(s20.x, s20.y, { button: 'right' });
const cycled20 = await waitStatus(/^2 of \d+ · /);
const cycleHover20 = await page.evaluate(() => { const { app, tools } = window.linkageDesigner; return { active: tools.queryActive, hover: app.hover && { type: app.hover.type, link: app.model.links[app.hover.id]?.name } }; });
await page.mouse.click(s20.x, s20.y);
const joined20 = await waitStatus(/Joint created|Nothing was moved|different link/);
await page.waitForTimeout(300);
const post20 = await workflowState();
const ok20 = /Polygon 1/.test(first20) && /Polygon 2/.test(cycled20) && /right-click: next/.test(cycled20) && cycleHover20.active && cycleHover20.hover?.type === 'edge' && cycleHover20.hover?.link === 'Polygon 2' && /Joint created/.test(joined20) && pre20.creases === 0 && post20.creases === 1 && post20.joints === 1 && !post20.status.startsWith('2 of');
console.log('query pick: right-click cycles to the coincident edge of the other panel', JSON.stringify({ first: first20.slice(0, 40), cycled: cycled20.slice(0, 70), hover: cycleHover20, joined: joined20.slice(0, 30), creases: [pre20.creases, post20.creases] }), ok20 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/13-query-pick.png` });
// 20b. A left-click through the cycle with a tool that writes no status (Select) leaves the plain feature description,
//      not the cycle's "left-click: use it" instructions; the pointer leaving the canvas clears them as well.
await page.click('button[title^="Select —"]');
await page.mouse.move(s20.x, s20.y);
await waitStatus(/^(edge|Crease)/);
await page.mouse.click(s20.x, s20.y, { button: 'right' });
const cycled20b = await waitStatus(/^2 of \d+ · /);
await page.mouse.click(s20.x, s20.y);
await page.waitForTimeout(300);
const after20b = await page.evaluate(() => { const { app, tools } = window.linkageDesigner; return { status: app.status, cycling: tools.queryActive, selection: app.selection?.type ?? null }; });
await page.mouse.click(s20.x, s20.y, { button: 'right' });
await waitStatus(/^\d+ of \d+ · /);
await page.mouse.move(5, 5); // the top bar: off the canvas
await page.waitForTimeout(300);
const left20b = await page.evaluate(() => { const { app, tools } = window.linkageDesigner; return { status: app.status, cycling: tools.queryActive, hover: app.hover }; });
const ok20b = /^2 of/.test(cycled20b) && !after20b.cycling && !/^\d+ of \d+ · /.test(after20b.status) && !/left-click: use it/.test(after20b.status) && after20b.selection !== null && !left20b.cycling && left20b.status === '' && left20b.hover === null;
console.log('cycle instructions leave the status bar with the click and with the pointer', JSON.stringify({ cycled: cycled20b.slice(0, 30), afterClick: after20b, afterLeave: { status: left20b.status, cycling: left20b.cycling } }), ok20b ? 'OK' : 'FAIL');
// 20c. The TOP datum plane is hovered wherever the canvas is empty, which must not colour it (the hover is silent);
//      a right-click query on it is an explicit choice and lights it in the selection colour. The spot is clear of the
//      RIGHT and FRONT planes, which the perspective top view projects as wedges over x < 0 and y > 0.
const planeMat = () => page.evaluate(() => { const { app } = window.linkageDesigner; const mesh = app.viewport.groups.construction.children.find((o) => o.userData?.id === 'plane_top' && o.userData.plane); return mesh ? { color: '#' + mesh.material.color.getHexString(), opacity: mesh.material.opacity, hover: app.hover?.id ?? null, queried: !!app.hover?.queried } : null; });
const empty20c = await W([2.5, -3.5, 0]);
await page.mouse.move(empty20c.x, empty20c.y);
await page.waitForTimeout(300);
const idle20c = await planeMat();
await page.mouse.click(empty20c.x, empty20c.y, { button: 'right' });
await waitStatus(/^\d+ of \d+ · TOP/);
await page.waitForTimeout(200);
const lit20c = await planeMat();
await page.keyboard.press('Escape');
await page.waitForTimeout(200);
const colors20c = await page.evaluate(() => ({ construction: window.linkageDesigner.app.settings.colors.construction.toLowerCase(), selection: window.linkageDesigner.app.settings.colors.selection.toLowerCase() }));
const ok20c = idle20c?.hover === 'plane_top' && !idle20c.queried && idle20c.color === colors20c.construction && idle20c.opacity === 0.04 && lit20c?.hover === 'plane_top' && lit20c.queried && lit20c.color === colors20c.selection && lit20c.opacity === 0.3;
console.log('datum plane: silent under the idle pointer, lit by a query click', JSON.stringify({ idle: idle20c, lit: lit20c, colors: colors20c }), ok20c ? 'OK' : 'FAIL');
// 20d. A right-drag of 60 px orbits (OrbitControls on the same element) and is not a query click, in the top view too
//      (the controls are rebuilt with the view's up vector, so the top view is no longer at the pole of a Z-up frame).
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('top'); app.viewport.fit([[-1, -4.5, 0], [3, 3, 0]]); });
await page.waitForTimeout(300);
const camBefore20 = await page.evaluate(() => window.linkageDesigner.app.viewport.camera.position.toArray());
const o20 = await W([1.5, -1, 0]);
await page.mouse.move(o20.x, o20.y);
await page.mouse.down({ button: 'right' });
for (let i = 1; i <= 6; i++) { await page.mouse.move(o20.x + i * 10, o20.y + i * 2); await page.waitForTimeout(40); }
await page.mouse.up({ button: 'right' });
await page.waitForTimeout(300);
const drag20 = await page.evaluate(() => { const { app, tools } = window.linkageDesigner; return { cam: app.viewport.camera.position.toArray(), status: app.status, cycling: tools.queryActive }; });
const orbited20 = Math.hypot(...[0, 1, 2].map((i) => drag20.cam[i] - camBefore20[i]));
console.log('right-drag still orbits (top view), no query cycle', JSON.stringify({ cameraMoved: +orbited20.toFixed(3), cycling: drag20.cycling, status: drag20.status.slice(0, 40) }), orbited20 > 1e-3 && !drag20.cycling && !/^\d+ of \d+ · /.test(drag20.status) ? 'OK' : 'FAIL');
// 20e. A right-drag that swings out and comes back to its press position is still a drag, not a query click
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('iso'); app.viewport.fit([[-1, -4.5, 0], [3, 3, 0]]); });
await page.waitForTimeout(300);
const camBefore20e = await page.evaluate(() => window.linkageDesigner.app.viewport.camera.position.toArray());
const o20e = await W([1.5, 0.8, 0]);
await page.mouse.move(o20e.x, o20e.y);
await page.mouse.down({ button: 'right' });
for (let i = 1; i <= 6; i++) { await page.mouse.move(o20e.x + i * 10, o20e.y + i * 2); await page.waitForTimeout(40); }
// the orbit is measured at the far end of the swing: the way back undoes it, so the camera ends where it started
const camFar20e = await page.evaluate(() => window.linkageDesigner.app.viewport.camera.position.toArray());
for (let i = 5; i >= 0; i--) { await page.mouse.move(o20e.x + i * 10, o20e.y + i * 2); await page.waitForTimeout(40); }
await page.mouse.up({ button: 'right' });
await page.waitForTimeout(300);
const drag20e = await page.evaluate(() => { const { app, tools } = window.linkageDesigner; return { cam: app.viewport.camera.position.toArray(), status: app.status, cycling: tools.queryActive }; });
const orbited20e = Math.hypot(...[0, 1, 2].map((i) => camFar20e[i] - camBefore20e[i]));
const returned20e = Math.hypot(...[0, 1, 2].map((i) => drag20e.cam[i] - camBefore20e[i]));
console.log('out-and-back right-drag: orbited and came back, no query cycle', JSON.stringify({ cameraMovedMidDrag: +orbited20e.toFixed(3), cameraOffAtEnd: +returned20e.toFixed(3), cycling: drag20e.cycling, status: drag20e.status.slice(0, 40) }), orbited20e > 1e-3 && returned20e < 1e-3 && !drag20e.cycling && !/^\d+ of \d+ · /.test(drag20e.status) ? 'OK' : 'FAIL');
// 21. 3-D placement (plan 2c): with "Place on sketch plane (2-D)" unticked, a bar clicked at two screen points in the
//     default (isometric-ish) view still gets both ends on the sketch plane TOP (z = 0), under the clicked points, but
//     carries no 2-D constraint (previously the ends landed on the view plane through the origin / previous point,
//     off the sketch plane). The option is ticked again afterwards.
const waitFor = async (fn, ms = 8000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { const v = await page.evaluate(fn); if (v) return v; await page.waitForTimeout(50); } return null; };
await page.evaluate(() => window.linkageDesigner.app.newModel());
await page.waitForTimeout(400);
await page.click('button[title^="Link —"]');
const mode2d21 = '.tool-options label.prop-row:has-text("Place on sketch plane") input';
await page.uncheck(mode2d21);
const ends21 = [[0.8, 0.6, 0], [2.6, 1.4, 0]];
const view21 = await page.evaluate(() => { const { app } = window.linkageDesigner; return { dir: app.viewport.viewDirection(), mode2d: app.toolOptions.mode2d }; });
for (const p of ends21) await clickWorld(p);
const bar21 = await waitFor(() => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links).find((x) => x.kind === 'bar'); return l ? { ends: l.pointIds.map((id) => [...m.points[id].pos]), bodyJoints: Object.values(m.joints).filter((j) => j.a.kind === 'body').length, links: Object.keys(m.links).length } : null; });
await page.check(mode2d21);
const restored21 = await page.evaluate(() => window.linkageDesigner.app.toolOptions.mode2d);
const onPlane21 = !!bar21 && bar21.ends.every((e) => Math.abs(e[2]) < 1e-9);
const underPointer21 = !!bar21 && bar21.ends.every((e, i) => Math.hypot(e[0] - ends21[i][0], e[1] - ends21[i][1]) < 0.1);
const ok21 = !view21.mode2d && Math.abs(view21.dir[2]) > 0.15 && onPlane21 && underPointer21 && bar21.bodyJoints === 0 && bar21.links === 1 && restored21;
console.log('3-D mode: a clicked bar lies on the sketch plane without the 2-D constraint', JSON.stringify({ mode2d: view21.mode2d, viewDirZ: +view21.dir[2].toFixed(3), ends: bar21?.ends.map((e) => e.map((v) => +v.toFixed(4))), onPlane: onPlane21, underPointer: underPointer21, bodyJoints: bar21?.bodyJoints, restored: restored21 }), ok21 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/14-3d-placement.png` });
// 22. Front view with "Place on sketch plane (2-D)" ticked: the camera sits on the TOP plane, so every off-centre ray
//     meets it at the camera; such a hit is a miss and the clicks land on the view plane through the previous point
//     (y = 0) instead of putting a point at the camera (previously nothing was created).
await page.evaluate(() => window.linkageDesigner.app.newModel());
await page.waitForTimeout(400);
await page.evaluate(() => window.linkageDesigner.app.viewport.setView('front'));
await page.waitForTimeout(300);
await page.click('button[title^="Link —"]');
const cam22 = await page.evaluate(() => window.linkageDesigner.app.viewport.camera.position.toArray());
const centre22 = await page.evaluate(() => { const r = document.querySelector('.viewport-canvas').getBoundingClientRect(); return { x: r.left + r.width / 2, y: r.top + r.height / 2 }; });
await page.mouse.click(centre22.x + 120, centre22.y + 60);
await page.waitForTimeout(150);
await page.mouse.click(centre22.x + 200, centre22.y + 100);
const bar22 = await waitFor(() => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links).find((x) => x.kind === 'bar'); return l ? { ends: l.pointIds.map((id) => [...m.points[id].pos]), links: Object.keys(m.links).length, mode2d: app.toolOptions.mode2d } : null; }, 3000);
const ok22 = !!bar22 && bar22.mode2d && bar22.links === 1 && bar22.ends.every((e) => Math.hypot(e[0] - cam22[0], e[1] - cam22[1], e[2] - cam22[2]) > 1 && Math.abs(e[1]) < 1e-6);
console.log('front view, 2-D mode: off-centre clicks make a bar on the view plane, no point at the camera', JSON.stringify({ camera: cam22.map((v) => +v.toFixed(3)), bar: bar22 && { ...bar22, ends: bar22.ends.map((e) => e.map((v) => +v.toFixed(3))) } }), ok22 ? 'OK' : 'FAIL');
// 23. Polygon tool in 2-D mode with its circumcircle point snapped to a vertex off the sketch plane (the top of a
//     prism): the snapped vertex is authoritative, so the polygon is built in the plane through its centre and that
//     vertex, joined there exactly (one spherical pin), without a 2-D constraint, and the status bar says so.
await page.evaluate(() => window.linkageDesigner.app.newModel());
await page.waitForTimeout(400);
await page.evaluate(() => { const { app } = window.linkageDesigner; app.viewport.setView('iso'); app.viewport.fit([[-2, -2, 0], [5, 2, 1]]); });
await page.waitForTimeout(300);
await page.click('button[title^="Prism"]');
await clickWorld([0, 0, 0]);
await clickWorld([1, 0, 0]);
const prism23 = await waitFor(() => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links).find((x) => x.kind === 'prism'); return l ? { id: l.id, tops: l.pointIds.slice(4).map((id) => [...m.points[id].pos]) } : null; }, 3000);
await page.click('button[title^="Polygon —"]');
await page.evaluate(() => window.linkageDesigner.app.setStatus(''));
await clickWorld([3.5, 0, 0]);
const topVertex23 = prism23.tops.reduce((b, p) => (p[0] - p[1] > b[0] - b[1] ? p : b), prism23.tops[0]); // the one nearest the iso camera
await clickWorld(topVertex23);
const status23 = await waitStatus(/Joint created/);
await page.waitForTimeout(300);
const poly23 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const l = Object.values(m.links).find((x) => x.kind === 'polygon'); if (!l) return null; const pts = l.pointIds.map((id) => [...m.points[id].pos]); return { pts, bodyJoints: Object.values(m.joints).filter((j) => j.a.kind === 'body' && j.a.linkId === l.id).length, joints: Object.values(m.joints).filter((j) => j.a.kind !== 'body').map((j) => j.type), violation: app.sim.violation }; });
const onVertex23 = !!poly23 && poly23.pts.some((p) => Math.hypot(p[0] - topVertex23[0], p[1] - topVertex23[1], p[2] - topVertex23[2]) < 1e-6);
// a regular polygon's centroid is its centre, clicked on the sketch plane (z = 0); the vertices then spread above and below it
const centroidZ23 = poly23 ? poly23.pts.reduce((s, p) => s + p[2], 0) / poly23.pts.length : NaN;
const tilted23 = !!poly23 && Math.abs(centroidZ23) < 1e-6 && Math.max(...poly23.pts.map((p) => Math.abs(p[2]))) > 0.1;
const ok23 = /Joint created \(1\)/.test(status23) && /no 2-D constraint/.test(status23) && onVertex23 && tilted23 && poly23.bodyJoints === 0 && poly23.joints.length === 1 && poly23.violation < 1e-8;
console.log('polygon snapped to an off-plane vertex in 2-D mode: tilted to meet it, joined, no 2-D constraint', JSON.stringify({ status: status23.slice(0, 120), onVertex: onVertex23, tilted: tilted23, sides: poly23?.pts.length, centroidZ: +centroidZ23.toFixed(6), bodyJoints: poly23?.bodyJoints, joints: poly23?.joints, violation: poly23?.violation }), ok23 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/15-polygon-off-plane.png` });
// ---------------------------------------------------------------------------------------------------------------
// Crease-pattern-first construction (docs/CONSTRUCTION_PLAN.md §5, Phase 3b)
// ---------------------------------------------------------------------------------------------------------------
// 24. Crease pattern tool: a square of border creases drawn by clicking its four corners and closing on the first
//     vertex (which snaps to the pattern vertex), then a valley diagonal between two corners, each polyline ended with
//     Enter. The pattern is selected from the model tree and "Generate panels" turns it into 2 panels joined by one
//     crease with fold.mv 'V', zero violation, the pattern hidden and the status text. Then the crease is selected,
//     its target set to 120° and "Fold to target" pressed: the fold-angle readout reads 120.0° · Valley (V).
await freshTopView([[-1, -1, 0], [4, 4, 0]]);
await page.click('button[title^="Crease pattern"]');
await page.selectOption('.tool-options select', 'B');
for (const q of [[0, 0, 0], [3, 0, 0], [3, 3, 0], [0, 3, 0], [0, 0, 0]]) {
  await clickWorld(q);
  await waitStatus(/Vertex placed|added|nothing was added/);
}
await page.keyboard.press('Enter');
await waitStatus(/Polyline ended/);
const square24 = await page.evaluate(() => { const { app } = window.linkageDesigner; const ps = Object.values(app.model.patterns ?? {}); const p = ps[0]; return p ? { patterns: ps.length, name: p.name, vertices: Object.keys(p.vertices).length, creases: Object.keys(p.creases).length, classes: Object.values(p.creases).map((c) => c.cls), selection: app.selection?.type } : null; });
await page.selectOption('.tool-options select', 'V');
await clickWorld([0, 0, 0]);
await waitStatus(/Vertex placed/);
await clickWorld([3, 3, 0]);
await waitStatus(/Crease \(valley\) added/);
await page.keyboard.press('Enter');
await waitStatus(/Polyline ended/);
const drawn24 = await page.evaluate(() => { const { app } = window.linkageDesigner; const ps = Object.values(app.model.patterns ?? {}); const p = ps[0]; return p ? { patterns: ps.length, vertices: Object.keys(p.vertices).length, creases: Object.keys(p.creases).length, valleys: Object.values(p.creases).filter((c) => c.cls === 'V').length, borders: Object.values(p.creases).filter((c) => c.cls === 'B').length, links: Object.keys(app.model.links).length } : null; });
const okDrawn24 = !!square24 && square24.patterns === 1 && square24.vertices === 4 && square24.creases === 4 && square24.classes.every((c) => c === 'B') && square24.selection === 'pattern' && !!drawn24 && drawn24.patterns === 1 && drawn24.vertices === 4 && drawn24.creases === 5 && drawn24.valleys === 1 && drawn24.borders === 4 && drawn24.links === 0;
console.log('crease pattern tool: square of borders closed on its first vertex, valley diagonal', JSON.stringify({ square: square24, drawn: drawn24 }), okDrawn24 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/16-crease-pattern-drawn.png` });
// select the pattern through its model-tree row and generate
await page.click('button[title^="Select —"]');
await page.evaluate(() => window.linkageDesigner.app.select(null));
await page.waitForTimeout(200);
const treeRow24 = await page.evaluate(() => { const row = [...document.querySelectorAll('.tree-item')].find((r) => r.querySelector('.tree-item__label')?.textContent === 'Pattern 1'); return row ? { badges: [...row.querySelectorAll('.tree-badge')].map((b) => b.textContent), group: row.closest('.tree-group')?.querySelector('.tree-group__summary')?.textContent ?? '' } : null; });
await page.click('.tree-item:has-text("Pattern 1")');
await page.waitForTimeout(300);
const props24 = await page.evaluate(() => ({ selection: window.linkageDesigner.app.selection, subtitle: document.querySelector('.props-panel .panel__subtitle')?.textContent ?? '', rows: [...document.querySelectorAll('.props-panel .prop-row')].map((r) => r.textContent.trim()), checks: [...document.querySelectorAll('.props-panel .crease-pattern p')].map((e) => e.textContent), buttons: [...document.querySelectorAll('.props-panel button')].map((b) => b.textContent.trim()) }));
await page.click('.props-panel button:has-text("Generate panels")');
const generated24 = await waitStatus(/Generated|Nothing to generate/);
await page.waitForTimeout(400);
const after24 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const p = Object.values(m.patterns ?? {})[0]; const creases = Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2); return { links: Object.values(m.links).map((l) => [l.name, l.kind, l.pointIds.length, l.ground]), creases: creases.map((j) => ({ id: j.id, mv: j.fold?.mv ?? null, target: j.fold?.target ?? null })), bodyJoints: Object.values(m.joints).filter((j) => j.a.kind === 'body').length, dof: app.sim.mobility?.dof, violation: app.sim.violation, hidden: !!p?.hidden, generated: p?.generated ? { links: p.generated.linkIds.length, joints: p.generated.jointIds.length } : null, selection: app.selection?.type, treeBadges: [...document.querySelectorAll('.tree-item')].find((r) => r.querySelector('.tree-item__label')?.textContent === 'Pattern 1') ? [...[...document.querySelectorAll('.tree-item')].find((r) => r.querySelector('.tree-item__label')?.textContent === 'Pattern 1').querySelectorAll('.tree-badge')].map((b) => b.textContent) : null }; });
const okGen24 = !!treeRow24 && /Crease patterns/.test(treeRow24.group) && treeRow24.badges.includes('5 creases') && props24.selection?.type === 'pattern' && /Crease pattern · Pattern 1/.test(props24.subtitle) && props24.rows.some((r) => /4 · 5 · 2/.test(r)) && props24.checks.some((t) => /No problems found/.test(t)) && props24.buttons.includes('Generate panels') && /Generated 2 panels and 1 crease\./.test(generated24) && !/warning/.test(generated24) && after24.links.length === 2 && after24.links.every((l) => l[1] === 'polygon' && l[2] === 3) && after24.links.filter((l) => l[3]).length === 1 && after24.creases.length === 1 && after24.creases[0].mv === 'V' && after24.bodyJoints === 0 && after24.violation < 1e-8 && after24.dof === 1 && after24.hidden && after24.generated?.links === 2 && after24.generated?.joints === 1 && after24.selection === 'link' && !!after24.treeBadges && after24.treeBadges.includes('hidden');
console.log('generate panels from the pattern: 2 panels, 1 valley crease, zero violation, pattern hidden', JSON.stringify({ treeRow: treeRow24, subtitle: props24.subtitle, counts: props24.rows.find((r) => /faces/.test(r)), checks: props24.checks, status: generated24, after: { ...after24, links: after24.links.map((l) => l[0]) } }), okGen24 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/17-crease-pattern-generated.png` });
// the generated crease: target 120°, Fold to target, readout 120.0° · Valley (V)
await page.evaluate((id) => window.linkageDesigner.app.select({ type: 'joint', id }), after24.creases[0].id);
await page.waitForTimeout(300);
const targetInput24 = '.props-panel label.prop-row:has-text("Target fold angle") input';
await page.fill(targetInput24, '120');
await page.press(targetInput24, 'Tab');
await page.waitForTimeout(100);
await page.click('.props-panel button:has-text("Fold to target")');
const folded24 = await waitStatus(/Folded to target|No pose was found|Nothing to fold|locked/);
await page.waitForTimeout(400);
const readout24 = await page.evaluate(() => { const { app } = window.linkageDesigner; const row = [...document.querySelectorAll('.props-panel .prop-row')].find((r) => /Fold angle \(current\)/.test(r.textContent)); const j = Object.values(app.model.joints).find((x) => x.type === 'revolute' && x.a.kind === 'edge'); return { row: row?.textContent.trim() ?? '', target: j?.fold?.target ?? null, mv: j?.fold?.mv ?? null, dof: app.sim.mobility?.dof, violation: app.sim.violation, selection: app.selection?.type }; });
const angle24 = Number((readout24.row.match(/(\d+(?:\.\d+)?)°/) ?? [])[1]);
const okFold24 = /Folded to target/.test(folded24) && Math.abs(angle24 - 120) < 0.5 && /Valley/.test(readout24.row) && readout24.target === 120 && readout24.mv === 'V' && readout24.violation < 1e-6 && readout24.selection === 'joint';
console.log('fold the generated crease to its target of 120°', JSON.stringify({ status: folded24.slice(0, 80), readout: readout24.row, angle: angle24, target: readout24.target, mv: readout24.mv, dof: readout24.dof, violation: readout24.violation }), okFold24 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/18-crease-pattern-folded.png` });
// 25. The Miura-ori sheet example (plan 3c) and the Fold command on a generated crease pattern. The example loads
//     pre-folded on its drawn branch: 16 panels, 24 creases, DOF 1, one fold driver at 160°, every crease label in the
//     tree with a class and an angle below 180°, the pattern kept hidden. "Regenerate panels" on that pattern (selected
//     from the tree) rebuilds the flat sheet: DOF above 1, nine flat vertices, the hint and the Fold button, no driver.
//     Fold then folds the whole sheet at once: the status reads "pre-folded as one sheet … 24 creases folded", DOF 1,
//     every crease off flat, the class read out in the Properties of four sampled creases equal to the class stored on
//     them (the drawn branch), one driver kept, one undo entry. Preview moves every panel.
await page.evaluate(() => { const { app } = window.linkageDesigner; app.setMode('construction'); app.loadExample('miuraSheet'); });
await page.waitForTimeout(1000);
const sheetState = () => page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const creases = Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2); const p = Object.values(m.patterns ?? {})[0]; const labels = [...document.querySelectorAll('.tree-item__label')].map((e) => e.textContent).filter((t) => /^Crease/.test(t)); return { links: Object.keys(m.links).length, creases: creases.length, creaseIds: creases.map((j) => j.id), stored: creases.map((j) => j.fold?.mv ?? null), drivers: m.drivers.map((d) => [d.kind, +Math.abs(d.value).toFixed(1)]), dof: app.sim.mobility?.dof, violation: app.sim.violation, ground: Object.values(m.links).find((l) => l.ground)?.name ?? null, pattern: p ? { name: p.name, hidden: !!p.hidden, generated: p.generated ? p.generated.jointIds.length : 0 } : null, labels: labels.length, folded: labels.filter((t) => /^Crease [MV] \d+°/.test(t) && !/ 180°/.test(t)).length, flatLoops: app.sim.creaseLoops.filter((l) => l.flat).length, hints: [...document.querySelectorAll('.mode-panel .hint.warn')].map((e) => e.textContent), foldBtn: !!document.querySelector('button[title^="Drives one crease"]'), undo: app.undoStack.length, status: app.status }; });
const loaded25 = await sheetState();
const okLoaded25 = loaded25.links === 16 && loaded25.creases === 24 && loaded25.dof === 1 && loaded25.violation < 1e-6 && loaded25.drivers.length === 1 && loaded25.drivers[0][0] === 'fold' && Math.abs(loaded25.drivers[0][1] - 160) < 0.1 && loaded25.folded === 24 && loaded25.flatLoops === 0 && !loaded25.foldBtn && !!loaded25.pattern?.hidden && loaded25.pattern.generated === 24 && loaded25.stored.every((c) => c === 'M' || c === 'V');
console.log('Miura-ori sheet example loads pre-folded', JSON.stringify({ ...loaded25, creaseIds: undefined, stored: undefined }), okLoaded25 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/19-miura-sheet-example.png` });
// regenerate the sheet from its pattern: flat again, nine flat vertices, the Fold button offered, the driver gone with the joints
await page.click('button[title^="Select —"]');
await page.evaluate(() => window.linkageDesigner.app.select(null));
await page.waitForTimeout(200);
await page.click('.tree-item:has-text("Miura-ori")');
await page.waitForTimeout(300);
await page.click('.props-panel button:has-text("Regenerate panels")');
const regenerated25 = await waitStatus(/Generated|Nothing to generate/);
await page.waitForTimeout(500);
const flat25 = await sheetState();
const okFlat25 = /Generated 16 panels and 24 creases\./.test(regenerated25) && flat25.links === 16 && flat25.creases === 24 && flat25.drivers.length === 0 && flat25.dof > 1 && flat25.flatLoops === 9 && flat25.foldBtn && flat25.folded === 0 && flat25.labels === 24 && flat25.hints.some((h) => /9 vertices are flat/.test(h)) && !!flat25.pattern?.hidden;
console.log('regenerate the sheet flat from its pattern', JSON.stringify({ status: regenerated25.slice(0, 80), links: flat25.links, creases: flat25.creases, drivers: flat25.drivers, dof: flat25.dof, flatLoops: flat25.flatLoops, foldBtn: flat25.foldBtn, hints: flat25.hints }), okFlat25 ? 'OK' : 'FAIL');
// Fold: the whole generated pattern leaves the flat state on its drawn branch
await page.click('button[title^="Drives one crease"]');
const folded25 = await waitStatus(/pre-folded|No folded state|Nothing to fold|already folded|locked/);
await page.waitForTimeout(500);
const post25 = await sheetState();
const sampled25 = [];
for (const i of [0, 7, 13, 23]) {
  const id = post25.creaseIds[i];
  await page.evaluate((id) => window.linkageDesigner.app.select({ type: 'joint', id }), id);
  await page.waitForTimeout(200);
  sampled25.push(await page.evaluate((id) => { const { app } = window.linkageDesigner; const row = [...document.querySelectorAll('.props-panel .prop-row')].find((r) => /Fold angle \(current\)/.test(r.textContent)); const text = row?.textContent ?? ''; return { stored: app.model.joints[id]?.fold?.mv ?? null, shown: /Mountain/.test(text) ? 'M' : /Valley/.test(text) ? 'V' : null, angle: Number((text.match(/(\d+(?:\.\d+)?)°/) ?? [])[1]) }; }, id));
}
const okFold25 = /pre-folded as one sheet/.test(folded25) && /24 creases folded/.test(folded25) && /DOF: 1\./.test(folded25) && post25.dof === 1 && post25.violation < 1e-6 && post25.drivers.length === 1 && post25.drivers[0][0] === 'fold' && post25.folded === 24 && post25.flatLoops === 0 && !post25.foldBtn && post25.undo === flat25.undo + 1 && sampled25.length === 4 && sampled25.every((s) => !!s.stored && s.shown === s.stored && s.angle < 179.5);
console.log('Fold folds the regenerated sheet as one pattern on the drawn branch', JSON.stringify({ status: folded25.slice(0, 140), dof: post25.dof, drivers: post25.drivers, folded: post25.folded, flatLoops: post25.flatLoops, undo: [flat25.undo, post25.undo], sampled: sampled25 }), okFold25 ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/20-miura-sheet-folded.png` });
await page.click('button.mode-tab:has-text("Preview")');
await page.waitForTimeout(3000);
const travel25 = await page.evaluate(() => { const { app } = window.linkageDesigner; const m = app.model; const sw = app.sim.sweep; if (!sw) return null; const ground = Object.values(m.links).find((l) => l.ground); const gp = ground.pointIds.map((id) => m.points[id].pos); const farFromGround = (id) => Math.min(...gp.map((g) => Math.hypot(...g.map((v, k) => v - m.points[id].pos[k])))); return Object.values(m.links).filter((l) => !l.ground).map((l) => { const id = l.pointIds.reduce((b, x) => (farFromGround(x) > farFromGround(b) ? x : b), l.pointIds[0]); const a = sw.poses.map((p) => p.positions.get(id)); let mx = 0; for (const q of a) mx = Math.max(mx, Math.hypot(...q.map((v, k) => v - a[0][k]))); return +mx.toFixed(2); }); });
console.log('preview after the sheet fold moves every panel', JSON.stringify(travel25), travel25 && travel25.length === 15 && travel25.every((d) => d > 0.5) ? 'OK' : 'FAIL');
await page.screenshot({ path: `${OUT}/21-miura-sheet-preview.png` });
await page.click('button.mode-tab:has-text("Construction")');
await page.waitForTimeout(300);
console.log('errors:', errors.length ? errors.join('\n') : 'none');
await browser.close();
server.kill();
process.exit(0);
