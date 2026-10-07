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
console.log('errors:', errors.length ? errors.join('\n') : 'none');
await browser.close();
server.kill();
process.exit(0);
