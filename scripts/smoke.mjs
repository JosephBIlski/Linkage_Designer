// Headless smoke test: builds must exist (npm run build). Usage: node scripts/smoke.mjs  (set SHOTS_DIR for screenshots)
import { chromium } from 'playwright';
import { spawn } from 'node:child_process';

const OUT = process.env.SHOTS_DIR ?? 'test-results';
const server = spawn('npx', ['vite', 'preview', '--port', '4173', '--strictPort'], { cwd: process.cwd(), stdio: 'pipe' });
await new Promise((r) => setTimeout(r, 2500));
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || undefined, args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 860 } });
const errors = [];
page.on('console', (msg) => { if (msg.type() === 'error' || msg.type() === 'warning') errors.push(`${msg.type()}: ${msg.text()}`); });
page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));
await page.goto('http://localhost:4173/');
await page.waitForTimeout(2500);
await page.screenshot({ path: `${OUT}/01-construction.png` });
const info1 = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  return { links: Object.keys(app.model.links).length, dof: app.sim.mobility?.dof, drivers: app.model.drivers.length, sweep: app.sim.sweep ? { range: app.sim.sweep.range, crank: app.sim.sweep.isCrank, poses: app.sim.sweep.poses.length } : null };
});
console.log('construction', JSON.stringify(info1));
// Simulation mode
await page.evaluate(() => window.linkageDesigner.app.setMode('simulation'));
await page.waitForTimeout(1500);
await page.screenshot({ path: `${OUT}/02-simulation.png` });
const info2 = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  const an = app.sim.analysis;
  return { designDOF: an?.designDOF, nullity: an?.nullity, spaces: an?.spaces.map((s) => ({ dim: s.dim, radius: s.radius })), converged: app.sim.design?.converged, residual: app.sim.design?.residual, message: app.sim.message };
});
console.log('simulation', JSON.stringify(info2));
// Drag an editing point programmatically: find its screen position
const drag = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  const st = app.renderState();
  const ep = st.editPoints[5];
  const s = app.viewport.worldToScreen(ep.pos);
  return { x: s.x, y: s.y, pose: ep.pose, pos: ep.pos };
});
console.log('edit point', JSON.stringify(drag));
await page.mouse.move(drag.x, drag.y);
await page.waitForTimeout(200);
await page.mouse.down();
for (let i = 1; i <= 10; i++) {
  await page.mouse.move(drag.x + i * 6, drag.y - i * 4);
  await page.waitForTimeout(60);
}
await page.mouse.up();
await page.waitForTimeout(1500);
await page.screenshot({ path: `${OUT}/03-simulation-dragged.png` });
const info3 = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  const an = app.sim.analysis;
  const t = app.model.targets[0];
  const st = app.renderState();
  const ep = st.editPoints.find((e) => t && e.pose === t.pose);
  return { targets: app.model.targets.length, target: t?.position, reached: ep?.pos, designDOF: an?.designDOF, converged: app.sim.design?.converged, residual: app.sim.design?.residual, message: app.sim.message, lengths: Object.values(app.model.links).map((l) => [l.name, l.rigidity.filter((r) => r.kind === 'dist' && !r.fixed).map((r) => +r.length.toFixed(3))]) };
});
console.log('after drag', JSON.stringify(info3));
// Preview mode
await page.evaluate(() => { window.linkageDesigner.app.setMode('preview'); window.linkageDesigner.app.preview.playing = true; });
await page.waitForTimeout(1200);
await page.screenshot({ path: `${OUT}/04-preview.png` });
// Other examples
for (const ex of ['sliderCrank', 'sphericalPendulum', 'miuraVertex']) {
  await page.evaluate((k) => { const { app } = window.linkageDesigner; app.setMode('construction'); app.loadExample(k); }, ex);
  await page.waitForTimeout(1200);
  await page.evaluate(() => window.linkageDesigner.app.setMode('simulation'));
  await page.waitForTimeout(1500);
  const info = await page.evaluate(() => {
    const { app } = window.linkageDesigner;
    return { dof: app.sim.mobility?.dof, grounded: app.sim.mobility?.grounded, sweep: app.sim.sweep ? { range: app.sim.sweep.range.map((v) => +v.toFixed(1)), crank: app.sim.sweep.isCrank, n: app.sim.sweep.poses.length } : null, designDOF: app.sim.analysis?.designDOF, dims: app.sim.analysis?.spaces.map((s) => s.dim), msg: app.sim.message };
  });
  console.log(ex, JSON.stringify(info));
  await page.screenshot({ path: `${OUT}/05-${ex}.png` });
}
// ---- Construction tools driven by real pointer events ----
await page.evaluate(() => { const { app } = window.linkageDesigner; app.setMode('construction'); app.newModel(); app.viewport.setView('top'); app.viewport.fit([[-1, -1, 0], [5, 3, 0]]); });
await page.waitForTimeout(600);
const W = async (p) => page.evaluate((p) => window.linkageDesigner.app.viewport.worldToScreen(p), p);
const clickWorld = async (p) => { const s = await W(p); await page.mouse.move(s.x, s.y); await page.waitForTimeout(40); await page.mouse.down(); await page.mouse.up(); await page.waitForTimeout(120); };
await page.click('button[title^="Link —"]');
await clickWorld([0, 0, 0]);
await clickWorld([2, 0, 0]);
// second link starting on the end of the first (snap → joint)
await clickWorld([2, 0, 0]);
await clickWorld([3, 1.5, 0]);
// third link from the free end of the second to the ground side and a 4th back to the origin: build a four-bar
await clickWorld([3, 1.5, 0]);
await clickWorld([5, 1, 0]);
await clickWorld([5, 1, 0]);
await clickWorld([0, 0, 0]);
await page.click('button[title^="Ground —"]');
await clickWorld([2.5, 0.5, 0]); // body of link 4 (from (5,1) to (0,0)) at its midpoint
await page.waitForTimeout(400);
const built = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  const m = app.model;
  return { links: Object.keys(m.links).length, joints: Object.values(m.joints).filter((j) => j.a.kind !== 'body').map((j) => j.type), ground: Object.values(m.links).find((l) => l.ground)?.name, dof: app.sim.mobility?.dof, violation: app.sim.violation };
});
console.log('built by tools', JSON.stringify(built));
// select tool: click a link end → popup; drag the end to change the length
await page.click('button[title^="Select —"]');
await clickWorld([3, 1.5, 0]);
await page.waitForTimeout(300);
const popupVisible = await page.evaluate(() => !document.querySelector('.popup').classList.contains('hidden'));
const s1 = await W([3, 1.5, 0]);
await page.mouse.move(s1.x, s1.y);
await page.mouse.down();
for (let i = 1; i <= 8; i++) { await page.mouse.move(s1.x + i * 5, s1.y - i * 3); await page.waitForTimeout(40); }
await page.mouse.up();
await page.waitForTimeout(400);
const afterDrag = await page.evaluate(() => {
  const { app } = window.linkageDesigner;
  return { popupSeen: true, dof: app.sim.mobility?.dof, violation: app.sim.violation, lengths: Object.values(app.model.links).map((l) => +l.rigidity.find((r) => r.kind === 'dist' && !r.fixed).length.toFixed(3)) };
});
console.log('popup visible', popupVisible, 'after end drag', JSON.stringify(afterDrag));
await page.screenshot({ path: `${OUT}/06-built-by-tools.png` });
// undo twice and check
await page.keyboard.press('Control+z');
await page.waitForTimeout(200);
const undone = await page.evaluate(() => Object.values(window.linkageDesigner.app.model.links).map((l) => +l.rigidity.find((r) => r.kind === 'dist' && !r.fixed).length.toFixed(3)));
console.log('after undo', JSON.stringify(undone));
// simulation on the hand-built four-bar: show a point, go to simulation
await page.evaluate(() => { const { app } = window.linkageDesigner; const l = Object.values(app.model.links)[1]; app.beginChange(); app.model.settings.displayPointIds = [l.pointIds[1]]; app.endChange(); app.setMode('simulation'); });
await page.waitForTimeout(1500);
const simBuilt = await page.evaluate(() => { const { app } = window.linkageDesigner; return { drivers: app.model.drivers.length, range: app.sim.sweep?.range.map((v) => +v.toFixed(1)), crank: app.sim.sweep?.isCrank, designDOF: app.sim.analysis?.designDOF, msg: app.sim.message }; });
console.log('sim on built', JSON.stringify(simBuilt));
await page.screenshot({ path: `${OUT}/07-built-simulation.png` });
if (process.env.EXPORT_EXAMPLES) {
  const fs = await import('node:fs');
  for (const ex of ['fourBar', 'sliderCrank', 'sphericalPendulum', 'miuraVertex']) {
    const json = await page.evaluate((k) => { const { app } = window.linkageDesigner; app.setMode('construction'); app.loadExample(k); return app.saveToJson(); }, ex);
    fs.writeFileSync(`examples/${ex}.linkage.json`, json);
  }
  console.log('examples exported');
}
console.log('console errors:', errors.length ? errors.join('\n') : 'none');
await browser.close();
server.kill();
process.exit(0);
