/**
 * DOM user interface: top bar (modes, menus), tool palette with options,
 * properties panel, simulation / preview panels, status bar with coordinate
 * input, link-end constraint pop-up, settings dialog and help.
 * All text comes from strings.ts.
 */
import type { App, Mode, ToolName } from '../app';
import {
  barLength,
  changeJointType,
  jointsAtPoint,
  removeJoint,
  setGround,
} from '../core/model';
import { commitSketch, solveSketch } from '../core/kinematics';
import type { Construction, ID, Joint, JointType, Link, Target } from '../core/types';
import { isConstructionRef } from '../core/types';
import { createColorPicker } from './colorPicker';
import { icon } from './icons';
import { DEFAULT_SETTINGS, type AppSettings } from './settings';
import { APP, EXAMPLES, HELP, JOINTS, MENU, MODES, PANEL, POPUP, PREVIEW, SETTINGS, SIM, STATUS, TOOLS, TOOL_OPTIONS } from './strings';
import { ToolManager, parseTypedPoint } from '../viewport/tools';

const JOINT_TYPES: JointType[] = ['spherical', 'revolute', 'planar', 'prismatic', 'cylindrical', 'screw'];
const TOOL_LIST: ToolName[] = ['select', 'bar', 'polygon', 'prism', 'cylinder', 'cpoint', 'caxis', 'cplane', 'joint', 'ground', 'driver', 'delete'];

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls?: string, text?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function button(label: string, onClick: () => void, opts: { icon?: string; title?: string; cls?: string } = {}): HTMLButtonElement {
  const b = el('button', `btn ${opts.cls ?? ''}`);
  if (opts.icon) {
    const i = el('span', 'btn__icon');
    i.innerHTML = icon(opts.icon);
    b.appendChild(i);
  }
  if (label) b.appendChild(el('span', 'btn__label', label));
  b.title = opts.title ?? label;
  b.addEventListener('click', onClick);
  return b;
}

function row(label: string, control: HTMLElement, cls = ''): HTMLElement {
  const r = el('label', `prop-row ${cls}`);
  r.appendChild(el('span', 'prop-row__label', label));
  r.appendChild(control);
  return r;
}

function numberInput(value: number, onChange: (v: number) => void, opts: { step?: number; min?: number; max?: number; digits?: number } = {}): HTMLInputElement {
  const i = el('input');
  i.type = 'number';
  i.step = String(opts.step ?? 0.1);
  if (opts.min !== undefined) i.min = String(opts.min);
  if (opts.max !== undefined) i.max = String(opts.max);
  i.value = Number.isFinite(value) ? String(Number(value.toFixed(opts.digits ?? 4))) : '';
  i.addEventListener('change', () => {
    const v = Number(i.value);
    if (Number.isFinite(v)) onChange(v);
  });
  return i;
}

function checkbox(checked: boolean, onChange: (v: boolean) => void): HTMLInputElement {
  const i = el('input');
  i.type = 'checkbox';
  i.checked = checked;
  i.addEventListener('change', () => onChange(i.checked));
  return i;
}

function select<T extends string>(options: { value: T; label: string }[], value: T, onChange: (v: T) => void): HTMLSelectElement {
  const s = el('select');
  for (const o of options) {
    const op = el('option', undefined, o.label);
    op.value = o.value;
    s.appendChild(op);
  }
  s.value = value;
  s.addEventListener('change', () => onChange(s.value as T));
  return s;
}

function textInput(value: string, onChange: (v: string) => void): HTMLInputElement {
  const i = el('input');
  i.type = 'text';
  i.value = value;
  i.addEventListener('change', () => onChange(i.value));
  return i;
}

export class UI {
  root: HTMLElement;
  private toolbar!: HTMLElement;
  private toolOptions!: HTMLElement;
  private props!: HTMLElement;
  private modePanel!: HTMLElement;
  private statusText!: HTMLElement;
  private dofChip!: HTMLElement;
  private coordInput!: HTMLInputElement;
  private popup!: HTMLElement;
  private popupPointId: ID | null = null;
  private modeTabs = new Map<Mode, HTMLButtonElement>();
  private toolButtons = new Map<ToolName, HTMLButtonElement>();
  private undoBtn!: HTMLButtonElement;
  private redoBtn!: HTMLButtonElement;
  private settingsDialog: HTMLElement | null = null;
  private lastPanelKey = '';
  private sliderValue: HTMLInputElement | null = null;
  private previewReadout: HTMLElement | null = null;

  constructor(
    private app: App,
    private tools: ToolManager,
    container: HTMLElement,
  ) {
    this.root = container;
    this.build();
    app.subscribe(() => this.refresh());
    tools.onPopup = (pid) => this.showPopup(pid);
    this.refresh();
    document.addEventListener('keydown', (e) => this.onKey(e));
    // keep popup glued to its point
    const tick = () => {
      this.positionPopup();
      requestAnimationFrame(tick);
    };
    tick();
  }

  // ---------------------------------------------------------------------------
  // Layout
  // ---------------------------------------------------------------------------

  private build(): void {
    const app = this.app;
    const top = el('header', 'topbar');
    const brand = el('div', 'brand');
    brand.appendChild(el('span', 'brand__title', APP.title));
    brand.appendChild(el('span', 'brand__sub', APP.subtitle));
    top.appendChild(brand);

    const tabs = el('div', 'mode-tabs');
    (['construction', 'simulation', 'preview'] as Mode[]).forEach((m) => {
      const b = button(MODES[m], () => app.setMode(m), { title: MODES.tooltips[m], cls: 'mode-tab' });
      this.modeTabs.set(m, b);
      tabs.appendChild(b);
    });
    top.appendChild(tabs);

    const menus = el('div', 'menus');
    menus.appendChild(this.menu(MENU.file, [
      [MENU.newModel, () => confirm(MENU.confirmNew) && app.newModel()],
      [MENU.open, () => this.openFile()],
      [MENU.save, () => app.download(`${app.fileName || 'mechanism'}.linkage.json`, app.saveToJson())],
      [MENU.exportCsv, () => app.download('output-paths.csv', app.exportCsv(), 'text/csv')],
      [MENU.exportObj, () => app.download('mechanism.obj', app.exportObj(), 'text/plain')],
      [MENU.exportPng, () => this.exportPng()],
    ]));
    menus.appendChild(this.menu(MENU.examples, Object.entries(EXAMPLES).map(([k, v]) => [v.label, () => app.loadExample(k), v.description] as [string, () => void, string])));
    menus.appendChild(this.menu(MENU.view, [
      [MENU.viewTop, () => app.viewport.setView('top')],
      [MENU.viewFront, () => app.viewport.setView('front')],
      [MENU.viewRight, () => app.viewport.setView('right')],
      [MENU.viewIso, () => app.viewport.setView('iso')],
      [MENU.viewFit, () => app.zoomToFit()],
      [MENU.viewOrtho, () => app.updateSettings({ orthographic: true })],
      [MENU.viewPersp, () => app.updateSettings({ orthographic: false })],
    ]));
    this.undoBtn = button('', () => app.undo(), { icon: 'undo', title: MENU.undo, cls: 'btn--icon' });
    this.redoBtn = button('', () => app.redo(), { icon: 'redo', title: MENU.redo, cls: 'btn--icon' });
    menus.appendChild(this.undoBtn);
    menus.appendChild(this.redoBtn);
    menus.appendChild(button('', () => app.zoomToFit(), { icon: 'fit', title: MENU.viewFit, cls: 'btn--icon' }));
    menus.appendChild(button('', () => this.openSettings(), { icon: 'settings', title: MENU.settings, cls: 'btn--icon' }));
    menus.appendChild(button('', () => this.openHelp(), { icon: 'help', title: MENU.help, cls: 'btn--icon' }));
    top.appendChild(menus);

    this.toolbar = el('nav', 'toolbar');
    for (const t of TOOL_LIST) {
      const def = TOOLS[t];
      const b = button(def.label, () => this.tools.setTool(t), { icon: t, title: `${def.label} — ${def.hint}`, cls: 'tool-btn' });
      this.toolButtons.set(t, b);
      this.toolbar.appendChild(b);
    }
    this.toolOptions = el('div', 'tool-options');
    const left = el('aside', 'left');
    left.append(this.toolbar, this.toolOptions);

    const right = el('aside', 'right');
    this.modePanel = el('section', 'panel mode-panel');
    this.props = el('section', 'panel props-panel');
    right.append(this.modePanel, this.props);

    const bottom = el('footer', 'statusbar');
    this.statusText = el('span', 'status-text');
    this.dofChip = el('span', 'chip');
    const coordWrap = el('div', 'coord');
    this.coordInput = el('input', 'coord__input');
    this.coordInput.placeholder = STATUS.coordPlaceholder;
    this.coordInput.title = STATUS.coordHelp;
    this.coordInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const tp = parseTypedPoint(this.coordInput.value);
        if (tp) {
          this.tools.applyTyped(tp);
          this.coordInput.value = '';
        }
        e.stopPropagation();
      }
      if (e.key === 'Escape') this.coordInput.blur();
    });
    coordWrap.appendChild(this.coordInput);
    bottom.append(this.statusText, this.dofChip, coordWrap);

    this.popup = el('div', 'popup hidden');
    this.root.append(top, left, right, bottom, this.popup);
  }

  private menu(label: string, items: [string, () => void, string?][]): HTMLElement {
    const wrap = el('div', 'menu');
    const b = button(label, () => wrap.classList.toggle('open'), { cls: 'menu__button' });
    const list = el('div', 'menu__list');
    for (const [text, fn, desc] of items) {
      const it = el('button', 'menu__item');
      it.appendChild(el('span', 'menu__item-label', text));
      if (desc) it.appendChild(el('span', 'menu__item-desc', desc));
      it.addEventListener('click', () => {
        wrap.classList.remove('open');
        fn();
      });
      list.appendChild(it);
    }
    wrap.append(b, list);
    document.addEventListener('click', (e) => {
      if (!wrap.contains(e.target as Node)) wrap.classList.remove('open');
    });
    return wrap;
  }

  // ---------------------------------------------------------------------------
  // Refresh
  // ---------------------------------------------------------------------------

  refresh(): void {
    const app = this.app;
    for (const [m, b] of this.modeTabs) b.classList.toggle('active', app.mode === m);
    for (const [t, b] of this.toolButtons) b.classList.toggle('active', app.tool === t);
    this.toolbar.classList.toggle('disabled', app.mode !== 'construction');
    this.undoBtn.disabled = !app.canUndo;
    this.redoBtn.disabled = !app.canRedo;
    this.statusText.textContent = app.status || app.hint;
    this.statusText.title = app.hint;
    const mob = app.sim.mobility;
    if (mob) {
      const extra = mob.grounded ? '' : ` (${SIM.rigidBodyModes}: ${mob.rigidBodyModes})`;
      this.dofChip.textContent = `${STATUS.dof}: ${mob.dof}${extra}` + (app.sim.violation > 1e-5 ? ` · ${STATUS.violated}` : '');
      this.dofChip.classList.toggle('chip--warn', app.sim.violation > 1e-5);
    } else this.dofChip.textContent = '';
    // avoid rebuilding panels while the user is typing in them
    const active = document.activeElement;
    const typing = active && (active.tagName === 'INPUT' || active.tagName === 'SELECT') && (this.props.contains(active) || this.modePanel.contains(active) || this.toolOptions.contains(active));
    if (typing && active !== this.sliderValue) {
      this.updateLiveReadouts();
      return;
    }
    this.renderToolOptions();
    this.renderModePanel();
    this.renderProps();
  }

  private updateLiveReadouts(): void {
    if (this.previewReadout) this.previewReadout.textContent = this.app.preview.value.toFixed(1);
  }

  // ---------------------------------------------------------------------------
  // Tool options
  // ---------------------------------------------------------------------------

  private renderToolOptions(): void {
    const app = this.app;
    const o = app.toolOptions;
    const box = this.toolOptions;
    box.replaceChildren();
    const t = app.tool;
    if (app.mode !== 'construction') return;
    box.appendChild(el('h3', 'panel__title', TOOLS[t].label));
    box.appendChild(el('p', 'hint', TOOLS[t].hint));
    if (t === 'bar' || t === 'polygon') box.appendChild(row(TOOL_OPTIONS.mode2d, checkbox(o.mode2d, (v) => (o.mode2d = v))));
    if (t === 'polygon' || t === 'prism') box.appendChild(row(TOOL_OPTIONS.sides, numberInput(o.sides, (v) => (o.sides = Math.max(3, Math.round(v))), { step: 1, min: 3 })));
    if (t === 'prism') box.appendChild(row(TOOL_OPTIONS.height, numberInput(o.height, (v) => (o.height = v), { step: 0.1 })));
    if (t === 'cylinder') box.appendChild(row(TOOL_OPTIONS.radius, numberInput(o.radius, (v) => (o.radius = v), { step: 0.1, min: 0.01 })));
    if (t === 'cplane') {
      box.appendChild(row(TOOL_OPTIONS.planeMode, select([{ value: 'three', label: TOOL_OPTIONS.planeThree }, { value: 'offset', label: TOOL_OPTIONS.planeOffset }], o.planeMode, (v) => { o.planeMode = v; this.refresh(); })));
      if (o.planeMode === 'offset') box.appendChild(row(TOOL_OPTIONS.offset, numberInput(o.offset, (v) => (o.offset = v))));
    }
    if (t === 'joint') {
      box.appendChild(row(TOOL_OPTIONS.jointType, select(JOINT_TYPES.map((j) => ({ value: j, label: `${JOINTS[j].label} (${JOINTS[j].dof})` })), o.jointType, (v) => { o.jointType = v; this.refresh(); })));
      box.appendChild(el('p', 'hint', JOINTS[o.jointType].description));
      if (o.jointType === 'screw') box.appendChild(row(TOOL_OPTIONS.pitch, numberInput(o.pitch, (v) => (o.pitch = v))));
    }
    if (t === 'bar') {
      box.appendChild(row(TOOL_OPTIONS.jointType, select(JOINT_TYPES.filter((j) => j === 'revolute' || j === 'spherical').map((j) => ({ value: j, label: JOINTS[j].label })), app.model.settings.defaultJoint, (v) => (app.model.settings.defaultJoint = v))));
    }
    box.appendChild(row(TOOL_OPTIONS.snapGrid, checkbox(app.settings.gridSnap, (v) => app.updateSettings({ gridSnap: v }))));
    if (app.settings.gridSnap) box.appendChild(row(TOOL_OPTIONS.gridStep, numberInput(app.settings.gridStep, (v) => app.updateSettings({ gridStep: Math.max(0.01, v) }), { step: 0.1 })));
    box.appendChild(el('p', 'hint hint--small', TOOLS.cancel));
  }

  // ---------------------------------------------------------------------------
  // Mode panel (simulation / preview)
  // ---------------------------------------------------------------------------

  private renderModePanel(): void {
    const app = this.app;
    const m = app.model;
    const box = this.modePanel;
    box.replaceChildren();
    this.sliderValue = null;
    this.previewReadout = null;
    const sim = app.sim;
    const summary = el('div', 'summary');
    summary.appendChild(el('h3', 'panel__title', PANEL.mechanism));
    const links = Object.values(m.links);
    summary.appendChild(el('p', 'hint', `${links.length} ${PANEL.links} · ${Object.values(m.joints).filter((j) => j.a.kind !== 'body').length} ${PANEL.joints} · ${m.settings.displayPointIds.length} ${PANEL.pointsShown}`));
    if (sim.mobility) {
      summary.appendChild(el('p', 'readout', `${SIM.motionDOF}: ${sim.mobility.dof}${sim.mobility.grounded ? '' : ` — ${SIM.notGrounded}`}`));
    }
    box.appendChild(summary);
    if (app.mode === 'construction') return;

    // driver selection (shared by simulation & preview)
    box.appendChild(el('h3', 'panel__title', app.mode === 'simulation' ? SIM.panelTitle : PREVIEW.panelTitle));
    if (m.drivers.length === 0) {
      box.appendChild(el('p', 'hint warn', SIM.noDriver));
    } else {
      const opts = m.drivers.map((d, i) => ({ value: String(i), label: this.driverLabel(d) }));
      box.appendChild(row(SIM.driver, select(opts, String(sim.activeDriver), (v) => app.setActiveDriver(Number(v)))));
      const d = m.drivers[sim.activeDriver];
      if (d?.range) box.appendChild(el('p', 'readout', `${SIM.range}: ${d.range[0].toFixed(1)} … ${d.range[1].toFixed(1)} ${d.kind === 'slide' ? '' : '°'} — ${d.isCrank ? SIM.crank : SIM.rocker}`));
    }
    if (sim.message) box.appendChild(el('p', 'hint warn', sim.message));

    if (app.mode === 'simulation') {
      box.appendChild(row(SIM.poseCount, numberInput(m.settings.poseCount, (v) => { app.beginChange(); m.settings.poseCount = Math.max(3, Math.min(64, Math.round(v))); app.endChange(); }, { step: 1, min: 3, max: 64 })));
      const soft = row(SIM.softAssumptions, checkbox(m.settings.softAssumptions, (v) => { app.beginChange(); m.settings.softAssumptions = v; app.endChange(); }));
      soft.title = SIM.softAssumptionsHelp;
      box.appendChild(soft);
      box.appendChild(el('p', 'hint hint--small', SIM.timing));
      const an = sim.analysis;
      if (an) {
        const stats = el('div', 'stats');
        stats.appendChild(this.stat(SIM.designDOF, String(an.designDOF), an.designDOF === 0 ? 'ok' : ''));
        stats.appendChild(this.stat(SIM.editPointsConstrained, String(an.targetCount)));
        box.appendChild(stats);
        if (an.designDOF === 0 && !an.overConstrained) box.appendChild(el('p', 'hint ok', SIM.fullySpecified));
        if (an.designDOF > 0 && !m.settings.softAssumptions) box.appendChild(el('p', 'hint warn', SIM.underSpecifiedHidden));
        for (const s of an.spaces) {
          const pt = m.points[s.pointId];
          if (!pt) continue;
          box.appendChild(el('p', 'readout', `${SIM.designSpaceFor} ${m.links[pt.linkId]?.name ?? ''} ${pt.name}: ${SIM.designSpaceDim[Math.min(3, s.dim)]}`));
        }
      }
      box.appendChild(row(SIM.showDesignSpace, checkbox(sim.showDesignSpace, (v) => { sim.showDesignSpace = v; app.requestRender(); })));
      box.appendChild(row(SIM.showEditPoints, checkbox(sim.showEditPoints, (v) => { sim.showEditPoints = v; app.requestRender(); })));
      box.appendChild(row(SIM.showPath, checkbox(sim.showPaths, (v) => { sim.showPaths = v; app.requestRender(); })));
      const actions = el('div', 'actions');
      actions.appendChild(button(SIM.clearTargets, () => { app.beginChange(); m.targets = []; app.endChange(); }));
      const solid = button(SIM.solidify, () => app.solidifyAssumptions());
      solid.title = SIM.solidifyHelp;
      actions.appendChild(solid);
      box.appendChild(actions);
    } else {
      const sweep = sim.sweep;
      if (!sweep) return;
      const [lo, hi] = sweep.range;
      const slider = el('input', 'slider');
      slider.type = 'range';
      slider.min = String(lo);
      slider.max = String(hi);
      slider.step = String((hi - lo) / 720);
      slider.value = String(app.preview.value);
      slider.addEventListener('input', () => app.setPreviewValue(Number(slider.value)));
      this.sliderValue = slider;
      const readout = el('span', 'readout', app.preview.value.toFixed(1));
      this.previewReadout = readout;
      const r = row(PREVIEW.value, slider);
      r.appendChild(readout);
      box.appendChild(r);
      const play = button(app.preview.playing ? PREVIEW.pause : PREVIEW.play, () => { app.preview.playing = !app.preview.playing; this.refresh(); }, { icon: app.preview.playing ? 'pause' : 'play' });
      box.appendChild(play);
      box.appendChild(row(PREVIEW.speed, numberInput(app.preview.speed, (v) => (app.preview.speed = v), { step: 10 })));
      box.appendChild(row(PREVIEW.trace, checkbox(sim.showPaths, (v) => { sim.showPaths = v; app.requestRender(); })));
    }
  }

  private stat(label: string, value: string, cls = ''): HTMLElement {
    const s = el('div', `stat ${cls}`);
    s.appendChild(el('span', 'stat__value', value));
    s.appendChild(el('span', 'stat__label', label));
    return s;
  }

  private driverLabel(d: import('../core/types').Driver): string {
    const m = this.app.model;
    if (d.kind === 'angle') return `${m.links[d.linkId ?? '']?.name ?? ''} · angle`;
    const j = d.jointId ? m.joints[d.jointId] : null;
    return `${j ? JOINTS[j.type].label : ''} · ${d.kind}`;
  }

  // ---------------------------------------------------------------------------
  // Properties panel
  // ---------------------------------------------------------------------------

  private renderProps(): void {
    const app = this.app;
    const m = app.model;
    const box = this.props;
    box.replaceChildren();
    box.appendChild(el('h3', 'panel__title', PANEL.properties));
    const sel = app.selection;
    if (!sel) {
      box.appendChild(el('p', 'hint', PANEL.nothingSelected));
      return;
    }
    if (sel.type === 'link' || sel.type === 'edge' || sel.type === 'face' || sel.type === 'axis') {
      const link = m.links[sel.id];
      if (link) this.linkProps(box, link);
    } else if (sel.type === 'vertex' && sel.pointId) {
      const link = m.links[sel.id];
      if (link) {
        this.pointProps(box, sel.pointId);
        box.appendChild(el('hr'));
        this.linkProps(box, link);
      }
    } else if (sel.type === 'joint') {
      const j = m.joints[sel.id];
      if (j) this.jointProps(box, j);
    } else if (sel.type === 'construction') {
      const c = m.construction[sel.id];
      if (c) this.constructionProps(box, c);
    } else if (sel.type === 'editPoint' && sel.pointId !== undefined && sel.pose !== undefined) {
      this.editPointProps(box, sel.pointId, sel.pose);
    }
  }

  private linkProps(box: HTMLElement, link: Link): void {
    const app = this.app;
    const m = app.model;
    const change = (fn: () => void) => {
      app.beginChange();
      fn();
      app.endChange();
    };
    box.appendChild(row(PANEL.name, textInput(link.name, (v) => change(() => (link.name = v)))));
    box.appendChild(row(PANEL.type, el('span', 'value', `${link.kind}`)));
    if (link.kind === 'bar') {
      box.appendChild(
        row(
          PANEL.length,
          numberInput(barLength(m, link), (v) => change(() => {
            const r = link.rigidity.find((c) => c.kind === 'dist' && !c.fixed);
            if (r && r.kind === 'dist') r.length = Math.max(1e-3, v);
            const res = solveSketch(m, {});
            commitSketch(m, res);
          }), { step: 0.1, min: 0.001 }),
        ),
      );
    }
    if (link.params.sides) box.appendChild(row(PANEL.sides, el('span', 'value', String(link.params.sides))));
    if (link.params.radius) box.appendChild(row(PANEL.radius, el('span', 'value', link.params.radius.toFixed(3))));
    if (link.params.height) box.appendChild(row(PANEL.height, el('span', 'value', link.params.height.toFixed(3))));
    box.appendChild(row(PANEL.locked, checkbox(link.locked, (v) => change(() => (link.locked = v)))));
    box.appendChild(row(PANEL.ground, checkbox(link.ground, (v) => change(() => setGround(m, v ? link.id : null)))));
    box.appendChild(row(PANEL.flexible, checkbox(link.flexible, (v) => change(() => (link.flexible = v)))));
    if (link.flexible) {
      const s = el('input');
      s.type = 'range';
      s.min = '0.02';
      s.max = '1';
      s.step = '0.02';
      s.value = String(link.stiffness);
      s.addEventListener('change', () => change(() => (link.stiffness = Number(s.value))));
      box.appendChild(row(PANEL.stiffness, s));
    }
    const colorIn = el('input');
    colorIn.type = 'color';
    colorIn.value = link.color ?? app.settings.colors.geometry;
    colorIn.addEventListener('change', () => change(() => (link.color = colorIn.value)));
    const colorRow = row(PANEL.color, colorIn);
    if (link.color) colorRow.appendChild(button(PANEL.resetColor, () => change(() => delete link.color), { cls: 'btn--small' }));
    box.appendChild(colorRow);
    // display toggles per vertex
    const show = el('div', 'show-points');
    show.appendChild(el('span', 'prop-row__label', PANEL.showPath));
    for (const pid of link.pointIds) {
      const pt = m.points[pid];
      const on = m.settings.displayPointIds.includes(pid);
      const b = button(pt.name, () => change(() => {
        const ids = m.settings.displayPointIds;
        m.settings.displayPointIds = on ? ids.filter((x) => x !== pid) : [...ids, pid];
      }), { cls: `btn--small ${on ? 'active' : ''}`, icon: 'eye' });
      show.appendChild(b);
    }
    box.appendChild(show);
    box.appendChild(button(PANEL.deleteItem, () => this.tools.deletePick({ type: 'link', id: link.id }), { icon: 'delete', cls: 'btn--danger' }));
  }

  private pointProps(box: HTMLElement, pointId: ID): void {
    const app = this.app;
    const m = app.model;
    const pt = m.points[pointId];
    if (!pt) return;
    box.appendChild(el('h4', 'panel__subtitle', `${m.links[pt.linkId]?.name ?? ''} · ${pt.name}`));
    const pos = el('div', 'vec');
    (['x', 'y', 'z'] as const).forEach((axis, i) => {
      pos.appendChild(
        numberInput(pt.pos[i], (v) => {
          app.beginChange();
          const target: [number, number, number] = [...pt.pos] as [number, number, number];
          target[i] = v;
          const link = m.links[pt.linkId];
          const res = solveSketch(m, { dragTargets: [{ pointId, pos: target, weight: 1 }], freePointIds: new Set([pointId]), allowGroundMove: link.ground });
          commitSketch(m, res, new Set([pointId]));
          app.endChange();
        }, { step: 0.1, digits: 4 }),
      );
      pos.lastElementChild!.setAttribute('title', axis);
    });
    box.appendChild(row(PANEL.position, pos));
    const js = jointsAtPoint(m, pointId);
    const list = el('div', 'joint-list');
    if (js.length === 0) list.appendChild(el('span', 'hint', PANEL.none));
    for (const j of js) {
      const item = button(`${JOINTS[j.type].label}`, () => app.select({ type: 'joint', id: j.id }), { icon: j.type, cls: 'btn--small' });
      list.appendChild(item);
    }
    box.appendChild(row(PANEL.pointConstraints, list));
    const on = m.settings.displayPointIds.includes(pointId);
    box.appendChild(
      row(
        PANEL.showPath,
        checkbox(on, (v) => {
          app.beginChange();
          const ids = m.settings.displayPointIds;
          m.settings.displayPointIds = v ? [...ids, pointId] : ids.filter((x) => x !== pointId);
          app.endChange();
        }),
      ),
    );
  }

  private jointProps(box: HTMLElement, j: Joint): void {
    const app = this.app;
    const m = app.model;
    box.appendChild(el('h4', 'panel__subtitle', JOINTS[j.type].label));
    box.appendChild(el('p', 'hint', JOINTS[j.type].description));
    const a = m.links[j.a.linkId]?.name ?? '';
    const b = isConstructionRef(j.b) ? m.construction[j.b.constructionId]?.name ?? '' : m.links[j.b.linkId]?.name ?? '';
    box.appendChild(row(PANEL.jointLinks, el('span', 'value', `${a} ↔ ${b}`)));
    if (j.a.kind !== 'body') {
      box.appendChild(
        row(
          PANEL.changeType,
          select(JOINT_TYPES.map((t) => ({ value: t, label: JOINTS[t].label })), j.type, (t) => {
            app.beginChange();
            const nj = changeJointType(m, j.id, t);
            if (nj) {
              const res = solveSketch(m, {});
              commitSketch(m, res);
              app.select({ type: 'joint', id: nj.id });
            } else app.setStatus(STATUS.jointIncompatible);
            app.endChange();
          }),
        ),
      );
    }
    if (j.axis) box.appendChild(row(PANEL.jointAxis, el('span', 'value', j.axis.map((v) => v.toFixed(2)).join(', '))));
    if (j.type === 'screw') box.appendChild(row(PANEL.jointPitch, numberInput(j.pitch ?? 1, (v) => { app.beginChange(); j.pitch = v; app.endChange(); })));
    if (j.type === 'revolute' && !isConstructionRef(j.b)) {
      const hinge = j.hinge ?? { enabled: false, stiffness: 0.3, restAngle: 0 };
      box.appendChild(row(PANEL.jointHinge, checkbox(hinge.enabled, (v) => { app.beginChange(); j.hinge = { ...hinge, enabled: v }; app.endChange(); })));
      if (hinge.enabled) {
        box.appendChild(row(PANEL.hingeRest, numberInput(hinge.restAngle, (v) => { app.beginChange(); j.hinge = { ...hinge, restAngle: v }; app.endChange(); }, { step: 5 })));
        box.appendChild(row(PANEL.hingeStiffness, numberInput(hinge.stiffness, (v) => { app.beginChange(); j.hinge = { ...hinge, stiffness: Math.max(0.01, Math.min(1, v)) }; app.endChange(); }, { step: 0.05, min: 0.01, max: 1 })));
      }
    }
    box.appendChild(button(PANEL.deleteItem, () => this.tools.deletePick({ type: 'joint', id: j.id }), { icon: 'delete', cls: 'btn--danger' }));
  }

  private constructionProps(box: HTMLElement, c: Construction): void {
    const app = this.app;
    const m = app.model;
    box.appendChild(el('h4', 'panel__subtitle', `${c.name} (${c.kind})`));
    box.appendChild(row(PANEL.name, textInput(c.name, (v) => { app.beginChange(); c.name = v; app.endChange(); })));
    box.appendChild(row(PANEL.constructionOrigin, el('span', 'value', c.origin.map((v) => v.toFixed(3)).join(', '))));
    if (c.dir) box.appendChild(row(PANEL.constructionDir, el('span', 'value', c.dir.map((v) => v.toFixed(3)).join(', '))));
    if (c.kind === 'plane') {
      const isSketch = m.settings.sketchPlaneId === c.id;
      box.appendChild(isSketch ? el('p', 'hint ok', PANEL.isSketchPlane) : button(PANEL.sketchPlane, () => { app.beginChange(); m.settings.sketchPlaneId = c.id; app.endChange(); }));
    }
    if (c.builtin) box.appendChild(el('p', 'hint', PANEL.builtin));
    else box.appendChild(button(PANEL.deleteItem, () => this.tools.deletePick({ type: 'construction', id: c.id }), { icon: 'delete', cls: 'btn--danger' }));
  }

  private editPointProps(box: HTMLElement, pointId: ID, pose: number): void {
    const app = this.app;
    const m = app.model;
    const pt = m.points[pointId];
    if (!pt) return;
    box.appendChild(el('h4', 'panel__subtitle', `${PANEL.editPoint}: ${m.links[pt.linkId]?.name ?? ''} ${pt.name}`));
    const value = app.sim.poseValues[pose];
    box.appendChild(row(PANEL.editPointPose, el('span', 'value', `${pose}${value !== undefined ? ` (${value.toFixed(1)})` : ''}`)));
    const t = app.targetFor(pointId, pose);
    const change = (fn: () => void) => {
      app.beginChange();
      fn();
      app.endChange();
    };
    if (!t) {
      box.appendChild(row(PANEL.editPointTarget, el('span', 'value', PANEL.editPointFree)));
    } else {
      const desc = t.kind === 'position' && t.position ? `${t.kind}: ${t.position.map((v) => v.toFixed(3)).join(', ')}` : `${t.kind}: ${m.construction[t.constructionId ?? '']?.name ?? ''}`;
      box.appendChild(row(PANEL.editPointTarget, el('span', 'value', desc)));
      if (t.kind === 'position' && t.position) {
        const pos = el('div', 'vec');
        (['x', 'y', 'z'] as const).forEach((_axis, i) => {
          pos.appendChild(numberInput(t.position![i], (v) => change(() => { t.position![i] = v; }), { step: 0.1 }));
        });
        box.appendChild(row(PANEL.position, pos));
      }
      box.appendChild(row(PANEL.editPointLocked, checkbox(t.locked, (v) => change(() => (t.locked = v)))));
      box.appendChild(button(PANEL.editPointRelease, () => change(() => (m.targets = m.targets.filter((x) => x !== t))), { icon: 'close' }));
    }
    // constrain to construction geometry
    const cons = Object.values(m.construction).filter((c) => !(c.kind === 'plane' && c.builtin && c.id === m.settings.sketchPlaneId) || true);
    const opts = [{ value: '', label: PANEL.constrainToNone }, ...cons.map((c) => ({ value: c.id, label: `${c.name} (${c.kind})` }))];
    box.appendChild(
      row(
        PANEL.constrainTo,
        select(opts, t && t.kind !== 'position' ? t.constructionId ?? '' : '', (cid) => {
          if (!cid) return;
          const c = m.construction[cid];
          const kind: Target['kind'] = c.kind === 'point' ? 'onPoint' : c.kind === 'axis' ? 'onAxis' : 'onPlane';
          change(() => {
            m.targets = m.targets.filter((x) => !(x.pointId === pointId && x.pose === pose));
            m.targets.push({ id: `t_${m.nextId++}`, pointId, pose, kind, constructionId: cid, locked: false });
          });
        }),
      ),
    );
  }

  // ---------------------------------------------------------------------------
  // Link-end pop-up (constraint icon + remove)
  // ---------------------------------------------------------------------------

  showPopup(pointId: ID | null): void {
    this.popupPointId = pointId;
    this.popup.replaceChildren();
    if (!pointId || this.app.mode !== 'construction') {
      this.popup.classList.add('hidden');
      return;
    }
    const app = this.app;
    const m = app.model;
    const js = jointsAtPoint(m, pointId).filter((j) => j.a.kind !== 'body');
    const link = m.links[m.points[pointId]?.linkId ?? ''];
    const j = js[0];
    const typeBtn = el('button', 'popup__btn');
    typeBtn.innerHTML = icon(j ? j.type : 'none');
    typeBtn.title = j ? `${JOINTS[j.type].label} — ${POPUP.changeConstraint}` : POPUP.noConstraint;
    typeBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      if (!j) {
        app.setStatus(TOOLS.joint.hint);
        return;
      }
      // cycle through compatible joint types
      const idx = JOINT_TYPES.indexOf(j.type);
      for (let k = 1; k <= JOINT_TYPES.length; k++) {
        const t = JOINT_TYPES[(idx + k) % JOINT_TYPES.length];
        app.beginChange();
        const nj = changeJointType(m, j.id, t);
        if (nj) {
          const res = solveSketch(m, {});
          commitSketch(m, res);
          app.endChange();
          app.setStatus(`${JOINTS[t].label}`);
          this.showPopup(pointId);
          return;
        }
        app.endChange({ skipUndo: true });
      }
    });
    const removeBtn = el('button', 'popup__btn popup__btn--remove');
    removeBtn.innerHTML = icon('close');
    removeBtn.title = POPUP.removeConstraint;
    removeBtn.disabled = !j;
    removeBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      if (!j) return;
      app.beginChange();
      removeJoint(m, j.id);
      app.endChange();
      this.showPopup(pointId);
    });
    const lockBtn = el('button', 'popup__btn popup__btn--lock');
    lockBtn.innerHTML = icon(link?.locked ? 'lock' : 'unlock');
    lockBtn.title = link?.locked ? POPUP.unlockLink : POPUP.lockLink;
    lockBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      if (!link) return;
      app.beginChange();
      link.locked = !link.locked;
      app.endChange();
      this.showPopup(pointId);
    });
    this.popup.append(typeBtn, removeBtn, lockBtn);
    this.popup.classList.remove('hidden');
    this.positionPopup();
  }

  private positionPopup(): void {
    if (!this.popupPointId || this.popup.classList.contains('hidden')) return;
    const pt = this.app.model.points[this.popupPointId];
    if (!pt) {
      this.popup.classList.add('hidden');
      return;
    }
    const s = this.app.viewport.worldToScreen(pt.pos);
    const rootRect = this.root.getBoundingClientRect();
    this.popup.style.left = `${s.x - rootRect.left + 14}px`;
    this.popup.style.top = `${s.y - rootRect.top - 18}px`;
  }

  // ---------------------------------------------------------------------------
  // Settings & help dialogs, files
  // ---------------------------------------------------------------------------

  private openSettings(): void {
    if (this.settingsDialog) this.settingsDialog.remove();
    const app = this.app;
    const dlg = el('div', 'dialog-backdrop');
    const box = el('div', 'dialog');
    box.appendChild(el('h2', 'dialog__title', SETTINGS.title));
    box.appendChild(el('h3', 'panel__title', SETTINGS.colors));
    const colorKeys: (keyof AppSettings['colors'])[] = ['geometry', 'ground', 'construction', 'designSpace', 'outputPath', 'editPointFree', 'editPointConstrained', 'background', 'gridMajor', 'gridMinor', 'selection'];
    const pickers: ReturnType<typeof createColorPicker>[] = [];
    for (const key of colorKeys) {
      const p = createColorPicker(SETTINGS[key as keyof typeof SETTINGS] as string, app.settings.colors[key], (hex) => app.updateSettings({ colors: { [key]: hex } }));
      pickers.push(p);
      box.appendChild(p.element);
    }
    const op = el('input');
    op.type = 'range';
    op.min = '0.05';
    op.max = '1';
    op.step = '0.05';
    op.value = String(app.settings.designSpaceOpacity);
    op.addEventListener('input', () => app.updateSettings({ designSpaceOpacity: Number(op.value) }));
    box.appendChild(row(SETTINGS.designSpaceOpacity, op));
    box.appendChild(el('h3', 'panel__title', SETTINGS.display));
    box.appendChild(row(SETTINGS.showLabels, checkbox(app.settings.showLabels, (v) => app.updateSettings({ showLabels: v }))));
    box.appendChild(row(SETTINGS.showConstruction, checkbox(app.settings.showConstruction, (v) => app.updateSettings({ showConstruction: v }))));
    box.appendChild(row(SETTINGS.showHelpers, checkbox(app.settings.showHelpers, (v) => app.updateSettings({ showHelpers: v }))));
    box.appendChild(row(SETTINGS.gridSnap, checkbox(app.settings.gridSnap, (v) => app.updateSettings({ gridSnap: v }))));
    box.appendChild(row(SETTINGS.gridStep, numberInput(app.settings.gridStep, (v) => app.updateSettings({ gridStep: Math.max(0.01, v) }), { step: 0.1 })));
    box.appendChild(row(SETTINGS.helperOffset, numberInput(app.model.settings.helperOffset, (v) => { app.model.settings.helperOffset = Math.max(0.01, v); }, { step: 0.1 })));
    const actions = el('div', 'actions');
    actions.appendChild(button(SETTINGS.resetDefaults, () => {
      app.updateSettings(structuredClone(DEFAULT_SETTINGS));
      pickers.forEach((p, i) => p.setValue(app.settings.colors[colorKeys[i]]));
    }));
    actions.appendChild(button(SETTINGS.close, () => dlg.remove(), { cls: 'btn--primary' }));
    box.appendChild(actions);
    dlg.appendChild(box);
    dlg.addEventListener('click', (e) => {
      if (e.target === dlg) dlg.remove();
    });
    this.root.appendChild(dlg);
    this.settingsDialog = dlg;
  }

  private openHelp(): void {
    const dlg = el('div', 'dialog-backdrop');
    const box = el('div', 'dialog');
    box.appendChild(el('h2', 'dialog__title', HELP.title));
    const ul = el('ul', 'help-list');
    for (const line of HELP.lines) ul.appendChild(el('li', undefined, line));
    box.appendChild(ul);
    box.appendChild(button(SETTINGS.close, () => dlg.remove(), { cls: 'btn--primary' }));
    dlg.appendChild(box);
    dlg.addEventListener('click', (e) => {
      if (e.target === dlg) dlg.remove();
    });
    this.root.appendChild(dlg);
  }

  private openFile(): void {
    const input = el('input');
    input.type = 'file';
    input.accept = '.json,application/json';
    input.addEventListener('change', async () => {
      const f = input.files?.[0];
      if (!f) return;
      try {
        const text = await f.text();
        if (Object.keys(this.app.model.links).length > 0 && !confirm(MENU.confirmLoad)) return;
        this.app.loadFromJson(text, f.name.replace(/\.linkage\.json$|\.json$/, ''));
      } catch (e) {
        alert(MENU.loadError + String(e));
      }
    });
    input.click();
  }

  private exportPng(): void {
    const url = this.app.viewport.screenshotDataUrl();
    const a = document.createElement('a');
    a.href = url;
    a.download = 'mechanism.png';
    a.click();
  }

  // ---------------------------------------------------------------------------
  // Keyboard
  // ---------------------------------------------------------------------------

  private onKey(e: KeyboardEvent): void {
    const target = e.target as HTMLElement;
    if (target && (target.tagName === 'INPUT' || target.tagName === 'SELECT' || target.tagName === 'TEXTAREA')) return;
    const app = this.app;
    if (e.key === 'Escape') {
      this.tools.cancel();
      this.showPopup(null);
    } else if (e.key === 'Delete' || e.key === 'Backspace') {
      if (app.selection) this.tools.deletePick({ type: app.selection.type, id: app.selection.id, pointId: app.selection.pointId, pose: app.selection.pose });
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
      e.preventDefault();
      if (e.shiftKey) app.redo();
      else app.undo();
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') {
      e.preventDefault();
      app.redo();
    } else if (app.mode === 'construction' && !e.ctrlKey && !e.metaKey) {
      const map: Record<string, ToolName> = { '1': 'select', '2': 'bar', '3': 'polygon', '4': 'prism', '5': 'cylinder', g: 'ground', j: 'joint', d: 'driver' };
      const t = map[e.key.toLowerCase()];
      if (t) this.tools.setTool(t);
      if (e.key.toLowerCase() === 'f') app.zoomToFit();
    } else if (e.key.toLowerCase() === 'f') app.zoomToFit();
    else if (e.key === ' ' && app.mode === 'preview') {
      e.preventDefault();
      app.preview.playing = !app.preview.playing;
      this.refresh();
    }
  }
}

