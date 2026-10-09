/**
 * DOM user interface: top bar (modes, menus), tool palette with options,
 * properties panel, simulation / preview panels, status bar with coordinate
 * input, link-end constraint pop-up, settings dialog and help.
 * All text comes from strings.ts.
 */
import type { App, Mode, ToolName } from '../app';
import {
  barLength,
  jointsAtPoint,
  removeJoint,
  setGround,
} from '../core/model';
import { consistencyTolerance, poseIsConsistent, tryChangeJointType, trySolveCommit } from '../core/feasibility';
import { DEFAULT_PREFOLD_DEG, creaseDihedralDeg, creaseMV, driveCrease, foldCreaseToTarget, isCrease, prefoldVertex } from '../core/fold';
import type { Construction, ID, Joint, JointType, Link, Target, Vec3 } from '../core/types';
import { isConstructionRef } from '../core/types';
import { createColorPicker } from './colorPicker';
import { icon } from './icons';
import { DEFAULT_SETTINGS, type AppSettings } from './settings';
import { APP, EXAMPLES, HELP, JOINTS, MENU, MODES, PANEL, POPUP, PREVIEW, SETTINGS, SIM, STATUS, TOOLS, TOOL_OPTIONS, TREE, creaseLabel, jointRefusedMessage } from './strings';
import { duplicateLink, extrudePolygon, translation } from '../core/patterns';
import { currentViolation, modelSize } from '../core/kinematics';
import { bodyPlaneJoint, setBodyPlane } from '../core/model';
import { add, cross, scale } from '../core/geometry';
import { ToolManager, parseTypedPoint } from '../viewport/tools';

const JOINT_TYPES: JointType[] = ['spherical', 'revolute', 'planar', 'prismatic', 'cylindrical', 'screw'];
const TOOL_LIST: ToolName[] = ['select', 'bar', 'sketch', 'polygon', 'prism', 'cylinder', 'edit', 'cpoint', 'caxis', 'cplane', 'joint', 'ground', 'driver', 'mirror', 'pattern', 'delete'];

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
  private tree!: HTMLElement;
  private treeOpen: Record<string, boolean> = { links: true, joints: true, construction: false, drivers: true, targets: true };
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
  /** The model object the panels were last built from; a replaced model (abortChange, undo, load) forces a rebuild. */
  private builtFor: import('../core/types').Model | null = null;

  constructor(
    private app: App,
    private tools: ToolManager,
    container: HTMLElement,
  ) {
    this.root = container;
    this.build();
    app.subscribe(() => this.refresh());
    app.onStatus = () => {
      this.statusText.textContent = app.status || app.hint;
      this.statusText.title = app.status || app.hint; // the whole message as a tooltip: long diagnoses are clipped in the bar
    };
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
    this.tree = el('section', 'panel tree-panel');
    this.modePanel = el('section', 'panel mode-panel');
    this.props = el('section', 'panel props-panel');
    right.append(this.tree, this.modePanel, this.props);

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
    this.statusText.title = app.status || app.hint;
    const mob = app.sim.mobility;
    if (mob) {
      const extra = mob.grounded ? '' : ` (${SIM.rigidBodyModes}: ${mob.rigidBodyModes})`;
      // the same tolerance that refuses Solidify / baking on Save, so the chip warns exactly when they would refuse
      const violated = app.sim.violation > consistencyTolerance(app.model);
      this.dofChip.textContent = `${STATUS.dof}: ${mob.dof}${extra}` + (violated ? ` · ${STATUS.violated}` : '');
      this.dofChip.classList.toggle('chip--warn', violated);
      const locked = app.sim.lockedCreaseIds.length;
      this.dofChip.title = locked ? SIM.creasesLocked(locked, this.foldAvailable()) : '';
    } else {
      this.dofChip.textContent = '';
      this.dofChip.title = '';
    }
    // avoid rebuilding panels while the user is typing in them, unless the model object was replaced (a refused
    // edit's abortChange, undo / redo, a loaded file): the widgets and their closures then belong to a discarded
    // model and would show the refused values and write edits into it
    const modelReplaced = this.builtFor !== app.model;
    this.builtFor = app.model;
    const active = document.activeElement;
    const typing = active && (active.tagName === 'INPUT' || active.tagName === 'SELECT') && (this.props.contains(active) || this.modePanel.contains(active) || this.toolOptions.contains(active) || this.tree.contains(active));
    if (typing && active !== this.sliderValue && !modelReplaced) {
      this.updateLiveReadouts();
      return;
    }
    this.renderToolOptions();
    this.renderTree();
    this.renderModePanel();
    this.renderProps();
  }

  // ---------------------------------------------------------------------------
  // Feature tree
  // ---------------------------------------------------------------------------

  private treeSignature = '';

  private renderTree(): void {
    const app = this.app;
    const m = app.model;
    const sel = app.selection;
    const sig = [
      Object.values(m.links).map((l) => `${l.id}:${l.name}:${l.kind}:${+l.ground}${+l.locked}${+l.flexible}${+!!l.hidden}`).join(','),
      // crease labels carry the fold angle rounded to 1° and the M/V class, so the tree follows the pose
      Object.values(m.joints).map((j) => `${j.id}:${j.type}:${this.jointLabel(j)}`).join(','),
      Object.values(m.construction).map((c) => `${c.id}:${c.name}`).join(','),
      m.settings.sketchPlaneId,
      m.drivers.map((d) => d.id).join(','),
      m.targets.map((t) => `${t.id}:${t.kind}:${+t.locked}`).join(','),
      app.mode,
      app.sim.activeDriver,
      sel ? `${sel.type}:${sel.id}:${sel.pointId ?? ''}:${sel.pose ?? ''}` : '',
    ].join('|');
    if (sig === this.treeSignature && this.tree.childElementCount > 0) return;
    this.treeSignature = sig;
    const box = this.tree;
    box.replaceChildren();
    box.appendChild(el('h3', 'panel__title', TREE.title));
    const group = (key: string, title: string, count: number, fill: (list: HTMLElement) => void) => {
      const details = el('details', 'tree-group');
      details.open = this.treeOpen[key] ?? true;
      details.addEventListener('toggle', () => (this.treeOpen[key] = details.open));
      const summary = el('summary', 'tree-group__summary', `${title} (${count})`);
      details.appendChild(summary);
      const list = el('div', 'tree-list');
      if (count === 0) list.appendChild(el('span', 'hint hint--small', TREE.empty));
      else fill(list);
      details.appendChild(list);
      box.appendChild(details);
    };
    const item = (label: string, active: boolean, onClick: () => void, opts: { badges?: string[]; hover?: () => void; rename?: (v: string) => void; extra?: HTMLElement; iconName?: string } = {}) => {
      const row = el('div', `tree-item ${active ? 'active' : ''}`);
      if (opts.iconName) {
        const ic = el('span', 'tree-item__icon');
        ic.innerHTML = icon(opts.iconName);
        row.appendChild(ic);
      }
      const name = el('span', 'tree-item__label', label);
      if (opts.rename) name.title = TREE.rename;
      row.appendChild(name);
      for (const b of opts.badges ?? []) row.appendChild(el('span', 'tree-badge', b));
      if (opts.extra) row.appendChild(opts.extra);
      row.addEventListener('click', onClick);
      if (opts.hover) {
        row.addEventListener('mouseenter', opts.hover);
        row.addEventListener('mouseleave', () => app.setHover(null));
      }
      if (opts.rename) {
        name.addEventListener('dblclick', (e) => {
          e.stopPropagation();
          const input = el('input', 'tree-item__rename');
          input.value = label;
          name.replaceWith(input);
          input.focus();
          input.select();
          let done = false;
          const finish = (apply: boolean) => {
            if (done) return;
            done = true;
            const v = input.value.trim();
            input.replaceWith(name); // restore the label first so refresh() is not blocked by the typing guard
            if (apply && v && v !== label) opts.rename!(v);
            else this.refresh();
          };
          input.addEventListener('keydown', (ev) => {
            if (ev.key === 'Enter') finish(true);
            else if (ev.key === 'Escape') finish(false);
            ev.stopPropagation();
          });
          input.addEventListener('blur', () => finish(true));
        });
      }
      return row;
    };
    const links = Object.values(m.links);
    group('links', TREE.links, links.length, (list) => {
      for (const link of links) {
        const badges: string[] = [];
        if (link.ground) badges.push(TREE.ground);
        if (link.locked) badges.push(TREE.locked);
        if (link.flexible) badges.push(TREE.flexible);
        if (link.hidden) badges.push(TREE.hidden);
        if (bodyPlaneJoint(m, link.id)) badges.push(TREE.planar2d);
        const eye = el('button', `tree-eye ${link.hidden ? 'off' : ''}`);
        eye.innerHTML = icon('eye');
        eye.title = link.hidden ? TREE.show : TREE.hide;
        eye.addEventListener('click', (e) => {
          e.stopPropagation();
          app.beginChange();
          link.hidden = !link.hidden;
          app.endChange();
        });
        const active = !!sel && (sel.type === 'link' || sel.type === 'vertex' || sel.type === 'edge' || sel.type === 'face' || sel.type === 'axis') && sel.id === link.id;
        list.appendChild(
          item(link.name, active, () => app.select({ type: 'link', id: link.id }), {
            badges,
            extra: eye,
            iconName: link.kind,
            hover: () => app.setHover({ type: 'edge', id: link.id, linkId: link.id, point: [0, 0, 0], distance: 0 }),
            rename: (v) => {
              app.beginChange();
              link.name = v;
              app.endChange();
            },
          }),
        );
      }
    });
    const joints = Object.values(m.joints).filter((j) => j.a.kind !== 'body');
    group('joints', TREE.joints, joints.length, (list) => {
      for (const j of joints) {
        list.appendChild(
          item(`${this.jointLabel(j)} · ${this.jointEnds(j)}`, sel?.type === 'joint' && sel.id === j.id, () => app.select({ type: 'joint', id: j.id }), {
            iconName: j.type,
            hover: () => app.setHover({ type: 'joint', id: j.id, point: [0, 0, 0], distance: 0 }),
          }),
        );
      }
    });
    const cons = Object.values(m.construction);
    group('construction', TREE.construction, cons.length, (list) => {
      for (const c of cons) {
        list.appendChild(
          item(`${c.name} (${c.kind})`, sel?.type === 'construction' && sel.id === c.id, () => app.select({ type: 'construction', id: c.id }), {
            iconName: c.kind === 'point' ? 'cpoint' : c.kind === 'axis' ? 'caxis' : 'cplane',
            badges: m.settings.sketchPlaneId === c.id ? [PANEL.isSketchPlane] : [],
            hover: () => app.setHover({ type: 'construction', id: c.id, point: [0, 0, 0], distance: 0 }),
            rename: c.builtin
              ? undefined
              : (v) => {
                  app.beginChange();
                  c.name = v;
                  app.endChange();
                },
          }),
        );
      }
    });
    group('drivers', TREE.drivers, m.drivers.length, (list) => {
      m.drivers.forEach((d, i) => {
        list.appendChild(item(this.driverLabel(d), app.sim.activeDriver === i && app.mode !== 'construction', () => app.setActiveDriver(i), { iconName: 'driver' }));
      });
    });
    group('targets', TREE.targets, m.targets.length, (list) => {
      for (const t of m.targets) {
        const pt = m.points[t.pointId];
        if (!pt) continue;
        const label = `${m.links[pt.linkId]?.name ?? ''} ${pt.name} · ${TREE.pose} ${t.pose} · ${t.kind}`;
        list.appendChild(
          item(label, sel?.type === 'editPoint' && sel.pointId === t.pointId && sel.pose === t.pose, () => app.select({ type: 'editPoint', id: t.pointId, pointId: t.pointId, pose: t.pose }), {
            iconName: t.locked ? 'lock' : 'target',
          }),
        );
      }
    });
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
    if (t === 'bar' || t === 'edit') {
      box.appendChild(row(TOOL_OPTIONS.jointType, select(JOINT_TYPES.filter((j) => j === 'revolute' || j === 'spherical').map((j) => ({ value: j, label: JOINTS[j].label })), app.model.settings.defaultJoint, (v) => (app.model.settings.defaultJoint = v))));
    }
    if (t === 'pattern') {
      box.appendChild(row(TOOL_OPTIONS.patternKind, select([{ value: 'linear', label: TOOL_OPTIONS.patternLinear }, { value: 'polar', label: TOOL_OPTIONS.patternPolar }], o.patternKind, (v) => { o.patternKind = v; this.refresh(); })));
      box.appendChild(row(TOOL_OPTIONS.patternCount, numberInput(o.patternCount, (v) => (o.patternCount = Math.max(1, Math.round(v))), { step: 1, min: 1 })));
      if (o.patternKind === 'polar') box.appendChild(row(TOOL_OPTIONS.patternAngle, numberInput(o.patternAngle, (v) => (o.patternAngle = v), { step: 15 })));
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
    if (app.notice) {
      const notice = el('div', 'notice');
      notice.appendChild(el('p', 'hint warn', app.notice));
      notice.appendChild(button(PANEL.noticeDismiss, () => app.setNotice(''), { cls: 'btn--small', title: PANEL.noticeDismissHelp }));
      summary.appendChild(notice);
    }
    const flatLoops = sim.creaseLoops.filter((l) => l.flat).length;
    if (sim.lockedCreaseIds.length) summary.appendChild(el('p', 'hint warn', SIM.creasesLocked(sim.lockedCreaseIds.length, flatLoops > 0)));
    if (flatLoops > 0) {
      summary.appendChild(el('p', 'hint warn', SIM.flatVertices(flatLoops)));
      const actions = el('div', 'actions');
      actions.appendChild(button(SIM.fold, () => this.foldFlatVertex(), { title: SIM.foldHelp }));
      summary.appendChild(actions);
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

  /** Short label of a joint for the tree: the type letter, or "Crease M 160°" for a crease (construction pose). */
  private jointLabel(j: Joint): string {
    const m = this.app.model;
    return isCrease(m, j) ? creaseLabel(creaseMV(m, j), creaseDihedralDeg(m, j)) : JOINTS[j.type].short;
  }

  /** "<link A> ↔ <link B or datum>" for a joint, used by the tree and the status bar. */
  private jointEnds(j: Joint): string {
    const m = this.app.model;
    const a = m.links[j.a.linkId]?.name ?? '';
    const b = isConstructionRef(j.b) ? m.construction[j.b.constructionId]?.name ?? '' : m.links[j.b.linkId]?.name ?? '';
    return `${a} ↔ ${b}`;
  }

  /**
   * Fold command (Mechanism panel): pre-fold the first flat crease loop so that every crease leaves the singular flat
   * state; one undo entry on success, nothing recorded and the model untouched on failure.
   */
  private foldFlatVertex(): void {
    const app = this.app;
    const m = app.model;
    const hadDrivers = m.drivers.length > 0;
    app.beginChange();
    const r = prefoldVertex(m);
    if (!r.ok || !r.creaseId) {
      app.abortChange();
      app.report(STATUS.foldFailed(r.reason ?? 'noBranch'));
      return;
    }
    const crease = m.joints[r.creaseId];
    app.select({ type: 'joint', id: r.creaseId });
    const folded = STATUS.folded(crease ? this.jointEnds(crease) : '', r.dof ?? 0);
    app.setStatus(!hadDrivers && m.drivers.length > 0 ? `${folded} ${STATUS.foldedDriverKept}` : folded);
    app.endChange();
  }

  private driverLabel(d: import('../core/types').Driver): string {
    const m = this.app.model;
    if (d.kind === 'angle') return `${m.links[d.linkId ?? '']?.name ?? ''} · angle`;
    const j = d.jointId ? m.joints[d.jointId] : null;
    return `${j ? (isCrease(m, j) ? JOINTS.crease.label : JOINTS[j.type].label) : ''} · ${d.kind}`;
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
          numberInput(barLength(m, link), (v) => {
            // the new rest length is kept only when the mechanism can be re-assembled with it
            app.beginChange();
            const r = link.rigidity.find((c) => c.kind === 'dist' && !c.fixed);
            if (r && r.kind === 'dist') r.length = Math.max(1e-3, v);
            this.resolveOrRevert(trySolveCommit(m, {}).ok);
          }, { step: 0.1, min: 0.001 }),
        ),
      );
    }
    if (link.params.sides) box.appendChild(row(PANEL.sides, el('span', 'value', String(link.params.sides))));
    if (link.params.radius) box.appendChild(row(PANEL.radius, el('span', 'value', link.params.radius.toFixed(3))));
    if (link.params.height) box.appendChild(row(PANEL.height, el('span', 'value', link.params.height.toFixed(3))));
    box.appendChild(row(PANEL.locked, checkbox(link.locked, (v) => change(() => (link.locked = v)))));
    box.appendChild(row(PANEL.hidden, checkbox(!!link.hidden, (v) => change(() => (link.hidden = v)))));
    box.appendChild(row(PANEL.ground, checkbox(link.ground, (v) => change(() => setGround(m, v ? link.id : null)))));
    if (link.kind === 'bar' || link.kind === 'polygon') {
      // the sketch-plane (2-D body) constraint is a hidden planar joint; it is presented as a property of the link
      const bp = bodyPlaneJoint(m, link.id);
      const planeName = bp && isConstructionRef(bp.b) ? m.construction[bp.b.constructionId]?.name : undefined;
      const planeRow = row(
        PANEL.keepOnSketchPlane,
        checkbox(!!bp, (v) => {
          // adding or removing the constraint re-solves the mechanism; a pose that cannot satisfy it is refused
          app.beginChange();
          setBodyPlane(m, link.id, v ? m.settings.sketchPlaneId : null);
          this.resolveOrRevert(trySolveCommit(m, { allowGroundMove: link.ground }).ok);
        }),
      );
      planeRow.title = PANEL.keepOnSketchPlaneHelp;
      if (planeName) planeRow.appendChild(el('span', 'value', PANEL.onPlane(planeName)));
      box.appendChild(planeRow);
    }
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
    if (link.kind === 'polygon') {
      let height = this.extrudeHeight;
      const h = numberInput(height, (v) => (height = v), { step: 0.1 });
      const r = row(TOOL_OPTIONS.extrudeHeight, h);
      const b = button(PANEL.extrude, () => {
        this.extrudeHeight = height;
        // extruding rebuilds the polygon's rest geometry from its current shape, which must not be a violated one
        if (!poseIsConsistent(m)) {
          app.setStatus(PANEL.extrudeRefused(currentViolation(m)));
          return;
        }
        change(() => {
          if (extrudePolygon(m, link, height)) app.setStatus(STATUS.extruded);
        });
      }, { icon: 'prism' });
      b.title = PANEL.extrudeHelp;
      r.appendChild(b);
      box.appendChild(r);
    }
    const actions = el('div', 'actions');
    actions.appendChild(button(PANEL.copyLink, () => this.duplicateSelected(), { icon: 'polygon', cls: 'btn--small' }));
    actions.appendChild(button(PANEL.deleteItem, () => this.tools.deletePick({ type: 'link', id: link.id }), { icon: 'delete', cls: 'btn--danger btn--small' }));
    box.appendChild(actions);
  }

  private extrudeHeight = 1;
  private clipboardLinkId: ID | null = null;

  /** Ctrl+C: remember the selected link. */
  copySelected(): void {
    const sel = this.app.selection;
    const id = sel && (sel.type === 'link' || sel.type === 'vertex' || sel.type === 'edge' || sel.type === 'face' || sel.type === 'axis') ? sel.id : null;
    if (!id || !this.app.model.links[id]) {
      this.app.setStatus(STATUS.nothingToCopy);
      return;
    }
    this.clipboardLinkId = id;
    this.app.setStatus(STATUS.copied);
  }

  /** Ctrl+V: paste a translated copy of the remembered link. */
  pasteCopied(): void {
    const app = this.app;
    const src = this.clipboardLinkId ? app.model.links[this.clipboardLinkId] : null;
    if (!src) {
      this.app.setStatus(STATUS.nothingToCopy);
      return;
    }
    const offset = Math.max(0.5, modelSize(app.model) * 0.15);
    // offset within the link's sketch plane (or the active one) so the copy keeps its 2-D constraint
    const bp = bodyPlaneJoint(app.model, src.id);
    const planeId = bp && isConstructionRef(bp.b) ? bp.b.constructionId : app.model.settings.sketchPlaneId;
    const plane = app.model.construction[planeId];
    const xd: Vec3 = plane?.xDir ?? [1, 0, 0];
    const yd: Vec3 = plane?.dir ? cross(plane.dir, xd) : [0, 1, 0];
    app.beginChange();
    const copy = duplicateLink(app.model, src, translation(add(scale(xd, offset), scale(yd, offset))), `${src.name} copy`);
    app.endChange();
    if (copy) {
      app.select({ type: 'link', id: copy.id });
      app.setStatus(STATUS.pasted);
    }
  }

  /** Properties → Duplicate: copy the selected link next to itself. */
  duplicateSelected(): void {
    this.copySelected();
    if (this.clipboardLinkId) this.pasteCopied();
  }

  /** Is the Fold command offered (a flat crease loop exists)? Keeps the locked-creases hint honest about suggesting it. */
  private foldAvailable(): boolean {
    return this.app.sim.creaseLoops.some((l) => l.flat);
  }

  /**
   * Finish a re-solved Properties edit: record it when the solve was accepted, otherwise restore the pre-edit model
   * and say so. The refused widget still has focus, so it is blurred first; refresh() then rebuilds the panel from
   * the restored model (abortChange replaces the model object), so the widget shows the model's value instead of
   * the refused one and later edits reach the live model.
   */
  private resolveOrRevert(ok: boolean): void {
    const app = this.app;
    if (ok) {
      app.endChange();
      return;
    }
    this.blurActive();
    const sel = app.selection;
    app.abortChange();
    app.select(sel);
    app.report(STATUS.editRefused);
  }

  /** Drop keyboard focus from the widget that triggered a refused edit (see resolveOrRevert). */
  private blurActive(): void {
    const active = document.activeElement;
    if (active instanceof HTMLElement && this.root.contains(active)) active.blur();
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
          this.resolveOrRevert(trySolveCommit(m, { dragTargets: [{ pointId, pos: target, weight: 1 }], freePointIds: new Set([pointId]), allowGroundMove: link.ground }).ok);
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
    const crease = isCrease(m, j);
    const info = crease ? JOINTS.crease : JOINTS[j.type];
    box.appendChild(el('h4', 'panel__subtitle', info.label));
    box.appendChild(el('p', 'hint', info.description));
    box.appendChild(row(PANEL.jointLinks, el('span', 'value', this.jointEnds(j))));
    if (crease) this.creaseProps(box, j);
    if (j.a.kind !== 'body') {
      box.appendChild(
        row(
          PANEL.changeType,
          select(JOINT_TYPES.map((t) => ({ value: t, label: JOINTS[t].label })), j.type, (t) => {
            // same pre-flight as the Joint tool: an incompatible or unsatisfiable type leaves the joint as it was
            const sel = app.selection;
            app.beginChange();
            const r = tryChangeJointType(m, j.id, t);
            if (r.ok && r.joint) {
              app.select({ type: 'joint', id: r.joint.id });
              app.endChange();
            } else {
              this.blurActive(); // the select keeps focus otherwise and refresh() would leave the stale panel
              app.abortChange();
              app.select(sel);
              app.report(r.diagnosis?.kind === 'incompatible' ? STATUS.jointIncompatible : jointRefusedMessage(r.diagnosis ?? { kind: 'infeasible', residual: r.residual }));
            }
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

  /**
   * Crease section of the joint Properties: the current fold angle and class (read-only), the mountain / valley
   * assignment and target fold angle stored in joint.fold (which move nothing by themselves), "Fold to target"
   * (foldCreaseToTarget inside one undoable change, restored and explained on refusal) and "Drive this crease"
   * (driveCrease, which pre-folds a flat vertex, then the new driver becomes the active one). The target is written
   * without rebuilding the panel (like the helper offset and default joint type), so that typing a value and
   * clicking "Fold to target" straight away is a single click; the fold itself is the undoable change.
   */
  private creaseProps(box: HTMLElement, j: Joint): void {
    const app = this.app;
    const m = app.model;
    const deg = creaseDihedralDeg(m, j);
    const mv = creaseMV(m, j);
    if (deg !== null) box.appendChild(row(PANEL.creaseAngle, el('span', 'value', PANEL.creaseAngleValue(deg, mv))));
    const mvRow = row(
      PANEL.creaseMV,
      select(
        [
          { value: '', label: PANEL.creaseMVUnset },
          { value: 'M', label: PANEL.creaseMountain },
          { value: 'V', label: PANEL.creaseValley },
        ],
        j.fold?.mv ?? '',
        (v) => {
          app.beginChange();
          if (v === '') delete j.fold?.mv;
          else j.fold = { ...(j.fold ?? {}), mv: v };
          if (j.fold && Object.keys(j.fold).length === 0) delete j.fold;
          app.endChange();
        },
      ),
    );
    mvRow.title = PANEL.creaseMVHelp;
    box.appendChild(mvRow);
    // default target: the stored one, else 160° for a flat crease (the pre-fold angle), else the current angle
    let target = j.fold?.target ?? (deg === null || mv === null ? DEFAULT_PREFOLD_DEG : Math.round(Math.abs(deg)));
    const targetRow = row(
      PANEL.creaseTarget,
      numberInput(target, (v) => {
        target = Math.max(0, Math.min(180, v));
        j.fold = { ...(j.fold ?? {}), target };
      }, { step: 5, min: 0, max: 180, digits: 1 }),
    );
    targetRow.title = PANEL.creaseTargetHelp;
    box.appendChild(targetRow);
    const actions = el('div', 'actions');
    const foldBtn = button(PANEL.creaseFoldTo, () => {
      const sel = app.selection;
      app.beginChange();
      const r = foldCreaseToTarget(m, j.id, target, j.fold?.mv);
      if (!r.ok) {
        app.abortChange();
        app.select(sel);
        app.report(STATUS.foldFailed(r.reason ?? 'unreachable'));
        return;
      }
      app.setStatus(STATUS.creaseFolded(`${this.jointLabel(j)} · ${this.jointEnds(j)}`, r.dof ?? 0));
      app.endChange();
    });
    foldBtn.title = PANEL.creaseFoldToHelp;
    actions.appendChild(foldBtn);
    const driveBtn = button(PANEL.creaseDrive, () => {
      app.beginChange();
      const r = driveCrease(m, j.id);
      if (!r.driver) {
        app.abortChange();
        return;
      }
      app.endChange(); // records an undo entry only when the model changed (a reused driver on a folded vertex changes nothing)
      app.setActiveDriver(m.drivers.indexOf(r.driver));
      const note = r.prefold ? (r.prefold.ok ? STATUS.prefolded : STATUS.flatVertexWarning) : '';
      const message = r.reused ? STATUS.driverReused : STATUS.driverSet;
      app.setStatus(note ? `${message} · ${note}` : message);
    }, { icon: 'driver' });
    driveBtn.title = PANEL.creaseDriveHelp;
    actions.appendChild(driveBtn);
    box.appendChild(actions);
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
    // the sketch-plane constraint is not a joint at this point, but it is why the point cannot leave the plane
    const bp = link ? bodyPlaneJoint(m, link.id) : null;
    const planeName = bp && isConstructionRef(bp.b) ? m.construction[bp.b.constructionId]?.name : undefined;
    if (planeName) typeBtn.title += ` · ${POPUP.onSketchPlane(planeName)}`;
    typeBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      if (!j) {
        app.setStatus(TOOLS.joint.hint);
        return;
      }
      // cycle to the next compatible joint type; a type the mechanism cannot be re-assembled with stops the cycle
      // with an explanation and the joint stays as it was (incompatible types are skipped, nothing is removed)
      const idx = JOINT_TYPES.indexOf(j.type);
      for (let k = 1; k <= JOINT_TYPES.length; k++) {
        const t = JOINT_TYPES[(idx + k) % JOINT_TYPES.length];
        const sel = app.selection;
        app.beginChange();
        const r = tryChangeJointType(app.model, j.id, t);
        if (r.ok) {
          app.endChange();
          app.setStatus(`${JOINTS[t].label}`);
          this.showPopup(pointId);
          return;
        }
        app.abortChange();
        app.select(sel);
        if (r.diagnosis?.kind !== 'incompatible') {
          app.setStatus(jointRefusedMessage(r.diagnosis ?? { kind: 'infeasible', residual: r.residual }));
          return;
        }
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
    const colorKeys: (keyof AppSettings['colors'])[] = ['geometry', 'ground', 'construction', 'designSpace', 'outputPath', 'editPointFree', 'editPointConstrained', 'background', 'gridMajor', 'gridMinor', 'selection', 'mountain', 'valley'];
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
    } else if (e.key === 'Backspace' && this.tools.popSketchSupported()) {
      e.preventDefault();
      this.tools.popSketchPoint();
    } else if (e.key === 'Delete' || e.key === 'Backspace') {
      if (app.selection) this.tools.deletePick({ type: app.selection.type, id: app.selection.id, pointId: app.selection.pointId, pose: app.selection.pose });
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
      e.preventDefault();
      if (e.shiftKey) app.redo();
      else app.undo();
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') {
      e.preventDefault();
      app.redo();
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'c' && app.mode === 'construction') {
      e.preventDefault();
      this.copySelected();
    } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'v' && app.mode === 'construction') {
      e.preventDefault();
      this.pasteCopied();
    } else if (e.key === 'Enter' && app.mode === 'construction') {
      this.tools.finishSketch();
    } else if (app.mode === 'construction' && !e.ctrlKey && !e.metaKey) {
      const map: Record<string, ToolName> = { '1': 'select', '2': 'bar', '3': 'polygon', '4': 'prism', '5': 'cylinder', g: 'ground', j: 'joint', d: 'driver', s: 'sketch', e: 'edit', m: 'mirror', p: 'pattern' };
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

