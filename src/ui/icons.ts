/** Inline SVG icons (24×24 viewBox). Stroke colour follows CSS `currentColor`. */
const svg = (body: string, extra = ''): string =>
  `<svg viewBox="0 0 24 24" width="20" height="20" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" ${extra}>${body}</svg>`;

export const ICONS: Record<string, string> = {
  select: svg('<path d="M5 3l14 9-6 1.5L10 20z"/>'),
  bar: svg('<circle cx="5" cy="18" r="2"/><circle cx="19" cy="6" r="2"/><path d="M6.5 16.5L17.5 7.5"/>'),
  polygon: svg('<path d="M12 3l8.5 6.2-3.2 10H6.7L3.5 9.2z"/>'),
  prism: svg('<path d="M4 8l8-4 8 4-8 4z"/><path d="M4 8v8l8 4 8-4V8"/><path d="M12 12v8"/>'),
  cylinder: svg('<ellipse cx="12" cy="6" rx="7" ry="2.5"/><path d="M5 6v12c0 1.4 3.1 2.5 7 2.5s7-1.1 7-2.5V6"/>'),
  cpoint: svg('<path d="M12 4v16M4 12h16"/><circle cx="12" cy="12" r="3"/>'),
  caxis: svg('<path d="M12 2v20"/><path d="M9 5l3-3 3 3M9 19l3 3 3-3"/>'),
  cplane: svg('<path d="M3 17l6-10h12l-6 10z"/>'),
  joint: svg('<circle cx="12" cy="12" r="3"/><path d="M12 9V3M12 15v6M9 12H3M15 12h6"/>'),
  ground: svg('<path d="M12 4v8"/><path d="M4 12h16"/><path d="M5 12l-2 4M9 12l-2 4M13 12l-2 4M17 12l-2 4M21 12l-2 4"/>'),
  driver: svg('<circle cx="12" cy="12" r="8"/><path d="M12 12l5-3"/><path d="M14 4.5l2.5 1.5-1 2.5"/>'),
  delete: svg('<path d="M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13"/>'),
  lock: svg('<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 018 0v4"/>'),
  unlock: svg('<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 017.5-2"/>'),
  close: svg('<path d="M6 6l12 12M18 6L6 18"/>'),
  play: svg('<path d="M7 4l12 8-12 8z"/>'),
  pause: svg('<path d="M7 4h4v16H7zM13 4h4v16h-4z"/>'),
  settings: svg('<circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 00.3 1.8l.1.1a2 2 0 11-2.8 2.8l-.1-.1a1.7 1.7 0 00-1.8-.3 1.7 1.7 0 00-1 1.5V21a2 2 0 11-4 0v-.1a1.7 1.7 0 00-1.1-1.5 1.7 1.7 0 00-1.8.3l-.1.1a2 2 0 11-2.8-2.8l.1-.1a1.7 1.7 0 00.3-1.8 1.7 1.7 0 00-1.5-1H3a2 2 0 110-4h.1a1.7 1.7 0 001.5-1.1 1.7 1.7 0 00-.3-1.8l-.1-.1a2 2 0 112.8-2.8l.1.1a1.7 1.7 0 001.8.3H9a1.7 1.7 0 001-1.5V3a2 2 0 114 0v.1a1.7 1.7 0 001 1.5 1.7 1.7 0 001.8-.3l.1-.1a2 2 0 112.8 2.8l-.1.1a1.7 1.7 0 00-.3 1.8V9a1.7 1.7 0 001.5 1H21a2 2 0 110 4h-.1a1.7 1.7 0 00-1.5 1z"/>'),
  undo: svg('<path d="M9 14L4 9l5-5"/><path d="M4 9h10a6 6 0 010 12h-3"/>'),
  redo: svg('<path d="M15 14l5-5-5-5"/><path d="M20 9H10a6 6 0 000 12h3"/>'),
  fit: svg('<path d="M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5"/><rect x="9" y="9" width="6" height="6"/>'),
  help: svg('<circle cx="12" cy="12" r="9"/><path d="M9.5 9.5a2.5 2.5 0 015 0c0 1.8-2.5 2-2.5 4"/><path d="M12 17h.01"/>'),
  eye: svg('<path d="M2 12s3.5-6 10-6 10 6 10 6-3.5 6-10 6S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>'),
  // joint type glyphs
  spherical: svg('<circle cx="12" cy="12" r="5"/><path d="M12 3v4M12 17v4M3 12h4M17 12h4"/>'),
  revolute: svg('<circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="1.5" fill="currentColor"/><path d="M12 2v3M12 19v3"/>'),
  planar: svg('<path d="M3 16l5-8h13l-5 8z"/><path d="M12 12l0-6"/>'),
  prismatic: svg('<rect x="3" y="9" width="18" height="6"/><rect x="8" y="7" width="8" height="10"/><path d="M1 12h2M21 12h2"/>'),
  cylindrical: svg('<rect x="3" y="9" width="18" height="6" rx="3"/><ellipse cx="12" cy="12" rx="4" ry="5"/>'),
  screw: svg('<path d="M4 12h16"/><path d="M6 8c2 2 2 6 0 8M10 8c2 2 2 6 0 8M14 8c2 2 2 6 0 8M18 8c2 2 2 6 0 8"/>'),
  none: svg('<circle cx="12" cy="12" r="7" stroke-dasharray="3 3"/>'),
  target: svg('<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>'),
  sketch: svg('<path d="M4 18l4-11 6 3 6-6" stroke-dasharray="3 2"/><circle cx="4" cy="18" r="1.6" fill="currentColor"/><circle cx="8" cy="7" r="1.6" fill="currentColor"/><circle cx="14" cy="10" r="1.6" fill="currentColor"/><circle cx="20" cy="4" r="1.6" fill="currentColor"/>'),
  edit: svg('<path d="M4 20l4-1 11-11-3-3L5 16z"/><path d="M14 7l3 3"/><circle cx="19" cy="17" r="2"/>'),
  mirror: svg('<path d="M12 3v18" stroke-dasharray="3 2"/><path d="M9 7L4 12l5 5z"/><path d="M15 7l5 5-5 5z"/>'),
  pattern: svg('<rect x="3" y="3" width="6" height="6"/><rect x="15" y="3" width="6" height="6"/><rect x="3" y="15" width="6" height="6"/><rect x="15" y="15" width="6" height="6"/>'),
};

export function icon(name: string): string {
  return ICONS[name] ?? ICONS.none;
}
