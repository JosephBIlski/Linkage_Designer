/**
 * Application settings (colours, display options), persisted in localStorage.
 * Default colours follow the original specification; change DEFAULT_SETTINGS
 * to adjust the defaults for future iterations.
 */

export interface ColorSettings {
  geometry: string;
  ground: string;
  construction: string;
  designSpace: string;
  outputPath: string;
  editPointFree: string;
  editPointConstrained: string;
  background: string;
  gridMajor: string;
  gridMinor: string;
  selection: string;
}

export interface AppSettings {
  colors: ColorSettings;
  designSpaceOpacity: number;
  outputPathOpacity: number;
  showLabels: boolean;
  showConstruction: boolean;
  showHelpers: boolean;
  gridSnap: boolean;
  gridStep: number;
  orthographic: boolean;
}

export const DEFAULT_SETTINGS: AppSettings = {
  colors: {
    geometry: '#35a4d3', // spec: default user geometry colour
    ground: '#5c6670',
    construction: '#8a6d3b',
    designSpace: '#c7007e', // spec: design space colour (30 % opacity)
    outputPath: '#9a0062', // spec: slightly darker, more opaque hue of the design space colour
    editPointFree: '#0eb062', // spec: unconstrained editing points
    editPointConstrained: '#94140a', // spec: constrained editing points
    background: '#e4e6e8', // spec: light grey background
    gridMajor: '#9aa0a6',
    gridMinor: '#c3c7cb',
    selection: '#ffb300',
  },
  designSpaceOpacity: 0.3,
  outputPathOpacity: 0.9,
  showLabels: true,
  showConstruction: true,
  showHelpers: false,
  gridSnap: false,
  gridStep: 0.5,
  orthographic: false,
};

const STORAGE_KEY = 'linkage-designer.settings.v1';

export function loadSettings(): AppSettings {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return structuredClone(DEFAULT_SETTINGS);
    const parsed = JSON.parse(raw) as Partial<AppSettings>;
    return { ...structuredClone(DEFAULT_SETTINGS), ...parsed, colors: { ...DEFAULT_SETTINGS.colors, ...(parsed.colors ?? {}) } };
  } catch {
    return structuredClone(DEFAULT_SETTINGS);
  }
}

export function saveSettings(s: AppSettings): void {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(s));
  } catch {
    /* ignore (private mode etc.) */
  }
}

// ---------------------------------------------------------------------------
// Colour conversions (HEX <-> RGB <-> HWB)
// ---------------------------------------------------------------------------

export interface RGB {
  r: number;
  g: number;
  b: number;
} // 0..255
export interface HWB {
  h: number;
  w: number;
  b: number;
} // h 0..360, w/b 0..100

export function hexToRgb(hex: string): RGB {
  let h = hex.trim().replace('#', '');
  if (h.length === 3) h = h.split('').map((c) => c + c).join('');
  const n = parseInt(h.slice(0, 6), 16);
  if (Number.isNaN(n)) return { r: 0, g: 0, b: 0 };
  return { r: (n >> 16) & 255, g: (n >> 8) & 255, b: n & 255 };
}

export function rgbToHex({ r, g, b }: RGB): string {
  const c = (v: number) => Math.max(0, Math.min(255, Math.round(v))).toString(16).padStart(2, '0');
  return `#${c(r)}${c(g)}${c(b)}`;
}

export function isValidHex(hex: string): boolean {
  return /^#?([0-9a-fA-F]{3}|[0-9a-fA-F]{6})$/.test(hex.trim());
}

export function rgbToHwb({ r, g, b }: RGB): HWB {
  const rn = r / 255;
  const gn = g / 255;
  const bn = b / 255;
  const max = Math.max(rn, gn, bn);
  const min = Math.min(rn, gn, bn);
  let h = 0;
  if (max !== min) {
    const d = max - min;
    if (max === rn) h = ((gn - bn) / d) % 6;
    else if (max === gn) h = (bn - rn) / d + 2;
    else h = (rn - gn) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  return { h, w: min * 100, b: (1 - max) * 100 };
}

export function hwbToRgb({ h, w, b }: HWB): RGB {
  let wn = w / 100;
  let bn = b / 100;
  if (wn + bn > 1) {
    const s = wn + bn;
    wn /= s;
    bn /= s;
  }
  // pure hue colour
  const hh = (((h % 360) + 360) % 360) / 60;
  const x = 1 - Math.abs((hh % 2) - 1);
  let rgb: [number, number, number];
  if (hh < 1) rgb = [1, x, 0];
  else if (hh < 2) rgb = [x, 1, 0];
  else if (hh < 3) rgb = [0, 1, x];
  else if (hh < 4) rgb = [0, x, 1];
  else if (hh < 5) rgb = [x, 0, 1];
  else rgb = [1, 0, x];
  const f = 1 - wn - bn;
  return { r: (rgb[0] * f + wn) * 255, g: (rgb[1] * f + wn) * 255, b: (rgb[2] * f + wn) * 255 };
}

export const hexToHwb = (hex: string): HWB => rgbToHwb(hexToRgb(hex));
export const hwbToHex = (hwb: HWB): string => rgbToHex(hwbToRgb(hwb));

/** Darken a HEX colour by a factor (0..1) in RGB space. */
export function darken(hex: string, factor: number): string {
  const { r, g, b } = hexToRgb(hex);
  return rgbToHex({ r: r * (1 - factor), g: g * (1 - factor), b: b * (1 - factor) });
}
