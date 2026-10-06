/**
 * Minimal HWB colour picker with a HEX field (as requested in the spec, so the
 * default colours can be copied out easily).
 */
import { SETTINGS } from './strings';
import { hexToHwb, hwbToHex, isValidHex, type HWB } from './settings';

export interface ColorPickerHandle {
  element: HTMLElement;
  setValue(hex: string): void;
}

export function createColorPicker(label: string, initial: string, onChange: (hex: string) => void): ColorPickerHandle {
  const root = document.createElement('div');
  root.className = 'color-picker';
  let hwb: HWB = hexToHwb(initial);

  const header = document.createElement('div');
  header.className = 'color-picker__header';
  const swatch = document.createElement('span');
  swatch.className = 'color-picker__swatch';
  const name = document.createElement('span');
  name.className = 'color-picker__label';
  name.textContent = label;
  const hex = document.createElement('input');
  hex.type = 'text';
  hex.className = 'color-picker__hex';
  hex.title = SETTINGS.hex;
  hex.spellcheck = false;
  header.append(swatch, name, hex);

  const sliders = document.createElement('div');
  sliders.className = 'color-picker__sliders';
  const mk = (lbl: string, max: number, key: keyof HWB) => {
    const row = document.createElement('label');
    row.className = 'color-picker__row';
    const span = document.createElement('span');
    span.textContent = lbl;
    const input = document.createElement('input');
    input.type = 'range';
    input.min = '0';
    input.max = String(max);
    input.step = '1';
    const val = document.createElement('span');
    val.className = 'color-picker__value';
    input.addEventListener('input', () => {
      hwb = { ...hwb, [key]: Number(input.value) };
      update(false);
      onChange(hwbToHex(hwb));
    });
    row.append(span, input, val);
    sliders.append(row);
    return { input, val };
  };
  const h = mk(SETTINGS.hue, 360, 'h');
  const w = mk(SETTINGS.whiteness, 100, 'w');
  const b = mk(SETTINGS.blackness, 100, 'b');

  const update = (fromHex: boolean) => {
    const value = hwbToHex(hwb);
    swatch.style.background = value;
    if (!fromHex) hex.value = value;
    h.input.value = String(Math.round(hwb.h));
    w.input.value = String(Math.round(hwb.w));
    b.input.value = String(Math.round(hwb.b));
    h.val.textContent = `${Math.round(hwb.h)}°`;
    w.val.textContent = `${Math.round(hwb.w)}%`;
    b.val.textContent = `${Math.round(hwb.b)}%`;
    h.input.style.background = `linear-gradient(to right, #f00, #ff0, #0f0, #0ff, #00f, #f0f, #f00)`;
  };
  hex.addEventListener('input', () => {
    if (isValidHex(hex.value)) {
      hwb = hexToHwb(hex.value.startsWith('#') ? hex.value : `#${hex.value}`);
      update(true);
      onChange(hwbToHex(hwb));
      hex.classList.remove('invalid');
    } else hex.classList.add('invalid');
  });
  hex.addEventListener('blur', () => update(false));

  root.append(header, sliders);
  update(false);
  return {
    element: root,
    setValue(v: string) {
      hwb = hexToHwb(v);
      update(false);
    },
  };
}
