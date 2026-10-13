/**
 * Build information for the version badge in the top bar: package version,
 * short commit hash, branch and build time, injected by vite.config.ts
 * (`define`). Falls back to placeholders when the constant is missing (for
 * example in a tool that imports the UI without Vite).
 */
export interface BuildInfo {
  version: string;
  commit: string;
  branch: string;
  /** ISO 8601 build time. */
  builtAt: string;
}

export const BUILD_INFO: BuildInfo =
  typeof __BUILD_INFO__ !== 'undefined' && __BUILD_INFO__ ? __BUILD_INFO__ : { version: '0.0.0', commit: 'unknown', branch: 'unknown', builtAt: '' };

/** `YYYY-MM-DD` of the build time, or an empty string. */
export function buildDate(info: BuildInfo = BUILD_INFO): string {
  return info.builtAt ? info.builtAt.slice(0, 10) : '';
}
