import { execSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { defineConfig } from 'vite';

/**
 * Build information shown in the app's version badge (top-right of the top
 * bar): package version, short commit, branch and build time. In GitHub
 * Actions the checkout is a detached HEAD, so the branch comes from
 * GITHUB_REF_NAME; git is queried otherwise, with safe fallbacks for
 * environments without git.
 */
function buildInfo(): { version: string; commit: string; branch: string; builtAt: string } {
  const pkg = JSON.parse(readFileSync(new URL('./package.json', import.meta.url), 'utf8')) as { version: string };
  const git = (args: string): string | null => {
    try {
      return execSync(`git ${args}`, { stdio: ['ignore', 'pipe', 'ignore'] }).toString().trim() || null;
    } catch {
      return null;
    }
  };
  const sha = process.env.GITHUB_SHA ?? git('rev-parse HEAD') ?? 'unknown';
  const branch = process.env.GITHUB_REF_NAME ?? git('rev-parse --abbrev-ref HEAD') ?? 'unknown';
  return { version: pkg.version, commit: sha.slice(0, 7), branch, builtAt: new Date().toISOString() };
}

// Base path is relative so the built app can be served from any sub-path
// (e.g. GitHub Pages at /Linkage_Designer/).
export default defineConfig({
  base: './',
  define: { __BUILD_INFO__: JSON.stringify(buildInfo()) },
  build: { target: 'es2022', sourcemap: true },
  test: { environment: 'node', include: ['tests/**/*.test.ts'] },
});
