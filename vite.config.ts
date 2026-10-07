import { defineConfig } from 'vite';

// Base path is relative so the built app can be served from any sub-path
// (e.g. GitHub Pages at /Linkage_Designer/).
export default defineConfig({
  base: './',
  build: { target: 'es2022', sourcemap: true },
  test: { environment: 'node', include: ['tests/**/*.test.ts'] },
});
