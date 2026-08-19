import { defineConfig } from 'vite';

export default defineConfig({
  base: '/LinkageSimulator/',

  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },

  preview: {
    port: 4173,
    strictPort: true,
    open: '/LinkageSimulator/',
  },
});