import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    // Enable COOP and COEP headers to allow SharedArrayBuffer and WebAssembly multi-threading/SIMD
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
});
