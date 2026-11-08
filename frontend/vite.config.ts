import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// The browser only ever talks to the Vite origin; REST goes through /api, the socket through /ws.
const BACKEND = process.env.BACKEND_URL ?? "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      "/api": { target: BACKEND, changeOrigin: true, rewrite: (p) => p.replace(/^\/api/, "") },
      "/ws": { target: BACKEND.replace(/^http/, "ws"), ws: true, changeOrigin: true },
    },
  },
});
