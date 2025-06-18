import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'
import mdx from '@mdx-js/rollup'

export default defineConfig(({ mode }) => ({
  base: mode === 'production' ? '/2025-Team-Innovators-Docs/' : '/',
  plugins: [
    react(),
    tailwindcss(),
    mdx()
  ],
}))
