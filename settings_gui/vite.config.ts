import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const robotTarget = process.env.ROBOT_URL || 'http://localhost:8088';

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': {
        target: robotTarget,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, ''),
      },
    },
  },
})
