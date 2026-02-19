import { defineConfig, type Plugin } from 'vite'
import react from '@vitejs/plugin-react'
import fs from 'node:fs'
import path from 'node:path'

const robotTarget = process.env.ROBOT_URL || 'http://localhost:8088';

const KNOWN_ENVS = new Set(['comp', 'test_drivebase']);
const KNOWN_FILES = new Set(['ShotMaps.json', 'VisionConstants.json']);

function localFilesPlugin(): Plugin {
  const constantsDir = (...parts: string[]) =>
    path.resolve(__dirname, '..', 'src', 'main', 'deploy', 'constants', ...parts);

  function validateEnvAndFile(env: string, filename: string, res: import('http').ServerResponse): boolean {
    if (!KNOWN_ENVS.has(env)) {
      res.statusCode = 400;
      res.end(JSON.stringify({ error: `Unknown environment: ${env}` }));
      return false;
    }
    if (!KNOWN_FILES.has(filename)) {
      res.statusCode = 400;
      res.end(JSON.stringify({ error: `Unknown filename: ${filename}` }));
      return false;
    }
    return true;
  }

  return {
    name: 'local-files',
    configureServer(server) {
      // GET /local-load/:env/:filename
      server.middlewares.use((req, res, next) => {
        const match = req.url?.match(/^\/local-load\/([^/]+)\/([^/]+)$/);
        if (!match || req.method !== 'GET') return next();

        const [, env, filename] = match;
        if (!validateEnvAndFile(env, filename, res)) return;

        try {
          const filePath = constantsDir(env, filename);
          const content = fs.readFileSync(filePath, 'utf-8');
          res.statusCode = 200;
          res.setHeader('Content-Type', 'application/json');
          res.end(content);
        } catch (e: unknown) {
          const code = (e as NodeJS.ErrnoException).code;
          res.statusCode = code === 'ENOENT' ? 404 : 500;
          res.end(JSON.stringify({ error: String(e) }));
        }
      });

      // POST /local-save/:env/:filename
      server.middlewares.use((req, res, next) => {
        const match = req.url?.match(/^\/local-save\/([^/]+)\/([^/]+)$/);
        if (!match || req.method !== 'POST') return next();

        const [, env, filename] = match;
        if (!validateEnvAndFile(env, filename, res)) return;

        let body = '';
        req.on('data', (chunk: Buffer) => { body += chunk.toString(); });
        req.on('end', () => {
          try {
            const json = JSON.parse(body);
            const dest = constantsDir(env, filename);
            fs.mkdirSync(path.dirname(dest), { recursive: true });
            fs.writeFileSync(dest, JSON.stringify(json, null, 2) + '\n');
            res.statusCode = 200;
            res.setHeader('Content-Type', 'application/json');
            res.end(JSON.stringify({ ok: true, path: dest }));
          } catch (e) {
            res.statusCode = 500;
            res.end(JSON.stringify({ error: String(e) }));
          }
        });
      });
    },
  };
}

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), localFilesPlugin()],
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
