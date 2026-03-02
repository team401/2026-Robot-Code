/**
 * build_autos.mjs
 *
 * 1. Compiles all TypeScript sources via `tsc`.
 * 2. Finds all compiled auto files (everything in dist/auto_generator/src/
 *    except helper modules like AutoLib.js and Shorthands.js).
 * 3. Runs write_autos.mjs to write a single Autos.json into src/main/deploy/.
 *
 * Usage (from auto_generator/):
 *   node scripts/build_autos.mjs
 */

import { execSync } from "child_process";
import * as fs from "fs";
import * as path from "path";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.resolve(__dirname, "..");

// Helper modules that should not be treated as auto files
const EXCLUDED_FILES = new Set(["AutoLib.js", "Shorthands.js"]);

// Step 1: Compile TypeScript
console.log("Compiling TypeScript...");
// Run TypeScript compilation then rewrite path aliases in emitted JS so Node can run imports like '@/typescript/...'
execSync("npx tsc && npx tsc-alias", { cwd: rootDir, stdio: "inherit" });

// Step 2: Find all compiled auto files (exclude helper modules)
const distSrcDir = path.join(rootDir, "dist", "auto_generator", "src");
const autoFiles = fs
  .readdirSync(distSrcDir)
  .filter((f) => f.endsWith(".js") && !EXCLUDED_FILES.has(f))
  .map((f) => path.join(distSrcDir, f));

if (autoFiles.length === 0) {
  console.warn("No auto files found in", distSrcDir);
  process.exit(0);
}

console.log(`Found ${autoFiles.length} auto file(s):`, autoFiles.map((f) => path.basename(f)));

// Step 3: Run the autos and write a single Autos.json
const runnerPath = path.join(__dirname, "write_autos.mjs");
execSync(`node "${runnerPath}" ${autoFiles.map((f) => `"${f}"`).join(" ")}`, {
  cwd: rootDir,
  stdio: "inherit",
});
