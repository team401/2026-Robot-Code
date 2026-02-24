/**
 * compile_autos.mjs
 *
 * 1. Compiles all TypeScript sources via `tsc`.
 * 2. Finds all compiled auto files (everything in dist/auto_generator/src/
 *    except AutoLib.js).
 * 3. Runs run_autos.mjs to write JSON files into src/main/deploy/autos/.
 *
 * Usage (from auto_generator/):
 *   node scripts/compile_autos.mjs
 */

import { execSync } from "child_process";
import * as fs from "fs";
import * as path from "path";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.resolve(__dirname, "..");

// Step 1: Compile TypeScript
console.log("Compiling TypeScript...");
execSync("npx tsc", { cwd: rootDir, stdio: "inherit" });

// Step 2: Find all compiled auto files (exclude AutoLib.js)
const distSrcDir = path.join(rootDir, "dist", "auto_generator", "src");
const autoFiles = fs
  .readdirSync(distSrcDir)
  .filter((f) => f.endsWith(".js") && f !== "AutoLib.js")
  .map((f) => path.join(distSrcDir, f));

if (autoFiles.length === 0) {
  console.warn("No auto files found in", distSrcDir);
  process.exit(0);
}

console.log(`Found ${autoFiles.length} auto file(s):`, autoFiles.map((f) => path.basename(f)));

// Step 3: Run the autos and write JSON output
const runnerPath = path.join(__dirname, "run_autos.mjs");
execSync(`node "${runnerPath}" ${autoFiles.map((f) => `"${f}"`).join(" ")}`, {
  cwd: rootDir,
  stdio: "inherit",
});
