/**
 * write_autos.mjs
 *
 * Imports all compiled auto JS files (passed as CLI args), then reads the
 * accumulated autos from AutoLib and writes a single Autos.json file into
 * src/main/deploy/Autos.json.
 *
 * The output format matches the Java Autos class:
 *   { "autos": { "AutoName": { ... AutoAction ... }, ... } }
 *
 * Usage (from auto_generator/):
 *   node scripts/write_autos.mjs dist/auto_generator/src/newTestAuto.js [...]
 */

import * as fs from "fs";
import * as path from "path";
import { fileURLToPath, pathToFileURL } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const OUTPUT_FILE = path.resolve(__dirname, "../../src/main/deploy/constants/comp/Autos.json");

const autoFiles = process.argv.slice(2);

if (autoFiles.length === 0) {
  console.error(
    "Usage: node scripts/write_autos.mjs <compiled_auto1.js> [compiled_auto2.js ...]"
  );
  process.exit(1);
}

// Import each auto file so it registers its autos with AutoLib
for (const autoFile of autoFiles) {
  const absolutePath = path.resolve(autoFile);
  await import(pathToFileURL(absolutePath).href);
}

// AutoLib is a singleton; import it from the compiled dist to read registered autos
const autoLibPath = path.resolve(
  __dirname,
  "../dist/auto_generator/src/AutoLib.js"
);
const AutoLib = await import(pathToFileURL(autoLibPath).href);

const autos = AutoLib.getAutos();

if (autos.size === 0) {
  console.warn("No autos were registered.");
} else {
  // Build the object matching the Java Autos class: { "autos": { name: AutoAction, ... } }
  const autosObj = {};
  for (const [name, action] of autos.entries()) {
    autosObj[name] = action;
  }

  const output = { autos: autosObj };
  const content = JSON.stringify(output, null, 4);

  // Ensure the parent directory exists
  fs.mkdirSync(path.dirname(OUTPUT_FILE), { recursive: true });
  fs.writeFileSync(OUTPUT_FILE, content, "utf-8");
  console.log(`Written ${autos.size} auto(s) to: ${OUTPUT_FILE}`);
}
