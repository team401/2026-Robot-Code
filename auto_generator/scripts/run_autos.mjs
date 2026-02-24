/**
 * run_autos.mjs
 *
 * Imports all compiled auto JS files (passed as CLI args), then reads the
 * accumulated autos from AutoLib and writes one JSON file per auto into
 * src/main/deploy/autos/.
 *
 * Usage (from auto_generator/):
 *   node scripts/run_autos.mjs dist/auto_generator/src/newTestAuto.js [...]
 */

import * as fs from "fs";
import * as path from "path";
import { fileURLToPath, pathToFileURL } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const OUTPUT_DIR = path.resolve(__dirname, "../../src/main/deploy/autos");

const autoFiles = process.argv.slice(2);

if (autoFiles.length === 0) {
  console.error(
    "Usage: node scripts/run_autos.mjs <compiled_auto1.js> [compiled_auto2.js ...]"
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
  fs.mkdirSync(OUTPUT_DIR, { recursive: true });
  for (const [name, commands] of autos.entries()) {
    const fileName = name.replace(/\s+/g, "_") + ".json";
    const outputPath = path.join(OUTPUT_DIR, fileName);
    const content = JSON.stringify({ auto: commands }, null, 4);
    fs.writeFileSync(outputPath, content, "utf-8");
    console.log(`Written: ${outputPath}`);
  }
}
