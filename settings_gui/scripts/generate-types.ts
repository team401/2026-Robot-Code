import * as fs from "fs";
import * as path from "path";

const USAGE = `
Usage: npx tsx scripts/generate-types.ts <json-file> [-o <output-dir>]

Reads a JSON constants file and generates TypeScript interface definitions.
Objects with exactly {value, unit} fields are inferred as a shared UnitValue type.

Arguments:
  <json-file>           Path to the JSON file to generate types from (required)
  -o, --output <dir>    Write a .d.ts file to this directory (created if needed).
                        If omitted, output is printed to stdout.
  -h, --help            Show this help message

Examples:
  # Print generated types to stdout
  npx tsx scripts/generate-types.ts ../src/main/deploy/constants/comp/ShotMaps.json

  # Write to src/types/ShotMaps.d.ts
  npx tsx scripts/generate-types.ts ../src/main/deploy/constants/comp/ShotMaps.json -o src/types
`.trim();

const UNIT_VALUE_KEYS = new Set(["value", "unit"]);

function isUnitValue(obj: unknown): boolean {
  if (typeof obj !== "object" || obj === null || Array.isArray(obj)) return false;
  const keys = Object.keys(obj);
  return keys.length === 2 && keys.every((k) => UNIT_VALUE_KEYS.has(k));
}

function capitalize(s: string): string {
  return s.charAt(0).toUpperCase() + s.slice(1);
}

interface InterfaceDef {
  name: string;
  fields: { name: string; type: string; optional: boolean }[];
}

function inferType(
  value: unknown,
  fieldName: string,
  interfaces: Map<string, InterfaceDef>
): string {
  if (typeof value === "number") return "number";
  if (typeof value === "string") return "string";
  if (typeof value === "boolean") return "boolean";
  if (value === null) return "unknown";

  if (Array.isArray(value)) {
    if (value.length === 0) return "unknown[]";
    const elemType = inferType(value[0], fieldName, interfaces);
    return `${elemType}[]`;
  }

  if (typeof value === "object") {
    if (isUnitValue(value)) return "UnitValue";

    const interfaceName = capitalize(fieldName).replace(/s$/, "");
    // Avoid duplicate names by checking
    if (!interfaces.has(interfaceName)) {
      const fields: InterfaceDef["fields"] = [];
      for (const [k, v] of Object.entries(value as Record<string, unknown>)) {
        fields.push({
          name: k,
          type: inferType(v, k, interfaces),
          optional: false,
        });
      }
      interfaces.set(interfaceName, { name: interfaceName, fields });
    }
    return interfaceName;
  }

  return "unknown";
}

function generateTypes(jsonPath: string): string {
  const raw = fs.readFileSync(jsonPath, "utf-8");
  const data = JSON.parse(raw) as Record<string, unknown>;
  const baseName = path.basename(jsonPath, ".json");

  const interfaces = new Map<string, InterfaceDef>();

  // Always emit UnitValue first
  interfaces.set("UnitValue", {
    name: "UnitValue",
    fields: [
      { name: "value", type: "number", optional: false },
      { name: "unit", type: "string", optional: false },
    ],
  });

  // Infer top-level interface
  const topFields: InterfaceDef["fields"] = [];
  for (const [k, v] of Object.entries(data)) {
    topFields.push({
      name: k,
      type: inferType(v, k, interfaces),
      optional: false,
    });
  }
  interfaces.set(baseName, { name: baseName, fields: topFields });

  // Render
  const lines: string[] = [];
  for (const iface of interfaces.values()) {
    lines.push(`export interface ${iface.name} {`);
    for (const f of iface.fields) {
      const opt = f.optional ? "?" : "";
      lines.push(`  ${f.name}${opt}: ${f.type};`);
    }
    lines.push(`}`);
    lines.push(``);
  }

  return lines.join("\n");
}

// --- CLI ---
const args = process.argv.slice(2);

if (args.includes("-h") || args.includes("--help")) {
  console.log(USAGE);
  process.exit(0);
}

// Parse arguments
let jsonFile: string | undefined;
let outputDir: string | undefined;

for (let i = 0; i < args.length; i++) {
  if (args[i] === "-o" || args[i] === "--output") {
    outputDir = args[++i];
    if (!outputDir) {
      console.error("Error: -o/--output requires a directory argument");
      process.exit(1);
    }
  } else if (args[i].startsWith("-")) {
    console.error(`Error: unknown option '${args[i]}'`);
    console.error(USAGE);
    process.exit(1);
  } else {
    jsonFile = args[i];
  }
}

if (!jsonFile) {
  console.error("Error: no JSON file specified\n");
  console.error(USAGE);
  process.exit(1);
}

const output = generateTypes(path.resolve(jsonFile));

if (outputDir) {
  const baseName = path.basename(jsonFile, ".json");
  const resolvedDir = path.resolve(outputDir);
  const outPath = path.join(resolvedDir, `${baseName}.d.ts`);
  fs.mkdirSync(resolvedDir, { recursive: true });
  fs.writeFileSync(outPath, output);
  console.error(`Generated ${outPath}`);
} else {
  process.stdout.write(output);
}
