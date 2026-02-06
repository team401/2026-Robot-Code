import * as fs from "fs";
import * as path from "path";

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

function generateTypes(jsonPath: string, outputDir: string): void {
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

  const outPath = path.join(outputDir, `${baseName}.d.ts`);
  fs.mkdirSync(outputDir, { recursive: true });
  fs.writeFileSync(outPath, lines.join("\n"));
  console.log(`Generated ${outPath}`);
}

// CLI: npx tsx scripts/generate-types.ts <json-file> [output-dir]
const jsonFile = process.argv[2] || "../src/main/deploy/constants/comp/ShotMaps.json";
const outputDir = process.argv[3] || "src/types";

generateTypes(path.resolve(jsonFile), path.resolve(outputDir));
