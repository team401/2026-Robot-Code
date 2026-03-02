import * as AutoAction from "@/typescript/AutoAction.js";
import { type AutoAction as AutoCommand, setAddCommandHook } from "@/typescript/AutoAction.js";

// Clean up by Claude Sonnet 4.6

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------

/** Stack of action arrays tracking the current nesting context. */
let commandPointers: AutoCommand[][] = [];

/** All registered autos, keyed by name. */
const autos = new Map<string, AutoCommand>();

// Wire up the hook so that .add() on any AutoAction instance calls addCommand.
setAddCommandHook(addCommand);

// ---------------------------------------------------------------------------
// Pointer-stack helpers (used internally and by shorthand builders)
// ---------------------------------------------------------------------------

/** Returns the action array at the top of the pointer stack, or undefined. */
export function getPointer(): AutoCommand[] | undefined {
  return commandPointers.at(-1);
}

export function pushPointer(pointer: AutoCommand[]): void {
  commandPointers.push(pointer);
}

export function popPointer(): void {
  commandPointers.pop();
}

/** Returns the last command added to the current pointer, or undefined. */
export function getLastCommand(): AutoCommand | undefined {
  return getPointer()?.at(-1);
}

/** Clears the entire pointer stack (called before building each auto). */
export function clearPointers(): void {
  commandPointers = [];
}

/** Appends a command to the current pointer. No-op if the stack is empty. */
export function addCommand(command: AutoCommand): void {
  getPointer()?.push(command);
}

// ---------------------------------------------------------------------------
// Auto registration
// ---------------------------------------------------------------------------

/**
 * Defines and registers a named autonomous routine.
 *
 * The `build` callback runs synchronously; commands created inside it are
 * appended to a top-level Sequence via the pointer stack. The stack is always
 * restored — even if `build` throws — so a failed auto doesn't corrupt
 * subsequent registrations.
 */
export function auto(name: string, build: () => void): void {
  clearPointers();
  const root: AutoCommand = new AutoAction.Sequence({});
  pushPointer(root.actions as AutoCommand[]);

  try {
    build();
  } finally {
    clearPointers();
  }

  autos.set(name, root);
}

export function getAuto(name: string): AutoCommand | undefined {
  return autos.get(name);
}

export function getAutos(): Map<string, AutoCommand> {
  return autos;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

/** Keys stripped from the serialized output (internal typing metadata). */
const SKIPPED_KEYS = new Set(["_unitType"]);

/** JSON replacer that drops internal typing metadata. */
function stripInternalKeys(key: string, value: unknown): unknown {
  return SKIPPED_KEYS.has(key) ? undefined : value;
}

/** Serializes all registered autos to a JSON string. */
export function serializeAutos(): string {
  return JSON.stringify(Object.fromEntries(autos), stripInternalKeys, 4);
}

