import * as AutoAction from "@/typescript/AutoAction.js";
import { type AutoAction as AutoCommand, setAddCommandHook } from "@/typescript/AutoAction.js";

// Cleaned up and improved by Claude Sonnet 4.6

// ---------------------------------------------------------------------------
// Internal pointer stack
// ---------------------------------------------------------------------------

/** Stack of action arrays tracking the current nesting context. */
let commandPointers: AutoCommand[][] = [];

function currentPointer(): AutoCommand[] | undefined {
  return commandPointers.at(-1);
}

function pushPointer(pointer: AutoCommand[]): void {
  commandPointers.push(pointer);
}

function popPointer(): void {
  commandPointers.pop();
}

function addCommand(command: AutoCommand): void {
  currentPointer()?.push(command);
}

// Wire up the hook so that .add() on any AutoAction instance calls addCommand.
setAddCommandHook(addCommand);

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/** Returns the last command added in the current context, or undefined. */
export function getLastCommand(): AutoCommand | undefined {
  return currentPointer()?.at(-1);
}

/**
 * Runs `build` with a container command as the active context, then adds the
 * finished container to the parent context. Use this to nest Sequence,
 * Parallel, or Race blocks without touching the pointer stack directly.
 *
 * @example
 * withContainer(new AutoAction.Parallel({}), () => {
 *   drive({ targetPose: … }).add();
 *   shoot({}).add();
 * });
 */
export function withContainer(
  container: AutoAction.Sequence | AutoAction.Parallel | AutoAction.Race,
  build: () => void
): void {
  pushPointer(container.actions as AutoCommand[]);
  try {
    build();
  } finally {
    popPointer();
  }
  addCommand(container);
}

// ---------------------------------------------------------------------------
// Auto registration
// ---------------------------------------------------------------------------

/** All registered autos, keyed by name. */
const autos = new Map<string, AutoCommand>();

/**
 * Defines and registers a named autonomous routine.
 *
 * The `build` callback runs synchronously; commands created inside it are
 * appended to a top-level Sequence via the pointer stack. The stack is always
 * restored — even if `build` throws — so a failed auto doesn't corrupt
 * subsequent registrations.
 */
export function auto(name: string, build: () => void): void {
  commandPointers = [];
  const root = new AutoAction.Sequence({});
  pushPointer(root.actions as AutoCommand[]);

  try {
    build();
  } finally {
    commandPointers = [];
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

