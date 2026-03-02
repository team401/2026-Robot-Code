import * as AutoAction  from "@/typescript/AutoAction.js";
import { type AutoAction as AutoCommand, setAddCommandHook } from "@/typescript/AutoAction.js";

let command_pointers: AutoCommand[][] = [];
let autos = new Map<string, AutoCommand>();

// Wire up the hook so that .add() on any AutoAction class calls addCommand
setAddCommandHook((command) => addCommand(command));

export function getPointer() {
  return command_pointers[command_pointers.length - 1];
}

export function pushPointer(pointer: AutoCommand[]) {
  command_pointers.push(pointer);
}

export function popPointer() {
  command_pointers.pop();
}

export function getLastCommand(): AutoCommand | undefined {
  const pointer = getPointer();
  if (pointer && pointer.length > 0) {
    return pointer[pointer.length - 1];
  }
  return undefined;
}

export function clearPointers() {
  command_pointers = [];
}

export function addCommand(command: AutoCommand) {
  const pointer = getPointer();
  if (pointer) {
    pointer.push(command);
  }
}

export function auto(name: string, commands: () => void) {
  clearPointers();
  const auto: AutoCommand = new AutoAction.Sequence({});
  const firstPointer = auto.actions as AutoCommand[];
  pushPointer(firstPointer);
  commands();
  if (command_pointers.length !== 1) {
    throw new Error("Pointer stack is not empty after auto command");
  }
  const endPointer = getPointer();
  if (endPointer != firstPointer) {
    throw new Error("Pointer stack was not properly managed");
  }
  command_pointers.pop();
  autos.set(name, auto);
}

export function getAuto(name: string): AutoCommand | undefined {
  return autos.get(name);
}

export function getAutos(): Map<string, AutoCommand> {
  return autos;
}

const skippedKeys = new Set(["_unitType"]);

function ignoreTyping(key: string, value: any) {
  if (skippedKeys.has(key)) {
    return undefined;
  }
  return value;
}

export function serializeAutos(): string {
  const obj: { [key: string]: AutoCommand } = {};
  autos.forEach((commands, name) => {
    obj[name] = commands;
  });
  return JSON.stringify(obj, ignoreTyping, 4);
}

