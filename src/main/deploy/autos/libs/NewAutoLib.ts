import { AutoAction } from "./HardTest";

type AutoCommand = AutoAction;

let command_pointers: AutoCommand[][] = [];
let autos = new Map<string, AutoCommand[]>();

export function getPointer() {
  return command_pointers[command_pointers.length - 1];
}

export function pushPointer(pointer: AutoCommand[]) {
  command_pointers.push(pointer);
}

export function popPointer() {
  command_pointers.pop();
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
  const auto: AutoCommand[] = [];
  const firstPointer = auto;
  pushPointer(firstPointer);
  commands();
  if (command_pointers.length !== 1) {
    throw new Error("Pointer stack is not empty after auto command");
  }
  var endPointer = getPointer();
  if (endPointer != firstPointer) {
    throw new Error("Pointer stack was not properly managed");
  }
  command_pointers.pop();
  autos.set(name, auto);
}

export function getAuto(name: string): AutoCommand[] | undefined {
  return autos.get(name);
}

export function getAutos(): Map<string, AutoCommand[]> {
  return autos;
}

export function serializeAutos(): string {
  const obj: { [key: string]: AutoCommand[] } = {};
  autos.forEach((commands, name) => {
    obj[name] = commands;
  });
  return JSON.stringify(obj);
}