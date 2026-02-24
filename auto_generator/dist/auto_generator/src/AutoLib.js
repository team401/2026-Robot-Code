let command_pointers = [];
let autos = new Map();
export function getPointer() {
    return command_pointers[command_pointers.length - 1];
}
export function pushPointer(pointer) {
    command_pointers.push(pointer);
}
export function popPointer() {
    command_pointers.pop();
}
export function clearPointers() {
    command_pointers = [];
}
export function addCommand(command) {
    const pointer = getPointer();
    if (pointer) {
        pointer.push(command);
    }
}
export function auto(name, commands) {
    clearPointers();
    const auto = {
        "type": "Sequence",
        "actions": []
    };
    const firstPointer = auto.actions;
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
export function getAuto(name) {
    return autos.get(name);
}
export function getAutos() {
    return autos;
}
export function serializeAutos() {
    const obj = {};
    autos.forEach((commands, name) => {
        obj[name] = commands;
    });
    return JSON.stringify(obj, null, 4);
}
//# sourceMappingURL=AutoLib.js.map