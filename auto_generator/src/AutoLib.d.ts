import type { AutoAction as AutoCommand } from "@/autos/AutoAction.js";
export declare function getPointer(): AutoCommand[] | undefined;
export declare function pushPointer(pointer: AutoCommand[]): void;
export declare function popPointer(): void;
export declare function clearPointers(): void;
export declare function addCommand(command: AutoCommand): void;
export declare function auto(name: string, commands: () => void): void;
export declare function getAuto(name: string): AutoCommand[] | undefined;
export declare function getAutos(): Map<string, AutoCommand[]>;
export declare function serializeAutos(): string;
//# sourceMappingURL=AutoLib.d.ts.map