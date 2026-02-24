import * as AutoLib from './AutoLib.js';
import { autopilot } from './Shorthands.js';
// This is purely for testing the system
AutoLib.auto("Random Auto", () => {
    for (let i = 0; i < 6; i++) {
        autopilot({ x: Math.random() * 5, y: Math.random() * 5, angle: Math.random() * 2 * Math.PI });
    }
});
//# sourceMappingURL=newTestAuto.js.map