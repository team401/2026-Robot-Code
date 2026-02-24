import * as AutoActions from '@/typescript/AutoAction.js';
import * as AutoLib from './AutoLib.js';
import { pose2d } from './Shorthands.js';


// This is purely for testing the system
AutoLib.auto("Random Auto", () => {
    for (let i = 0; i < 6; i++) {
        new AutoActions.AutoPilotAction({
            target: new AutoActions.APTarget({
                reference: pose2d({ x: Math.random() * 5, y: Math.random() * 5, angle: Math.random() * 2 * Math.PI })
            })
        }).add();
    }
});