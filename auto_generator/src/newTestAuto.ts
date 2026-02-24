import type { AutoPilotAction, Sequence } from '@/typescript/AutoAction.js';
import * as AutoLib from './AutoLib.js';

// This is purely for testing the system
AutoLib.auto("Random Auto", () => {
    for (let i = 0; i < 6; i++) {
        let x = Math.random() * 5; // Random x between 0 and 5
        let y = Math.random() * 5; // Random y between 0 and 5
        let angle = Math.random() * 2 * Math.PI; // Random angle between 0 and 2π
        AutoLib.addCommand(
            {
                type: "DriveToPose",
                target: {
                    reference: {
                        translation: {
                            x,
                            y
                        },
                        rotation: {
                            radians: angle
                        }
                    }
                }
            } as unknown as AutoPilotAction
        );
    }
});
