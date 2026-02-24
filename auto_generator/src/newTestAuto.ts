import type { AutoPilotAction } from '@/autos/AutoAction.js';
import * as AutoLib from './AutoLib.js';

AutoLib.auto("Random Auto", () => {
  AutoLib.addCommand(
    {
        "type": "DriveToPose",
        "target": {
            "reference": {
                "translation": {
                    "x": 0,
                    "y": 0
                },
                "rotation": {
                    "radians": 0
                }
            }
        }
    } as unknown as AutoPilotAction
  );
});