import { AutoPilotAction } from './libs/HardTest';
import * as AutoLib from './libs/NewAutoLib';

AutoLib.auto("Test Auto", () => {
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

console.log("Autos:", AutoLib.getAutos());