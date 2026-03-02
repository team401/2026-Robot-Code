import * as AutoActions from '@/typescript/AutoAction.js';
import * as AutoLib from './AutoLib.js';
import { pose2d } from './Shorthands.js';

function linearDrive({ targetPose }: { targetPose: AutoActions.Pose2d }) {
    return new AutoActions.AutoPilotAction({
        target: new AutoActions.APTarget({
            reference: targetPose
        })
    })
}


// TODO: Add locations for the alliance and add alliance utils to make it so you can have 
// alliance relative coordinates for the autos.

// TODO: Replace all locations with actual coordinates
// TODO: Make it use AutoPilotAction with entry angle and exit velocity for trench.

AutoLib.auto("Test Auto", () => {
    // Center of Alliance Zone
    linearDrive({ targetPose: pose2d({ x: 1, y: 0, angleDegrees: 0 }) });
    // Middle of trench on the right side of the field
    linearDrive({ targetPose: pose2d({ x: 3, y: -1, angleDegrees: 0 }) });
    // Center of the field
    linearDrive({ targetPose: pose2d({ x: 4, y: 0, angleDegrees: 0 }) });
});