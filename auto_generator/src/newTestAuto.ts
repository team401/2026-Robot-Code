import * as AutoLib from './AutoLib.js';
import { autoPilot, pose2d, pose3dToPose2d, rotation2d, translation2dToPose2d, xBasedAutoPilotAction, reference, translate2dPose, translation2d, wait, transform2dPose } from './Shorthands.js';
import * as AutoActions from '@/typescript/AutoAction.js';
import { FieldConstants } from './FieldLocations.js';
import { Meter } from '@/typescript/Units.js';

// TODO: Add alliance-relative coordinate utilities.
// TODO: Replace placeholder coordinates with real field positions.
// TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.

AutoLib.auto("Test Auto", () => {
    // NOTE: These coordinates are placeholders and should be replaced with actual field positions.
    autoPilot({ targetPose: FieldConstants.Alliance.center });             // Center of Alliance Zone
    xBasedAutoPilotAction({ targetPose: pose3dToPose2d({ pose3d: FieldConstants.RightTrench.openingFloorCenter() }),
        entryAngle: rotation2d({ angleDegrees: 180 }),
        velocity: 100.0,
    });            // Middle of trench (right side)
    autoPilot({ targetPose: pose2d({ x: FieldConstants.LinesVertical.center, y: FieldConstants.LinesHorizontal.center, angleDegrees: 90}), rotationRadius: Meter.of(2.5),
        entryAngle: rotation2d({ angleDegrees: -90 })
    });             // Center of field
    xBasedAutoPilotAction(
        {
            targetPose: transform2dPose(
                {
                    pose: pose3dToPose2d({ pose3d: FieldConstants.RightTrench.openingFloorCenter() }),
                    transform: new AutoActions.Transform2d({ translation: {x: 0, y: 0}, rotation: { degrees: 180 } })
                }),
            entryAngle: rotation2d({ angleDegrees: 0 }),
            velocity: 100.0,
        });
    autoPilot({ 
        targetPose: transform2dPose({
            pose: translation2dToPose2d(FieldConstants.Outpost.centerPoint()), 
            transform: new AutoActions.Transform2d({ translation: {x: 1, y: 0}, rotation: { degrees: 180} })
        }),
    }); 
    reference("LeftClimbLineup");
});
