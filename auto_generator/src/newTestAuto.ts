import * as AutoLib from './AutoLib.js';
import { autoPilot, pose2d, pose3dToPose2d } from './Shorthands.js';
import { FieldConstants } from './FieldLocations.js';

// TODO: Add alliance-relative coordinate utilities.
// TODO: Replace placeholder coordinates with real field positions.
// TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.

AutoLib.auto("Test Auto", () => {
    // NOTE: These coordinates are placeholders and should be replaced with actual field positions.
    autoPilot({ targetPose: pose2d({ x: FieldConstants.LinesVertical.allianceZone / 2.0,
        y: FieldConstants.LinesHorizontal.center
    }) });             // Center of Alliance Zone
    autoPilot({ targetPose: pose3dToPose2d({ pose3d: FieldConstants.RightTrench.openingFloorCenter() }) });            // Middle of trench (right side)
    autoPilot({ targetPose: pose2d({ x: FieldConstants.LinesVertical.center, y: FieldConstants.LinesHorizontal.center }) });             // Center of field
});
