import * as AutoLib from './AutoLib.js';
import { autoPilot, pose2d } from './Shorthands.js';

// TODO: Add alliance-relative coordinate utilities.
// TODO: Replace placeholder coordinates with real field positions.
// TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.

AutoLib.auto("Test Auto", () => {
    // NOTE: These coordinates are placeholders and should be replaced with actual field positions.
    autoPilot({ targetPose: pose2d({ x: 1, y: 0 }) });             // Center of Alliance Zone
    autoPilot({ targetPose: pose2d({ x: 3, y: -1 }) });            // Middle of trench (right side)
    autoPilot({ targetPose: pose2d({ x: 4, y: 0 }) });             // Center of field
});
