import * as AutoActions from '@/typescript/AutoAction.js';
import { FieldConstants } from './FieldLocations.js';
import * as AutoLib from './AutoLib.js';
import * as Units from '@/typescript/Units.js';
import { pose2d, translate2dPose, translation2dToPose2d } from './Shorthands.js';

// TODO: Maybe make these loaded from the constants files in the main robot code instead of hardcoded here,
// to avoid duplication and potential inconsistencies.

export const climbOffset: AutoActions.Translation2d = new AutoActions.Translation2d({ x: 0.4, y: 0 });

export const leftClimbLocation = translate2dPose({ pose: translation2dToPose2d(FieldConstants.Tower.leftUpright()), translation: climbOffset });
export const rightClimbLocation = translate2dPose({ pose: translation2dToPose2d(FieldConstants.Tower.rightUpright()), translation: climbOffset });

export const climbLineupVelocity = 100.0;

export const climbLeftLineupEntryAngle = new AutoActions.Rotation2d({ degrees: 0 });
export const climbRightLineupEntryAngle = climbLeftLineupEntryAngle;