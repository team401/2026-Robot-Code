import * as AutoActions from '@/typescript/AutoAction.js';
import { FieldConstants } from './FieldLocations.js';
import * as AutoLib from './AutoLib.js';
import * as Units from '@/typescript/Units.js';
import { pose2d, rotation2d, transform2dPose, translate2dPose, translation2d, translation2dToPose2d } from './Shorthands.js';

// TODO: Maybe make these loaded from the constants files in the main robot code instead of hardcoded here,
// to avoid duplication and potential inconsistencies.

export const climbOffset: AutoActions.Transform2d = new AutoActions.Transform2d({ 
    translation: translation2d({
        x: 0.31,
        y: 0
    }),
    rotation: rotation2d({
        angleDegrees: -90.0
    })
});

export const leftClimbLocation = transform2dPose({ pose: translation2dToPose2d(FieldConstants.Tower.leftUpright()), transform: climbOffset });
export const rightClimbLocation = transform2dPose({ pose: translation2dToPose2d(FieldConstants.Tower.rightUpright()), transform: climbOffset });

export const climbLineupVelocity = 100.0;

export const climbLeftLineupEntryAngle = new AutoActions.Rotation2d({ degrees: 0 });
export const climbRightLineupEntryAngle = climbLeftLineupEntryAngle;