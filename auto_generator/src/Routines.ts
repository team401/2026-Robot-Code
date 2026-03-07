import * as AutoLib from './AutoLib.js';
import { autoPilot, parallel, pose2d, pose3dToPose2d, rotation2d, sequence, xBasedAutoPilotAction, transform2dPose } from './Shorthands.js';
import * as AutoActions from '@/typescript/AutoAction.js';
import { FieldConstants } from './FieldLocations.js';
import * as Constants from './Constants.js';
import type { Pose2d } from './typescript/AutoAction.js';


function climbLineup({ targetPose, entryAngle, velocity }: {
    targetPose: AutoActions.Pose2d;
    entryAngle?: AutoActions.Rotation2d;
    velocity?: number;
}) {
    sequence(() => {
        parallel(() => {
            sequence(() => {
                xBasedAutoPilotAction({targetPose: 
                    transform2dPose({
                        pose:FieldConstants.Alliance.center,
                        transform: Constants.climbOffset
                    })});
                autoPilot({
                    targetPose,
                    ...(entryAngle !== undefined && { entryAngle }),
                    ...(velocity !== undefined && { velocity }),
                });
            });
            new AutoActions.ClimbSearchAction({}).add();
        });
        new AutoActions.ClimbHangAction({}).add();
    });
}

const _climbLineup = ({ targetPose, entryAngle, velocity }: {
    targetPose: AutoActions.Pose2d;
    entryAngle?: AutoActions.Rotation2d;
    velocity?: number;
}) => {
    return () => climbLineup({ 
        targetPose, 
        ...(entryAngle !== undefined && { entryAngle }), 
        ...(velocity !== undefined && { velocity })
    });
}


AutoLib.auto("LeftClimbLineup", _climbLineup({
    targetPose: Constants.leftClimbLocation,
    entryAngle: Constants.climbLeftLineupEntryAngle,
    velocity: Constants.climbLineupVelocity
}));

AutoLib.auto("RightClimbLineup", _climbLineup({
    targetPose: Constants.rightClimbLocation,
    entryAngle: Constants.climbRightLineupEntryAngle,
    velocity: Constants.climbLineupVelocity
}));
