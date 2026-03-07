import * as AutoActions from '@/typescript/AutoAction.js';
import * as AutoLib from './AutoLib.js';
import * as Units from '@/typescript/Units.js';

// Cleaned up and improved by Claude Sonnet 4.6

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

export function translation2d({ x = 0, y = 0 }: { x?: number; y?: number }) {
    return new AutoActions.Translation2d({ x, y });
}

export function rotation2d({ angleDegrees = 0 }: { angleDegrees?: number }) {
    return new AutoActions.Rotation2d({ degrees: angleDegrees });
}

export function pose2d({ x = 0, y = 0, angleDegrees = 0 }: { x?: number; y?: number; angleDegrees?: number }) {
    return new AutoActions.Pose2d({
        translation: translation2d({ x, y }),
        rotation: rotation2d({ angleDegrees }),
    });
}

export function translation3d({ x = 0, y = 0, z = 0 }: { x?: number; y?: number; z?: number }) {
    return new AutoActions.Translation3d({ x, y, z });
}

export function rotation3d({ rollDegrees = 0, pitchDegrees = 0, yawDegrees = 0 }: { rollDegrees?: number; pitchDegrees?: number; yawDegrees?: number }) {
    return new AutoActions.Rotation3d({ roll: rollDegrees, pitch: pitchDegrees, yaw: yawDegrees });
}

export function pose3d({ x = 0, y = 0, z = 0, rollDegrees = 0, pitchDegrees = 0, yawDegrees = 0 }: { x?: number; y?: number; z?: number; rollDegrees?: number; pitchDegrees?: number; yawDegrees?: number }) {
    return new AutoActions.Pose3d({
        translation: translation3d({ x, y, z }),
        rotation: rotation3d({ rollDegrees, pitchDegrees, yawDegrees }),
    });
}

export function translation3dToPose2d({ x = 0, y = 0, z = 0 }: { x?: number; y?: number; z?: number }) {
    return new AutoActions.Pose2d({
        translation: translation2d({ x, y }),
        rotation: rotation2d({}),
    });
}

export function translation2dToPose2d(translation: AutoActions.Translation2d): AutoActions.Pose2d {
    return new AutoActions.Pose2d({
        translation,
        rotation: rotation2d({}),
    });
}

export function pose2dToPose3d({pose2d, z = 0}: {pose2d: AutoActions.Pose2d; z?: number}): AutoActions.Pose3d {
    return new AutoActions.Pose3d({
        translation: translation3d({ x: pose2d.translation?.x ?? 0, y: pose2d.translation?.y ?? 0, z }),
        rotation: rotation3d({ rollDegrees: 0, pitchDegrees: 0, yawDegrees: pose2d.rotation?.degrees ?? 0 }),
    });
}

export function pose3dToPose2d({pose3d}: {pose3d: AutoActions.Pose3d}): AutoActions.Pose2d {
    return new AutoActions.Pose2d({
        translation: translation2d({ x: pose3d.translation?.x ?? 0, y: pose3d.translation?.y ?? 0 }),
        rotation: rotation2d({ angleDegrees: pose3d.rotation ? (pose3d.rotation.yaw ?? 0) : 0 }),
    });
}

export function translate2dPose({ pose, translation }: { pose: AutoActions.Pose2d; translation: AutoActions.Translation2d }): AutoActions.Pose2d {
    const currentX = pose.translation?.x ?? 0;
    const currentY = pose.translation?.y ?? 0;
    const deltaX = translation.x ?? 0;
    const deltaY = translation.y ?? 0;
    return new AutoActions.Pose2d({
        translation: translation2d({ x: currentX + deltaX, y: currentY + deltaY }),
        rotation: pose.rotation ?? rotation2d({}),
    });
}

// ---------------------------------------------------------------------------
// Primitive commands
// ---------------------------------------------------------------------------

export function wait({ seconds }: { seconds: number }) {
    return new AutoActions.Wait({ delay: Units.Second.of(seconds) }).add();
}

export function print({ message }: { message: string }) {
    return new AutoActions.Print({ message }).add();
}

export function autoPilot({ targetPose, entryAngle, velocity }: {
    targetPose: AutoActions.Pose2d;
    entryAngle?: AutoActions.Rotation2d;
    velocity?: number;
}) {
    return new AutoActions.AutoPilotAction({
        target: new AutoActions.APTarget({
            reference: targetPose,
            ...(entryAngle !== undefined && { entryAngle }),
            ...(velocity !== undefined && { velocity }),
        }),
    }).add();
}

export function xBasedAutoPilotAction({ targetPose, entryAngle, velocity }: {
    targetPose: AutoActions.Pose2d;
    entryAngle?: AutoActions.Rotation2d;
    velocity?: number;
}) {
    return new AutoActions.XBasedAutoPilotAction({
        target: new AutoActions.APTarget({
            reference: targetPose,
            ...(entryAngle !== undefined && { entryAngle }),
            ...(velocity !== undefined && { velocity }),
        }),
    }).add();
}

// ---------------------------------------------------------------------------
// Container helpers
// ---------------------------------------------------------------------------

export function sequence(build: () => void) {
    AutoLib.withContainer(new AutoActions.Sequence({}), build);
}

export function parallel(build: () => void) {
    AutoLib.withContainer(new AutoActions.Parallel({}), build);
}

export function race(build: () => void) {
    AutoLib.withContainer(new AutoActions.Race({}), build);
}

export function reference(auto: string) {
    return new AutoActions.AutoReference({ name: auto }).add();
}