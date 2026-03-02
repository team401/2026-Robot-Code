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
