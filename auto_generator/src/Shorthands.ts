// Probably want to modify where this is importing from and how it is named later, but for now this is just a test file to make sure the system is working
// And make it easier to write new autos without needing to write out the full class names every time

import * as AutoActions from '@/typescript/AutoAction.js';
import * as Units from '@/typescript/Units.js';

export function translation2d({ x, y }: { x: number | undefined; y: number | undefined }) {
    return new AutoActions.Translation2d({ x: x ?? 0, y: y ?? 0 });
}

export function rotation2d({ angleDegrees }: { angleDegrees: number | undefined }) {
    return new AutoActions.Rotation2d({ degrees: angleDegrees ?? 0 });
}

export function pose2d({ x, y, angleDegrees }: { x: number | undefined; y: number | undefined; angleDegrees: number | undefined }) {
    return new AutoActions.Pose2d({
        translation: translation2d({ x, y }),
        rotation: rotation2d({ angleDegrees })
    });
}

export function wait({ seconds }: { seconds: number }) {
    return new AutoActions.Wait({ delay: Units.Second.of(seconds) }).add();
}

export function print({ message }: { message: string }) {
    return new AutoActions.Print({ message }).add();
}