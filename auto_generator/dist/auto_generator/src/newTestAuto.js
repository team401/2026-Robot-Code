import * as AutoLib from './AutoLib.js';
function generateRandomAuto(depth) {
    for (let i = 0; i < Math.round(Math.random() * 2) + 1; i++) {
        let x = Math.random() * 5; // Random x between 0 and 5
        let y = Math.random() * 5; // Random y between 0 and 5
        let angle = Math.random() * 2 * Math.PI; // Random angle between 0 and 2π
        AutoLib.addCommand({
            type: "DriveToPose",
            target: {
                reference: {
                    translation: {
                        x,
                        y
                    },
                    rotation: {
                        radians: angle
                    }
                }
            }
        });
        if (Math.random() < 0.5) {
            if (depth < 4) {
                let seq = {
                    type: "Sequence",
                    actions: []
                };
                AutoLib.addCommand(seq);
                AutoLib.pushPointer(seq.actions);
                generateRandomAuto(depth + 1);
                AutoLib.popPointer();
            }
        }
    }
}
AutoLib.auto("Random Auto", () => {
    generateRandomAuto(0);
    generateRandomAuto(0);
});
//# sourceMappingURL=newTestAuto.js.map