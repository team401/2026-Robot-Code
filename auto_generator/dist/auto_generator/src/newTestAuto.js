import * as AutoLib from './AutoLib.js';
AutoLib.auto("Test Auto", () => {
    AutoLib.addCommand({
        "type": "DriveToPose",
        "target": {
            "reference": {
                "translation": {
                    "x": 0,
                    "y": 0
                },
                "rotation": {
                    "radians": 0
                }
            }
        }
    });
});
console.log("Autos:", AutoLib.getAutos());
console.log("Json Autos", AutoLib.serializeAutos());
//# sourceMappingURL=newTestAuto.js.map