const AutoLib = require("./libs/AutoLib.js");
const Geometry = require("./libs/GeometryLib.js");
const Units = require("./libs/UnitsLib.js");


function goToCenter() {
    AutoLib.autoPilotToPose(new Geometry.sPose2d({x: 8.25, y:4.1, rotation: 0.0}));
}
// No need for this as it is only one command
// goToCenter = makeRoutine(goToCenter);

AutoLib.startAuto();

function degToRad(degrees) {
    return degrees * (Math.PI / 180);
}

AutoLib.autoPilot({
    target: new AutoLib.apTarget({
        pose: new Geometry.sPose2d({x: -15.0, y: 3.9, rotation: -1.57079}),
        rotationRadius: 1.5,
        entryAngle: Geometry.Rotation2d({})
    }),
    profile: new AutoLib.apProfile({
        constraints: new AutoLib.apConstraints({
            acceleration: 6,
            jerk: 3
        }),
        errorXY: 0.02,
        errorTheta: degToRad(15)
    })
});

// AutoLib.auto("testAuto", () => {

// });

AutoLib.endAuto();
