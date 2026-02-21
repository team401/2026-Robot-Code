const AutoLib = require("./libs/AutoLib.js");
const Geometry = require("./libs/GeometryLib.js");
const Units = require("./libs/UnitsLib.js");


function goToCenter() {
    AutoLib.autoPilotToPose(new Geometry.sPose2d({x: 8.25, y:4.1, rotation: 0.0}));
}
// No need for this as it is only one command
// goToCenter = makeRoutine(goToCenter);

AutoLib.startAuto();

AutoLib.autoPilotToPose(new Geometry.sPose2d({x: 1.0, y: 1.0, rotation: 2.1}));
goToCenter();
AutoLib.autoPilotToPose(new Geometry.sPose2d({x: 15.0, y: 1.0, rotation: 0.7}));
AutoLib.parallel(
    () => AutoLib.sequence(() => {
        goToCenter();
    })
)

AutoLib.auto("testAuto", () => {

});

AutoLib.endAuto();
