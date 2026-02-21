const Geometry = require("./GeometryLib.js");
const Units = require("./UnitsLib.js");

let auto;
const pointers = [];

function push(command) {
    pointers[pointers.length - 1].push(command);
}

function pushPointer(pointer) {
    pointers.push(pointer);
}

function popPointer() {
    pointers.pop();
}

function createCommand(type, data) {
    return {
        type: type,
        ...data
    }
}

function _wrapper(command, pointer, commands){
    push(command)
    pushPointer(pointer);
    for (const command of commands) {
        command();
    }
    popPointer();
}

function sequence(...commands) {
    const s = createCommand("sequence", { actions: [] });
    _wrapper(s, s.actions, commands);
    return s;
}

const seq = sequence;

function parallel(...commands) {
    const par = createCommand("parallel", { actions: [] });
    _wrapper(par, par.actions, commands);
    return par;
}

function deadline(deadline, ...commands) {
    const dl = createCommand("deadline", { deadline: deadline, others: [] });
    _wrapper(dl, dl.others, commands);
    return dl;
}

function race(...commands) {
    const r = createCommand("race", { actions: [] });
    _wrapper(r, r.actions, commands);
    return r;
}

function makeRoutine(func, ...args) {
    return _ => seq(func, ...args);
}

function climb() {
    push(createCommand("climb", {}));
}

function climbLeft() {
    push(createCommand("climbLeft", {}));
}

function climbRight() {
    push(createCommand("climbRight", {}));
}

function wait(seconds) {
    push(createCommand("wait", { seconds: seconds }));
}

// Velocity is in m/s, entry angle is in radians, rotation distance is in meters
function apTarget({ pose = Geometry.Pose2d({}), exitVelocity = null, entryAngle = null, rotationRadius = null }) {
    return {
        reference: pose,
        velocity: (exitVelocity === null) ? null : Units.MetersPerSecond(exitVelocity),
        entryAngle: (entryAngle === null) ? null : Geometry.Rotation2d({rotation: entryAngle}),
        rotationRadius: (rotationRadius === null) ? null : Units.Meters(rotationRadius)
    }
}

// Velocity is in m/s, acceleration is in m/s^2, jerk is in m/s^3
function apConstraints({ velocity = null, acceleration = null, jerk = null }) {
    return {
        velocity: velocity,
        acceleration: acceleration,
        jerk: jerk
    }
}

// Beeline radius is in meters, errorXY is in meters, errorTheta is in radians
function apProfile({ constraints = null, beelineRadius = null, errorXY = null, errorTheta = null }) {
    return {
        constraints: constraints,
        beelineRadius: (beelineRadius === null) ? null : Units.Meters(beelineRadius),
        errorXY: (errorXY === null) ? null : Units.Meters(errorXY),
        errorTheta: (errorTheta === null) ? null : Units.Radians(errorTheta)
    }
}

function pidGains({ kP = 0, kI = 0, kD = 0 }) {
    return {
        kP: kP,
        kI: kI,
        kD: kD,
    }
}

function autoPilot({ target = apTarget({}), profile = null, gains = null }) {
    push(createCommand("autoPilot", {
        target: target,
        profile: profile,
        pidGains: gains
    }));
}

function autoPilotToPose(pose = Geometry.Pose2d({}), entryAngle = 0.0) {
    autoPilot(
        {
            "target": apTarget({
                pose,
                entryAngle
            })
        }
    )
}

function startAuto() {
    auto = createCommand("Auto", {
        "commands": []
    });
    pushPointer(auto.commands);
}

// Custom replacer function to skip null values
function ignoreNulls(key, value) {
  // If the value is null, skip it by returning undefined
  return value === null ? undefined : value;
}

function endAuto() {
    console.log(JSON.stringify(auto, ignoreNulls, 2));
    // TODO: Send auto to robot
    popPointer();
}


// Export all functions as an object
module.exports = {
    sequence,
    parallel,
    deadline,
    race,
    startAuto,
    makeRoutine,
    climb,
    climbLeft,
    climbRight,
    wait,
    apTarget,
    apConstraints,
    apProfile,
    pidGains,
    autoPilot,
    autoPilotToPose,
    endAuto,
    seq
};