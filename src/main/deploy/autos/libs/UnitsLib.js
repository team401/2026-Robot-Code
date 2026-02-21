

function Unit(unit, value) {
    return {
        unit: unit,
        value: value
    }
}

function Meters(value) {
    return Unit("Meters", value)
}
function Radians(value) {
    return Unit("Radians", value)
}
function Seconds(value) {
    return Unit("Seconds", value)
}
function MetersPerSecond(value) {
    return Unit("MetersPerSecond", value)
}

module.exports = {
    Unit,
    Meters,
    Radians,
    Seconds,
    MetersPerSecond
}