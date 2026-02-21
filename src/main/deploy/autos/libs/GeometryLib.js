
function Translation2d({x = 0.0, y = 0.0}) {
    return {
        x: x,
        y: y
    }
}

function Rotation2d({rotation = 0.0}) {
    return {
        rotation: rotation
    }
}

// Pose2d is in meters and radians
function Pose2d({translation = Translation2d({}), rotation = Rotation2d({})}) {
    return {
        translation: translation,
        rotation: rotation
    }
}

function sPose2d({x = 0.0, y = 0.0, rotation = 0.0}) {
    return Pose2d({
        translation: Translation2d({x: x, y: y}),
        rotation: Rotation2d({rotation: rotation})
    })
}


function Transform2d({translation = Translation2d({}), rotation = Rotation2d({})}) {
    return {
        translation: translation,
        rotation: rotation
    }
}

function sTransform2d({x = 0.0, y = 0.0, rotation = 0.0}) {
    return Transform2d({
        translation: Translation2d({x: x, y: y}),
        rotation: Rotation2d({rotation: rotation})
    })
}


module.exports = {
    Translation2d,
    Rotation2d,
    Pose2d,
    sPose2d,
    Transform2d,
    sTransform2d
}