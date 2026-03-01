# Memory Allocation Optimizations

**Optimized by:** Claude 3.5 Sonnet (Anthropic)
**Date:** March 2026
**Goal:** Reduce garbage collection pressure on the roboRIO's constrained 100MB heap

## Overview

These optimizations reduce per-cycle memory allocations to minimize garbage collection (GC) pauses on the roboRIO. On a 2-core, 32-bit ARM processor with limited heap space, frequent allocations lead to:
- More frequent GC cycles
- Stop-the-world pauses that can cause loop overruns
- Inconsistent loop timing

The primary strategies used:
1. **Pre-allocated arrays** - Reuse fixed-size arrays instead of creating new ones each cycle
2. **Mutable measures** - Use WPILib's `MutAngle`/`MutAngularVelocity` with `mut_replace()` instead of `Unit.of()`
3. **Primitive math** - Compute rotations and transforms using doubles instead of allocating geometry objects
4. **Object reuse** - Cache and reuse objects like `EnhancedLine2d` via mutable setters

## Files Modified

### Drive System

#### `ModuleIOTalonFX.java`
- **Change:** Added pre-allocated arrays for odometry data
- **Before:** Used `stream().mapToDouble().toArray()` which allocates intermediate objects and a new array each cycle
- **After:** Drain queues into pre-allocated `double[]` and `Rotation2d[]` arrays using loops
- **Impact:** Eliminates ~3 array allocations + stream overhead per module per cycle (×4 modules)

#### `GyroIOPigeon2.java`
- **Change:** Same pattern as ModuleIOTalonFX
- **Before:** Stream-based queue draining
- **After:** Loop-based draining into pre-allocated arrays
- **Impact:** Eliminates array allocations for gyro odometry data

#### `Drive.java`
- **Change:** Pre-allocated `SwerveModulePosition[]` arrays for odometry loop
- **Before:** Created new `SwerveModulePosition` arrays per odometry sample
- **After:** Reuse `modulePositions[4]` and `moduleDeltas[4]` arrays, update in place
- **Impact:** Eliminates 2 array allocations × number of odometry samples per cycle

#### `Module.java`
- **Change:** Pre-allocated odometry positions array + cached logger key string
- **Before:** Created new `SwerveModulePosition[]` each cycle, concatenated logger key string
- **After:** Reuse pre-allocated array, cache the logger key at construction
- **Impact:** Eliminates array allocation + string concatenation per module per cycle

#### `ModuleIOSim.java`
- **Change:** Pre-allocated arrays matching real robot pattern
- **Impact:** Maintains consistency with real robot code, avoids allocations in sim

### Subsystems (Mutable Measures)

#### `ShooterSubsystem.java`
- **Change:** `MutAngularVelocity cachedVelocity` for `getVelocity()` method
- **Before:** `return RadiansPerSecond.of(inputs.velocityRadPerSec)` allocates new Measure
- **After:** `return cachedVelocity.mut_replace(inputs.velocityRadPerSec, RadiansPerSecond)`
- **Impact:** Eliminates 1 Measure allocation per cycle

#### `HopperSubsystem.java`
- **Change:** `MutAngularVelocity cachedVelocity` for `getHopperVelocity()` method
- **Impact:** Eliminates 1 Measure allocation per cycle

#### `IndexerSubsystem.java`
- **Change:** `MutAngularVelocity cachedVelocity` for `getVelocity()` method
- **Impact:** Eliminates 1 Measure allocation per cycle

#### `TurretSubsystem.java`
- **Change:** `MutAngle cachedAngle` + `MutAngularVelocity cachedVelocity`
- **Impact:** Eliminates 2 Measure allocations per cycle

#### `HoodSubsystem.java`
- **Change:** `MutAngle cachedAngle` + `MutAngularVelocity cachedVelocity`
- **Impact:** Eliminates 2 Measure allocations per cycle

#### `IntakeSubsystem.java`
- **Change:** `MutAngle cachedPivotAngle` for `getCurrentPivotAngle()` method
- **Impact:** Eliminates 1 Measure allocation per cycle

### Coordination Layer

#### `CoordinationLayer.java`

**Test mode distance calculation:**
- **Before:** `new Pose3d().plus().getTranslation().toTranslation2d().getDistance(new Translation2d())`
- **After:** Manual math: `Math.sqrt(dx * dx + dy * dy)`
- **Impact:** Eliminates Pose3d, Translation3d, Translation2d allocations

**`shouldStowHoodBasedOnMovement()` method:**
- **Before:**
  - `new Translation2d().rotateBy()` for field-centric speeds
  - `new Pose3d().plus().getTranslation().toTranslation2d()` for shooter position
  - `new EnhancedLine2d()` each cycle
  - `new Translation2d[]{}` for logging
- **After:**
  - Manual rotation using `cos`/`sin` primitives
  - Manual transform calculation
  - Cached `EnhancedLine2d` reused via `set()` method
  - Pre-allocated `cachedShooterTrajectory` array
- **Impact:** Eliminates ~6-8 geometry object allocations per cycle

**`runShotCalculatorWithDrive()` method:**
- **Before:**
  - `new Pose3d(robotPose).plus(robotToShooter).getTranslation()` - 2 Pose3d + Translation3d
  - `ChassisSpeeds.fromRobotRelativeSpeeds()` - new ChassisSpeeds
  - `new Translation3d(0, 0, omega)` for omega vector
  - `robotToShooterTranslation.rotateBy(new Rotation3d())` - Rotation3d + Translation3d
  - `omega_vec.cross()` - Vector<N3> allocation
  - `new Translation2d()` for shooter velocity
  - `ShotCalculations.calculateShotFromMap()` returning new `MapBasedShotInfo` record
- **After:**
  - All calculations done with primitive `double` math
  - Cross product computed manually: `vRotX = -omega * ry`, `vRotY = omega * rx`
  - Uses `calculateShotFromMapInPlace()` with cached `MutableMapBasedShotInfo`
- **Impact:** Eliminates ~10+ geometry/math object allocations per cycle

### Shot Calculations

#### `ShotCalculations.java`

**New `MutableMapBasedShotInfo` class:**
- Mutable alternative to `MapBasedShotInfo` record
- `set()` method to update values in place
- Same getter methods as the record for API compatibility

**New `calculateShotFromMapInPlace()` method:**
- Takes primitive `double` parameters instead of `Translation3d`/`Translation2d`
- Stores output in a `MutableMapBasedShotInfo` parameter
- All distance calculations use `Math.sqrt(dx*dx + dy*dy)` instead of `Translation2d.getDistance()`
- **Impact:** Eliminates record allocation + multiple Translation2d/Translation3d allocations per cycle

### Utilities

#### `EnhancedLine2d.java`
- **Change:** Made fields non-final, added `set(Translation2d start, Translation2d end)` method
- **Before:** Immutable, required `new EnhancedLine2d()` each time endpoints changed
- **After:** Can reuse same instance by calling `set()`
- **Impact:** Eliminates EnhancedLine2d allocation in `shouldStowHoodBasedOnMovement()`

## Estimated Impact

### Per-Cycle Allocation Reduction

| Area | Allocations Eliminated |
|------|----------------------|
| Drive odometry (4 modules + gyro) | ~15-20 arrays/objects |
| Subsystem measures | ~8 Measure objects |
| Coordination layer | ~15-20 geometry objects |
| Shot calculations | ~10 geometry objects + 1 record |
| **Total** | **~50+ object allocations per cycle** |

### Expected Results

- **Simulation:** Reduced allocation rate by ~200-500 KB/s
- **Real robot:** Should see proportionally larger improvement due to:
  - Smaller heap (100MB vs multi-GB)
  - Less powerful GC
  - No spare CPU cores to hide GC latency

## Notes

### What Was NOT Changed

- **Optional usage:** `Optional.ifPresent()` has negligible overhead - the Optional objects are stored as fields (created once), and `ifPresent()` is just a null check
- **Logging allocations:** Some `Translation3d` allocations remain for `Logger.recordOutput()` calls - these can be disabled in competition if needed
- **WPILib geometry classes:** `Rotation2d`, `Pose2d`, `SwerveModulePosition` are immutable by design - we work around this where critical

### Tradeoffs

- **Mutable measures:** Callers must not store references to returned measures, as values will change. This is documented in method Javadocs.
- **Code complexity:** Primitive math is less readable than geometry class operations, but comments explain the equivalent operations.
- **EnhancedLine2d mutability:** Changed from immutable to mutable pattern - callers should be aware the object can change.

## Testing

All changes verified to:
1. Compile successfully with `./gradlew build`
2. Pass existing tests
3. Run correctly in simulation

Real robot testing pending (robot access unavailable at time of optimization).
