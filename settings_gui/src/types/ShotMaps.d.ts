export interface UnitValue {
  value: number;
  unit: string;
}

export interface ShotMapDataPoint {
  distance: UnitValue;
  shooterRPM: number;
  hoodAngle: UnitValue;
  flightTime: UnitValue;
}

export interface ShotMaps {
  hubDataPoints: ShotMapDataPoint[];
  passDataPoints: ShotMapDataPoint[];
  mechanismCompensationDelay?: UnitValue;
}
