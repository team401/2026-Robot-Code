export interface JSONMeasure {
  value: number;
  unit: string;
}

export interface ShotMapDataPoint {
  distance: JSONMeasure;
  shooterRPM: number;
  hoodAngle: JSONMeasure;
  flightTime: JSONMeasure;
}

export interface ShotMaps {
  hubDataPoints: ShotMapDataPoint[];
  passDataPoints: ShotMapDataPoint[];
  mechanismCompensationDelay: JSONMeasure;
}

