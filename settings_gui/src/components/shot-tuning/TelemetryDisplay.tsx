import { Box, Typography } from '@mui/material';
import type { Telemetry } from '../../services/nt4';
import CheckIcon from '@mui/icons-material/Check';
import type { DistanceSource, TargetCoordinates } from '../../types/ShotTuning';

interface TelemetryDisplayProps {
  telemetry: Telemetry;
  distanceSource: DistanceSource;
  targetCoordinates: TargetCoordinates;
}

function fmt(value: number, decimals = 3): string {
  return value.toFixed(decimals);
}

export function TelemetryDisplay({ telemetry, distanceSource, targetCoordinates }: TelemetryDisplayProps) {
  const ntDist = telemetry.distanceToHubMeters;
  const odometryDist = telemetry.distanceMeters;

  return (
    <Box>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Live Telemetry
      </Typography>
      <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 0.5 }}>
        <Typography variant="body2" color="text.secondary">NetworkTables Distance:</Typography>
        <Typography variant="body2" fontFamily="monospace" sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          {ntDist !== null ? fmt(ntDist) : '--'} m
          {distanceSource === 'networkTables' && <CheckIcon color="success" sx={{ fontSize: 20 }} />}
        </Typography>

        <Typography variant="body2" color="text.secondary">Odometry Distance:</Typography>
        <Typography variant="body2" fontFamily="monospace" sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
          {fmt(odometryDist)} m
          {distanceSource === 'odometry' && <CheckIcon color="success" sx={{ fontSize: 20 }} />}
        </Typography>

        <Typography variant="body2" color="text.secondary">Target (x, y):</Typography>
        <Typography variant="body2" fontFamily="monospace">
          ({fmt(targetCoordinates.x)}, {fmt(targetCoordinates.y)})
        </Typography>

        <Typography variant="body2" color="text.secondary">Shooter Setpoint:</Typography>
        <Typography variant="body2" fontFamily="monospace">{fmt(telemetry.shooterRPM, 1)} RPM</Typography>

        <Typography variant="body2" color="text.secondary">Hood Setpoint:</Typography>
        <Typography variant="body2" fontFamily="monospace">{fmt(telemetry.hoodAngleDegrees, 2)} deg</Typography>

        <Typography variant="body2" color="text.secondary">Pose (x, y):</Typography>
        <Typography variant="body2" fontFamily="monospace">
          ({fmt(telemetry.robotPoseX)}, {fmt(telemetry.robotPoseY)})
        </Typography>
      </Box>
    </Box>
  );
}
