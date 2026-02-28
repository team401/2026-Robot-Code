import { Box, Typography } from '@mui/material';
import type { Telemetry } from '../../services/nt4';

interface TelemetryDisplayProps {
  telemetry: Telemetry;
}

function fmt(value: number, decimals = 3): string {
  return value.toFixed(decimals);
}

export function TelemetryDisplay({ telemetry }: TelemetryDisplayProps) {
  const ntDist = telemetry.distanceToHubMeters;
  const poseDist = telemetry.distanceMeters;

  return (
    <Box>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Live Telemetry
      </Typography>
      <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 0.5 }}>
        <Typography variant="body2" color="text.secondary">Distance to Hub:</Typography>
        <Typography variant="body2" fontFamily="monospace">
          {ntDist !== null ? fmt(ntDist) : '--'} m
          {' '}
          <Typography component="span" variant="caption" color="text.secondary">
            (pose: {fmt(poseDist)} m)
          </Typography>
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
