import { Box, Typography } from '@mui/material';
import type { Telemetry } from '../../services/nt4';

interface TelemetryDisplayProps {
  telemetry: Telemetry;
}

function fmt(value: number, decimals = 3): string {
  return value.toFixed(decimals);
}

export function TelemetryDisplay({ telemetry }: TelemetryDisplayProps) {
  return (
    <Box>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Live Telemetry
      </Typography>
      <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 0.5 }}>
        <Typography variant="body2" color="text.secondary">Distance to Hub:</Typography>
        <Typography variant="body2" fontFamily="monospace">{fmt(telemetry.distanceMeters)} m</Typography>

        <Typography variant="body2" color="text.secondary">Shooter Velocity:</Typography>
        <Typography variant="body2" fontFamily="monospace">{fmt(telemetry.shooterRPMRadPerSec * 60 / (2 * Math.PI), 1)} RPM</Typography>

        <Typography variant="body2" color="text.secondary">Hood Angle:</Typography>
        <Typography variant="body2" fontFamily="monospace">{fmt(telemetry.hoodAngleRadians * 180 / Math.PI, 2)} deg</Typography>

        <Typography variant="body2" color="text.secondary">Pose (x, y):</Typography>
        <Typography variant="body2" fontFamily="monospace">
          ({fmt(telemetry.robotPoseX)}, {fmt(telemetry.robotPoseY)})
        </Typography>
      </Box>
    </Box>
  );
}
