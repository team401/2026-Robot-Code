import { useMemo, useState } from 'react';
import {
  Alert,
  Box,
  Checkbox,
  FormControlLabel,
  Paper,
  Stack,
  useTheme,
} from '@mui/material';
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts';
import type { ShotMapDataPoint } from '../../types/ShotMaps';
import {
  buildChartData,
  buildCubicSeries,
  buildQuadraticSeries,
  buildSplineInterpolationSeries,
  type XYPoint,
} from '../../utils/shotMapRegression';

interface ShotMapChartProps {
  chartId: string;
  rows: ShotMapDataPoint[];
}

type FitVisibility = {
  quadratic: boolean;
  cubic: boolean;
  spline: boolean;
};

const DEFAULT_FIT_VISIBILITY: FitVisibility = {
  quadratic: false,
  cubic: false,
  spline: false,
};

const METRIC_CONFIG = {
  shooterRPM: {
    actualKey: 'actualShooterRPM',
    quadraticKey: 'quadraticShooterRPM',
    cubicKey: 'cubicShooterRPM',
    splineKey: 'splineShooterRPM',
    color: '#1565c0',
  },
  hoodAngle: {
    actualKey: 'actualHoodAngle',
    quadraticKey: 'quadraticHoodAngle',
    cubicKey: 'cubicHoodAngle',
    splineKey: 'splineHoodAngle',
    color: '#2e7d32',
  },
  flightTime: {
    actualKey: 'actualFlightTime',
    quadraticKey: 'quadraticFlightTime',
    cubicKey: 'cubicFlightTime',
    splineKey: 'splineFlightTime',
    color: '#ef6c00',
  },
} as const;

function toSeries(rows: ShotMapDataPoint[]) {
  const sortedRows = [...rows].sort((a, b) => a.distance.value - b.distance.value);

  return {
    shooterRPM: sortedRows.map((row) => ({ x: row.distance.value, y: row.shooterRPM })),
    hoodAngle: sortedRows.map((row) => ({ x: row.distance.value, y: row.hoodAngle.value })),
    flightTime: sortedRows.map((row) => ({ x: row.distance.value, y: row.flightTime.value })),
  } satisfies Record<keyof typeof METRIC_CONFIG, XYPoint[]>;
}

function formatDistance(value: number) {
  return Number.isFinite(value) ? value.toFixed(2) : '';
}

export function ShotMapChart({ chartId, rows }: ShotMapChartProps) {
  const theme = useTheme();
  const [fitVisibilityByChart, setFitVisibilityByChart] = useState<Record<string, FitVisibility>>({
    [chartId]: DEFAULT_FIT_VISIBILITY,
  });
  const fitVisibility = fitVisibilityByChart[chartId] ?? DEFAULT_FIT_VISIBILITY;

  const { chartData, hasData, xDomain } = useMemo(() => {
    const series = toSeries(rows);
    const distances = rows.map((row) => row.distance.value).filter(Number.isFinite);
    const minDistance = distances.length > 0 ? Math.min(...distances) : 0;
    const maxDistance = distances.length > 0 ? Math.max(...distances) : 0;
    const computedDomain: [number, number] =
      Math.abs(maxDistance - minDistance) < 1e-10
        ? [Math.max(0, minDistance - 0.25), maxDistance + 0.25]
        : [Math.max(0, minDistance), maxDistance];

    const fitSeries = {
      [METRIC_CONFIG.shooterRPM.quadraticKey]: buildQuadraticSeries(series.shooterRPM),
      [METRIC_CONFIG.shooterRPM.cubicKey]: buildCubicSeries(series.shooterRPM),
      [METRIC_CONFIG.shooterRPM.splineKey]: buildSplineInterpolationSeries(series.shooterRPM),
      [METRIC_CONFIG.hoodAngle.quadraticKey]: buildQuadraticSeries(series.hoodAngle),
      [METRIC_CONFIG.hoodAngle.cubicKey]: buildCubicSeries(series.hoodAngle),
      [METRIC_CONFIG.hoodAngle.splineKey]: buildSplineInterpolationSeries(series.hoodAngle),
      [METRIC_CONFIG.flightTime.quadraticKey]: buildQuadraticSeries(series.flightTime),
      [METRIC_CONFIG.flightTime.cubicKey]: buildCubicSeries(series.flightTime),
      [METRIC_CONFIG.flightTime.splineKey]: buildSplineInterpolationSeries(series.flightTime),
    };

    const actualSeries = {
      [METRIC_CONFIG.shooterRPM.actualKey]: series.shooterRPM,
      [METRIC_CONFIG.hoodAngle.actualKey]: series.hoodAngle,
      [METRIC_CONFIG.flightTime.actualKey]: series.flightTime,
    };

    return {
      chartData: buildChartData(actualSeries, fitSeries),
      hasData: rows.length > 0,
      xDomain: computedDomain,
    };
  }, [rows]);

  if (!hasData) {
    return (
      <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
        <Alert severity="info">Add or load data points to see the chart and regression overlays.</Alert>
      </Paper>
    );
  }

  return (
    <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
      <Box sx={{ width: '100%', height: { xs: 380, md: 460 } }}>
        <ResponsiveContainer>
          <LineChart
            data={chartData}
            margin={{ top: 12, right: 24, bottom: 28, left: 24 }}
          >
            <CartesianGrid stroke={theme.palette.divider} strokeDasharray="3 3" />
            <XAxis
              type="number"
              dataKey="distance"
              domain={xDomain}
              tickFormatter={formatDistance}
              label={{ value: 'Distance (m)', position: 'insideBottom', offset: -8 }}
            />
            <YAxis
              yAxisId="rpm"
              orientation="left"
              width={80}
              axisLine={{ stroke: METRIC_CONFIG.shooterRPM.color }}
              tickLine={{ stroke: METRIC_CONFIG.shooterRPM.color }}
              tick={{ fill: METRIC_CONFIG.shooterRPM.color }}
              tickFormatter={(value: number) => value.toFixed(0)}
              label={{
                value: 'Shooter RPM',
                angle: -90,
                position: 'insideLeft',
                dx: 15,
                fill: METRIC_CONFIG.shooterRPM.color,
              }}
            />
            <YAxis
              yAxisId="hood"
              orientation="left"
              width={96}
              axisLine={{ stroke: METRIC_CONFIG.hoodAngle.color }}
              tickLine={{ stroke: METRIC_CONFIG.hoodAngle.color }}
              tick={{ fill: METRIC_CONFIG.hoodAngle.color }}
              tickFormatter={(value: number) => value.toFixed(1)}
              label={{
                value: 'Hood Angle (deg)',
                angle: -90,
                position: 'insideLeft',
                dx: 30,
                fill: METRIC_CONFIG.hoodAngle.color,
              }}
            />
            <YAxis
              yAxisId="flightTime"
              orientation="right"
              width={76}
              axisLine={{ stroke: METRIC_CONFIG.flightTime.color }}
              tickLine={{ stroke: METRIC_CONFIG.flightTime.color }}
              tick={{ fill: METRIC_CONFIG.flightTime.color }}
              tickFormatter={(value: number) => value.toFixed(2)}
              label={{
                value: 'Flight Time (s)',
                angle: 90,
                position: 'insideRight',
                fill: METRIC_CONFIG.flightTime.color,
                dx: -15
              }}
            />
            <Tooltip
              labelFormatter={(value) => `Distance: ${formatDistance(Number(value))} m`}
            />
            <Legend wrapperStyle={{ paddingTop: 8, transform: 'translateY(4px)' }} />

            <Line
              yAxisId="rpm"
              type="linear"
              dataKey={METRIC_CONFIG.shooterRPM.actualKey}
              name="Shooter RPM"
              stroke={METRIC_CONFIG.shooterRPM.color}
              strokeWidth={3}
              dot={{ r: 4 }}
              connectNulls
              isAnimationActive={false}
            />
            {fitVisibility.quadratic && (
              <Line
                yAxisId="rpm"
                type="linear"
                dataKey={METRIC_CONFIG.shooterRPM.quadraticKey}
                name="Shooter RPM Quadratic Fit"
                stroke={METRIC_CONFIG.shooterRPM.color}
                strokeDasharray="6 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
              />
            )}
            {fitVisibility.cubic && (
              <Line
                yAxisId="rpm"
                type="linear"
                dataKey={METRIC_CONFIG.shooterRPM.cubicKey}
                name="Shooter RPM Cubic Fit"
                stroke={METRIC_CONFIG.shooterRPM.color}
                strokeDasharray="2 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.8}
              />
            )}
            {fitVisibility.spline && (
              <Line
                yAxisId="rpm"
                type="linear"
                dataKey={METRIC_CONFIG.shooterRPM.splineKey}
                name="Shooter RPM Spline"
                stroke={METRIC_CONFIG.shooterRPM.color}
                strokeDasharray="12 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.7}
              />
            )}

            <Line
              yAxisId="hood"
              type="linear"
              dataKey={METRIC_CONFIG.hoodAngle.actualKey}
              name="Hood Angle"
              stroke={METRIC_CONFIG.hoodAngle.color}
              strokeWidth={3}
              dot={{ r: 4 }}
              connectNulls
              isAnimationActive={false}
            />
            {fitVisibility.quadratic && (
              <Line
                yAxisId="hood"
                type="linear"
                dataKey={METRIC_CONFIG.hoodAngle.quadraticKey}
                name="Hood Angle Quadratic Fit"
                stroke={METRIC_CONFIG.hoodAngle.color}
                strokeDasharray="6 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
              />
            )}
            {fitVisibility.cubic && (
              <Line
                yAxisId="hood"
                type="linear"
                dataKey={METRIC_CONFIG.hoodAngle.cubicKey}
                name="Hood Angle Cubic Fit"
                stroke={METRIC_CONFIG.hoodAngle.color}
                strokeDasharray="2 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.8}
              />
            )}
            {fitVisibility.spline && (
              <Line
                yAxisId="hood"
                type="linear"
                dataKey={METRIC_CONFIG.hoodAngle.splineKey}
                name="Hood Angle Spline"
                stroke={METRIC_CONFIG.hoodAngle.color}
                strokeDasharray="12 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.7}
              />
            )}

            <Line
              yAxisId="flightTime"
              type="linear"
              dataKey={METRIC_CONFIG.flightTime.actualKey}
              name="Flight Time"
              stroke={METRIC_CONFIG.flightTime.color}
              strokeWidth={3}
              dot={{ r: 4 }}
              connectNulls
              isAnimationActive={false}
            />
            {fitVisibility.quadratic && (
              <Line
                yAxisId="flightTime"
                type="linear"
                dataKey={METRIC_CONFIG.flightTime.quadraticKey}
                name="Flight Time Quadratic Fit"
                stroke={METRIC_CONFIG.flightTime.color}
                strokeDasharray="6 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
              />
            )}
            {fitVisibility.cubic && (
              <Line
                yAxisId="flightTime"
                type="linear"
                dataKey={METRIC_CONFIG.flightTime.cubicKey}
                name="Flight Time Cubic Fit"
                stroke={METRIC_CONFIG.flightTime.color}
                strokeDasharray="2 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.8}
              />
            )}
            {fitVisibility.spline && (
              <Line
                yAxisId="flightTime"
                type="linear"
                dataKey={METRIC_CONFIG.flightTime.splineKey}
                name="Flight Time Spline"
                stroke={METRIC_CONFIG.flightTime.color}
                strokeDasharray="12 4"
                dot={false}
                connectNulls
                isAnimationActive={false}
                strokeOpacity={0.7}
              />
            )}
          </LineChart>
        </ResponsiveContainer>
      </Box>
      <Stack direction={{ xs: 'column', sm: 'row' }} spacing={1} sx={{ mt: 2 }}>
        <FormControlLabel
          control={(
            <Checkbox
              checked={fitVisibility.quadratic}
              onChange={(event) =>
                setFitVisibilityByChart((current) => ({
                  ...current,
                  [chartId]: {
                    ...fitVisibility,
                    quadratic: event.target.checked,
                  },
                }))
              }
            />
          )}
          label="Show Quadratic Fit"
        />
        <FormControlLabel
          control={(
            <Checkbox
              checked={fitVisibility.cubic}
              onChange={(event) =>
                setFitVisibilityByChart((current) => ({
                  ...current,
                  [chartId]: {
                    ...fitVisibility,
                    cubic: event.target.checked,
                  },
                }))
              }
            />
          )}
          label="Show Cubic Fit"
        />
        <FormControlLabel
          control={(
            <Checkbox
              checked={fitVisibility.spline}
              onChange={(event) =>
                setFitVisibilityByChart((current) => ({
                  ...current,
                  [chartId]: {
                    ...fitVisibility,
                    spline: event.target.checked,
                  },
                }))
              }
            />
          )}
          label="Show Spline Fit"
        />
      </Stack>
    </Paper>
  );
}
