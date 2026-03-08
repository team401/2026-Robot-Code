import { useState, useEffect, useCallback, useRef } from 'react';
import {
  Box,
  Typography,
  Alert,
  Snackbar,
  Paper,
  Grid,
  Stack,
  TextField,
  ToggleButton,
  ToggleButtonGroup,
} from '@mui/material';
import CheckIcon from '@mui/icons-material/Check';
import type { DistanceSource, TargetCoordinates, TuningAttempt } from '../types/ShotTuning';
import { DEFAULT_DISTANCE_SOURCE, DEFAULT_TARGET_COORDINATES } from '../types/ShotTuning';
import { loadAttempts, saveAttempts, uploadClip, deleteClip } from '../services/shotTuningStorage';
import { loadLocal } from '../services/api';
import { NTConnectionStatus } from '../components/shot-tuning/NTConnectionStatus';
import { TelemetryDisplay } from '../components/shot-tuning/TelemetryDisplay';
import { RecordingControls } from '../components/shot-tuning/RecordingControls';
import { AttemptsList } from '../components/shot-tuning/AttemptsList';
import { VideoReplayPlayer } from '../components/shot-tuning/VideoReplayPlayer';
import type { NT4Service, Telemetry } from '../services/nt4';
import { createNT4Service } from '../services/nt4';
import { useConnection } from '../contexts/ConnectionContext';

interface RobotInfo {
  robotToShooter: { translation: { x: number; y: number; z: number } };
}

interface DistanceSettings {
  distanceSource: DistanceSource;
  targetCoordinates: TargetCoordinates;
}

const DISTANCE_SETTINGS_STORAGE_KEY = 'shot-map-tuning.distance-settings';

function loadDistanceSettings(): DistanceSettings {
  try {
    const raw = localStorage.getItem(DISTANCE_SETTINGS_STORAGE_KEY);
    if (!raw) {
      return {
        distanceSource: DEFAULT_DISTANCE_SOURCE,
        targetCoordinates: DEFAULT_TARGET_COORDINATES,
      };
    }
    const parsed = JSON.parse(raw) as Partial<DistanceSettings>;
    const source = parsed.distanceSource === 'odometry' ? 'odometry' : DEFAULT_DISTANCE_SOURCE;
    const x = typeof parsed.targetCoordinates?.x === 'number' ? parsed.targetCoordinates.x : DEFAULT_TARGET_COORDINATES.x;
    const y = typeof parsed.targetCoordinates?.y === 'number' ? parsed.targetCoordinates.y : DEFAULT_TARGET_COORDINATES.y;
    return {
      distanceSource: source,
      targetCoordinates: { x, y },
    };
  } catch {
    return {
      distanceSource: DEFAULT_DISTANCE_SOURCE,
      targetCoordinates: DEFAULT_TARGET_COORDINATES,
    };
  }
}

export function ShotMapTuning() {
  const { environment } = useConnection();
  const [attempts, setAttempts] = useState<TuningAttempt[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [snack, setSnack] = useState<{ message: string; severity: 'success' | 'error' } | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [distanceSettings, setDistanceSettings] = useState<DistanceSettings>(loadDistanceSettings);
  const [shooterOffset, setShooterOffset] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [telemetry, setTelemetry] = useState<Telemetry>({
    distanceMeters: 0,
    distanceToHubMeters: null,
    shooterRPM: 0,
    hoodAngleDegrees: 0,
    robotPoseX: 0,
    robotPoseY: 0,
  });
  const [recordingFps, setRecordingFps] = useState(30);
  const nt4Ref = useRef<NT4Service | null>(null);
  const telemetryRef = useRef(telemetry);
  telemetryRef.current = telemetry;
  // Snapshot taken at the moment the user presses "Start Recording"
  const startSnapshotRef = useRef<Telemetry>(telemetry);
  const startDistanceSettingsRef = useRef<DistanceSettings>(distanceSettings);

  useEffect(() => {
    localStorage.setItem(DISTANCE_SETTINGS_STORAGE_KEY, JSON.stringify(distanceSettings));
  }, [distanceSettings]);

  const fetchAttempts = useCallback(async () => {
    try {
      const data = await loadAttempts();
      setAttempts(data);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to load attempts');
    }
  }, []);

  useEffect(() => { fetchAttempts(); }, [fetchAttempts]);

  // Load robot geometry once on startup so distance is measured from the shooter, not the robot center.
  useEffect(() => {
    loadLocal<RobotInfo>(environment, 'RobotInfo.json')
      .then((info) => {
        const { x, y } = info.robotToShooter.translation;
        setShooterOffset({ x, y });
      })
      .catch(() => { /* keep default {0,0} if unavailable */ });
  }, [environment]);

  const handleConnect = useCallback(async (address: string) => {
    if (nt4Ref.current) {
      nt4Ref.current.disconnect();
    }
    const svc = await createNT4Service(address, setTelemetry, {
      shooterOffset,
      targetCoordinates: distanceSettings.targetCoordinates,
    });
    nt4Ref.current = svc;
    return svc;
  }, [distanceSettings.targetCoordinates, shooterOffset]);

  const handleDisconnect = useCallback(() => {
    if (nt4Ref.current) {
      nt4Ref.current.disconnect();
      nt4Ref.current = null;
    }
  }, []);

  useEffect(() => {
    return () => {
      if (nt4Ref.current) nt4Ref.current.disconnect();
    };
  }, []);

  useEffect(() => {
    nt4Ref.current?.updateDistanceConfig({
      shooterOffset,
      targetCoordinates: distanceSettings.targetCoordinates,
    });
  }, [distanceSettings.targetCoordinates, shooterOffset]);

  const handleRecordStart = useCallback(() => {
    startSnapshotRef.current = telemetryRef.current;
    startDistanceSettingsRef.current = distanceSettings;
  }, [distanceSettings]);

  const handleStore = useCallback(async (blob: Blob) => {
    const id = crypto.randomUUID();
    const snap = startSnapshotRef.current;
    const settingsAtStart = startDistanceSettingsRef.current;
    const sampleDistanceMeters = settingsAtStart.distanceSource === 'networkTables'
      ? snap.distanceToHubMeters ?? snap.distanceMeters
      : snap.distanceMeters;
    const attempt: TuningAttempt = {
      id,
      createdAt: new Date().toISOString(),
      distanceMeters: snap.distanceMeters,
      distanceToHubMeters: snap.distanceToHubMeters,
      sampleDistanceMeters,
      distanceSource: settingsAtStart.distanceSource,
      targetCoordinates: settingsAtStart.targetCoordinates,
      shooterRPM: snap.shooterRPM,
      hoodAngleDegrees: snap.hoodAngleDegrees,
      robotPoseX: snap.robotPoseX,
      robotPoseY: snap.robotPoseY,
      leavesShooterTimeSec: null,
      hitTargetTimeSec: null,
      flightTimeSec: null,
      exportedToShotMap: null,
      fps: recordingFps,
    };
    try {
      await uploadClip(id, blob);
      const next = [...attempts, attempt];
      await saveAttempts(next);
      setAttempts(next);
      setSelectedId(id);
      setSnack({ message: 'Attempt stored', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to store attempt');
    }
  }, [attempts, recordingFps]);

  const handleDelete = useCallback(async (id: string) => {
    try {
      await deleteClip(id);
      const next = attempts.filter((a) => a.id !== id);
      await saveAttempts(next);
      setAttempts(next);
      if (selectedId === id) setSelectedId(null);
      setSnack({ message: 'Attempt deleted', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to delete attempt');
    }
  }, [attempts, selectedId]);

  const handleUpdateAttempt = useCallback(async (updated: TuningAttempt) => {
    const next = attempts.map((a) => a.id === updated.id ? updated : a);
    try {
      await saveAttempts(next);
      setAttempts(next);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to update attempt');
    }
  }, [attempts]);

  const selectedAttempt = attempts.find((a) => a.id === selectedId) ?? null;

  return (
    <Box>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 2 }}>
        <Typography variant="h5" sx={{ whiteSpace: 'nowrap' }}>Shot Map Tuning</Typography>
        <Paper sx={{ p: 1.5, flex: 1 }}>
          <NTConnectionStatus onConnect={handleConnect} onDisconnect={handleDisconnect} />
        </Paper>
      </Box>

      {error && <Alert severity="error" sx={{ mb: 2 }} onClose={() => setError(null)}>{error}</Alert>}

      <Grid container spacing={2}>
        {/* Left column: Recording */}
        <Grid size={{ xs: 12, md: 6 }}>
          <Paper sx={{ p: 2, mb: 2 }}>
            <Stack spacing={2}>
              <Box>
                <ToggleButtonGroup
                  value={distanceSettings.distanceSource}
                  exclusive
                  onChange={(_, value: DistanceSource | null) => {
                    if (!value) return;
                    setDistanceSettings((current) => ({
                      ...current,
                      distanceSource: value,
                    }));
                  }}
                  size="small"
                >
                  <ToggleButton value="networkTables">
                    {distanceSettings.distanceSource === 'networkTables' && <CheckIcon fontSize="small" sx={{ mr: 0.5 }} />}
                    Read dist from NT
                  </ToggleButton>
                  <ToggleButton value="odometry">
                    {distanceSettings.distanceSource === 'odometry' && <CheckIcon fontSize="small" sx={{ mr: 0.5 }} />}
                    Compute distance via Odometry
                  </ToggleButton>
                </ToggleButtonGroup>
              </Box>

              {distanceSettings.distanceSource === 'odometry' && (
                <Box>
                  <Stack direction={{ xs: 'column', sm: 'row' }} spacing={2} alignItems={{ xs: 'stretch', sm: 'center' }}>
                    <TextField
                      label="Target X (m)"
                      type="number"
                      value={distanceSettings.targetCoordinates.x}
                      onChange={(event) => {
                        const value = Number.parseFloat(event.target.value);
                        setDistanceSettings((current) => ({
                          ...current,
                          targetCoordinates: {
                            ...current.targetCoordinates,
                            x: Number.isFinite(value) ? value : 0,
                          },
                        }));
                      }}
                      inputProps={{ step: '0.1' }}
                      size="small"
                      fullWidth
                    />
                    <TextField
                      label="Target Y (m)"
                      type="number"
                      value={distanceSettings.targetCoordinates.y}
                      onChange={(event) => {
                        const value = Number.parseFloat(event.target.value);
                        setDistanceSettings((current) => ({
                          ...current,
                          targetCoordinates: {
                            ...current.targetCoordinates,
                            y: Number.isFinite(value) ? value : 0,
                          },
                        }));
                      }}
                      inputProps={{ step: '0.1' }}
                      size="small"
                      fullWidth
                    />
                    <Box sx={{ display: 'flex', justifyContent: { xs: 'flex-start', sm: 'center' } }}>
                      <Typography
                        component="button"
                        type="button"
                        onClick={() => {
                          setDistanceSettings((current) => ({
                            ...current,
                            targetCoordinates: { ...DEFAULT_TARGET_COORDINATES },
                          }));
                        }}
                        sx={{
                          border: 'none',
                          background: 'none',
                          p: 0,
                          m: 0,
                          color: 'primary.main',
                          cursor: 'pointer',
                          font: 'inherit',
                          fontSize: '0.75rem',
                          textDecoration: 'underline',
                          whiteSpace: 'nowrap',
                        }}
                      >
                        reset to
                        <br />
                        hub coords
                      </Typography>
                    </Box>
                  </Stack>
                </Box>
              )}

              <TelemetryDisplay
                telemetry={telemetry}
                distanceSource={distanceSettings.distanceSource}
                targetCoordinates={distanceSettings.targetCoordinates}
              />
            </Stack>
          </Paper>

          <Paper sx={{ p: 2, mb: 2 }}>
            <RecordingControls onStore={handleStore} onFpsChange={setRecordingFps} onRecordStart={handleRecordStart} />
          </Paper>
        </Grid>

        {/* Right column: Replay + Attempts */}
        <Grid size={{ xs: 12, md: 6 }}>
          {selectedAttempt && (
            <Paper sx={{ p: 2, mb: 2 }}>
              <VideoReplayPlayer
                attempt={selectedAttempt}
                onUpdate={handleUpdateAttempt}
                attempts={attempts}
                onAttemptsChange={async (next) => {
                  await saveAttempts(next);
                  setAttempts(next);
                }}
                fps={selectedAttempt.fps ?? 30}
              />
            </Paper>
          )}

          <Paper sx={{ p: 2 }}>
            <AttemptsList
              attempts={attempts}
              selectedId={selectedId}
              onSelect={setSelectedId}
              onDelete={handleDelete}
            />
          </Paper>
        </Grid>
      </Grid>

      <Snackbar open={snack !== null} autoHideDuration={3000} onClose={() => setSnack(null)}>
        <Alert severity={snack?.severity ?? 'success'} variant="filled" onClose={() => setSnack(null)}>
          {snack?.message}
        </Alert>
      </Snackbar>
    </Box>
  );
}
