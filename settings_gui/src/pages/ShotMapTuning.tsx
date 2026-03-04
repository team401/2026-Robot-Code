import { useState, useEffect, useCallback, useRef } from 'react';
import {
  Box,
  Typography,
  Alert,
  Snackbar,
  Paper,
  Grid,
} from '@mui/material';
import type { TuningAttempt } from '../types/ShotTuning';
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

export function ShotMapTuning() {
  const { environment } = useConnection();
  const [attempts, setAttempts] = useState<TuningAttempt[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [snack, setSnack] = useState<{ message: string; severity: 'success' | 'error' } | null>(null);
  const [error, setError] = useState<string | null>(null);
  const shooterOffsetRef = useRef<{ x: number; y: number }>({ x: 0, y: 0 });
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
        shooterOffsetRef.current = { x, y };
      })
      .catch(() => { /* keep default {0,0} if unavailable */ });
  }, [environment]);

  const handleConnect = useCallback(async (address: string) => {
    if (nt4Ref.current) {
      nt4Ref.current.disconnect();
    }
    const svc = await createNT4Service(address, setTelemetry, shooterOffsetRef.current);
    nt4Ref.current = svc;
    return svc;
  }, []);

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

  const handleRecordStart = useCallback(() => {
    startSnapshotRef.current = telemetryRef.current;
  }, []);

  const handleStore = useCallback(async (blob: Blob) => {
    const id = crypto.randomUUID();
    const snap = startSnapshotRef.current;
    const attempt: TuningAttempt = {
      id,
      createdAt: new Date().toISOString(),
      distanceMeters: snap.distanceMeters,
      distanceToHubMeters: snap.distanceToHubMeters,
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
  }, [attempts]);

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
            <TelemetryDisplay telemetry={telemetry} />
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
