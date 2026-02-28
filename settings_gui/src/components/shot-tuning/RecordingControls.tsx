import { useState, useRef, useEffect, useCallback } from 'react';
import { Box, Button, Typography, Alert, Select, MenuItem, InputLabel, FormControl, ToggleButtonGroup, ToggleButton, Tooltip } from '@mui/material';
import FiberManualRecordIcon from '@mui/icons-material/FiberManualRecord';
import StopIcon from '@mui/icons-material/Stop';
import DeleteIcon from '@mui/icons-material/Delete';
import SaveIcon from '@mui/icons-material/Save';

const FPS_OPTIONS = [30, 60, 120] as const;

interface RecordingControlsProps {
  onStore: (blob: Blob) => Promise<void>;
  onFpsChange?: (fps: number) => void;
}

function pickCodec(): string {
  for (const codec of ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm']) {
    if (MediaRecorder.isTypeSupported(codec)) return codec;
  }
  return 'video/webm';
}

export function RecordingControls({ onStore, onFpsChange }: RecordingControlsProps) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const streamRef = useRef<MediaStream | null>(null);
  const chunksRef = useRef<Blob[]>([]);

  const [cameras, setCameras] = useState<MediaDeviceInfo[]>([]);
  const [selectedCameraId, setSelectedCameraId] = useState<string>('');
  const [selectedFps, setSelectedFps] = useState<number>(30);
  const [actualFps, setActualFps] = useState<number | null>(null);
  const [cameraReady, setCameraReady] = useState(false);
  const [cameraError, setCameraError] = useState<string | null>(null);
  const [recording, setRecording] = useState(false);
  const [pendingBlob, setPendingBlob] = useState<Blob | null>(null);
  const [saving, setSaving] = useState(false);

  // Enumerate cameras after initial permission grant
  const enumerateCameras = useCallback(async () => {
    const devices = await navigator.mediaDevices.enumerateDevices();
    const videoDevices = devices.filter((d) => d.kind === 'videoinput');
    setCameras(videoDevices);
    return videoDevices;
  }, []);

  // Open a specific camera by deviceId (or default if empty)
  const openCamera = useCallback(async (deviceId?: string, fps?: number) => {
    // Stop existing stream
    streamRef.current?.getTracks().forEach((t) => t.stop());
    setCameraReady(false);
    setCameraError(null);
    setActualFps(null);

    const targetFps = fps ?? selectedFps;
    try {
      const videoConstraints: MediaTrackConstraints = {
        frameRate: { ideal: targetFps },
      };
      if (deviceId) videoConstraints.deviceId = { exact: deviceId };
      const stream = await navigator.mediaDevices.getUserMedia({ video: videoConstraints });
      streamRef.current = stream;
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
      setCameraReady(true);

      // Report actual frame rate
      const activeTrack = stream.getVideoTracks()[0];
      const settings = activeTrack?.getSettings();
      const deliveredFps = settings?.frameRate ?? null;
      setActualFps(deliveredFps);
      if (deliveredFps) onFpsChange?.(deliveredFps);

      // After permission is granted, enumerate to get labels
      const videoDevices = await enumerateCameras();
      // If no deviceId was specified, figure out which one we got
      if (!deviceId && videoDevices.length > 0) {
        const activeDeviceId = settings?.deviceId;
        if (activeDeviceId) setSelectedCameraId(activeDeviceId);
      }
    } catch (err) {
      setCameraError(err instanceof Error ? err.message : String(err));
    }
  }, [enumerateCameras, selectedFps, onFpsChange]);

  // Acquire default camera on mount
  useEffect(() => {
    openCamera();
    return () => {
      streamRef.current?.getTracks().forEach((t) => t.stop());
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const handleCameraChange = useCallback((deviceId: string) => {
    setSelectedCameraId(deviceId);
    openCamera(deviceId);
  }, [openCamera]);

  const handleFpsChange = useCallback((fps: number) => {
    setSelectedFps(fps);
    openCamera(selectedCameraId || undefined, fps);
  }, [openCamera, selectedCameraId]);

  const handleStart = useCallback(() => {
    if (!streamRef.current) return;
    chunksRef.current = [];
    const mimeType = pickCodec();
    const recorder = new MediaRecorder(streamRef.current, { mimeType });
    recorder.ondataavailable = (e) => {
      if (e.data.size > 0) chunksRef.current.push(e.data);
    };
    recorder.onstop = () => {
      const blob = new Blob(chunksRef.current, { type: mimeType });
      setPendingBlob(blob);
    };
    recorder.start();
    recorderRef.current = recorder;
    setRecording(true);
    setPendingBlob(null);
  }, []);

  const handleStop = useCallback(() => {
    recorderRef.current?.stop();
    recorderRef.current = null;
    setRecording(false);
  }, []);

  const handleDiscard = useCallback(() => {
    setPendingBlob(null);
  }, []);

  const handleStore = useCallback(async () => {
    if (!pendingBlob) return;
    setSaving(true);
    try {
      await onStore(pendingBlob);
      setPendingBlob(null);
    } finally {
      setSaving(false);
    }
  }, [pendingBlob, onStore]);

  return (
    <Box>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Recording
      </Typography>

      {cameras.length > 1 && (
        <FormControl size="small" fullWidth sx={{ mb: 1 }}>
          <InputLabel>Camera</InputLabel>
          <Select
            value={selectedCameraId}
            label="Camera"
            onChange={(e) => handleCameraChange(e.target.value)}
            disabled={recording}
          >
            {cameras.map((cam) => (
              <MenuItem key={cam.deviceId} value={cam.deviceId}>
                {cam.label || `Camera ${cameras.indexOf(cam) + 1}`}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
      )}

      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
        <Typography variant="body2" color="text.secondary">FPS:</Typography>
        <ToggleButtonGroup
          value={selectedFps}
          exclusive
          size="small"
          onChange={(_, v) => { if (v !== null) handleFpsChange(v); }}
          disabled={recording}
        >
          {FPS_OPTIONS.map((fps) => (
            <ToggleButton key={fps} value={fps}>{fps}</ToggleButton>
          ))}
        </ToggleButtonGroup>
        {actualFps !== null && actualFps !== selectedFps && (
          <Tooltip title="Camera may not support the requested frame rate" arrow>
            <Typography variant="caption" color="warning.main">
              actual: {Math.round(actualFps)} fps
            </Typography>
          </Tooltip>
        )}
        {actualFps !== null && actualFps === selectedFps && (
          <Typography variant="caption" color="text.secondary">
            {Math.round(actualFps)} fps
          </Typography>
        )}
      </Box>

      {cameraError && (
        <Alert severity="error" sx={{ mb: 1 }}>{cameraError}</Alert>
      )}

      <Box sx={{ mb: 1, position: 'relative' }}>
        <video
          ref={videoRef}
          autoPlay
          muted
          playsInline
          style={{ width: '100%', borderRadius: 4, background: '#000' }}
        />
        {recording && (
          <Box sx={{
            position: 'absolute', top: 8, right: 8,
            bgcolor: 'error.main', color: 'white',
            px: 1, py: 0.25, borderRadius: 1,
            fontSize: '0.75rem', fontWeight: 'bold',
          }}>
            REC
          </Box>
        )}
      </Box>

      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
        {!recording && !pendingBlob && (
          <Button
            variant="contained"
            color="error"
            startIcon={<FiberManualRecordIcon />}
            onClick={handleStart}
            disabled={!cameraReady}
          >
            Start Recording
          </Button>
        )}

        {recording && (
          <Button
            variant="contained"
            startIcon={<StopIcon />}
            onClick={handleStop}
          >
            Stop
          </Button>
        )}

        {pendingBlob && (
          <>
            <Button
              variant="outlined"
              color="error"
              startIcon={<DeleteIcon />}
              onClick={handleDiscard}
            >
              Discard
            </Button>
            <Button
              variant="contained"
              color="success"
              startIcon={<SaveIcon />}
              onClick={handleStore}
              disabled={saving}
            >
              {saving ? 'Saving...' : 'Store Attempt'}
            </Button>
          </>
        )}
      </Box>
    </Box>
  );
}
