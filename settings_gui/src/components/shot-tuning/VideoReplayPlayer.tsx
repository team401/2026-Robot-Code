import { useRef, useState, useEffect, useCallback } from 'react';
import {
  Alert,
  Box,
  Button,
  Typography,
  IconButton,
  Divider,
  Slider,
  Tooltip,
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import type { TuningAttempt } from '../../types/ShotTuning';
import type { ShotMaps, ShotMapDataPoint } from '../../types/ShotMaps';
import { clipUrl } from '../../services/shotTuningStorage';
import { loadLocal, saveLocal } from '../../services/api';
import { useConnection } from '../../contexts/ConnectionContext';
import { shotMapsLocalSignal } from '../../services/shotMapsStore';

interface VideoReplayPlayerProps {
  attempt: TuningAttempt;
  onUpdate: (attempt: TuningAttempt) => Promise<void>;
  attempts: TuningAttempt[];
  onAttemptsChange: (attempts: TuningAttempt[]) => Promise<void>;
  fps?: number;
}


export function VideoReplayPlayer({ attempt, onUpdate, attempts, onAttemptsChange, fps = 30 }: VideoReplayPlayerProps) {
  const { environment } = useConnection();
  const FRAME_STEP = 1 / fps;
  const videoRef = useRef<HTMLVideoElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [playing, setPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const [exportError, setExportError] = useState<string | null>(null);


  // Webm files often report wrong duration until fully buffered.
  // Seek to a huge time to force the browser to find the real end,
  // then read the corrected duration and seek back to 0.
  const fixWebmDuration = useCallback(() => {
    const video = videoRef.current;
    if (!video) return;
    if (video.duration && isFinite(video.duration) && video.duration > 0.5) {
      setDuration(video.duration);
      return;
    }
    const onSeeked = () => {
      video.removeEventListener('seeked', onSeeked);
      if (isFinite(video.duration)) {
        setDuration(video.duration);
      }
      video.currentTime = 0;
    };
    video.addEventListener('seeked', onSeeked);
    video.currentTime = 1e10;
  }, []);

  useEffect(() => {
    setPlaying(false);
    setCurrentTime(0);
    setDuration(0);
  }, [attempt.id]);

  const handleTimeUpdate = useCallback(() => {
    if (videoRef.current) setCurrentTime(videoRef.current.currentTime);
  }, []);

  const handleLoadedMetadata = useCallback(() => {
    fixWebmDuration();
  }, [fixWebmDuration]);

  const handleDurationChange = useCallback(() => {
    const video = videoRef.current;
    if (video && isFinite(video.duration) && video.duration > 0) {
      setDuration(video.duration);
    }
  }, []);

  const togglePlay = useCallback(() => {
    if (!videoRef.current) return;
    if (playing) {
      videoRef.current.pause();
    } else {
      videoRef.current.play();
    }
    setPlaying(!playing);
  }, [playing]);

  const stepFrame = useCallback((delta: number) => {
    const video = videoRef.current;
    if (!video) return;
    video.pause();
    setPlaying(false);
    const target = Math.max(0, Math.min(duration || Infinity, video.currentTime + delta));
    video.currentTime = target;
  }, [duration]);

  const seekTo = useCallback((time: number) => {
    const video = videoRef.current;
    if (!video) return;
    video.pause();
    setPlaying(false);
    video.currentTime = time;
  }, []);

  // Focus the container on mount and whenever the attempt changes so keyboard
  // shortcuts work immediately without the user having to click first.
  useEffect(() => {
    containerRef.current?.focus();
  }, [attempt.id]);

  const markLeavesShooter = useCallback(async () => {
    const updated = { ...attempt, leavesShooterTimeSec: currentTime };
    if (updated.hitTargetTimeSec !== null) {
      updated.flightTimeSec = updated.hitTargetTimeSec - currentTime;
    }
    await onUpdate(updated);
  }, [attempt, currentTime, onUpdate]);

  const markHitTarget = useCallback(async () => {
    const updated = { ...attempt, hitTargetTimeSec: currentTime };
    if (updated.leavesShooterTimeSec !== null) {
      updated.flightTimeSec = currentTime - updated.leavesShooterTimeSec;
    }
    await onUpdate(updated);
  }, [attempt, currentTime, onUpdate]);

  const handleKeyDown = useCallback((e: React.KeyboardEvent) => {
    const target = e.target as HTMLElement;
    const tag = target.tagName;
    // Let form controls handle their own keys entirely.
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;

    const isSlider = target.getAttribute('role') === 'slider';

    switch (e.key) {
      case 'ArrowLeft':
      case 'j':
        // Skip arrow keys when slider is focused — it handles them for fine scrubbing.
        if (isSlider && e.key === 'ArrowLeft') return;
        e.preventDefault();
        stepFrame(-FRAME_STEP);
        break;
      case 'ArrowRight':
      case 'k':
        if (isSlider && e.key === 'ArrowRight') return;
        e.preventDefault();
        stepFrame(FRAME_STEP);
        break;
      case 'h':
        e.preventDefault();
        stepFrame(-FRAME_STEP * 10);
        break;
      case 'l':
        e.preventDefault();
        stepFrame(FRAME_STEP * 10);
        break;
      case 's':
        e.preventDefault();
        markLeavesShooter();
        break;
      case 'e':
        e.preventDefault();
        markHitTarget();
        break;
      case ' ':
        // Don't intercept Space when a button has focus (it would double-fire).
        if (tag !== 'BUTTON') {
          e.preventDefault();
          togglePlay();
        }
        break;
    }
  }, [stepFrame, togglePlay, markLeavesShooter, markHitTarget, FRAME_STEP]);

  const handleExport = async (target: 'hub' | 'pass') => {
    if (attempt.flightTimeSec === null) return;
    setExportError(null);
    const distance = attempt.distanceToHubMeters ?? attempt.distanceMeters;
    const newPoint: ShotMapDataPoint = {
      distance: { value: distance, unit: 'Meter' },
      shooterRPM: attempt.shooterRPM,
      hoodAngle: { value: attempt.hoodAngleDegrees, unit: 'Degree' },
      flightTime: { value: attempt.flightTimeSec, unit: 'Second' },
    };
    try {
      // Always load fresh so we never overwrite concurrent edits.
      const current = await loadLocal<ShotMaps>(environment, 'ShotMaps.json');
      if (target === 'hub') current.hubDataPoints.push(newPoint);
      else current.passDataPoints.push(newPoint);
      await saveLocal(environment, 'ShotMaps.json', current);
      // Signal the Shot Maps tab to reload from local on its next mount.
      shotMapsLocalSignal.markDirty();
    } catch (e) {
      setExportError(e instanceof Error ? e.message : 'Failed to save to local file');
      return;
    }

    const updated = { ...attempt, exportedToShotMap: target };
    const next = attempts.map((a) => a.id === updated.id ? updated : a);
    await onAttemptsChange(next);
  };

  return (
    <Box ref={containerRef} tabIndex={-1} sx={{ outline: 'none' }} onKeyDownCapture={handleKeyDown}>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Video Replay
      </Typography>

      <video
        ref={videoRef}
        src={clipUrl(attempt.id)}
        onTimeUpdate={handleTimeUpdate}
        onLoadedMetadata={handleLoadedMetadata}
        onDurationChange={handleDurationChange}
        onEnded={() => setPlaying(false)}
        // Clicking the video would give native focus to the <video> element, which
        // intercepts keyboard events in some browsers. Redirect focus to our container.
        onClick={() => containerRef.current?.focus()}
        style={{ width: '100%', borderRadius: 4, background: '#000', cursor: 'pointer' }}
      />

      {/* Scrub slider + flight-window indicator */}
      {duration > 0 && (
        <>
          <Slider
            value={currentTime}
            min={0}
            max={duration}
            step={0.001}
            onChange={(_, v) => seekTo(v as number)}
            onChangeCommitted={() => containerRef.current?.focus()}
            size="small"
            sx={{ mt: 0.5, mb: 0 }}
          />
          {/* Only show the indicator once at least one mark is set */}
          {(attempt.leavesShooterTimeSec !== null || attempt.hitTargetTimeSec !== null) && (
            <Box sx={{ position: 'relative', height: 8, borderRadius: 1, overflow: 'hidden', mb: 0.5, bgcolor: 'divider' }}>
              {/* Amber highlight for the flight window */}
              {attempt.leavesShooterTimeSec !== null && attempt.hitTargetTimeSec !== null && (
                <Box sx={{
                  position: 'absolute', top: 0, bottom: 0,
                  left: `${(attempt.leavesShooterTimeSec / duration) * 100}%`,
                  width: `${((attempt.hitTargetTimeSec - attempt.leavesShooterTimeSec) / duration) * 100}%`,
                  bgcolor: 'warning.main', opacity: 0.6,
                }} />
              )}
              {/* Green tick: leaves shooter */}
              {attempt.leavesShooterTimeSec !== null && (
                <Box sx={{
                  position: 'absolute', top: 0, bottom: 0, width: 2,
                  left: `${(attempt.leavesShooterTimeSec / duration) * 100}%`,
                  transform: 'translateX(-1px)', bgcolor: 'success.main',
                }} />
              )}
              {/* Red tick: hits target */}
              {attempt.hitTargetTimeSec !== null && (
                <Box sx={{
                  position: 'absolute', top: 0, bottom: 0, width: 2,
                  left: `${(attempt.hitTargetTimeSec / duration) * 100}%`,
                  transform: 'translateX(-1px)', bgcolor: 'error.main',
                }} />
              )}
            </Box>
          )}
        </>
      )}

      {/* Playback controls */}
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5, mt: 0.5, flexWrap: 'wrap' }}>
        <Tooltip title="Back 10 frames (h)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(-FRAME_STEP * 10)}>
            -10 Frames
          </Button>
        </Tooltip>
        <Tooltip title="Back 1 frame (j / Left Arrow)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(-FRAME_STEP)}>
            -1 Frame
          </Button>
        </Tooltip>
        <Tooltip title={playing ? 'Pause (Space)' : 'Play (Space)'} arrow>
          <IconButton size="small" onClick={togglePlay}>
            {playing ? <PauseIcon /> : <PlayArrowIcon />}
          </IconButton>
        </Tooltip>
        <Tooltip title="Forward 1 frame (k / Right Arrow)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(FRAME_STEP)}>
            +1 Frame
          </Button>
        </Tooltip>
        <Tooltip title="Forward 10 frames (l)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(FRAME_STEP * 10)}>
            +10 Frames
          </Button>
        </Tooltip>
        <Typography variant="body2" fontFamily="monospace" sx={{ ml: 1 }}>
          {currentTime.toFixed(3)}s / {duration.toFixed(3)}s
        </Typography>
      </Box>

      <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
        h: -10 frames | j / Left: -1 frame | Space: play/pause | k / Right: +1 frame | l: +10 frames | s: mark start | e: mark end
      </Typography>

      <Divider sx={{ my: 1 }} />

      {/* Timestamp marking */}
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 1 }}>
        <Tooltip title="Set the current frame as the moment the ball leaves the shooter" arrow>
          <Button size="small" variant="outlined" onClick={markLeavesShooter}>
            Mark Leaves Shooter ({attempt.leavesShooterTimeSec?.toFixed(3) ?? '--'}s)
          </Button>
        </Tooltip>
        <Tooltip title="Set the current frame as the moment the ball hits the target" arrow>
          <Button size="small" variant="outlined" onClick={markHitTarget}>
            Mark Hit Target ({attempt.hitTargetTimeSec?.toFixed(3) ?? '--'}s)
          </Button>
        </Tooltip>
      </Box>

      {attempt.flightTimeSec !== null && (
        <Typography variant="body2" sx={{ mb: 1 }}>
          Flight time: <strong>{attempt.flightTimeSec.toFixed(3)}s</strong>
        </Typography>
      )}

      {/* Telemetry summary */}
      <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 0.25, mb: 1 }}>
        <Typography variant="caption" color="text.secondary">Distance:</Typography>
        <Typography variant="caption" fontFamily="monospace">
          {(attempt.distanceToHubMeters ?? attempt.distanceMeters).toFixed(3)} m
          {attempt.distanceToHubMeters !== null && (
            <Typography component="span" variant="caption" color="text.secondary">
              {' '}(pose: {attempt.distanceMeters.toFixed(3)} m)
            </Typography>
          )}
        </Typography>
        <Typography variant="caption" color="text.secondary">Shooter:</Typography>
        <Typography variant="caption" fontFamily="monospace">{attempt.shooterRPM.toFixed(1)} RPM</Typography>
        <Typography variant="caption" color="text.secondary">Hood:</Typography>
        <Typography variant="caption" fontFamily="monospace">{attempt.hoodAngleDegrees.toFixed(2)} deg</Typography>
      </Box>

      <Divider sx={{ my: 1 }} />

      {/* Export to Shot Maps */}
      <Box sx={{ display: 'flex', gap: 1 }}>
        <Tooltip title="Load ShotMaps.json, append this point to the hub table, save back to local file" arrow>
          <span>
            <Button
              size="small"
              variant="contained"
              disabled={attempt.flightTimeSec === null}
              onClick={() => handleExport('hub')}
            >
              {attempt.exportedToShotMap === 'hub' ? '✓ Add to Hub Shot Map' : 'Add to Hub Shot Map'}
            </Button>
          </span>
        </Tooltip>
        <Tooltip title="Load ShotMaps.json, append this point to the pass table, save back to local file" arrow>
          <span>
            <Button
              size="small"
              variant="contained"
              color="secondary"
              disabled={attempt.flightTimeSec === null}
              onClick={() => handleExport('pass')}
            >
              {attempt.exportedToShotMap === 'pass' ? '✓ Add to Pass Shot Map' : 'Add to Pass Shot Map'}
            </Button>
          </span>
        </Tooltip>
      </Box>
      {exportError && (
        <Alert severity="error" sx={{ mt: 1 }} onClose={() => setExportError(null)}>
          {exportError}
        </Alert>
      )}
    </Box>
  );
}
