import { useRef, useState, useEffect, useCallback } from 'react';
import {
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
import type { ShotMapDataPoint } from '../../types/ShotMaps';
import { clipUrl } from '../../services/shotTuningStorage';
import { shotMapsStore } from '../../services/shotMapsStore';

interface VideoReplayPlayerProps {
  attempt: TuningAttempt;
  onUpdate: (attempt: TuningAttempt) => Promise<void>;
  attempts: TuningAttempt[];
  onAttemptsChange: (attempts: TuningAttempt[]) => Promise<void>;
  fps?: number;
}


export function VideoReplayPlayer({ attempt, onUpdate, attempts, onAttemptsChange, fps = 30 }: VideoReplayPlayerProps) {
  const FRAME_STEP = 1 / fps;
  const videoRef = useRef<HTMLVideoElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [playing, setPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);


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

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;

      switch (e.key) {
        case 'ArrowLeft':
        case 'j':
          e.preventDefault();
          stepFrame(-FRAME_STEP);
          break;
        case 'ArrowRight':
        case 'k':
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
        case ' ':
          e.preventDefault();
          togglePlay();
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [stepFrame, togglePlay]);

  const markLeavesShooter = async () => {
    const updated = { ...attempt, leavesShooterTimeSec: currentTime };
    if (updated.hitTargetTimeSec !== null) {
      updated.flightTimeSec = updated.hitTargetTimeSec - currentTime;
    }
    await onUpdate(updated);
  };

  const markHitTarget = async () => {
    const updated = { ...attempt, hitTargetTimeSec: currentTime };
    if (updated.leavesShooterTimeSec !== null) {
      updated.flightTimeSec = currentTime - updated.leavesShooterTimeSec;
    }
    await onUpdate(updated);
  };

  const handleExport = async (target: 'hub' | 'pass') => {
    if (attempt.flightTimeSec === null) return;
    // Use NT-reported distance if available, fall back to pose-computed
    const distance = attempt.distanceToHubMeters ?? attempt.distanceMeters;
    const newPoint: ShotMapDataPoint = {
      distance: { value: distance, unit: 'Meter' },
      shooterRPM: attempt.shooterRPM,
      hoodAngle: { value: attempt.hoodAngleDegrees, unit: 'Degree' },
      flightTime: { value: attempt.flightTimeSec, unit: 'Second' },
    };
    // Push into the shared in-memory store so ShotMapsEditor reflects it immediately.
    // No file write here — the user saves explicitly from the Shot Maps tab.
    shotMapsStore.addPoint(newPoint, target);

    const updated = { ...attempt, exportedToShotMap: target };
    const next = attempts.map((a) => a.id === updated.id ? updated : a);
    await onAttemptsChange(next);
  };

  return (
    <Box ref={containerRef} tabIndex={-1} sx={{ outline: 'none' }}>
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
        style={{ width: '100%', borderRadius: 4, background: '#000' }}
      />

      {/* Scrub slider */}
      {duration > 0 && (
        <Slider
          value={currentTime}
          min={0}
          max={duration}
          step={0.001}
          onChange={(_, v) => seekTo(v as number)}
          size="small"
          sx={{ mt: 0.5, mb: 0 }}
        />
      )}

      {/* Playback controls */}
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5, mt: 0.5, flexWrap: 'wrap' }}>
        <Tooltip title="Back 1 frame (Left Arrow)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(-FRAME_STEP)}>
            -1 Frame
          </Button>
        </Tooltip>
        <Tooltip title={playing ? 'Pause (Space)' : 'Play (Space)'} arrow>
          <IconButton size="small" onClick={togglePlay}>
            {playing ? <PauseIcon /> : <PlayArrowIcon />}
          </IconButton>
        </Tooltip>
        <Tooltip title="Forward 1 frame (Right Arrow)" arrow>
          <Button size="small" variant="outlined" onClick={() => stepFrame(FRAME_STEP)}>
            +1 Frame
          </Button>
        </Tooltip>
        <Typography variant="body2" fontFamily="monospace" sx={{ ml: 1 }}>
          {currentTime.toFixed(3)}s / {duration.toFixed(3)}s
        </Typography>
      </Box>

      <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
        j / Left: -1 frame | k / Right: +1 frame | h: -10 frames | l: +10 frames | Space: play/pause
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
        <Tooltip title="Append this data point to the hub interpolation table in ShotMaps.json" arrow>
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
        <Tooltip title="Append this data point to the pass interpolation table in ShotMaps.json" arrow>
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
    </Box>
  );
}
