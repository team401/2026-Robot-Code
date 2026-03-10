import {
  Box,
  Typography,
  List,
  ListItemButton,
  ListItemText,
  ListItemSecondaryAction,
  IconButton,
  Chip,
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import type { TuningAttempt } from '../../types/ShotTuning';
import { getAttemptDistanceSource, getAttemptSampleDistance } from '../../types/ShotTuning';

interface AttemptsListProps {
  attempts: TuningAttempt[];
  selectedId: string | null;
  onSelect: (id: string) => void;
  onDelete: (id: string) => void;
}

export function AttemptsList({ attempts, selectedId, onSelect, onDelete }: AttemptsListProps) {
  const sorted = [...attempts].sort((a, b) => b.createdAt.localeCompare(a.createdAt));

  return (
    <Box>
      <Typography variant="subtitle1" fontWeight="bold" sx={{ mb: 1 }}>
        Attempts ({attempts.length})
      </Typography>

      {attempts.length === 0 ? (
        <Typography variant="body2" color="text.secondary">
          No attempts recorded yet. Use the recording controls to capture a shot.
        </Typography>
      ) : (
        <List dense disablePadding>
          {sorted.map((attempt) => (
            <ListItemButton
              key={attempt.id}
              selected={attempt.id === selectedId}
              onClick={() => onSelect(attempt.id)}
              sx={{ borderRadius: 1 }}
            >
              <ListItemText
                primary={
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
                    <Typography variant="body2">
                      {getAttemptSampleDistance(attempt).toFixed(2)}m
                    </Typography>
                    <Chip
                      label={getAttemptDistanceSource(attempt) === 'networkTables' ? 'NT' : 'Odom'}
                      size="small"
                      variant="outlined"
                    />
                    {attempt.flightTimeSec !== null && (
                      <Chip label={`${attempt.flightTimeSec.toFixed(3)}s`} size="small" color="primary" variant="outlined" />
                    )}
                    {attempt.exportedToShotMap && (
                      <Chip label={attempt.exportedToShotMap} size="small" color="success" />
                    )}
                  </Box>
                }
                secondary={new Date(attempt.createdAt).toLocaleString()}
              />
              <ListItemSecondaryAction>
                <IconButton
                  edge="end"
                  size="small"
                  onClick={(e) => { e.stopPropagation(); onDelete(attempt.id); }}
                >
                  <DeleteIcon fontSize="small" />
                </IconButton>
              </ListItemSecondaryAction>
            </ListItemButton>
          ))}
        </List>
      )}
    </Box>
  );
}
