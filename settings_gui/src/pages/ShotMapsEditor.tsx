import { useEffect, useState, useCallback } from 'react';
import {
  Box,
  Button,
  IconButton,
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  TextField,
  Typography,
  Alert,
  Snackbar,
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';
import SaveIcon from '@mui/icons-material/Save';
import SendIcon from '@mui/icons-material/Send';
import RefreshIcon from '@mui/icons-material/Refresh';
import FileDownloadIcon from '@mui/icons-material/FileDownload';
import { useConnection } from '../contexts/ConnectionContext';
import { getData, putData, postData, saveLocal } from '../services/api';
import type { ShotMaps, ShotMapDataPoint } from '../types/ShotMaps';

function defaultDataPoint(): ShotMapDataPoint {
  return {
    distance: { value: 0, unit: 'Meter' },
    shooterRPM: 0,
    hoodAngle: { value: 0, unit: 'Degree' },
    flightTime: { value: 0, unit: 'Second' },
  };
}

interface DataPointTableProps {
  title: string;
  rows: ShotMapDataPoint[];
  onChange: (rows: ShotMapDataPoint[]) => void;
}

function DataPointTable({ title, rows, onChange }: DataPointTableProps) {
  const updateRow = (index: number, updated: ShotMapDataPoint) => {
    const next = [...rows];
    next[index] = updated;
    onChange(next);
  };

  const deleteRow = (index: number) => {
    onChange(rows.filter((_, i) => i !== index));
  };

  const addRow = () => {
    onChange([...rows, defaultDataPoint()]);
  };

  return (
    <Box sx={{ mb: 3 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1, gap: 1 }}>
        <Typography variant="h6">{title}</Typography>
        <Button startIcon={<AddIcon />} size="small" onClick={addRow}>
          Add Row
        </Button>
      </Box>
      <TableContainer component={Paper} variant="outlined">
        <Table size="small">
          <TableHead>
            <TableRow>
              <TableCell>Distance (m)</TableCell>
              <TableCell>Shooter RPM</TableCell>
              <TableCell>Hood Angle (deg)</TableCell>
              <TableCell>Flight Time (s)</TableCell>
              <TableCell width={48} />
            </TableRow>
          </TableHead>
          <TableBody>
            {rows.map((row, i) => (
              <TableRow key={i}>
                <TableCell>
                  <TextField
                    type="number"
                    size="small"
                    value={row.distance.value}
                    onChange={(e) =>
                      updateRow(i, {
                        ...row,
                        distance: { ...row.distance, value: parseFloat(e.target.value) || 0 },
                      })
                    }
                  />
                </TableCell>
                <TableCell>
                  <TextField
                    type="number"
                    size="small"
                    value={row.shooterRPM}
                    onChange={(e) =>
                      updateRow(i, { ...row, shooterRPM: parseFloat(e.target.value) || 0 })
                    }
                  />
                </TableCell>
                <TableCell>
                  <TextField
                    type="number"
                    size="small"
                    value={row.hoodAngle.value}
                    onChange={(e) =>
                      updateRow(i, {
                        ...row,
                        hoodAngle: { ...row.hoodAngle, value: parseFloat(e.target.value) || 0 },
                      })
                    }
                  />
                </TableCell>
                <TableCell>
                  <TextField
                    type="number"
                    size="small"
                    value={row.flightTime.value}
                    onChange={(e) =>
                      updateRow(i, {
                        ...row,
                        flightTime: { ...row.flightTime, value: parseFloat(e.target.value) || 0 },
                      })
                    }
                  />
                </TableCell>
                <TableCell>
                  <IconButton size="small" color="error" onClick={() => deleteRow(i)}>
                    <DeleteIcon fontSize="small" />
                  </IconButton>
                </TableCell>
              </TableRow>
            ))}
            {rows.length === 0 && (
              <TableRow>
                <TableCell colSpan={5} align="center">
                  No data points. Click "Add Row" to add one.
                </TableCell>
              </TableRow>
            )}
          </TableBody>
        </Table>
      </TableContainer>
    </Box>
  );
}

export function ShotMapsEditor() {
  const { environment } = useConnection();
  const [data, setData] = useState<ShotMaps | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [snack, setSnack] = useState<{ message: string; severity: 'success' | 'error' } | null>(null);

  const fetchData = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const result = await getData<ShotMaps>(environment, 'shotmaps');
      setData(result);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to fetch');
    } finally {
      setLoading(false);
    }
  }, [environment]);

  useEffect(() => {
    fetchData();
  }, [fetchData]);

  const handleSave = async () => {
    if (!data) return;
    setError(null);
    try {
      await putData(environment, 'shotmaps', data);
      setSnack({ message: 'Saved successfully', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to save');
    }
  };

  const handleSaveAndActivate = async () => {
    if (!data) return;
    setError(null);
    try {
      await putData(environment, 'shotmaps', data);
      const result = await postData<{ success?: boolean }>(environment, 'shotmaps');
      if (result.success) {
        setSnack({ message: 'Activated successfully', severity: 'success' });
      } else {
        setSnack({ message: 'Activation failed', severity: 'error' });
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to save & activate');
    }
  };

  const handleSaveLocal = async () => {
    if (!data) return;
    setError(null);
    try {
      await saveLocal(environment, 'ShotMaps.json', data);
      setSnack({ message: 'Saved to local source tree', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to save locally');
    }
  };

  if (loading) return <Typography>Loading...</Typography>;

  if (!data) {
    return (
      <Box>
        {error && <Alert severity="error" sx={{ mb: 2 }}>{error}</Alert>}
        <Button startIcon={<RefreshIcon />} onClick={fetchData}>Retry</Button>
      </Box>
    );
  }

  return (
    <Box>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 2, gap: 2 }}>
        <Typography variant="h5">Shot Maps</Typography>
        <Button variant="contained" startIcon={<SaveIcon />} onClick={handleSave}>
          Save to Robot
        </Button>
        <Button variant="contained" color="secondary" startIcon={<SendIcon />} onClick={handleSaveAndActivate}>
          Save to Robot & Activate
        </Button>
        <Button startIcon={<RefreshIcon />} onClick={fetchData}>
          Refresh
        </Button>
        <Button variant="outlined" startIcon={<FileDownloadIcon />} onClick={handleSaveLocal} sx={{ ml: 'auto' }}>
          Save to Local
        </Button>
      </Box>

      {error && <Alert severity="error" sx={{ mb: 2 }}>{error}</Alert>}

      <DataPointTable
        title="Hub Data Points"
        rows={data.hubDataPoints}
        onChange={(hubDataPoints) => setData({ ...data, hubDataPoints })}
      />

      <DataPointTable
        title="Pass Data Points"
        rows={data.passDataPoints}
        onChange={(passDataPoints) => setData({ ...data, passDataPoints })}
      />

      {data.mechanismCompensationDelay !== undefined && (
        <Box sx={{ mb: 3 }}>
          <Typography variant="h6" sx={{ mb: 1 }}>Mechanism Compensation Delay</Typography>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <TextField
              type="number"
              size="small"
              label="Value (s)"
              value={data.mechanismCompensationDelay.value}
              onChange={(e) =>
                setData({
                  ...data,
                  mechanismCompensationDelay: {
                    ...data.mechanismCompensationDelay!,
                    value: parseFloat(e.target.value) || 0,
                  },
                })
              }
            />
          </Box>
        </Box>
      )}

      <Snackbar open={snack !== null} autoHideDuration={3000} onClose={() => setSnack(null)}>
        <Alert severity={snack?.severity ?? 'success'} variant="filled" onClose={() => setSnack(null)}>
          {snack?.message}
        </Alert>
      </Snackbar>
    </Box>
  );
}
