import { useEffect, useState, useCallback } from 'react';
import {
  Box,
  Button,
  IconButton,
  Paper,
  Tab,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Tabs,
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
import SortIcon from '@mui/icons-material/Sort';
import FileDownloadIcon from '@mui/icons-material/FileDownload';
import FileUploadIcon from '@mui/icons-material/FileUpload';
import { useConnection } from '../contexts/ConnectionContext';
import { getData, putData, postData, saveLocal, loadLocal } from '../services/api';
import type { ShotMaps, ShotMapDataPoint } from '../types/ShotMaps';
import { shotMapsLocalSignal } from '../services/shotMapsStore';
import { ShotMapChart } from '../components/shot-maps/ShotMapChart';

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

  const sortByDistance = () => {
    onChange([...rows].sort((a, b) => a.distance.value - b.distance.value));
  };

  return (
    <Box sx={{ mb: 3 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1, gap: 1 }}>
        <Typography variant="h6">{title}</Typography>
        <Button startIcon={<AddIcon />} size="small" onClick={addRow}>
          Add Row
        </Button>
        <Button startIcon={<SortIcon />} size="small" onClick={sortByDistance}>
          Sort by Distance
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
  const [activeTab, setActiveTab] = useState<'hub' | 'pass'>('hub');

  const fetchData = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      let result: ShotMaps;
      if (shotMapsLocalSignal.consume()) {
        // The tuning tab saved a new point to the local file — reload from there.
        result = await loadLocal<ShotMaps>(environment, 'ShotMaps.json');
      } else {
        result = await getData<ShotMaps>(environment, 'shotmaps');
      }
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

  const handleLoadLocal = async () => {
    setError(null);
    try {
      const result = await loadLocal<ShotMaps>(environment, 'ShotMaps.json');
      setData(result);
      setSnack({ message: 'Loaded from local source tree', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to load from local');
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

  const activeConfig = activeTab === 'hub'
    ? {
        chartId: 'hub',
        title: 'Hub Data Points',
        rows: data.hubDataPoints,
        onChange: (rows: ShotMapDataPoint[]) => setData({ ...data, hubDataPoints: rows }),
      }
    : {
        chartId: 'pass',
        title: 'Pass Data Points',
        rows: data.passDataPoints,
        onChange: (rows: ShotMapDataPoint[]) => setData({ ...data, passDataPoints: rows }),
      };

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
        <Button variant="outlined" startIcon={<FileUploadIcon />} onClick={handleLoadLocal} sx={{ ml: 'auto' }}>
          Load from Local
        </Button>
        <Button
          variant="outlined"
          startIcon={<FileDownloadIcon />}
          onClick={handleSaveLocal}
        >
          Save to Local
        </Button>
      </Box>

      {error && <Alert severity="error" sx={{ mb: 2 }}>{error}</Alert>}

      {(data.mechanismCompensationDelay !== undefined || data.rpmCompensation !== undefined) && (
        <Box sx={{ display: 'flex', gap: 2, mb: 2 }}>
          {data.mechanismCompensationDelay !== undefined && (
            <TextField
              type="number"
              size="small"
              label="Mechanism Compensation Delay (s)"
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
          )}
          {data.rpmCompensation !== undefined && (
            <TextField
              type="number"
              size="small"
              label="RPM Compensation (RPM)"
              value={data.rpmCompensation.value}
              onChange={(e) =>
                setData({
                  ...data,
                  rpmCompensation: {
                    ...data.rpmCompensation!,
                    value: parseFloat(e.target.value) || 0,
                  },
                })
              }
            />
          )}
        </Box>
      )}

      <Paper variant="outlined" sx={{ mb: 2 }}>
        <Tabs
          value={activeTab}
          onChange={(_, value: 'hub' | 'pass') => setActiveTab(value)}
          variant="fullWidth"
        >
          <Tab value="hub" label="Hub Data Points" />
          <Tab value="pass" label="Pass Data Points" />
        </Tabs>
      </Paper>

      <ShotMapChart chartId={activeConfig.chartId} rows={activeConfig.rows} />

      <DataPointTable
        title={activeConfig.title}
        rows={activeConfig.rows}
        onChange={activeConfig.onChange}
      />

      <Snackbar open={snack !== null} autoHideDuration={3000} onClose={() => setSnack(null)}>
        <Alert severity={snack?.severity ?? 'success'} variant="filled" onClose={() => setSnack(null)}>
          {snack?.message}
        </Alert>
      </Snackbar>
    </Box>
  );
}
