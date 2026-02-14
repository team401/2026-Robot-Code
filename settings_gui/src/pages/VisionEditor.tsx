import { useEffect, useState, useCallback } from 'react';
import {
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Alert,
  Box,
  Button,
  Grid,
  IconButton,
  Slider,
  Snackbar,
  TextField,
  Typography,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';
import SaveIcon from '@mui/icons-material/Save';
import SendIcon from '@mui/icons-material/Send';
import RefreshIcon from '@mui/icons-material/Refresh';
import { useConnection } from '../contexts/ConnectionContext';
import { getData, putData, postData } from '../services/api';
import type {
  VisionWireFormat,
  VisionConfig,
  CameraConfig,
  CameraTransform,
  GainConstants,
} from '../types/Vision';

// --- Wire format conversion ---

function wireToConfig(wire: VisionWireFormat): VisionConfig {
  const cameras: { num: number; cam: CameraConfig }[] = [];
  for (const key of Object.keys(wire)) {
    const m = key.match(/^camera(\d+)Index$/);
    if (m) {
      const n = parseInt(m[1], 10);
      const transform = (wire[`camera${n}Transform`] as CameraTransform) ?? defaultTransform();
      const r = transform.rotation;
      cameras.push({
        num: n,
        cam: {
          index: wire[`camera${n}Index`] as number,
          name: (wire[`camera${n}Name`] as string) ?? '',
          transform: {
            translation: transform.translation,
            rotation: {
              roll: Math.round(r.roll * 100) / 100,
              pitch: Math.round(r.pitch * 100) / 100,
              yaw: Math.round(r.yaw * 100) / 100,
            },
          },
        },
      });
    }
  }
  cameras.sort((a, b) => a.num - b.num);
  return {
    gainConstants: wire.gainConstants,
    cameras: cameras.map((c) => c.cam),
  };
}

function configToWire(config: VisionConfig): VisionWireFormat {
  const wire: VisionWireFormat = { gainConstants: config.gainConstants };
  config.cameras.forEach((cam, i) => {
    const n = i + 1;
    wire[`camera${n}Index`] = cam.index;
    wire[`camera${n}Name`] = cam.name;
    wire[`camera${n}Transform`] = cam.transform;
  });
  return wire;
}

function defaultTransform(): CameraTransform {
  return {
    rotation: { roll: 0, pitch: 0, yaw: 0 },
    translation: { x: 0, y: 0, z: 0 },
  };
}

// --- SliderField ---

interface SliderFieldProps {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit: string;
  onChange: (v: number) => void;
}

function SliderField({ label, value, min, max, step, unit, onChange }: SliderFieldProps) {
  const clamp = (v: number) => Math.min(max, Math.max(min, v));

  return (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5, mb: 1 }}>
      <Typography sx={{ minWidth: 40 }}>{label}</Typography>
      <TextField
        type="number"
        size="small"
        sx={{ width: 120 }}
        inputProps={{ step, min, max }}
        value={value}
        onChange={(e) => {
          const parsed = parseFloat(e.target.value);
          if (!isNaN(parsed)) onChange(clamp(parsed));
        }}
      />
      <Typography variant="caption" color="text.secondary">
        {min}
      </Typography>
      <Slider
        value={value}
        min={min}
        max={max}
        step={step}
        valueLabelDisplay="auto"
        onChange={(_, v) => onChange(v as number)}
        sx={{ flex: 1 }}
      />
      <Typography variant="caption" color="text.secondary">
        {max}
      </Typography>
      <Typography variant="body2" color="text.secondary" sx={{ minWidth: 30 }}>
        {unit}
      </Typography>
    </Box>
  );
}

// --- GainConstantsSection ---

const gainFields: { key: keyof GainConstants; label: string }[] = [
  { key: 'maxAcceptedDistanceMeters', label: 'Max Accepted Distance (m)' },
  { key: 'linearStdDevFactor', label: 'Linear Std Dev Factor' },
  { key: 'angularStdDevFactor', label: 'Angular Std Dev Factor' },
  { key: 'maxZCutoff', label: 'Max Z Cutoff' },
  { key: 'maxSingleTagAmbiguity', label: 'Max Single Tag Ambiguity' },
  { key: 'maxAmbiguity', label: 'Max Ambiguity' },
];

interface GainConstantsSectionProps {
  gain: GainConstants;
  onChange: (g: GainConstants) => void;
}

function GainConstantsSection({ gain, onChange }: GainConstantsSectionProps) {
  return (
    <Box sx={{ mb: 3 }}>
      <Typography variant="h6" sx={{ mb: 1 }}>
        Gain Constants
      </Typography>
      <Grid container spacing={2}>
        {gainFields.map(({ key, label }) => (
          <Grid item xs={12} sm={6} key={key}>
            <TextField
              type="number"
              size="small"
              fullWidth
              label={label}
              value={gain[key]}
              onChange={(e) =>
                onChange({ ...gain, [key]: parseFloat(e.target.value) || 0 })
              }
            />
          </Grid>
        ))}
      </Grid>
    </Box>
  );
}

// --- CameraCard ---

interface CameraCardProps {
  camera: CameraConfig;
  index: number;
  onChange: (cam: CameraConfig) => void;
  onDelete: () => void;
}

function CameraCard({ camera, index, onChange, onDelete }: CameraCardProps) {
  const updateTranslation = (axis: 'x' | 'y' | 'z', v: number) =>
    onChange({
      ...camera,
      transform: {
        ...camera.transform,
        translation: { ...camera.transform.translation, [axis]: v },
      },
    });

  const updateRotation = (axis: 'roll' | 'pitch' | 'yaw', v: number) =>
    onChange({
      ...camera,
      transform: {
        ...camera.transform,
        rotation: { ...camera.transform.rotation, [axis]: Math.round(v * 100) / 100 },
      },
    });

  return (
    <Accordion defaultExpanded>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography>
          Camera {index + 1}: {camera.name || '(unnamed)'}
        </Typography>
      </AccordionSummary>
      <AccordionDetails>
        <Box sx={{ display: 'flex', gap: 2, mb: 2 }}>
          <TextField
            size="small"
            label="Camera Name"
            value={camera.name}
            onChange={(e) => onChange({ ...camera, name: e.target.value })}
          />
          <TextField
            type="number"
            size="small"
            label="Hardware Index"
            inputProps={{ step: 1 }}
            value={camera.index}
            onChange={(e) =>
              onChange({ ...camera, index: parseInt(e.target.value, 10) || 0 })
            }
          />
        </Box>

        <Typography variant="subtitle2" sx={{ mb: 0.5 }}>
          Translation
        </Typography>
        {(['x', 'y', 'z'] as const).map((axis) => (
          <SliderField
            key={axis}
            label={axis}
            value={camera.transform.translation[axis]}
            min={-0.5}
            max={0.5}
            step={0.001}
            unit="m"
            onChange={(v) => updateTranslation(axis, v)}
          />
        ))}

        <Typography variant="subtitle2" sx={{ mt: 1, mb: 0.5 }}>
          Rotation
        </Typography>
        {(['roll', 'pitch', 'yaw'] as const).map((axis) => (
          <SliderField
            key={axis}
            label={axis}
            value={camera.transform.rotation[axis]}
            min={-180}
            max={180}
            step={0.01}
            unit="deg"
            onChange={(v) => updateRotation(axis, v)}
          />
        ))}

        <Box sx={{ mt: 1, display: 'flex', justifyContent: 'flex-end' }}>
          <IconButton color="error" onClick={onDelete}>
            <DeleteIcon />
          </IconButton>
        </Box>
      </AccordionDetails>
    </Accordion>
  );
}

// --- VisionEditor (main page) ---

export function VisionEditor() {
  const { environment } = useConnection();
  const [data, setData] = useState<VisionConfig | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [snack, setSnack] = useState<{ message: string; severity: 'success' | 'error' } | null>(
    null,
  );

  const fetchData = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const wire = await getData<VisionWireFormat>(environment, 'vision');
      setData(wireToConfig(wire));
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
      await putData(environment, 'vision', configToWire(data));
      setSnack({ message: 'Saved successfully', severity: 'success' });
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to save');
    }
  };

  const handleSaveAndActivate = async () => {
    if (!data) return;
    setError(null);
    try {
      await putData(environment, 'vision', configToWire(data));
      const result = await postData<{ success?: boolean }>(environment, 'vision');
      if (result.success) {
        setSnack({ message: 'Activated successfully', severity: 'success' });
      } else {
        setSnack({ message: 'Activation failed', severity: 'error' });
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to save & activate');
    }
  };

  const updateCamera = (index: number, cam: CameraConfig) => {
    if (!data) return;
    const cameras = [...data.cameras];
    cameras[index] = cam;
    setData({ ...data, cameras });
  };

  const deleteCamera = (index: number) => {
    if (!data) return;
    setData({ ...data, cameras: data.cameras.filter((_, i) => i !== index) });
  };

  const addCamera = () => {
    if (!data) return;
    const nextIndex =
      data.cameras.length > 0 ? Math.max(...data.cameras.map((c) => c.index)) + 1 : 0;
    setData({
      ...data,
      cameras: [...data.cameras, { index: nextIndex, name: '', transform: defaultTransform() }],
    });
  };

  if (loading) return <Typography>Loading...</Typography>;

  if (!data) {
    return (
      <Box>
        {error && (
          <Alert severity="error" sx={{ mb: 2 }}>
            {error}
          </Alert>
        )}
        <Button startIcon={<RefreshIcon />} onClick={fetchData}>
          Retry
        </Button>
      </Box>
    );
  }

  return (
    <Box>
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 2, gap: 2 }}>
        <Typography variant="h5">Vision</Typography>
        <Button variant="contained" startIcon={<SaveIcon />} onClick={handleSave}>
          Save
        </Button>
        <Button
          variant="contained"
          color="secondary"
          startIcon={<SendIcon />}
          onClick={handleSaveAndActivate}
        >
          Save & Activate
        </Button>
        <Button startIcon={<RefreshIcon />} onClick={fetchData}>
          Refresh
        </Button>
      </Box>

      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      <GainConstantsSection
        gain={data.gainConstants}
        onChange={(gainConstants) => setData({ ...data, gainConstants })}
      />

      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1, gap: 1 }}>
        <Typography variant="h6">Cameras</Typography>
        <Button startIcon={<AddIcon />} size="small" onClick={addCamera}>
          Add Camera
        </Button>
      </Box>

      {data.cameras.map((cam, i) => (
        <CameraCard
          key={i}
          camera={cam}
          index={i}
          onChange={(c) => updateCamera(i, c)}
          onDelete={() => deleteCamera(i)}
        />
      ))}

      {data.cameras.length === 0 && (
        <Typography color="text.secondary" sx={{ mt: 1 }}>
          No cameras configured. Click "Add Camera" to add one.
        </Typography>
      )}

      <Snackbar open={snack !== null} autoHideDuration={3000} onClose={() => setSnack(null)}>
        <Alert
          severity={snack?.severity ?? 'success'}
          variant="filled"
          onClose={() => setSnack(null)}
        >
          {snack?.message}
        </Alert>
      </Snackbar>
    </Box>
  );
}
