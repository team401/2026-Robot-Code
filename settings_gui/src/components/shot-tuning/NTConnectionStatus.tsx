import { useState, useEffect, useRef } from 'react';
import {
  Box,
  Button,
  TextField,
  Typography,
  Chip,
  ToggleButtonGroup,
  ToggleButton,
  Collapse,
  IconButton,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ExpandLessIcon from '@mui/icons-material/ExpandLess';
import type { NT4Service } from '../../services/nt4';

interface NTConnectionStatusProps {
  onConnect: (address: string) => Promise<NT4Service>;
  onDisconnect: () => void;
}

const PRESETS = [
  { label: 'Sim (localhost)', value: 'localhost' },
  { label: 'Robot (10.4.1.2)', value: '10.4.1.2' },
];

export function NTConnectionStatus({ onConnect, onDisconnect }: NTConnectionStatusProps) {
  const [address, setAddress] = useState('localhost');
  const [connected, setConnected] = useState(false);
  const [active, setActive] = useState(false);
  const [expanded, setExpanded] = useState(true);
  const cleanupRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    return () => {
      cleanupRef.current?.();
    };
  }, []);

  const handleConnect = async () => {
    setActive(true);
    const svc = await onConnect(address);
    cleanupRef.current = svc.addConnectionListener((conn) => {
      setConnected(conn);
      if (conn) setExpanded(false);
    });
  };

  const handleDisconnect = () => {
    cleanupRef.current?.();
    cleanupRef.current = null;
    onDisconnect();
    setConnected(false);
    setActive(false);
    setExpanded(true);
  };

  return (
    <Box>
      <Box
        sx={{ display: 'flex', alignItems: 'center', gap: 1, cursor: 'pointer' }}
        onClick={() => setExpanded((v) => !v)}
      >
        <Typography variant="subtitle1" fontWeight="bold">NT4 Connection</Typography>
        <Chip
          label={connected ? `Connected to ${address}` : active ? 'Connecting...' : 'Disconnected'}
          color={connected ? 'success' : active ? 'warning' : 'default'}
          size="small"
        />
        <IconButton size="small" sx={{ ml: 'auto' }}>
          {expanded ? <ExpandLessIcon /> : <ExpandMoreIcon />}
        </IconButton>
      </Box>

      <Collapse in={expanded}>
        <Box sx={{ mt: 1 }}>
          <ToggleButtonGroup
            value={address}
            exclusive
            onChange={(_, v) => { if (v) setAddress(v); }}
            size="small"
            sx={{ mb: 2 }}
          >
            {PRESETS.map((p) => (
              <ToggleButton key={p.value} value={p.value}>{p.label}</ToggleButton>
            ))}
          </ToggleButtonGroup>

          <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
            <TextField
              size="small"
              label="Address"
              value={address}
              onChange={(e) => setAddress(e.target.value)}
              sx={{ flex: 1 }}
            />
            {!active ? (
              <Button variant="contained" size="small" onClick={handleConnect}>Connect</Button>
            ) : (
              <Button variant="outlined" size="small" color="error" onClick={handleDisconnect}>Disconnect</Button>
            )}
          </Box>
        </Box>
      </Collapse>
    </Box>
  );
}
