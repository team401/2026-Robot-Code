import { useState, useEffect, useRef } from 'react';
import {
  Box,
  Button,
  TextField,
  Typography,
  Chip,
  ToggleButtonGroup,
  ToggleButton,
} from '@mui/material';
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
  const cleanupRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    return () => {
      cleanupRef.current?.();
    };
  }, []);

  const handleConnect = async () => {
    setActive(true);
    const svc = await onConnect(address);
    cleanupRef.current = svc.addConnectionListener((conn) => setConnected(conn));
  };

  const handleDisconnect = () => {
    cleanupRef.current?.();
    cleanupRef.current = null;
    onDisconnect();
    setConnected(false);
    setActive(false);
  };

  return (
    <Box>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
        <Typography variant="subtitle1" fontWeight="bold">NT4 Connection</Typography>
        <Chip
          label={connected ? 'Connected' : active ? 'Connecting...' : 'Disconnected'}
          color={connected ? 'success' : active ? 'warning' : 'default'}
          size="small"
        />
      </Box>

      <ToggleButtonGroup
        value={address}
        exclusive
        onChange={(_, v) => { if (v) setAddress(v); }}
        size="small"
        sx={{ mb: 1 }}
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
  );
}
