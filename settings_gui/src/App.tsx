import { useState, useMemo } from 'react';
import { BrowserRouter, Routes, Route, useNavigate, Navigate } from 'react-router-dom';
import { ThemeProvider, CssBaseline } from '@mui/material';
import {
  AppBar,
  Toolbar,
  Typography,
  IconButton,
  Box,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Drawer,
  List,
  ListItemButton,
  ListItemText,
} from '@mui/material';
import LightModeIcon from '@mui/icons-material/LightMode';
import DarkModeIcon from '@mui/icons-material/DarkMode';
import MenuIcon from '@mui/icons-material/Menu';
import { lightTheme, darkTheme } from './theme';
import { ConnectionProvider, useConnection, ENVIRONMENTS } from './contexts/ConnectionContext';
import { endpoints } from './endpoints/registry';

const DRAWER_WIDTH = 200;

function AppShell({ darkMode, onToggleTheme }: { darkMode: boolean; onToggleTheme: () => void }) {
  const navigate = useNavigate();
  const { environment, setEnvironment } = useConnection();
  const [drawerOpen, setDrawerOpen] = useState(false);

  return (
    <Box sx={{ display: 'flex' }}>
      <AppBar position="fixed" sx={{ zIndex: (t) => t.zIndex.drawer + 1 }}>
        <Toolbar>
          <IconButton
            color="inherit"
            edge="start"
            onClick={() => setDrawerOpen(!drawerOpen)}
            sx={{ mr: 2, display: { md: 'none' } }}
          >
            <MenuIcon />
          </IconButton>
          <Typography variant="h6" sx={{ flexGrow: 1 }}>
            Team 401 Settings
          </Typography>

          <FormControl size="small" sx={{ minWidth: 140, mr: 1 }}>
            <InputLabel sx={{ color: 'inherit' }}>Environment</InputLabel>
            <Select
              value={environment}
              label="Environment"
              onChange={(e) => setEnvironment(e.target.value)}
              sx={{ color: 'inherit', '.MuiOutlinedInput-notchedOutline': { borderColor: 'rgba(255,255,255,0.5)' } }}
            >
              {ENVIRONMENTS.map((env) => (
                <MenuItem key={env} value={env}>{env}</MenuItem>
              ))}
            </Select>
          </FormControl>

          <IconButton color="inherit" onClick={onToggleTheme}>
            {darkMode ? <LightModeIcon /> : <DarkModeIcon />}
          </IconButton>
        </Toolbar>
      </AppBar>

      {/* Desktop sidebar */}
      <Drawer
        variant="permanent"
        sx={{
          display: { xs: 'none', md: 'block' },
          width: DRAWER_WIDTH,
          flexShrink: 0,
          '& .MuiDrawer-paper': { width: DRAWER_WIDTH, boxSizing: 'border-box' },
        }}
      >
        <Toolbar />
        <List>
          {endpoints.map((ep) => (
            <ListItemButton key={ep.name} onClick={() => navigate(`/${ep.name}`)}>
              <ListItemText primary={ep.label} />
            </ListItemButton>
          ))}
        </List>
      </Drawer>

      {/* Mobile drawer */}
      <Drawer
        variant="temporary"
        open={drawerOpen}
        onClose={() => setDrawerOpen(false)}
        sx={{ display: { xs: 'block', md: 'none' }, '& .MuiDrawer-paper': { width: DRAWER_WIDTH } }}
      >
        <Toolbar />
        <List>
          {endpoints.map((ep) => (
            <ListItemButton key={ep.name} onClick={() => { navigate(`/${ep.name}`); setDrawerOpen(false); }}>
              <ListItemText primary={ep.label} />
            </ListItemButton>
          ))}
        </List>
      </Drawer>

      <Box component="main" sx={{ flexGrow: 1, p: 3, width: { md: `calc(100% - ${DRAWER_WIDTH}px)` } }}>
        <Toolbar />
        <Routes>
          {endpoints.map((ep) => (
            <Route key={ep.name} path={`/${ep.name}`} element={<ep.component />} />
          ))}
          <Route path="*" element={<Navigate to={`/${endpoints[0].name}`} replace />} />
        </Routes>
      </Box>
    </Box>
  );
}

export default function App() {
  const [darkMode, setDarkMode] = useState(false);
  const theme = useMemo(() => (darkMode ? darkTheme : lightTheme), [darkMode]);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <BrowserRouter>
        <ConnectionProvider>
          <AppShell darkMode={darkMode} onToggleTheme={() => setDarkMode(!darkMode)} />
        </ConnectionProvider>
      </BrowserRouter>
    </ThemeProvider>
  );
}
