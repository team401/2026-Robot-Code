import { createContext, useContext, useState, type ReactNode } from 'react';

interface ConnectionState {
  environment: string;
  setEnvironment: (env: string) => void;
  baseUrl: string;
}

const ConnectionContext = createContext<ConnectionState | null>(null);

const ENVIRONMENTS = ['comp', 'test_drivebase'];

export function ConnectionProvider({ children }: { children: ReactNode }) {
  const [environment, setEnvironment] = useState(ENVIRONMENTS[0]);

  return (
    <ConnectionContext.Provider value={{ environment, setEnvironment, baseUrl: '/api' }}>
      {children}
    </ConnectionContext.Provider>
  );
}

export function useConnection() {
  const ctx = useContext(ConnectionContext);
  if (!ctx) throw new Error('useConnection must be used within ConnectionProvider');
  return ctx;
}

export { ENVIRONMENTS };
