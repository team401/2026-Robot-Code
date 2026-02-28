import type { ShotMapDataPoint } from '../types/ShotMaps';

export interface PendingPoint {
  point: ShotMapDataPoint;
  target: 'hub' | 'pass';
}

type Listener = (point: ShotMapDataPoint, target: 'hub' | 'pass') => void;

// Module-level singleton — survives tab switches within the same page session.
const pending: PendingPoint[] = [];
const listeners = new Set<Listener>();

export const shotMapsStore = {
  /** Called by the tuning tab to push a new data point without saving to disk. */
  addPoint(point: ShotMapDataPoint, target: 'hub' | 'pass') {
    pending.push({ point, target });
    listeners.forEach((cb) => cb(point, target));
  },

  /** Called by the editor when it mounts to pick up points added before it loaded. */
  drainPending(): PendingPoint[] {
    return pending.splice(0);
  },

  /** Subscribe to future addPoint calls. Returns unsubscribe function. */
  subscribe(cb: Listener): () => void {
    listeners.add(cb);
    return () => listeners.delete(cb);
  },
};
