import type { TuningAttempt } from '../types/ShotTuning';

export async function loadAttempts(): Promise<TuningAttempt[]> {
  const res = await fetch('/shot-tuning/attempts');
  if (!res.ok) {
    if (res.status === 404) return [];
    throw new Error(`Load attempts failed: ${res.status}`);
  }
  return res.json() as Promise<TuningAttempt[]>;
}

export async function saveAttempts(attempts: TuningAttempt[]): Promise<void> {
  const res = await fetch('/shot-tuning/attempts', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(attempts),
  });
  if (!res.ok) throw new Error(`Save attempts failed: ${res.status}`);
}

export async function uploadClip(id: string, blob: Blob): Promise<void> {
  const res = await fetch(`/shot-tuning/clips/${id}`, {
    method: 'POST',
    headers: { 'Content-Type': 'video/webm' },
    body: blob,
  });
  if (!res.ok) throw new Error(`Upload clip failed: ${res.status}`);
}

export function clipUrl(id: string): string {
  return `/shot-tuning/clips/${id}`;
}

export async function deleteClip(id: string): Promise<void> {
  const res = await fetch(`/shot-tuning/clips/${id}`, { method: 'DELETE' });
  if (!res.ok && res.status !== 404) throw new Error(`Delete clip failed: ${res.status}`);
}
