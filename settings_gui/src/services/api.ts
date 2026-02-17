const BASE_URL = '/api';

export async function getData<T>(env: string, endpoint: string): Promise<T> {
  const res = await fetch(`${BASE_URL}/${env}/${endpoint}`);
  if (!res.ok) throw new Error(`GET failed: ${res.status} ${res.statusText}`);
  return res.json() as Promise<T>;
}

export async function putData<T>(env: string, endpoint: string, data: T): Promise<void> {
  const res = await fetch(`${BASE_URL}/${env}/${endpoint}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
  if (!res.ok) throw new Error(`PUT failed: ${res.status} ${res.statusText}`);
}

export async function postData<R>(env: string, endpoint: string): Promise<R> {
  const res = await fetch(`${BASE_URL}/${env}/${endpoint}`, {
    method: 'POST',
  });
  if (!res.ok) throw new Error(`POST failed: ${res.status} ${res.statusText}`);
  return res.json() as Promise<R>;
}

export async function loadLocal<T>(env: string, filename: string): Promise<T> {
  const res = await fetch(`/local-load/${env}/${filename}`);
  if (!res.ok) throw new Error(`Local load failed: ${res.status} ${res.statusText}`);
  return res.json() as Promise<T>;
}

export async function saveLocal(env: string, filename: string, data: unknown): Promise<void> {
  const res = await fetch(`/local-save/${env}/${filename}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
  if (!res.ok) throw new Error(`Local save failed: ${res.status} ${res.statusText}`);
}
