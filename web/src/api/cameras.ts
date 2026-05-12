export interface Camera {
  id: number
  name: string
  created_at: number
}

async function readError(res: Response): Promise<string> {
  try {
    const body = await res.json()
    if (body && typeof body.error === 'string') return body.error
  } catch {
    // fall through
  }
  return `HTTP ${res.status}`
}

async function asJson<T>(res: Response): Promise<T> {
  if (!res.ok) throw new Error(await readError(res))
  return (await res.json()) as T
}

export async function listCameras(): Promise<Camera[]> {
  return asJson<Camera[]>(await fetch('/api/cameras'))
}

export async function createCamera(name: string): Promise<Camera> {
  const res = await fetch('/api/cameras', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  return asJson<Camera>(res)
}

export async function updateCamera(id: number, name: string): Promise<Camera> {
  const res = await fetch(`/api/cameras/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  return asJson<Camera>(res)
}

export async function deleteCamera(id: number): Promise<void> {
  const res = await fetch(`/api/cameras/${id}`, { method: 'DELETE' })
  if (!res.ok) throw new Error(await readError(res))
}
