import { asJson, readError } from './http'

export interface Camera {
  id: number
  name: string
  serial: string
  online: boolean
  created_at: number
}

export interface AvailableCamera {
  serial: string
  model: string
  vendor: string
}

export async function listCameras(): Promise<Camera[]> {
  return asJson<Camera[]>(await fetch('/api/cameras'))
}

export async function listAvailableCameras(): Promise<AvailableCamera[]> {
  return asJson<AvailableCamera[]>(await fetch('/api/cameras/available'))
}

export async function createCamera(name: string, serial: string): Promise<Camera> {
  const res = await fetch('/api/cameras', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, serial }),
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
