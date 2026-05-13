import { asJson, readError } from './http'

export interface Camera {
  id: number
  name: string
  serial: string
  mode: string | null
  // Geometry + max FPS for the currently-running mode. null when the camera is
  // offline or its producer didn't report a value.
  mode_width: number | null
  mode_height: number | null
  mode_max_fps: number | null
  online: boolean
  created_at: number
}

export interface AvailableCamera {
  serial: string
  model: string
  vendor: string
}

export interface CameraMode {
  name: string
  display_name: string
  description: string
  width: number | null
  height: number | null
  max_fps: number | null
}

export interface CameraModesResponse {
  supported: boolean
  current: string | null
  options: CameraMode[]
}

// Raised by getCameraModes when the camera exists in the DB but isn't online,
// so the modal can disable the dropdown without parsing error strings.
export class CameraOfflineError extends Error {
  constructor() {
    super('camera is offline')
    this.name = 'CameraOfflineError'
  }
}

export async function listCameras(): Promise<Camera[]> {
  return asJson<Camera[]>(await fetch('/api/cameras'))
}

export async function listAvailableCameras(): Promise<AvailableCamera[]> {
  return asJson<AvailableCamera[]>(await fetch('/api/cameras/available'))
}

export async function createCamera(input: {
  name: string
  serial: string
  mode?: string
}): Promise<Camera> {
  const body: Record<string, unknown> = { name: input.name, serial: input.serial }
  if (input.mode) body.mode = input.mode
  const res = await fetch('/api/cameras', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  return asJson<Camera>(res)
}

export async function updateCamera(
  id: number,
  patch: { name?: string; mode?: string },
): Promise<Camera> {
  const res = await fetch(`/api/cameras/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patch),
  })
  return asJson<Camera>(res)
}

export async function deleteCamera(id: number): Promise<void> {
  const res = await fetch(`/api/cameras/${id}`, { method: 'DELETE' })
  if (!res.ok) throw new Error(await readError(res))
}

export async function getCameraModes(id: number): Promise<CameraModesResponse> {
  const res = await fetch(`/api/cameras/${id}/modes`)
  if (res.status === 409) throw new CameraOfflineError()
  return asJson<CameraModesResponse>(res)
}

export async function getAvailableCameraModes(
  serial: string,
): Promise<CameraModesResponse> {
  const res = await fetch(`/api/cameras/available/${encodeURIComponent(serial)}/modes`)
  return asJson<CameraModesResponse>(res)
}
