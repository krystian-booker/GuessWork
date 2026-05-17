import { asJson, readError } from './http'

export interface Camera {
  id: number
  name: string
  serial: string
  // Lens focal length in millimeters. Drives the Kalibr focal-length hint
  // and the auto-selected camera model (pinhole-radtan vs pinhole-equi) in
  // the calibration command. Server-side conversion via the fixed sensor
  // pixel pitch.
  focal_length_mm: number
  mode: string | null
  // Geometry + max FPS for the currently-running mode. null when the camera is
  // offline or its producer didn't report a value.
  mode_width: number | null
  mode_height: number | null
  mode_max_fps: number | null
  // Live-tunable settings. null means "use camera default".
  gain_auto: boolean | null
  gain: number | null
  exposure_auto: boolean | null
  exposure: number | null
  online: boolean
  created_at: number
  // Unix seconds when the calibration was last uploaded; null if uncalibrated.
  calibrated_at: number | null
  // RMS reprojection-error sigma in pixels, extracted from the camchain we
  // stored. null when uncalibrated, or when the calibration predates the
  // guesswork_meta enrichment (uploaded manually before the auto-pipeline).
  // Rule of thumb: ≤ 0.5 px is a good calibration; > 0.5 px is poor.
  reprojection_error_px: number | null
}

// Threshold + presentation helpers for the Good / Poor visual cue shared
// between the cameras table and the calibrate page, so a borderline value
// always looks the same.
export const GOOD_REPROJ_ERROR_PX = 0.5

export type CalibrationQuality = 'good' | 'poor' | 'unknown'

export function calibrationQuality(reprojErrorPx: number | null): CalibrationQuality {
  if (reprojErrorPx == null) return 'unknown'
  return reprojErrorPx <= GOOD_REPROJ_ERROR_PX ? 'good' : 'poor'
}

export function calibrationQualityColor(q: CalibrationQuality): string {
  switch (q) {
    case 'good':    return '#15803d'
    case 'poor':    return '#b91c1c'
    case 'unknown': return '#6b7280'
  }
}

export interface SettingRange {
  min: number
  max: number
  unit: string
}

export interface CameraSettingsLimits {
  gain: SettingRange | null
  exposure: SettingRange | null
}

export interface CameraSettingsPatch {
  gain_auto?: boolean
  gain?: number
  exposure_auto?: boolean
  exposure?: number
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

export async function getCamera(id: number): Promise<Camera> {
  return asJson<Camera>(await fetch(`/api/cameras/${id}`))
}

export async function listAvailableCameras(): Promise<AvailableCamera[]> {
  return asJson<AvailableCamera[]>(await fetch('/api/cameras/available'))
}

export async function createCamera(input: {
  name: string
  serial: string
  focal_length_mm: number
  mode?: string
}): Promise<Camera> {
  const body: Record<string, unknown> = {
    name: input.name,
    serial: input.serial,
    focal_length_mm: input.focal_length_mm,
  }
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
  patch: { name?: string; mode?: string; focal_length_mm?: number } & CameraSettingsPatch,
): Promise<Camera> {
  const res = await fetch(`/api/cameras/${id}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patch),
  })
  return asJson<Camera>(res)
}

export async function getCameraSettingsLimits(
  id: number,
): Promise<CameraSettingsLimits> {
  const res = await fetch(`/api/cameras/${id}/settings/limits`)
  if (res.status === 409) throw new CameraOfflineError()
  return asJson<CameraSettingsLimits>(res)
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
