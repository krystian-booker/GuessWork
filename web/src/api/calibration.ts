import { asJson, readError } from './http'

export interface RecordingStatus {
  session_id: string
  path: string
  frames_written: number
  frames_dropped: number
  elapsed_ms: number
}

export interface RecordingResult extends RecordingStatus {
  suggested_command: string
}

// Raw Kalibr camchain YAML text. The server stores and returns it verbatim;
// the UI parses with js-yaml on demand for display purposes.
export type CalibrationYaml = string

export interface CameraCalibration {
  camera_id: number
  calibrated_at: number | null
  calibration: CalibrationYaml | null
}

// Subset of a Kalibr camchain.yaml that the UI displays. The on-disk shape
// can include more cameras, baselines, etc. — we only read cam0 for the
// mono-intrinsic PoC.
export interface KalibrCameraEntry {
  camera_model?: string                            // e.g. "pinhole"
  distortion_model?: string                        // e.g. "radtan", "equidistant"
  intrinsics?: [number, number, number, number]    // fx, fy, cx, cy
  distortion_coeffs?: number[]
  resolution?: [number, number]
  rostopic?: string
}

export interface KalibrCamchain {
  cam0?: KalibrCameraEntry
}

export async function startRecording(
  cameraId: number,
): Promise<RecordingStatus> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/recording`, {
    method: 'POST',
  })
  return asJson<RecordingStatus>(res)
}

// Returns null when no session is active (server replies 404). All other
// errors surface as exceptions.
export async function getRecording(
  cameraId: number,
): Promise<RecordingStatus | null> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/recording`)
  if (res.status === 404) return null
  return asJson<RecordingStatus>(res)
}

export async function stopRecording(
  cameraId: number,
): Promise<RecordingResult> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/recording`, {
    method: 'DELETE',
  })
  return asJson<RecordingResult>(res)
}

// Returns null when the camera has no calibration uploaded (server replies
// 404 with "camera is not calibrated").
export async function getCalibration(
  cameraId: number,
): Promise<CameraCalibration | null> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration`)
  if (res.status === 404) return null
  return asJson<CameraCalibration>(res)
}

export async function uploadCalibration(
  cameraId: number,
  yaml: string,
): Promise<CameraCalibration> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/x-yaml' },
    body: yaml,
  })
  return asJson<CameraCalibration>(res)
}

export async function deleteCalibration(cameraId: number): Promise<void> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration`, {
    method: 'DELETE',
  })
  if (!res.ok) throw new Error(await readError(res))
}
