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

// basalt_calibrate's output JSON. We don't enforce the full schema on the
// client — the server stores whatever we PUT and parses it back on GET.
export type CalibrationDocument = Record<string, unknown>

export interface CameraCalibration {
  camera_id: number
  calibrated_at: number | null
  calibration: CalibrationDocument | string | null
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
  document: CalibrationDocument,
): Promise<CameraCalibration> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(document),
  })
  return asJson<CameraCalibration>(res)
}

export async function deleteCalibration(cameraId: number): Promise<void> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration`, {
    method: 'DELETE',
  })
  if (!res.ok) throw new Error(await readError(res))
}
