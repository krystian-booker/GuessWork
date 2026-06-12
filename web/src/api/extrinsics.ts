import { getJsonOrNull, readError, send, sendJson, ApiError } from './http'
import type { CalibrationJob } from './calibration'

export interface ExtrinsicsCameraStat {
  camera_id: number
  topic: string
  frames_written: number
  frames_dropped: number
}

export interface ExtrinsicsRecordingStatus {
  session_id: string
  path: string
  cameras: ExtrinsicsCameraStat[]
  imu_written: number
  imu_dropped: number
  elapsed_ms: number
}

export interface ExtrinsicsRecordingResult extends ExtrinsicsRecordingStatus {
  model: string
  suggested_command: string
}

export interface StopExtrinsicsResponse {
  recording_result: ExtrinsicsRecordingResult
  job: CalibrationJob | null
  job_error?: string
}

// camera_ids order matters: camera_ids[i] records to /cam<i>/image_raw.
export async function startExtrinsicsRecording(
  cameraIds: number[],
): Promise<ExtrinsicsRecordingStatus> {
  return sendJson('/api/calibration/extrinsics/recording', 'POST', { camera_ids: cameraIds })
}

export async function getExtrinsicsRecording(): Promise<ExtrinsicsRecordingStatus | null> {
  return getJsonOrNull<ExtrinsicsRecordingStatus>('/api/calibration/extrinsics/recording')
}

export async function stopExtrinsicsRecording(): Promise<StopExtrinsicsResponse> {
  return sendJson('/api/calibration/extrinsics/recording', 'DELETE')
}

export async function getExtrinsicsJob(): Promise<CalibrationJob | null> {
  return getJsonOrNull<CalibrationJob>('/api/calibration/extrinsics/job')
}

export async function cancelExtrinsicsJob(): Promise<boolean> {
  const res = await fetch('/api/calibration/extrinsics/job', { method: 'DELETE' })
  if (res.status === 404) return false
  if (!res.ok) throw new ApiError(await readError(res), res.status)
  return true
}

export const EXTRINSICS_JOB_LOG_URL = '/api/calibration/extrinsics/job/log'

export interface CameraExtrinsics {
  camera_id: number
  extrinsics_calibrated_at: number | null
  // camchain-imucam YAML re-keyed to cam0 (T_cam_imu + refined intrinsics).
  extrinsics: string | null
}

export async function getCameraExtrinsics(cameraId: number): Promise<CameraExtrinsics | null> {
  return getJsonOrNull<CameraExtrinsics>(`/api/cameras/${cameraId}/extrinsics`)
}

export async function deleteCameraExtrinsics(cameraId: number): Promise<void> {
  return send(`/api/cameras/${cameraId}/extrinsics`, 'DELETE')
}
