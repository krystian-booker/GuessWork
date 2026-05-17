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

// State machine for the auto-launched Kalibr subprocess. Mirrors the
// SubprocessState enum on the server.
export type JobState = 'pending' | 'running' | 'succeeded' | 'failed' | 'cancelled'

export interface CalibrationJob {
  state: JobState
  model: string                       // "pinhole-radtan" or "pinhole-equi"
  exit_code: number
  started_at_ms: number
  ended_at_ms: number
  log_bytes: number
  calibration_stored: boolean
  upload_error: string | null         // set when state==failed *because* the
                                      // post-run camchain upload step failed
}

// Server response when stopping a recording — the previous "RecordingResult"
// shape lives under `recording_result`, plus an initial CalibrationJob.
// `job_error` is populated only when start_kalibr_job rejected (e.g. another
// job is already running for a different camera); in that case `job` is null.
export interface StopRecordingResponse {
  recording_result: RecordingResult
  job: CalibrationJob | null
  job_error?: string
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
): Promise<StopRecordingResponse> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/recording`, {
    method: 'DELETE',
  })
  return asJson<StopRecordingResponse>(res)
}

// Polled status snapshot for an in-progress Kalibr job. Returns null if no
// job is active for this camera (server replies 404).
export async function getJob(
  cameraId: number,
): Promise<CalibrationJob | null> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/job`)
  if (res.status === 404) return null
  return asJson<CalibrationJob>(res)
}

// Returns true if a cancel was issued, false if there was nothing to cancel.
export async function cancelJob(cameraId: number): Promise<boolean> {
  const res = await fetch(`/api/cameras/${cameraId}/calibration/job`, {
    method: 'DELETE',
  })
  if (res.status === 404) return false
  if (!res.ok) throw new Error(await readError(res))
  return true
}

export interface JobLogStreamHandlers {
  onChunk: (chunk: string) => void
  // Fires once when the server sends `event: done`. Includes the terminal
  // state + whether the camchain was auto-uploaded. After this fires the
  // EventSource is closed; subsequent invocations are not delivered.
  onDone: (summary: {
    state: JobState
    exit_code: number
    calibration_stored: boolean
    upload_error: string | null
  }) => void
  // EventSource error (network blip, server restart). Caller decides whether
  // to retry — `close()` is still safe to call.
  onError?: (e: Event) => void
}

// Opens an EventSource subscribing to the job's log stream. Returns a close()
// function the caller MUST invoke on unmount (or when the job ends, but
// onDone also closes internally).
export function streamJobLog(
  cameraId: number,
  handlers: JobLogStreamHandlers,
): () => void {
  const es = new EventSource(`/api/cameras/${cameraId}/calibration/job/log`)
  // Default `message` events carry log chunks (the server emits `data: …`
  // without an explicit `event:` header for log frames).
  es.onmessage = (e) => {
    handlers.onChunk(e.data)
  }
  es.addEventListener('done', (e) => {
    try {
      const summary = JSON.parse((e as MessageEvent).data)
      handlers.onDone(summary)
    } catch {
      // If the done payload is malformed (shouldn't happen), close anyway so
      // we don't keep the connection open.
      handlers.onDone({
        state: 'failed',
        exit_code: -1,
        calibration_stored: false,
        upload_error: 'malformed done event from server',
      })
    } finally {
      es.close()
    }
  })
  if (handlers.onError) {
    es.onerror = (e) => handlers.onError!(e)
  }
  return () => es.close()
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
