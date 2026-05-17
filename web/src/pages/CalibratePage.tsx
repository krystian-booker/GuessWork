import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Link, useParams } from 'react-router-dom'
import yaml from 'js-yaml'
import {
  cancelJob,
  deleteCalibration,
  getCalibration,
  getJob,
  getRecording,
  startRecording,
  stopRecording,
  streamJobLog,
  type CalibrationJob,
  type CalibrationYaml,
  type CameraCalibration,
  type JobState,
  type KalibrCamchain,
  type RecordingStatus,
} from '../api/calibration'
import {
  calibrationQuality,
  calibrationQualityColor,
  getCamera,
  type Camera,
} from '../api/cameras'
import Stream from '../Stream'
import { dangerButtonStyle, primaryButtonStyle } from '../components/buttonStyles'

const cardStyle: React.CSSProperties = {
  border: '1px solid #e5e7eb',
  borderRadius: 8,
  padding: 16,
  marginBottom: 16,
  background: '#fff',
}

const logBlockStyle: React.CSSProperties = {
  background: '#0b1020',
  color: '#e5e7eb',
  borderRadius: 6,
  padding: 12,
  fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
  fontSize: 12,
  lineHeight: 1.45,
  whiteSpace: 'pre-wrap',
  wordBreak: 'break-all',
  maxHeight: 360,
  overflow: 'auto',
  margin: 0,
  marginTop: 8,
}

function formatMs(ms: number): string {
  const total = Math.floor(ms / 1000)
  const m = Math.floor(total / 60)
  const s = total % 60
  return `${m}m ${s.toString().padStart(2, '0')}s`
}

function jobElapsedMs(job: CalibrationJob, nowMs: number): number {
  if (job.state === 'running') return Math.max(0, nowMs - job.started_at_ms)
  return Math.max(0, job.ended_at_ms - job.started_at_ms)
}

function jobStateLabel(state: JobState): string {
  switch (state) {
    case 'pending':   return 'Pending'
    case 'running':   return 'Running'
    case 'succeeded': return 'Succeeded'
    case 'failed':    return 'Failed'
    case 'cancelled': return 'Cancelled'
  }
}

function ReprojErrorLine({ reprojErrorPx }: { reprojErrorPx: number }) {
  const q = calibrationQuality(reprojErrorPx)
  const label = q === 'good'
    ? '✓ Good calibration'
    : '⚠ Poor calibration — consider re-recording'
  return (
    <p style={{ marginTop: 8 }}>
      <strong style={{ color: calibrationQualityColor(q) }}>{label}</strong>
      {' — reprojection error '}
      <code>{reprojErrorPx.toFixed(3)} px</code>
      {' '}<span style={{ color: '#6b7280', fontSize: 13 }}>
        (RMS sigma; rule of thumb: ≤ 0.5 px is good)
      </span>
    </p>
  )
}

function jobStateColor(state: JobState): string {
  switch (state) {
    case 'running':   return '#1d4ed8'
    case 'succeeded': return '#15803d'
    case 'failed':    return '#b91c1c'
    case 'cancelled': return '#7c2d12'
    case 'pending':   return '#6b7280'
  }
}

// Subset of the guesswork_meta block KalibrJob writes into the camchain
// alongside the standard Kalibr keys. Absent for pre-enrichment uploads.
interface GuessworkMeta {
  reprojection_error_px?: number
  reprojection_error_u_px?: number
  reprojection_error_v_px?: number
}

// Best-effort intrinsics summary for the "current calibration" card; cam0
// only (stereo entries are out of scope for the mono-intrinsic PoC).
function summariseCamchain(text: CalibrationYaml | null): {
  model?: string
  distortion_model?: string
  width?: number
  height?: number
  fx?: number
  fy?: number
  cx?: number
  cy?: number
  distortion?: number[]
  reproj_error_px?: number
} | null {
  if (!text) return null
  let doc: KalibrCamchain & { guesswork_meta?: GuessworkMeta }
  try {
    doc = yaml.load(text) as KalibrCamchain & { guesswork_meta?: GuessworkMeta }
  } catch {
    return null
  }
  const c = doc?.cam0
  if (!c) return null
  const intr = c.intrinsics ?? []
  const res  = c.resolution ?? []
  const dist = Array.isArray(c.distortion_coeffs) ? c.distortion_coeffs : undefined
  const meta = doc.guesswork_meta
  return {
    model:            c.camera_model,
    distortion_model: c.distortion_model,
    width:            typeof res[0] === 'number' ? res[0] : undefined,
    height:           typeof res[1] === 'number' ? res[1] : undefined,
    fx: typeof intr[0] === 'number' ? intr[0] : undefined,
    fy: typeof intr[1] === 'number' ? intr[1] : undefined,
    cx: typeof intr[2] === 'number' ? intr[2] : undefined,
    cy: typeof intr[3] === 'number' ? intr[3] : undefined,
    distortion: dist,
    reproj_error_px: typeof meta?.reprojection_error_px === 'number'
                       ? meta.reprojection_error_px
                       : undefined,
  }
}

export default function CalibratePage() {
  const { id } = useParams<{ id: string }>()
  const cameraId = id ? Number(id) : NaN

  const [camera, setCamera] = useState<Camera | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)

  const [recording, setRecording] = useState<RecordingStatus | null>(null)
  const [recordError, setRecordError] = useState<string | null>(null)
  const [recordBusy, setRecordBusy] = useState(false)

  const [job, setJob] = useState<CalibrationJob | null>(null)
  const [jobLog, setJobLog] = useState<string>('')
  const [jobError, setJobError] = useState<string | null>(null)
  const [nowMs, setNowMs] = useState<number>(() => Date.now())

  const [calibration, setCalibration] = useState<CameraCalibration | null>(null)
  const [calibError, setCalibError] = useState<string | null>(null)
  const [calibBusy, setCalibBusy] = useState(false)

  const logRef = useRef<HTMLPreElement | null>(null)

  useEffect(() => {
    if (Number.isNaN(cameraId)) {
      setLoadError('invalid camera id')
      return
    }
    let cancelled = false
    ;(async () => {
      try {
        const [c, rec, j, cal] = await Promise.all([
          getCamera(cameraId),
          getRecording(cameraId),
          getJob(cameraId),
          getCalibration(cameraId),
        ])
        if (cancelled) return
        setCamera(c)
        setRecording(rec)
        setJob(j)
        setCalibration(cal)
      } catch (e) {
        if (!cancelled) {
          setLoadError(e instanceof Error ? e.message : String(e))
        }
      }
    })()
    return () => { cancelled = true }
  }, [cameraId])

  const isRecording = recording !== null
  useEffect(() => {
    if (!isRecording) return
    let cancelled = false
    const tick = async () => {
      try {
        const r = await getRecording(cameraId)
        if (!cancelled) setRecording(r)
      } catch {
        // transient — next tick may succeed
      }
    }
    const t = setInterval(tick, 500)
    return () => { cancelled = true; clearInterval(t) }
  }, [isRecording, cameraId])

  const isJobActive = job?.state === 'running' || job?.state === 'pending'
  useEffect(() => {
    if (!isJobActive) return
    const t = setInterval(() => setNowMs(Date.now()), 1000)
    return () => clearInterval(t)
  }, [isJobActive])

  useEffect(() => {
    if (!job || job.state !== 'running') return
    const close = streamJobLog(cameraId, {
      onChunk: (chunk) => {
        setJobLog((prev) => prev + chunk)
      },
      onDone: async (summary) => {
        setJob((prev) =>
          prev
            ? {
                ...prev,
                state: summary.state,
                exit_code: summary.exit_code,
                calibration_stored: summary.calibration_stored,
                upload_error: summary.upload_error,
                ended_at_ms: Date.now(),
              }
            : prev,
        )
        if (summary.state === 'succeeded' && summary.calibration_stored) {
          try {
            setCalibration(await getCalibration(cameraId))
          } catch (e) {
            setJobError(e instanceof Error ? e.message : String(e))
          }
        }
      },
      // EventSource retries transient blips itself. Persistent failures
      // remain visible as a stuck "Running" — the page-load effect on
      // navigation back reconciles.
      onError: () => {},
    })
    return close
  }, [job?.state, cameraId])

  // Auto-scroll the log on new content unless the user has scrolled up.
  useEffect(() => {
    const el = logRef.current
    if (!el) return
    const distanceFromBottom = el.scrollHeight - el.scrollTop - el.clientHeight
    if (distanceFromBottom < 100) {
      el.scrollTop = el.scrollHeight
    }
  }, [jobLog])

  const runAction = useCallback(
    (setBusy: (b: boolean) => void, setError: (m: string | null) => void) =>
      async (fn: () => Promise<void>) => {
        setBusy(true)
        setError(null)
        try {
          await fn()
        } catch (e) {
          setError(e instanceof Error ? e.message : String(e))
        } finally {
          setBusy(false)
        }
      },
    [],
  )
  const runRecord = runAction(setRecordBusy, setRecordError)
  const runCalib  = runAction(setCalibBusy,  setCalibError)

  const onStart = () => {
    if (recordBusy) return
    // Starting a new recording invalidates any prior job result the page
    // was displaying — clear the log so it doesn't bleed into the next run.
    setJob(null)
    setJobLog('')
    setJobError(null)
    runRecord(async () => { setRecording(await startRecording(cameraId)) })
  }

  const onStop = () => {
    if (recordBusy) return
    runRecord(async () => {
      const resp = await stopRecording(cameraId)
      setRecording(null)
      if (resp.job) {
        setJobLog('')
        setJob(resp.job)
      } else if (resp.job_error) {
        setJobError(resp.job_error)
      }
    })
  }

  const onCancelJob = () => {
    if (!job || job.state !== 'running') return
    ;(async () => {
      try {
        await cancelJob(cameraId)
      } catch (e) {
        setJobError(e instanceof Error ? e.message : String(e))
      }
    })()
  }

  const onClearCalibration = () => {
    if (calibBusy) return
    runCalib(async () => {
      await deleteCalibration(cameraId)
      setCalibration(null)
    })
  }

  const summary = useMemo(
    () => (calibration ? summariseCamchain(calibration.calibration) : null),
    [calibration],
  )

  if (Number.isNaN(cameraId)) {
    return <p style={{ color: 'crimson' }}>Invalid camera id.</p>
  }
  if (loadError) {
    return <p style={{ color: 'crimson' }}>{loadError}</p>
  }
  if (!camera) {
    return <p>Loading…</p>
  }

  return (
    <div>
      <div style={{ display: 'flex', alignItems: 'baseline', gap: 12, marginBottom: 16 }}>
        <h2 style={{ margin: 0 }}>Calibrate: {camera.name}</h2>
        <Link to="/cameras" style={{ fontSize: 14 }}>← Cameras</Link>
      </div>

      {camera.online ? (
        <Stream cameraId={cameraId} />
      ) : (
        <div style={cardStyle}>
          <p style={{ margin: 0, color: '#9a3412' }}>
            Camera is offline. Plug it in or check connections — you can't record without a live feed.
          </p>
        </div>
      )}

      <div style={cardStyle}>
        <h3 style={{ marginTop: 0 }}>1. Record an AprilGrid sequence</h3>
        <p style={{ color: '#4b5563', marginTop: 0 }}>
          Move the camera around an AprilGrid target so the pattern is seen from many angles and
          covers the whole frame. 30–90 seconds is typical. Stopping the recording automatically
          launches Kalibr.
        </p>
        {recording ? (
          <>
            <p style={{ margin: '8px 0' }}>
              <strong>Recording…</strong> session <code>{recording.session_id}</code>
              {' '}— {formatMs(recording.elapsed_ms)} —
              {' '}{recording.frames_written} frames
              {recording.frames_dropped > 0 && (
                <span style={{ color: '#b45309' }}>{' '}({recording.frames_dropped} dropped)</span>
              )}
            </p>
            <button type="button" style={dangerButtonStyle} onClick={onStop} disabled={recordBusy}>
              Stop recording
            </button>
          </>
        ) : (
          <button
            type="button"
            style={primaryButtonStyle}
            onClick={onStart}
            disabled={recordBusy || !camera.online || isJobActive}
          >
            Start recording
          </button>
        )}
        {recordError && (
          <p style={{ color: 'crimson', marginTop: 8 }}>{recordError}</p>
        )}
      </div>

      {job && (
        <div style={cardStyle}>
          <h3 style={{ marginTop: 0 }}>2. Kalibr</h3>
          <p style={{ margin: '4px 0' }}>
            <strong style={{ color: jobStateColor(job.state) }}>
              {jobStateLabel(job.state)}
            </strong>
            {' — '}
            {formatMs(jobElapsedMs(job, nowMs))}
            {' · '}
            model <code>{job.model}</code>
            {' · '}
            {jobLog.length.toLocaleString()} bytes of log
          </p>
          {job.state === 'succeeded' && job.calibration_stored && (
            <p style={{ color: '#15803d', margin: '4px 0' }}>
              Calibration auto-uploaded — see card below.
            </p>
          )}
          {job.state === 'succeeded' && !job.calibration_stored && (
            <p style={{ color: '#b91c1c', margin: '4px 0' }}>
              Kalibr exited cleanly but the camchain wasn't stored
              {job.upload_error ? <>: {job.upload_error}</> : '.'}
            </p>
          )}
          {job.state === 'failed' && (
            <p style={{ color: '#b91c1c', margin: '4px 0' }}>
              Exit code {job.exit_code}
              {job.upload_error ? <> · {job.upload_error}</> : ''}
              {' '}— scan the log below for the underlying error.
            </p>
          )}
          {job.state === 'cancelled' && (
            <p style={{ color: '#7c2d12', margin: '4px 0' }}>
              Cancelled by user; no calibration was stored.
            </p>
          )}
          {jobError && (
            <p style={{ color: 'crimson', margin: '4px 0' }}>{jobError}</p>
          )}

          {job.state === 'running' && (
            <button type="button" style={dangerButtonStyle} onClick={onCancelJob}>
              Cancel
            </button>
          )}

          <details style={{ marginTop: 12 }} open={job.state !== 'running'}>
            <summary style={{ cursor: 'pointer', color: '#4b5563', fontSize: 14 }}>
              Show container log
            </summary>
            <pre ref={logRef} style={logBlockStyle}>
              {jobLog || '(no output yet)'}
            </pre>
          </details>
        </div>
      )}

      <div style={cardStyle}>
        <h3 style={{ marginTop: 0 }}>Current calibration</h3>
        {calibration ? (
          <>
            <p style={{ marginTop: 0 }}>
              {calibration.calibrated_at ? (
                <>Uploaded <strong>
                  {new Date(calibration.calibrated_at * 1000).toLocaleString()}
                </strong></>
              ) : (
                'Uploaded'
              )}
              {summary?.model && <> · model <code>{summary.model}</code></>}
              {summary?.width != null && summary?.height != null &&
                <> · {summary.width}×{summary.height}</>}
            </p>
            {summary && (summary.fx != null || summary.cx != null) && (
              <p style={{ color: '#4b5563', marginTop: 4 }}>
                fx={summary.fx?.toFixed(2)}, fy={summary.fy?.toFixed(2)},
                cx={summary.cx?.toFixed(2)}, cy={summary.cy?.toFixed(2)}
              </p>
            )}
            {summary?.distortion && summary.distortion.length > 0 && (
              <p style={{ color: '#4b5563', marginTop: 4 }}>
                distortion ({summary.distortion_model ?? 'unknown'}):
                {' '}[{summary.distortion.map((d) => d.toFixed(4)).join(', ')}]
              </p>
            )}
            {summary?.reproj_error_px != null && (
              <ReprojErrorLine reprojErrorPx={summary.reproj_error_px} />
            )}
            <button
              type="button"
              style={dangerButtonStyle}
              onClick={onClearCalibration}
              disabled={calibBusy}
            >
              Delete calibration
            </button>
            {calibError && (
              <p style={{ color: 'crimson', marginTop: 8 }}>{calibError}</p>
            )}
          </>
        ) : (
          <p style={{ color: '#6b7280', margin: 0 }}>No calibration uploaded yet.</p>
        )}
      </div>
    </div>
  )
}
