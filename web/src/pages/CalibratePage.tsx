import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Link, useParams } from 'react-router-dom'
import {
  deleteCalibration,
  getCalibration,
  getRecording,
  startRecording,
  stopRecording,
  uploadCalibration,
  type CalibrationDocument,
  type CameraCalibration,
  type RecordingResult,
  type RecordingStatus,
} from '../api/calibration'
import { getCamera, type Camera } from '../api/cameras'
import Stream from '../Stream'
import { dangerButtonStyle, neutralButtonStyle, primaryButtonStyle } from '../components/buttonStyles'

const cardStyle: React.CSSProperties = {
  border: '1px solid #e5e7eb',
  borderRadius: 8,
  padding: 16,
  marginBottom: 16,
  background: '#fff',
}

const codeBlockStyle: React.CSSProperties = {
  background: '#0b1020',
  color: '#e5e7eb',
  borderRadius: 6,
  padding: 12,
  fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
  fontSize: 13,
  whiteSpace: 'pre-wrap',
  wordBreak: 'break-all',
}

function formatMs(ms: number): string {
  const total = Math.floor(ms / 1000)
  const m = Math.floor(total / 60)
  const s = total % 60
  return `${m}m ${s.toString().padStart(2, '0')}s`
}

// Best-effort intrinsics summary for the "current calibration" card. Walks the
// basalt_calibrate JSON for a recognisable shape; falls back to a row count.
function summariseCalibration(doc: CalibrationDocument | string | null): {
  model?: string
  width?: number
  height?: number
  fx?: number
  fy?: number
  cx?: number
  cy?: number
} | null {
  if (!doc || typeof doc === 'string') return null
  // Basalt wraps everything under "value0" via cereal.
  const v0 = (doc as Record<string, unknown>)['value0']
  const root = (v0 && typeof v0 === 'object') ? (v0 as Record<string, unknown>) : doc
  const intrinsics = (root['intrinsics'] as unknown[]) || []
  const resolution = (root['resolution'] as unknown[]) || []
  if (!Array.isArray(intrinsics) || intrinsics.length === 0) return null
  const first = intrinsics[0] as Record<string, unknown> | undefined
  if (!first || typeof first !== 'object') return null
  const intr = (first['intrinsics'] as Record<string, unknown> | undefined) || first
  const cam0Res = Array.isArray(resolution) && Array.isArray(resolution[0]) ? resolution[0] : null
  return {
    model: typeof first['camera_type'] === 'string' ? (first['camera_type'] as string) : undefined,
    width: cam0Res ? Number(cam0Res[0]) : undefined,
    height: cam0Res ? Number(cam0Res[1]) : undefined,
    fx: typeof intr['fx'] === 'number' ? (intr['fx'] as number) : undefined,
    fy: typeof intr['fy'] === 'number' ? (intr['fy'] as number) : undefined,
    cx: typeof intr['cx'] === 'number' ? (intr['cx'] as number) : undefined,
    cy: typeof intr['cy'] === 'number' ? (intr['cy'] as number) : undefined,
  }
}

export default function CalibratePage() {
  const { id } = useParams<{ id: string }>()
  const cameraId = id ? Number(id) : NaN

  const [camera, setCamera] = useState<Camera | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)

  const [recording, setRecording] = useState<RecordingStatus | null>(null)
  const [lastResult, setLastResult] = useState<RecordingResult | null>(null)
  const [recordError, setRecordError] = useState<string | null>(null)
  const [recordBusy, setRecordBusy] = useState(false)

  const [calibration, setCalibration] = useState<CameraCalibration | null>(null)
  const [calibError, setCalibError] = useState<string | null>(null)
  const [calibBusy, setCalibBusy] = useState(false)

  const fileRef = useRef<HTMLInputElement>(null)
  const [copied, setCopied] = useState(false)

  // Initial load — camera, current recording (if any), current calibration.
  useEffect(() => {
    if (Number.isNaN(cameraId)) {
      setLoadError('invalid camera id')
      return
    }
    let cancelled = false
    ;(async () => {
      try {
        const [c, rec, cal] = await Promise.all([
          getCamera(cameraId),
          getRecording(cameraId),
          getCalibration(cameraId),
        ])
        if (cancelled) return
        setCamera(c)
        setRecording(rec)
        setCalibration(cal)
      } catch (e) {
        if (!cancelled) {
          setLoadError(e instanceof Error ? e.message : String(e))
        }
      }
    })()
    return () => { cancelled = true }
  }, [cameraId])

  // Poll the recording status while a session is active. Keyed on the boolean
  // so per-tick status updates don't tear down and re-create the interval.
  const isRecording = recording !== null
  useEffect(() => {
    if (!isRecording) return
    let cancelled = false
    const tick = async () => {
      try {
        const r = await getRecording(cameraId)
        if (!cancelled) setRecording(r)
      } catch {
        // ignore transient errors; the next tick may succeed
      }
    }
    const t = setInterval(tick, 500)
    return () => { cancelled = true; clearInterval(t) }
  }, [isRecording, cameraId])

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
    setLastResult(null)
    runRecord(async () => { setRecording(await startRecording(cameraId)) })
  }

  const onStop = () => {
    if (recordBusy) return
    runRecord(async () => {
      setLastResult(await stopRecording(cameraId))
      setRecording(null)
    })
  }

  const onCopyCommand = async () => {
    if (!lastResult) return
    try {
      await navigator.clipboard.writeText(lastResult.suggested_command)
      setCopied(true)
      setTimeout(() => setCopied(false), 1500)
    } catch {
      setRecordError('copy to clipboard failed (clipboard permissions?)')
    }
  }

  const onUploadFile = (file: File) => {
    if (calibBusy) return
    runCalib(async () => {
      const parsed = JSON.parse(await file.text()) as CalibrationDocument
      setCalibration(await uploadCalibration(cameraId, parsed))
      if (fileRef.current) fileRef.current.value = ''
    })
  }

  const onClearCalibration = () => {
    if (calibBusy) return
    runCalib(async () => {
      await deleteCalibration(cameraId)
      setCalibration(null)
    })
  }

  const summary = useMemo(
    () => (calibration ? summariseCalibration(calibration.calibration) : null),
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
          covers the whole frame. 30–90 seconds is typical.
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
            disabled={recordBusy || !camera.online}
          >
            Start recording
          </button>
        )}
        {recordError && (
          <p style={{ color: 'crimson', marginTop: 8 }}>{recordError}</p>
        )}
      </div>

      {lastResult && (
        <div style={cardStyle}>
          <h3 style={{ marginTop: 0 }}>2. Run Basalt</h3>
          <p style={{ color: '#4b5563', marginTop: 0 }}>
            Run this command in a terminal. Basalt's calibration GUI will open; follow the
            on-screen steps until it writes <code>calibration.json</code> to the result path.
          </p>
          <pre style={codeBlockStyle}>{lastResult.suggested_command}</pre>
          <div style={{ display: 'flex', gap: 8, alignItems: 'center', marginTop: 8 }}>
            <button type="button" style={neutralButtonStyle} onClick={onCopyCommand}>
              {copied ? 'Copied!' : 'Copy command'}
            </button>
            <span style={{ color: '#6b7280', fontSize: 13 }}>
              Dataset path: <code>{lastResult.path}</code>
              {' '}({lastResult.frames_written} frames)
            </span>
          </div>
        </div>
      )}

      <div style={cardStyle}>
        <h3 style={{ marginTop: 0 }}>3. Upload calibration.json</h3>
        <p style={{ color: '#4b5563', marginTop: 0 }}>
          After <code>basalt_calibrate</code> finishes, pick the resulting
          {' '}<code>calibration.json</code> from the result directory.
        </p>
        <input
          ref={fileRef}
          type="file"
          accept="application/json,.json"
          onChange={(e) => {
            const f = e.target.files?.[0]
            if (f) onUploadFile(f)
          }}
          disabled={calibBusy}
        />
        {calibError && (
          <p style={{ color: 'crimson', marginTop: 8 }}>{calibError}</p>
        )}
      </div>

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
            <button
              type="button"
              style={dangerButtonStyle}
              onClick={onClearCalibration}
              disabled={calibBusy}
            >
              Delete calibration
            </button>
          </>
        ) : (
          <p style={{ color: '#6b7280', margin: 0 }}>No calibration uploaded yet.</p>
        )}
      </div>
    </div>
  )
}
