import { useEffect, useMemo, useState } from 'react'
import { fetchStatus, type Status } from '../api/status'
import Stream from '../Stream'
import CameraSettingsPanel from '../components/CameraSettingsPanel'

const inputStyle: React.CSSProperties = {
  padding: '6px 10px',
  fontSize: 14,
  border: '1px solid #ccc',
  borderRadius: 6,
  background: '#fff',
}

export default function StreamPage() {
  const [status, setStatus] = useState<Status | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [selectedId, setSelectedId] = useState<number | null>(null)

  useEffect(() => {
    let cancelled = false
    const tick = async () => {
      try {
        const s = await fetchStatus()
        if (!cancelled) {
          setStatus(s)
          setError(null)
        }
      } catch (e) {
        if (!cancelled) setError(e instanceof Error ? e.message : String(e))
      }
    }
    tick()
    const id = setInterval(tick, 1000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [])

  const onlineCameras = useMemo(
    () => (status?.cameras ?? []).filter((c) => c.online),
    [status]
  )

  // Keep the dropdown selection valid: pick the first online camera by default,
  // and reset if the current selection goes offline.
  useEffect(() => {
    if (onlineCameras.length === 0) {
      if (selectedId !== null) setSelectedId(null)
      return
    }
    if (selectedId === null || !onlineCameras.some((c) => c.id === selectedId)) {
      setSelectedId(onlineCameras[0].id)
    }
  }, [onlineCameras, selectedId])

  return (
    <>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12, marginBottom: 16 }}>
        <label htmlFor="camera-select" style={{ fontWeight: 600 }}>
          Camera:
        </label>
        <select
          id="camera-select"
          style={inputStyle}
          value={selectedId ?? ''}
          onChange={(e) => setSelectedId(Number(e.target.value))}
          disabled={onlineCameras.length === 0}
        >
          {onlineCameras.length === 0 ? (
            <option value="">No cameras online</option>
          ) : (
            onlineCameras.map((c) => (
              <option key={c.id} value={c.id}>
                {c.name} ({c.serial})
              </option>
            ))
          )}
        </select>
      </div>

      {selectedId !== null ? (
        <div
          style={{
            display: 'flex',
            gap: 16,
            alignItems: 'flex-start',
            flexWrap: 'wrap',
            marginBottom: 24,
          }}
        >
          <div style={{ flex: '0 1 auto' }}>
            <Stream cameraId={selectedId} />
          </div>
          <div style={{ flex: '1 1 320px', maxWidth: 360 }}>
            <CameraSettingsPanel cameraId={selectedId} />
          </div>
        </div>
      ) : (
        <p style={{ color: '#666' }}>
          No cameras online. Add one on the <a href="/cameras">Cameras</a> page or plug a camera in.
        </p>
      )}

      <h2 style={{ marginBottom: 8 }}>Status</h2>
      {error && (
        <p style={{ color: 'crimson' }}>Failed to fetch /api/status: {error}</p>
      )}
      {status ? (
        <pre style={{ background: '#f4f4f4', padding: 16, borderRadius: 8 }}>
          {JSON.stringify(status, null, 2)}
        </pre>
      ) : (
        !error && <p>Loading…</p>
      )}
    </>
  )
}
